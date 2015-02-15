/* Copyright (C) 2015  Adam Green (https://github.com/adamgreen)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
#include <assert.h>
#include <mbed.h>
#include "dma.h"
#include "interlock.h"
#include "ws2822s.h"


// This class utilizes DMA based UART hardware to send data to WS2822S LEDs.
// It was only coded to work on the LPC1768.
#ifndef TARGET_LPC176X
    #error("This WS2822S class was only coded to work on the LPC1768.")
#endif


// Static members
// Maintain a linked list of WS2822S objects that can be checked from the global DMA ISR.
WS2822S* WS2822S::m_pHead = NULL;
WS2822S* WS2822S::m_pTail = NULL;
// DMA channel to be used for the next WS2822S object allocated.
// Since the UART is pretty slow and the channel is transmitting instead of receiving, it should use a low priority
// channel.  Channel 7 is the lowest priority but I am reserving it for memory to memory operations.
uint32_t WS2822S::m_nextDmaChannel = 6;


// Place buffers used by DMA code in separate RAM bank to optimize performance.
static __attribute__((section("AHBSRAM0"),aligned)) uint8_t g_dmaHeap0[16 * 1024];
static uint8_t* g_pDmaHeap0 = g_dmaHeap0;

static uint8_t* dmaHeap0Alloc(uint32_t size)
{
    static const uint8_t* pEnd = g_dmaHeap0 + sizeof(g_dmaHeap0);
    uint8_t*              pCurr = g_pDmaHeap0;
    uint8_t*              pNew = pCurr + size;

    if (pNew > pEnd)
        return NULL;
    g_pDmaHeap0 = pNew;
    return pCurr;
}



WS2822S::WS2822S(PinName tx, int baudRate /* = 250000 */) : Serial(tx, NC)
{
    // 8 data bits, no parity, and 2 stop bits.
    format(8, SerialBase::None, 2);
    baud(baudRate);

    // Minimum time to keep UART in break state at start of new packet is 2 frames (22 bit) times.
    m_minPacketBreakTime = 1000000 * 22 / baudRate;

    // Minimum time to keep UART in mark state between packets is 88usec + time to flush last byte out of shift
    // register (11 bit times).
    m_minPacketMarkTime = 88 + (1000000 * 11 / baudRate);

    addToList();

    m_frameCount = 0;
    m_markLongEnough = false;
    m_pPacket = NULL;
    m_pChannel = NULL;
    m_uartTx = 0;
    m_packetSize = 0;
    m_dmaChannel = 0;

    // Initialize timers.
    startMarkTimeout();
    m_timer.start();
}

void WS2822S::addToList()
{
    // Add this controller to the linked list of controllers to be checked from the DMA handler.
    m_pNext = NULL;
    if (m_pHead == NULL)
        m_pHead = this;
    else
        m_pTail->m_pNext = this;
    m_pTail = m_pNext;
}

void WS2822S::startMarkTimeout()
{
    // Want to keep UART in mark state for at least 88 usec.
    m_timeout.attach_us(this, &WS2822S::markCompleteHandler, m_minPacketMarkTime);
}

void WS2822S::markCompleteHandler()
{
    m_markLongEnough = true;
}



bool WS2822S::init(uint32_t ledCount)
{
    if (!initDmaChannel())
        return false;
    if (!initPacketBuffer(ledCount))
        return false;

    // Enable DMA for UART.
    static const uint32_t fifoEnable = (1 << 0);
    static const uint32_t dmaEnable = (1 << 3);
    _serial.uart->FCR = fifoEnable | dmaEnable;

    initGPDMA();

    return true;
}

bool WS2822S::initDmaChannel()
{
    if (m_nextDmaChannel > 7)
        return false;
    m_dmaChannel = m_nextDmaChannel--;
    m_pChannel = (LPC_GPDMACH_TypeDef*)((uint8_t*)LPC_GPDMACH0 + 0x20 * m_dmaChannel);
    m_uartTx = DMA_PERIPHERAL_UART0TX_MAT0_0 + (_serial.index * 2);
    return true;
}

bool WS2822S::initPacketBuffer(uint32_t ledCount)
{
    // Packet needs a start code frame and then 3 frames per LED (red, green, blue).
    m_packetSize = 1 + ledCount * 3;
    m_pPacket = dmaHeap0Alloc(m_packetSize);
    if (!m_pPacket)
        return false;
    memset(m_pPacket, 0, m_packetSize);
    return true;
}

void WS2822S::initGPDMA()
{
    enableGpdmaPower();
    enableGpdmaInLittleEndianMode();

    // Make sure that GPDMA is configured for UART and not TimerMatch.
    LPC_SC->DMAREQSEL &= ~(3 << (_serial.index * 2));

    NVIC_SetVector(DMA_IRQn, (uint32_t)WS2822S::dmaISR);
    NVIC_EnableIRQ(DMA_IRQn);
}

void WS2822S::dmaISR(void)
{
    WS2822S* pCurr = m_pHead;
    uint32_t dmaInterrupts = LPC_GPDMA->DMACIntStat;

    // Walk the linked list of registers WS2822S objects and let each one check to see if its DMA channel has completed.
    while (pCurr)
    {
        pCurr->dmaHandler(dmaInterrupts);
        pCurr = pCurr->m_pNext;
    }
}

void WS2822S::dmaHandler(uint32_t dmaInterrupts)
{
    if (dmaInterrupts & (1 << m_dmaChannel))
    {
        // The UART transmit channel triggered an interupt to indicate that UART FIFO has been filled.
        // Clear the interrupts for this channel.
        LPC_GPDMA->DMACIntTCClear = 1 << m_dmaChannel;

        // Now wait for UART FIFO to empty.
        waitForTxToComplete();
    }
}

void WS2822S::waitForTxToComplete()
{
    attach(this, &WS2822S::uartTxCompleteHandler, TxIrq);
}

void WS2822S::uartTxCompleteHandler()
{
    // Now that UART transmit FIFO is empty, start the timer to make sure that there is enough MARK state between
    // packets.
    startMarkTimeout();
}



void WS2822S::set(const RGBData* pRGB)
{
    waitForMarkBetweenPackets();

    // Start break condition now.
    m_timer.reset();
    serial_break_set(&_serial);

    // Copy the RGB data into the packet, right after the first byte which is the start code frame.
    memcpy(m_pPacket + 1, pRGB, m_packetSize - 1);

    // Make sure that we have been in break state for atleast two frames (22 bits).
    uint32_t elapsedTime = m_timer.read_us();
    if (elapsedTime < m_minPacketBreakTime)
        // Need to wait for a bit longer so schedule future transmit.
        startBreakTimeout(m_minPacketBreakTime - elapsedTime);
    else
        // The copy took longer than minimum break time so just start the transmit now.
        startDmaUartTx();

    m_frameCount++;
}

void WS2822S::waitForMarkBetweenPackets()
{
    while (!m_markLongEnough)
    {
    }
    m_markLongEnough = false;
}

void WS2822S::startBreakTimeout(uint32_t usecsToWait)
{
    m_timeout.attach_us(this, &WS2822S::breakCompleteHandler, usecsToWait);
}

void WS2822S::breakCompleteHandler()
{
    // The BREAK state has been held long enough so we can start the UART transmit.
    startDmaUartTx();
}

void WS2822S::startDmaUartTx()
{
    serial_break_clear(&_serial);

    // Prep DMA channel to send bytes via the UART.
    m_pChannel->DMACCSrcAddr  = (uint32_t)m_pPacket;
    m_pChannel->DMACCDestAddr = (uint32_t)&_serial.uart->THR;
    m_pChannel->DMACCLLI      = 0;
    m_pChannel->DMACCControl  = DMACCxCONTROL_I | DMACCxCONTROL_SI |
                     (DMACCxCONTROL_BURSTSIZE_1 << DMACCxCONTROL_SBSIZE_SHIFT) |
                     (DMACCxCONTROL_BURSTSIZE_1 << DMACCxCONTROL_DBSIZE_SHIFT) |
                     (m_packetSize & DMACCxCONTROL_TRANSFER_SIZE_MASK);

    // Enable DMA channel.
    m_pChannel->DMACCConfig = DMACCxCONFIG_ENABLE |
                   (m_uartTx << DMACCxCONFIG_DEST_PERIPHERAL_SHIFT) |
                   DMACCxCONFIG_TRANSFER_TYPE_M2P |
                   DMACCxCONFIG_ITC;
}



uint32_t WS2822S::getFrameCount()
{
    uint32_t frameCount = m_frameCount;
    m_frameCount = 0;
    return frameCount;
}
