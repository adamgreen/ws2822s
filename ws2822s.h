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
#ifndef WS2822S_H_
#define WS2822S_H_

#include <mbed.h>


// The RGB data for each LED is packed into this structure.
struct RGBData
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};


class WS2822S : public Serial
{
public:
    WS2822S(PinName tx, int baudRate = 250000);

    bool     init(uint32_t ledCount);
    void     set(const RGBData* pRGB);
    uint32_t getFrameCount();

protected:
    void addToList();
    bool initDmaChannel();
    bool initPacketBuffer(uint32_t ledCount);
    void initGPDMA();
    void startMarkTimeout();
    void markCompleteHandler();
    void waitForMarkBetweenPackets();
    void startBreakTimeout(uint32_t usecsToWait);
    void breakCompleteHandler();
    void startDmaUartTx();
    void waitForTxToComplete();
    void uartTxCompleteHandler();
    void dmaHandler(uint32_t dmaInterrupts);

    WS2822S*                m_pNext;
    uint8_t*                m_pPacket;
    LPC_GPDMACH_TypeDef*    m_pChannel;
    uint32_t                m_uartTx;
    uint32_t                m_packetSize;
    uint32_t                m_frameCount;
    uint32_t                m_minPacketMarkTime;
    uint32_t                m_minPacketBreakTime;
    uint32_t                m_dmaChannel;
    volatile bool           m_markLongEnough;
    volatile bool           m_waitingForTx;
    Timeout                 m_timeout;
    Timer                   m_timer;

    static void dmaISR();
    static WS2822S* m_pHead;
    static WS2822S* m_pTail;
    static uint32_t m_nextDmaChannel;
};

#endif // WS2822S_H_
