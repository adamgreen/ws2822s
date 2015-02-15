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
#include <mbed.h>
#include "ws2822s.h"


#define LED_COUNT 3


DigitalOut myled(LED1);
int main()
{
    static WS2822S ledControl(p9, 250000);
    static Timer   timer;
    RGBData leds[LED_COUNT] = { {1, 1, 1}, {8, 8, 8}, {0x40, 0x40, 0x40} };

    if (!ledControl.init(LED_COUNT))
    {
        printf("error: Failed ledControl.init()\n");
        return -1;
    }

    timer.start();
    while(1)
    {
        // Dump frame count every 5 seconds.
        if (timer.read_ms() > 5000)
        {
            timer.reset();
            printf("frame count: %lu\n", ledControl.getFrameCount());
            myled = !myled;
        }

        ledControl.set(leds);
    }
}
