/*
  SAMR3 - Device
    Created on: 01.01.2020
    Author: Georgi Angelov

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA   
 */

#include <Arduino.h>

class Device
{
public:
    Device() {}

    inline int getSerialNumber(uint8_t *buf, uint32_t size) { return serial_number(buf, size); }

    void getSerialNumber(String &str)
    {
        uint8_t buf[16];
        char hex[3];
        str.reserve(33);
        serial_number(buf, 16);
        for (int i = 0; i < 8; i++)
        {
            sprintf(hex, "%02X", buf[i]);
            str.concat(hex);
        }
    }

    int resetFrom() { return get_reset_cause(); }

    static const char* getResetCause()
    {
        const char* retStr = "n/a";
        int cause = get_reset_cause();
        switch(cause)
        {
            case RESET_CAUSE_BACKUP: retStr = "backup reset"; break;
            case RESET_CAUSE_SOFTWARE: retStr = "software reset"; break;
            case RESET_CAUSE_WDT: retStr = "WDT"; break;
            case RESET_CAUSE_EXTERNAL_RESET: retStr=" external reset"; break;
            case RESET_CAUSE_BOD33: retStr = "BOD33"; break;
            case RESET_CAUSE_BOD12: retStr = "BOD12"; break;
            case RESET_CAUSE_POR: retStr = "POR"; break;
            default: break;
        }

        return retStr;
    }

    void reset()
    {
        delay(100);
        NVIC_SystemReset();
    }

    void sleep(bool wd = 1)
    {
        WDT->CTRLA.bit.ENABLE = wd; /* watchdog timer if need */
        delay(100);
        sys_set_sleep_mode(SLEEP_MODE_STANDBY);
        sys_sleep();
    }

    inline uint32_t speed() { return SystemCoreClock; }
};

Device dev;