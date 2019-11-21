#include <stdint.h>
#include <samr3.h>

void init_system_clock(void);
void init_systick(void);

void system_init(void)
{
    /* disable the watchdog timer */
    WDT->CTRLA.bit.ENABLE = 0;

    /* Various bits in the INTFLAG register can be set to one at startup. This will ensure that these bits are cleared */
    OSCCTRL->INTFLAG.reg = OSCCTRL_INTFLAG_DFLLRDY;
    SUPC->INTFLAG.reg = SUPC_INTFLAG_BOD33RDY | SUPC_INTFLAG_BOD33DET;

    /* FLASH ... TODO make the best for 48Mhz */
    NVMCTRL->CTRLB.bit.RWS = 2;         // wait states ??? at 48MHz ???
    PM->INTFLAG.reg = PM_INTFLAG_PLRDY; /* Switch to PL2 to be sure configuration of GCLK0 is safe */
    PM->PLCFG.reg = PM_PLCFG_PLSEL_PL2; /* Switch performance level = SYSTEM_PERFORMANCE_LEVEL_2 */
    while (!PM->INTFLAG.reg)
    { /* Waiting performance level ready */
    }

    init_system_clock();
    
    init_systick();

#ifndef DISABLE_WATCHDOG
    WDT->CTRLA.bit.ENABLE = 1;
#endif
}