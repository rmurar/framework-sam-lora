#include <stdint.h>
#include <samr3.h>
#include "umm_malloc.h"

void init_system_clock(void);
void init_systick(void);

void system_init(void)
{
    /* disable the watchdog timer */
    WDT->CTRLA.reg = 0;
    while (WDT->SYNCBUSY.reg)
    {
    }

    /* Various bits in the INTFLAG register can be set to one at startup. This will ensure that these bits are cleared */
    OSCCTRL->INTFLAG.reg = OSCCTRL_INTFLAG_DFLLRDY;
    SUPC->INTFLAG.reg = SUPC_INTFLAG_BOD33RDY | SUPC_INTFLAG_BOD33DET;

    /* FLASH */
    NVMCTRL->CTRLB.bit.RWS = 3; /* wait states ? */

#if 1
    /* PERFORMANCE LEVEL 2 */
    PM->INTFLAG.reg = PM_INTFLAG_PLRDY; /* Switch to PL2 to be sure configuration of GCLK0 is safe */
    PM->PLCFG.reg = PM_PLCFG_PLSEL_PL2; /* Switch performance level = SYSTEM_PERFORMANCE_LEVEL_2 */
    while (!PM->INTFLAG.reg)
    { /* Waiting performance level ready */
    }
#endif 

    /* turn on only needed APB peripherals */
    MCLK->APBAMASK.reg =
        MCLK_APBAMASK_PM
        |MCLK_APBAMASK_MCLK
        |MCLK_APBAMASK_RSTC
        |MCLK_APBAMASK_OSCCTRL
        |MCLK_APBAMASK_OSC32KCTRL
        |MCLK_APBAMASK_SUPC
        |MCLK_APBAMASK_GCLK
        |MCLK_APBAMASK_WDT
        |MCLK_APBAMASK_RTC
        |MCLK_APBAMASK_EIC
        |MCLK_APBAMASK_PORT
        //|MCLK_APBAMASK_TAL
        ;

    init_system_clock();
    init_systick();

#if 1
    /* BOD33 disabled */
    SUPC->BOD33.reg &= ~SUPC_BOD33_ENABLE;
    /* VDDCORE is supplied BUCK converter */
    SUPC->VREG.bit.SEL = SUPC_VREG_SEL_BUCK_Val;
#endif 

#ifndef DISABLE_UMM
    umm_init();
#endif

#ifndef DISABLE_WATCHDOG
    /*is active except backup mode.*/
    WDT->CTRLA.reg = WDT_CTRLA_ENABLE;
    while (WDT->SYNCBUSY.reg)
    {
    }
#endif
}