#include "variant.h"
#include <interface.h>

uint32_t SystemCoreClock = 4000000; // default freq;

static void waitForDFLL(void)
{
    while ((OSCCTRL->STATUS.reg & OSCCTRL_STATUS_DFLLRDY) == 0)
    {
    }
}

static void enable_XOSC32K(void)
{
    /* Enable external XOSC32K oscilator */
    OSC32KCTRL->XOSC32K.reg = (OSC32KCTRL_XOSC32K_ENABLE |
                               OSC32KCTRL_XOSC32K_STARTUP(0) |
                               OSC32KCTRL_XOSC32K_XTALEN |
                               OSC32KCTRL_XOSC32K_EN32K |
                               OSC32KCTRL_XOSC32K_EN1K); // for RTC
    while (0 == (OSC32KCTRL->STATUS.reg & OSC32KCTRL_STATUS_XOSC32KRDY))
    {
    }
}

static void _osc32k_setup(void)
{
#if INTERNAL_OSC32_SOURCE
    uint32_t * pCalibrationArea;
    uint32_t osc32kcal;

    /* Read OSC32KCAL, calibration data for OSC32 !!! */
    pCalibrationArea = (uint32_t*) NVMCTRL_OTP5;
    osc32kcal = ( (*pCalibrationArea) & 0x1FC0 ) >> 6;

    /* RTC use Low Power Internal Oscillator at 32kHz */
    OSC32KCTRL->OSC32K.reg = OSC32KCTRL_OSC32K_RUNSTDBY
                           | OSC32KCTRL_OSC32K_EN32K
                           | OSC32KCTRL_OSC32K_CALIB(osc32kcal)
                           | OSC32KCTRL_OSC32K_ENABLE;

    /* Wait OSC32K Ready */
    while (!OSC32KCTRL->STATUS.bit.OSC32KRDY) {}
#endif /* INTERNAL_OSC32_SOURCE */
}

static void _xosc32k_setup(void)
{
#if EXTERNAL_OSC32_SOURCE
    /* RTC uses External 32,768KHz Oscillator */
    OSC32KCTRL->XOSC32K.reg = OSC32KCTRL_XOSC32K_XTALEN
                            | OSC32KCTRL_XOSC32K_RUNSTDBY
                            | OSC32KCTRL_XOSC32K_EN32K
                            | OSC32KCTRL_XOSC32K_ENABLE;

    /* Wait XOSC32K Ready */
    while (!OSC32KCTRL->STATUS.bit.XOSC32KRDY) {}
#endif
}

void sam0_gclk_enable(uint8_t id)
{
    switch(id) {
        case VARIANT_GCLK_48MHZ:
            _gclk_setup(VARIANT_GCLK_48MHZ, GCLK_GENCTRL_GENEN |  GCLK_GENCTRL_SRC_DFLL48M);
            break;
        default:
            break;
    }
}

uint32_t sam0_gclk_freq(uint8_t id)
{
    switch (id) {
    case VARIANT_GCLK_MAIN:
        return VARIANT_MCK;
    case VARIANT_GCLK_TIMER:
#if (VARIANT_MCK == 48000000U) || (VARIANT_MCK == 16000000U) || (VARIANT_MCK == 8000000U)
        return 8000000;
#else
        return 4000000;
#endif
    case VARIANT_GCLK_32KHZ:
        return 32768;
    case VARIANT_GCLK_48MHZ:
        return 48000000;
    default:
        return 0;
    }
}

static void _dfll_setup(void)
{
//    if (!USE_DFLL) {
//        return;
//    }

    GCLK->PCHCTRL[OSCCTRL_GCLK_ID_DFLL48].reg = GCLK_PCHCTRL_CHEN |
                                                GCLK_PCHCTRL_GEN_GCLK2;

    /* wait for sync */
    while (!(GCLK->PCHCTRL[OSCCTRL_GCLK_ID_DFLL48].reg & GCLK_PCHCTRL_CHEN)) {}

    OSCCTRL->DFLLCTRL.reg = OSCCTRL_DFLLCTRL_ENABLE;
    /* Wait for write synchronization */
    while (!(OSCCTRL->STATUS.reg & OSCCTRL_STATUS_DFLLRDY)) {}
    OSCCTRL->DFLLVAL.reg = OSCCTRL_DFLLVAL_COARSE((*(uint32_t*)NVMCTRL_OTP5)
                           >> 26) |  OSCCTRL_DFLLVAL_FINE(512);

    /* Wait for write synchronization */
    while (!(OSCCTRL->STATUS.reg & OSCCTRL_STATUS_DFLLRDY)) {}
    /* Generate a 48 Mhz clock from the 32KHz */
    OSCCTRL->DFLLMUL.reg = OSCCTRL_DFLLMUL_CSTEP(0x08) |
                           OSCCTRL_DFLLMUL_FSTEP(0x08) |
                           OSCCTRL_DFLLMUL_MUL((48000000U/32768));

    /* Disable DFLL before setting its configuration */
    OSCCTRL->DFLLCTRL.reg = 0;
    while (!(OSCCTRL->STATUS.reg & OSCCTRL_STATUS_DFLLRDY)) {}
    /* Write full configuration to DFLL control register */
    OSCCTRL->DFLLCTRL.reg =  OSCCTRL_DFLLCTRL_WAITLOCK |
                             OSCCTRL_DFLLCTRL_MODE |
                             OSCCTRL_DFLLCTRL_CCDIS |
                             OSCCTRL_DFLLCTRL_BPLCKC |
                             OSCCTRL_DFLLCTRL_ENABLE;

    /* Ensure COARSE and FINE are locked */
    while ((!(OSCCTRL->STATUS.bit.DFLLLCKC)) && (!(OSCCTRL->STATUS.bit.DFLLLCKF))) {}
    while (!(OSCCTRL->STATUS.bit.DFLLRDY)) {}

    /* Enable NVMCTRL */
    MCLK->APBBMASK.reg |= MCLK_APBBMASK_NVMCTRL;
    /* Set Wait State to meet requirements */
    NVMCTRL->CTRLB.reg |= NVMCTRL_CTRLB_RWS(3);
}

/*  from XOSC32K to DFLL48M  */
void init_system_clock(void)
{
    /* Turn on the digital interface clock: IS ON */
    //system_apb_clock_set_mask(SYSTEM_CLOCK_APB_APBA, MCLK_APBAMASK_GCLK);

    /* Software reset the GCLK module to ensure it is re-initialized correctly */
    GCLK->CTRLA.reg = GCLK_CTRLA_SWRST;
    while (GCLK->CTRLA.reg & GCLK_CTRLA_SWRST) {}
    while (GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_SWRST) {}

#if (VARIANT_MCK > 12000000U)
    PM->PLCFG.reg = PM_PLCFG_PLSEL_PL2;
    while (!PM->INTFLAG.bit.PLRDY) {}
#endif

    /* set OSC16M according to VARIANT_MCK */
#if (VARIANT_MCK == 48000000U) || (VARIANT_MCK == 16000000U)
    OSCCTRL->OSC16MCTRL.bit.FSEL = OSCCTRL_OSC16MCTRL_FSEL_16_Val;
#elif (VARIANT_MCK == 12000000U)
    OSCCTRL->OSC16MCTRL.bit.FSEL = OSCCTRL_OSC16MCTRL_FSEL_12_Val;
#elif (VARIANT_MCK == 8000000U)
    OSCCTRL->OSC16MCTRL.bit.FSEL = OSCCTRL_OSC16MCTRL_FSEL_8_Val;
#elif (VARIANT_MCK == 4000000U)
    OSCCTRL->OSC16MCTRL.bit.FSEL = OSCCTRL_OSC16MCTRL_FSEL_4_Val;
#else
#error "Please select a valid CPU frequency"
#endif

    OSCCTRL->OSC16MCTRL.bit.ONDEMAND = 0;
    OSCCTRL->OSC16MCTRL.bit.RUNSTDBY = 0;

    _osc32k_setup();
    _xosc32k_setup();

#if EXTERNAL_OSC32_SOURCE
    _gclk_setup(VARIANT_GCLK_32KHZ, GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_XOSC32K);
#else
    gclk_setup(VARIANT_GCLK_32KHZ, GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSCULP32K);
#endif

    _dfll_setup();

    /* Setup GCLK generators */
    gclk_setup(VARIANT_GCLK_MAIN, GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M);

    /* Ensure APB Backup domain clock is within the 6MHZ limit, BUPDIV value
       must be a power of 2 and between 1(2^0) and 128(2^7) */
    for (unsigned i = 0; i < 8; i++) {
        if (VARIANT_MCK / (1 << i) <= 6000000) {
            MCLK->BUPDIV.reg = (1 << i);
            while (!MCLK->INTFLAG.bit.CKRDY) {}
            break;
    }
    }
    /* clock used by timers */
    gclk_setup(VARIANT_GCLK_TIMER, GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M
                | GCLK_GENCTRL_DIV(VARIANT_MCK/sam0_gclk_freq(VARIANT_GCLK_TIMER)));

#ifdef MODULE_PERIPH_PM
    PM->CTRLA.reg = PM_CTRLA_MASK & (~PM_CTRLA_IORET);

    /* disable brownout detection
     * (Caused unexplicable reboots from sleep on saml21. /KS)
     */
    SUPC->BOD33.bit.ENABLE=0;
#endif


#if 0
    /* CPU and BUS clocks */
    MCLK->BUPDIV.reg = MCLK_BUPDIV_BUPDIV(1 << 0);
    MCLK->LPDIV.reg = MCLK_LPDIV_LPDIV(1 << 0);
    MCLK->CPUDIV.reg = MCLK_CPUDIV_CPUDIV(1 << 0);

    //enable_XOSC32K();

    /* SWITCH GCLK1 TO OSCULP32K */
    gclk_setup(1, GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSCULP32K);

    /* GCLK_DFLL48M_REF */
    gclk_channel_setup(0, GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_WRTLOCK); // SET TO GCLK1


//    /* SWITCH MAIN GCLK0 to DFLL48M output. */
//    gclk_setup(0, GCLK_GENCTRL_GENEN |
//                      GCLK_GENCTRL_SRC_DFLL48M |
//                      GCLK_GENCTRL_DIV(0) |
//                      GCLK_GENCTRL_IDC);
#endif
    SystemCoreClock = VARIANT_MCK; // 48000000ul;
}

void init_systick(void)
{
    /* Set Systick to 1ms interval, common to all Cortex-M variants */
    if (SysTick_Config(SystemCoreClock / 1000))
    {
        while (1)
        { // Capture error
        }
    }
}

/** Tick Counter united by ms */
volatile uint32_t _ulTickCount = 0;

void SysTick_Handler(void)
{
    _ulTickCount++; // Increment tick count each ms
}