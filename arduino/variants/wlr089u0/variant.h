/*
  SAMR3 - variant
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

#ifndef __VARIANT_H__
#define __VARIANT_H__

#include <wiring_analog.h>

#ifdef __cplusplus
extern "C"
{
#endif
// C ware //////////////////////////////////////////////////////////////////////////

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#define VARIANT_MCK (48000000ul)

enum {
    VARIANT_GCLK_MAIN  = 0,                 /**< Main clock */
    VARIANT_GCLK_TIMER = 1,                 /**< 4/8MHz clock for timers */
    VARIANT_GCLK_32KHZ = 2,                 /**< 32 kHz clock */
    VARIANT_GCLK_48MHZ = 3,                 /**< 48MHz clock */
};



enum PinNumber
{
    MODULE_PIN_PA07 = 0,
    MODULE_PIN_PA08,
    MODULE_PIN_PA27,
    MODULE_PIN_PA16,
    MODULE_PIN_PA17,
    MODULE_PIN_PA18,
    MODULE_PIN_PA19,
    MODULE_PIN_PA15,
    MODULE_PIN_PA14,
    MODULE_PIN_PB22,
    MODULE_PIN_PA23,
    MODULE_PIN_PB23,
    MODULE_PIN_PB02,
    MODULE_PIN_PA22,
    MODULE_PIN_PA25,
    MODULE_PIN_PA24,
    MODULE_PIN_PA28,
    MODULE_PIN_PA05,
    MODULE_PIN_PA04,
    MODULE_PIN_PB03,
    MODULE_PIN_PA06,
    MODULE_PIN_ADC_PA07,
    MODULE_PIN_ADC_PA06,
    MODULE_PIN_I2C_SDA_PA16,
    MODULE_PIN_I2C_SCL_PA17,
    MODULE_PIN_STATUS_LED_PA15,
    MODULE_PIN_SPI_MOSI_PB22,
    MODULE_PIN_SPI_CS_PA23,
    MODULE_PIN_SPI_SCK_PB23,
    MODULE_PIN_SPI_MISO_PB02,
    MODULE_PIN_SERCOM_RX_PA05,
    MODULE_PIN_SERCOM_TX_PA04,
    MODULE_PIN_SUPC_VBAT_PB03,
    MODULE_PIN_RF_SEL_PB31,
    MODULE_PIN_RF_MOSI_PB30,
    MODULE_PIN_RF_MISO_PC19,
    MODULE_PIN_RF_SCK_PC18,
    MODULE_PIN_RF_NRST_PB15,
    MODULE_PIN_RF_DIO0_PB16,
    MODULE_PIN_RF_DIO1_PA11,
    MODULE_PIN_RF_DIO2_PA12,
    MODULE_PIN_RF_DIO3_PB17,
    MODULE_PIN_RF_DIO4_PA10,
    MODULE_PIN_RF_DIO5_PB00,
    MODULE_PIN_RF_OTHER_PA09,
    MODULE_PIN_RF_OTHER_PA13,

    __MODULE_PINS_DESCRIPTIONS_COUNT

};

#define NUM_PIN_DESCRIPTION_ENTRIES (__MODULE_PINS_DESCRIPTIONS_COUNT) 

/* LED from table */
//#define LED_ON                (0) 
//#define LED_OFF               (1) 

#define LED                   PinNumber::MODULE_PIN_STATUS_LED_PA15
#define LED_BUILTIN           LED

/* Analog pins */
//#define PIN_A0                (4)
//#define PIN_A1                (5)
//static const uint8_t A0       = PIN_A0;
//static const uint8_t A1       = PIN_A1;

//#define PIN_DAC0            (14ul)
//static const uint8_t DAC0   = PIN_DAC0;

#define ADC_RESOLUTION		    (12)
#define VARIANT_AR_DEFAULT	AR_DEFAULT
//#define REMAP_ANALOG_PIN_ID(pin)	if ( pin < A0 ) pin += A0


#define PIN_SUPC_VBAT         (PinNumber::MODULE_PIN_SUPC_VBAT_PB03)



/* Serial from table */
#define PIN_SERIAL_TX         (PinNumber::MODULE_PIN_SERCOM_TX_PA04)
#define PIN_SERIAL_RX         (PinNumber::MODULE_PIN_SERCOM_RX_PA05) 


/* I2C from table */
#define PIN_WIRE_SDA          (PinNumber::MODULE_PIN_I2C_SDA_PA16) 
#define PIN_WIRE_SCL          (PinNumber::MODULE_PIN_I2C_SCL_PA17) 


/* SPI from table */
#define PIN_SPI_MOSI          (PinNumber::MODULE_PIN_SPI_MOSI_PB22)
#define PIN_SPI_MISO          (PinNumber::MODULE_PIN_SPI_MISO_PB02)
#define PIN_SPI_SCK           (PinNumber::MODULE_PIN_SPI_SCK_PB23)
#define PIN_SPI_SS            (PinNumber::MODULE_PIN_SPI_CS_PA23)


/* RF from table */
#define RF_SEL                (PinNumber::MODULE_PIN_RF_SEL_PB31)  
#define RF_MOSI               (PinNumber::MODULE_PIN_RF_MOSI_PB30)  
#define RF_MISO               (PinNumber::MODULE_PIN_RF_MISO_PC19)  
#define RF_SCK                (PinNumber::MODULE_PIN_RF_SCK_PC18)  
#define RF_RST                (PinNumber::MODULE_PIN_RF_NRST_PB15)   
#define RF_DIO0               (PinNumber::MODULE_PIN_RF_DIO0_PB16)
#define RF_DIO1               (PinNumber::MODULE_PIN_RF_DIO1_PA11)
#define RF_DIO2               (PinNumber::MODULE_PIN_RF_DIO2_PA12)
#define RF_DIO3               (PinNumber::MODULE_PIN_RF_DIO3_PB17)
#define RF_DIO4               (PinNumber::MODULE_PIN_RF_DIO4_PA10)
#define RF_DIO5               (PinNumber::MODULE_PIN_RF_DIO5_PB00)
#define RF_TCXO               (PinNumber::MODULE_PIN_RF_OTHER_PA09)
#define RF_SWITCH             (PinNumber::MODULE_PIN_RF_OTHER_PA13)

/* * */

#define analogInputToDigitalPin(p)  ((p < 6u) ? (p) + 14u : -1)
#define digitalPinToPort(P)         ( &(PORT->Group[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)      ( 1 << g_APinDescription[P].ulPin )
#define portOutputRegister(port)    ( &(port->OUT.reg) )
#define portInputRegister(port)     ( &(port->IN.reg) )
#define portModeRegister(port)      ( &(port->DIR.reg) )
#define digitalPinHasPWM(P)         ( g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )
#define digitalPinToInterrupt(P)    ( P )

extern uint32_t SystemCoreClock;

#ifdef __cplusplus
}

// CPP ware ////////////////////////////////////////////////////////////////////////

void initVariant();

#include <SERCOM.h>
extern SERCOM sercom0; // Serial
extern SERCOM sercom1; // I2C
extern SERCOM sercom2;
extern SERCOM sercom3; // Serial1
extern SERCOM sercom4; // RESERVED FOR RF
extern SERCOM sercom5; // SPI

#include <Uart.h>
extern Uart Serial;
extern Uart Serial1;

#define SAMR34XPRO /* for libraries */

#endif //__cplusplus
#endif /* __VARIANT_H__ */