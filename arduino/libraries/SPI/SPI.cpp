/*
 * SPI Master library for Arduino Zero.
 * Copyright (c) 2015 Arduino LLC
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "SPI.h"
#include <Arduino.h>
#include <wiring_private.h>

#define SPI_IMODE_NONE 0
#define SPI_IMODE_EXTINT 1
#define SPI_IMODE_GLOBAL 2

const SPISettings DEFAULT_SPI_SETTINGS = SPISettings();

SPIClass::SPIClass(SERCOM *p_sercom, uint8_t uc_pinMISO, uint8_t uc_pinSCK, uint8_t uc_pinMOSI, SercomSpiTXPad PadTx, SercomRXPad PadRx)
{
  initialized = false;
  while (p_sercom == NULL) // assert
  {
  }
  _p_sercom = p_sercom;
  // pins
  _uc_pinMiso = uc_pinMISO;
  _uc_pinSCK = uc_pinSCK;
  _uc_pinMosi = uc_pinMOSI;
  // SERCOM pads
  _padTx = PadTx;
  _padRx = PadRx;
}

void SPIClass::begin()
{
  init();
  uint32_t peripheral = (_p_sercom == &sercom4) ? PIO_SERCOM_RF : PIO_SERCOM;
  pinPeripheral(_uc_pinMiso, peripheral);
  pinPeripheral(_uc_pinMosi, peripheral);
  pinPeripheral(_uc_pinSCK, peripheral);
  config(DEFAULT_SPI_SETTINGS);
}

void SPIClass::init()
{
  if (initialized)
    return;
  interruptMode = SPI_IMODE_NONE;
  interruptSave = 0;
  interruptMask = 0;
  initialized = true;
}

void SPIClass::config(SPISettings settings)
{
  _p_sercom->disableSPI();
  _p_sercom->initSPI(_padTx, _padRx, SPI_CHAR_SIZE_8_BITS, settings.bitOrder);
  _p_sercom->initSPIClock(settings.dataMode, settings.clockFreq);
  _p_sercom->enableSPI();
}

void SPIClass::end()
{
  _p_sercom->resetSPI();
  initialized = false;
}

#ifndef interruptsStatus
#define interruptsStatus() __interruptsStatus()
static inline unsigned char __interruptsStatus(void) __attribute__((always_inline, unused));
static inline unsigned char __interruptsStatus(void)
{
  // See http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0497a/CHDBIBGJ.html
  return (__get_PRIMASK() ? 0 : 1);
}
#endif

void SPIClass::usingInterrupt(int interruptNumber)
{
  if ((interruptNumber == NOT_AN_INTERRUPT) || (interruptNumber == EINT_NMI))
    return;
  uint8_t irestore = interruptsStatus();
  noInterrupts();
  if (interruptNumber >= EXTERNAL_NUM_INTERRUPTS)
    interruptMode = SPI_IMODE_GLOBAL;
  else
  {
    interruptMode |= SPI_IMODE_EXTINT;
    interruptMask |= (1 << interruptNumber);
  }
  if (irestore)
    interrupts();
}

void SPIClass::notUsingInterrupt(int interruptNumber)
{
  if ((interruptNumber == NOT_AN_INTERRUPT) || (interruptNumber == EINT_NMI))
    return;
  if (interruptMode & SPI_IMODE_GLOBAL)
    return; // can't go back, as there is no reference count
  uint8_t irestore = interruptsStatus();
  noInterrupts();
  interruptMask &= ~(1 << interruptNumber);
  if (interruptMask == 0)
    interruptMode = SPI_IMODE_NONE;
  if (irestore)
    interrupts();
}

void SPIClass::beginTransaction(SPISettings settings)
{
  if (interruptMode != SPI_IMODE_NONE)
  {
    if (interruptMode & SPI_IMODE_GLOBAL)
    {
      interruptSave = interruptsStatus();
      noInterrupts();
    }
    else if (interruptMode & SPI_IMODE_EXTINT)
      EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT(interruptMask);
  }
  config(settings);
}

void SPIClass::endTransaction(void)
{
  if (interruptMode != SPI_IMODE_NONE)
  {
    if (interruptMode & SPI_IMODE_GLOBAL)
    {
      if (interruptSave)
        interrupts();
    }
    else if (interruptMode & SPI_IMODE_EXTINT)
      EIC->INTENSET.reg = EIC_INTENSET_EXTINT(interruptMask);
  }
}

void SPIClass::setBitOrder(BitOrder order)
{
  if (order == LSBFIRST)
  {
    _p_sercom->setDataOrderSPI(LSB_FIRST);
  }
  else
  {
    _p_sercom->setDataOrderSPI(MSB_FIRST);
  }
}

void SPIClass::setDataMode(uint8_t mode)
{
  switch (mode)
  {
  case SPI_MODE0:
    _p_sercom->setClockModeSPI(SERCOM_SPI_MODE_0);
    break;
  case SPI_MODE1:
    _p_sercom->setClockModeSPI(SERCOM_SPI_MODE_1);
    break;
  case SPI_MODE2:
    _p_sercom->setClockModeSPI(SERCOM_SPI_MODE_2);
    break;
  case SPI_MODE3:
    _p_sercom->setClockModeSPI(SERCOM_SPI_MODE_3);
    break;
  default:
    break;
  }
}

void SPIClass::setClockDivider(uint8_t div)
{
  if (div < SPI_MIN_CLOCK_DIVIDER)
  {
    _p_sercom->setBaudrateSPI(SPI_MIN_CLOCK_DIVIDER);
  }
  else
  {
    _p_sercom->setBaudrateSPI(div);
  }
}

byte SPIClass::transfer(uint8_t data)
{
  return _p_sercom->transferDataSPI(data);
}

uint16_t SPIClass::transfer16(uint16_t data)
{
  union {
    uint16_t val;
    struct
    {
      uint8_t lsb;
      uint8_t msb;
    };
  } t;
  t.val = data;
  if (_p_sercom->getDataOrderSPI() == LSB_FIRST)
  {
    t.lsb = transfer(t.lsb);
    t.msb = transfer(t.msb);
  }
  else
  {
    t.msb = transfer(t.msb);
    t.lsb = transfer(t.lsb);
  }
  return t.val;
}

void SPIClass::transfer(void *buf, size_t count)
{
  uint8_t *buffer = reinterpret_cast<uint8_t *>(buf);
  for (size_t i = 0; i < count; i++)
  {
    *buffer = transfer(*buffer);
    buffer++;
  }
}

void SPIClass::attachInterrupt()
{
  // Should be enableInterrupt()
}

void SPIClass::detachInterrupt()
{
  // Should be disableInterrupt()
}

#ifdef SAMR34XPRO /* not set yet */
SPIClass SPI(&sercom3, PIN_SPI_MISO, PIN_SPI_SCK, PIN_SPI_MOSI, (SercomSpiTXPad)1, (SercomRXPad)0);
SPIClass SPI_RF(&sercom4, RF_MISO, RF_SCK, RF_MOSI, (SercomSpiTXPad)1, (SercomRXPad)0);
#endif