/*
 * TivaTwoWire.cpp
 *
 * Simple non-blocking version of an I2C interface.
 * This class replaces Wire.cpp, with the difference
 * that 'endTransmission' merely begins the transmission,
 * it doesn't actually wait until it completes.
 * 
 * This version also has a FIFO depth of only 8 characters,
 * corresponding to the hardware FIFO.
 * 
 * Call status() to find whether or not the I2C
 * transaction is complete.
 *
 * Derived from:
 * Wire.cpp - library for Energia's LM4F port
 * TwoWire.cpp - hardware library for Wiring
 */
/*
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
  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
  Modified 7 March 2016 by Patrick Allison
 */


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "wiring_private.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/i2c.h"

#include "TivaTwoWire.h"

//% \brief Setup the peripheral.
//%
//% Note that the peripheral choice is hard-coded here! We use
//% I2C0 (SYSCTL_PERIPH_I2C0), using PB2 (SCL) PB3 (SDA).
//%
//% For I2C5 on PB4/PB5, for instance, it'd be

static const unsigned long i2cPeriph = SYSCTL_PERIPH_I2C5;
static const unsigned long i2cConfigSCL = GPIO_PB4_I2C5SCL;
static const unsigned long i2cConfigSDA = GPIO_PB5_I2C5SDA;
static const unsigned long i2cBase = I2C5_BASE;
static const unsigned long i2cPinSDA = GPIO_PIN_5;
static const unsigned long i2cPinSCL = GPIO_PIN_4;
static const unsigned long i2cPinBase = GPIO_PORTB_BASE;

TivaTwoWire Wire;

uint8_t TivaTwoWire::thisAddress = 0;
size_t TivaTwoWire::thisCount = 0;

TivaTwoWire::TivaTwoWire() {

}

void TivaTwoWire::begin(void) {
  ROM_SysCtlPeripheralEnable(i2cPeriph);
  ROM_GPIOPinConfigure(i2cConfigSCL);
  ROM_GPIOPinConfigure(i2cConfigSDA);
  ROM_GPIOPinTypeI2C(i2cPinBase, i2cPinSDA);
  ROM_GPIOPinTypeI2CSCL(i2cPinBase, i2cPinSCL);
  ROM_I2CMasterInitExpClk(i2cBase, F_CPU, false);
  // Force stop.
  if (!ROM_GPIOPinRead(i2cPinBase, i2cPinSCL))
    forceStop();

  if (ROM_I2CMasterBusBusy(i2cBase) || ROM_I2CMasterErr(i2cBase)
      || !ROM_GPIOPinRead(i2cPinBase, i2cPinSCL)) {
    uint8_t doI = 0;
    ROM_GPIOPinTypeGPIOOutput(i2cPinBase, i2cPinSCL);
    unsigned long mask;
    do {
      for (unsigned long i;i<10;i++) {
	ROM_SysCtlDelay(F_CPU/100000/3);
	mask = (i%2) ? i2cPinSCL : 0;
	ROM_GPIOPinWrite(i2cPinBase, i2cPinSCL, mask);
      }
      doI++;
    } while(ROM_I2CMasterBusBusy(i2cBase) && doI < 100);

    ROM_GPIOPinTypeI2CSCL(i2cPinBase, i2cPinSCL);
    if (!ROM_GPIOPinRead(i2cPinBase, i2cPinSCL)) forceStop();
  }

  ROM_I2CTxFIFOConfigSet(i2cBase, I2C_FIFO_CFG_TX_MASTER);
  ROM_I2CRxFIFOConfigSet(i2cBase, I2C_FIFO_CFG_RX_MASTER);
  ROM_I2CTxFIFOFlush(i2cBase);
  ROM_I2CRxFIFOFlush(i2cBase);
}

void TivaTwoWire::forceStop() {
  ROM_GPIOPinTypeGPIOOutput(i2cPinBase, i2cPinSCL | i2cPinSDA);
  ROM_GPIOPinWrite(i2cPinBase, i2cPinSDA, 0);
  ROM_GPIOPinWrite(i2cPinBase, i2cPinSCL, i2cPinSCL);
  ROM_GPIOPinWrite(i2cPinBase, i2cPinSDA, i2cPinSDA);
  ROM_GPIOPinTypeI2C(i2cPinBase, i2cPinSDA);
  ROM_GPIOPinTypeI2CSCL(i2cPinBase, i2cPinSCL);
  ROM_SysCtlPeripheralReset(i2cPeriph);
  while (!ROM_SysCtlPeripheralReady(i2cPeriph));
  ROM_I2CMasterInitExpClk(i2cBase, F_CPU, false);
}

unsigned long TivaTwoWire::status() {
  if (ROM_I2CMasterBusy(i2cBase)) return 1;
  return ROM_I2CMasterErr(i2cBase);
}

int TivaTwoWire::available() {
  return thisCount-ROM_I2CMasterBurstCountGet(i2cBase);
}

uint8_t TivaTwoWire::requestFrom(uint8_t address, uint8_t quantity) {
  requestFrom_nonblock(address, quantity);
  while (ROM_I2CMasterBusy(i2cBase));
  if (status() != I2C_MASTER_ERR_NONE) return 0;
  return available();
}
  
void TivaTwoWire::requestFrom_nonblock(uint8_t address, uint8_t quantity) {
  thisCount = quantity;
  ROM_I2CRxFIFOFlush(i2cBase);
  ROM_I2CMasterSlaveAddrSet(i2cBase, address, true);
  ROM_I2CMasterBurstLengthSet(i2cBase, quantity);
  ROM_I2CMasterControl(i2cBase, I2C_MASTER_CMD_FIFO_SINGLE_RECEIVE);
  while (!(ROM_I2CMasterBusy(i2cBase)));
}

void TivaTwoWire::beginTransmission(uint8_t address) {
  thisAddress = address;
  thisCount = 0;
  ROM_I2CTxFIFOFlush(i2cBase);
}

void TivaTwoWire::beginTransmission(int address) {
  beginTransmission((uint8_t) address);
}

uint8_t TivaTwoWire::endTransmission() {
  endTransmission_nonblock();
  while (ROM_I2CMasterBusy(i2cBase));
  return status();
}

void TivaTwoWire::endTransmission_nonblock() {
  if (!thisCount) return;

  ROM_I2CMasterSlaveAddrSet(i2cBase, thisAddress, false);
  ROM_I2CMasterBurstLengthSet(i2cBase, thisCount);
  // The name here is deceptive, because these 'cmd'
  // things are stupid.
  // You're actually setting BURST | STOP | START.
  // This means that the whole transfer (start + address + data + stop)
  // will occur.
  ROM_I2CMasterControl(i2cBase, I2C_MASTER_CMD_FIFO_SINGLE_RECEIVE);
  // Wait until the I2C master *becomes* busy.
  while (!(ROM_I2CMasterBusy(i2cBase)));
}

size_t TivaTwoWire::write(uint8_t data) {
  if (thisCount < 8) {
    ROM_I2CFIFODataPut(i2cBase, data);
    thisCount++;
    return 1;
  }
  return 0;
}

size_t TivaTwoWire::write(const uint8_t *data, size_t quantity) {
  size_t nchars;
  nchars = 0;
  do {
    ROM_I2CFIFODataPut(i2cBase, *data);
    data++;
    nchars++;
    thisCount++;
  } while (thisCount < 8 && nchars < quantity);
  return nchars;
}

int TivaTwoWire::read(void) {
  int value = -1;
  if (!(ROM_I2CFIFOStatus(i2cBase) & I2C_FIFO_RX_EMPTY)) {
    return ROM_I2CFIFODataGet(i2cBase);
  }
  return value;
}

void TivaTwoWire::flush() {
  ROM_I2CRxFIFOFlush(i2cBase);
  ROM_I2CTxFIFOFlush(i2cBase);
}

int TivaTwoWire::peek() {
  return -1;
}
