#include "EthernetBootloader.h"
#include "inc/hw_nvic.h"
#include "inc/hw_sysctl.h"
#include "driverlib/flash.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"

#define ROM_APITABLE            ((uint32_t *)0x01000010)
#define ROM_EMACTABLE           ((uint32_t *)(ROM_APITABLE[42]))
#define ROM_UpdateEMAC                                                        \
        ((void (*)(uint32_t ui32Clock))ROM_EMACTABLE[71])

void EthernetBootloader::begin() {
  _server.begin(magicPort);
}

void EthernetBootloader::handle() {
	int packetSize;
	packetSize = _server.parsePacket();
	if (packetSize != 0) {
		if (packetSize == magicPacketSize) {
		  unsigned char inb;
		  unsigned char i,j,k;
		  uint32_t user[2];
		  uint32_t tmp;

		  Serial.println("EBL: Checking Magic Packet...");
		  for (i=0;i<6;i++) {
			  inb = _server.read();
			  if (inb != magicMarker) {
				  Serial.print("EBL: Incorrect Magic Packet");
				  Serial.print(": Marker ");
				  Serial.print(i);
				  Serial.print(" = ");
				  Serial.println(inb, HEX);
				  _server.flush();
				  return;
			  }
		  }
		  FlashUserGet(&user[0], &user[1]);
		  for (i=0;i<4;i++) {
			  for (j=0;j<2;j++) {
				  	  tmp = user[j];
				  	  for (k=0;k<3;k++) {
				  		  unsigned char chk;
				  		  chk = tmp & 0xFF;
				  		  inb = _server.read();
				  		  if (inb != chk) {
				  			  Serial.print("EBL: Incorrect Magic Packet");
				  			  Serial.print(": MAC byte ");
				  			  Serial.print(i*6+j*3+k);
				  			  Serial.print(" expect ");
				  			  Serial.print(chk, HEX);
				  			  Serial.print(" got ");
				  			  Serial.println(inb, HEX);
				  			  _server.flush();
				  			  return;
				  		  }
				  		  tmp >>= 8;
				  	  }
			  }
		  }
		  Serial.println("EBL: Magic Packet OK!");
		  delay(500);
		  tmp = SysCtlClockGet();
		  // Disable all interrupts.
		  HWREG(NVIC_DIS0) = 0xffffffff;
		  HWREG(NVIC_DIS1) = 0xffffffff;
		  HWREG(NVIC_DIS2) = 0xffffffff;
		  HWREG(NVIC_DIS3) = 0xffffffff;
		  HWREG(NVIC_DIS4) = 0xffffffff;
		  // Disable SysTick.
		  SysTickIntDisable();
		  SysTickDisable();
		  // Call the bootloader.
		  ROM_UpdateEMAC(tmp);
		} else {
			Serial.print("EBL: Incorrect Magic Packet");
			Serial.print(": Length ");
			Serial.println(packetSize);
			_server.flush();
			return;
		}
	}
}
