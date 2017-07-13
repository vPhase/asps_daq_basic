#include <SerialServer.h>
#include <WebServer.h>
#include <TivaTwoWire.h>  
#include <Cmd.h>
#include <Ethernet.h>
#include <EthernetBootloader.h>
#include "driverlib/watchdog.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "driverlib/i2c.h"
#include "driverlib/uart.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "inc/hw_flash.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"

bool feedWatchdog;

void defaultPage(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete);
void serialPage(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete);
void bslPage(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete);


const char *cmd_banner = ">>> ASPS-DAQ Basic Setup Interface";
const char *cmd_prompt = "ADAQ> ";
const char *cmd_unrecog = "Unknown command.";

#define BOOTCFG_MASK      0x7FFF00EC
// NW = 0, PORT = 0 (A), pin = 2 (010), POL = 0 (Low), EN = 0,
// KEY = 1, DBG1 = 1, DBG0 = 0.
// so byte 1 is 000 010 0 0 = 0x08
#define ASPS_DAQ_BOOTCFG  0x7FFF08FE

#define ASPSDAQ_BOARD_ID_ADDRESS 0
#define ASPSDAQ_IP_ADDRESS 8

// Enable TIVA <-> Heater BSL comms pin (inverted)
#define TIVA_EN_BSL_B      58
// Reset ASPS-POWER pin (inverted)
#define RST_ASPS_PWR_B     79
// ASPS-POWER RX (our transmit!)/multiplexed TEST line
#define ASPSPWR_RX         66
// Reset heater (not inverted)
#define TIVA_RST_MSP430    53
// Heater test (not inverted)
#define MSP430_TEST        11

char boardID[9];
#define VERSION "v0.6"

SerialServer *bridgeSerial = NULL;
unsigned char bridgeExitMatch = 0;
#define EXIT_LENGTH 3
#define EXIT_CHAR '+'

typedef enum {
  ETHERNET_NOT_STARTED = 0,
  ETHERNET_STARTED_WAIT_DHCP = 1,
  ETHERNET_READY = 2
} EthernetState;

EthernetState eState;
EthernetBootloader boot;
SerialServer ser1(23, 1);
SerialServer ser4(24, 4);
SerialServer ser5(25, 5);
SerialServer ser7(26, 7);

#define SENSOR_UPDATE_PERIOD 1000
unsigned long sensorUpdateTime = 0;
// Our sensors:
// We have:
// 5 currents     (PK3, PE3, PE4, PE5, PD6) = (68, 26, 2, 6, 87) = (AUX, SBC, SFE, ARAFE, FIBER)
// 6 faults       (PK2, PQ2, PP1, PQ1, PP0) = (67, 55, 44, 52, 43)
// 3 temperatures 
// On pins (PQ0, PD3, PD1, PD2, PD0) = (47, 7, 15, 42, 14)
const unsigned char currentPins[5] =  { 68, 26, 2, 6, 87 };
const unsigned char faultPins[5] = {67,55,44,52,43};
const unsigned char onPins[5] = {47,7,15,42,14};
unsigned char onState[5];

typedef enum {
  SENSOR_STATE_ADC0 = 0,
  SENSOR_STATE_ADC1 = 1,
  SENSOR_STATE_ADC2 = 2,
  SENSOR_STATE_ADC3 = 3,
  SENSOR_STATE_ADC4 = 4,
  SENSOR_STATE_TEMPSENSOR = 5,
  SENSOR_STATE_FAULTS = 6,
  SENSOR_STATE_I2C_0 = 7,
  SENSOR_STATE_I2C_0_READ = 8,
  SENSOR_STATE_I2C_1 = 9,
  SENSOR_STATE_I2C_1_READ = 10,
  SENSOR_STATE_I2C_1_COMPLETE = 11,
  SENSOR_STATE_WAIT = 12
} SensorState;
SensorState sState = SENSOR_STATE_ADC0;

typedef struct {
  unsigned int current[5];
  unsigned char fault;
  int temps[3];
} Sensors;
Sensors curSensors;

WebServer webServer("", 80);

void setup() {
  uint32_t user0, user1;
  uint32_t tmp;
  uint32_t boardID_raw[2];
  uint32_t reset_type;
  uint32_t ip_address;
  
  ROM_EEPROMInit();  
  ROM_EEPROMRead(&ip_address, ASPSDAQ_IP_ADDRESS, 8);  

  // Figure out the bootloader crap.
  Serial.begin(38400);
  Serial.println("");
  Serial.println("ASPS-DAQ Basic " VERSION);
  Serial.print("Reset Cause(s):");
  reset_type = SysCtlResetCauseGet();
  if (reset_type & SYSCTL_CAUSE_WDOG0) {
    Serial.print(" Watchdog");
  }
  if (reset_type & SYSCTL_CAUSE_LDO) {
    Serial.print(" LDO");
  }
  if (reset_type & SYSCTL_CAUSE_SW) {
    Serial.print(" SW"); 
  }
  if (reset_type & SYSCTL_CAUSE_EXT) {
    Serial.print(" EXT");
  }
  if (reset_type & SYSCTL_CAUSE_BOR) {
    Serial.print(" BOR");
  }
  if (reset_type & SYSCTL_CAUSE_POR) {
    Serial.print(" POR");
  }
  Serial.println("");
  SysCtlResetCauseClear(reset_type);
  
  // Check the BOOTCFG register.
  tmp = HWREG(FLASH_BOOTCFG);
  if (tmp & FLASH_BOOTCFG_NW) {
    Serial.println("Updating BOOTCFG...");
    tmp = (tmp & BOOTCFG_MASK) | (ASPS_DAQ_BOOTCFG & ~BOOTCFG_MASK);
    HWREG(FLASH_FMD) = tmp;
    HWREG(FLASH_FMA) = 0x75100000;
    HWREG(FLASH_FMC) = FLASH_FMC_WRKEY | FLASH_FMC_COMT;
    while (HWREG(FLASH_FMC) & FLASH_FMC_COMT);
    Serial.println("BOOTCFG updated.");
  }  else {
    Serial.print("BOOTCFG up-to-date: ");
    Serial.println(tmp, HEX);
  }
  // print out the Ethernet MAC
  getMacAddress(0, NULL);
  ROM_FlashUserGet(&user0, &user1);
  if (user0 == 0xFFFFFFFF && user1 == 0xFFFFFFFF) {
    Serial.println("Not starting Ethernet - no MAC address");
    eState = ETHERNET_NOT_STARTED;
  } else {    
    if (ip_address == 0xFFFFFFFF ) {
      Ethernet.begin_nonblock();
      eState = ETHERNET_STARTED_WAIT_DHCP;
    } else {
      Ethernet.begin(ip_address);
      eState = ETHERNET_READY;
      beginEthernet();
    }
    Ethernet.enableLinkLed();
    Ethernet.enableActivityLed();
  }
  // set up the FAULT pulls, on state
  for (unsigned int i=0;i<5;i++) {
    pinMode(faultPins[i], INPUT_PULLUP);
    onState[i] = 1;
    digitalWrite(onPins[i], 0);
  }
  // Pull up the output reset to look nice on a scope.
  pinMode(RST_ASPS_PWR_B, INPUT_PULLUP);
  
  // Enable heater comms.
  pinMode(TIVA_EN_BSL_B, OUTPUT);
  digitalWrite(TIVA_EN_BSL_B, 0);
  // Copy board ID.
  copyBoardID();
  
  Serial.end();
  ser1.beginSerial(9600);
  ser4.beginSerial(9600);
  ser5.beginSerial(9600);
  ser7.beginSerial(9600);
  // this command crap needs to be fixed. I hate these libraries.
  cmdInit(38400);
  cmdAdd("getmac", getMacAddress);
  cmdAdd("setmac", setMacAddress);
  cmdAdd("savemac", saveMacAddress);
  cmdAdd("setid", setBoardID);
  cmdAdd("bridge", serialBridge);
  cmdAdd("ip", showip);
  cmdAdd("status", printStatus);
  cmdAdd("control", control);
  cmdAdd("identify", doIdentify);
  cmdAdd("bsl", doBsl);
  cmdAdd("reboot", doReboot);
  cmdAdd("loop",doLoop);
  analogRead(TEMPSENSOR);
  Wire.begin();

  // Prep to feed watchdog.
  feedWatchdog = true;  
  // Enable the watchdog peripheral
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);
  // Set timeout of watchdog to 2 sec
  MAP_WatchdogReloadSet(WATCHDOG0_BASE, F_CPU * 2);
  // Reset when watchdog expires
  MAP_WatchdogResetEnable(WATCHDOG0_BASE);
  // Register the watchdog interrupt handler
  WatchdogIntRegister(WATCHDOG0_BASE, &WatchdogIntHandler);
  // Enable the watchdog
  MAP_WatchdogEnable(WATCHDOG0_BASE);
}

void bslReset(bool heater, bool bsl) {
  if (!heater) {
    if (!bsl) {
      // ASPS-POWER, normal reset
      digitalWrite(RST_ASPS_PWR_B, 1);
      pinMode(RST_ASPS_PWR_B, OUTPUT);
      digitalWrite(RST_ASPS_PWR_B, 0);
      digitalWrite(RST_ASPS_PWR_B, 1);
      pinMode(RST_ASPS_PWR_B, INPUT_PULLUP);
    } else {
      // ASPS-POWER, BSL reset
      Serial4.end();
      digitalWrite(RST_ASPS_PWR_B, 1);
      digitalWrite(ASPSPWR_RX, 1);
      pinMode(RST_ASPS_PWR_B, OUTPUT);
      pinMode(ASPSPWR_RX, OUTPUT);
      digitalWrite(RST_ASPS_PWR_B, 0);
      delayMicroseconds(500);
      digitalWrite(ASPSPWR_RX, 0);
      delayMicroseconds(100);
      digitalWrite(ASPSPWR_RX, 1);
      delayMicroseconds(100);
      digitalWrite(ASPSPWR_RX, 0);         
      // THIS DELAY SHOULD BE TUNED TO MAKE SURE ASPSPWR'S RESET ONESHOT HAS COMPLETED
      delay(5);
      digitalWrite(ASPSPWR_RX, 1);
      pinMode(ASPSPWR_RX, INPUT);
      pinMode(RST_ASPS_PWR_B, INPUT_PULLUP);
      Serial4.begin(9600);
    }
  } else {
    if (!bsl) {
      // Heater, normal reset
      digitalWrite(TIVA_RST_MSP430, 0);
      pinMode(TIVA_RST_MSP430, OUTPUT);
      digitalWrite(TIVA_RST_MSP430, 1);
      digitalWrite(TIVA_RST_MSP430, 0);
      pinMode(TIVA_RST_MSP430, INPUT);
    } else {
      // Heater, BSL reset
      digitalWrite(TIVA_RST_MSP430, 0);
      digitalWrite(MSP430_TEST, 0);
      pinMode(TIVA_RST_MSP430, OUTPUT);
      pinMode(MSP430_TEST, OUTPUT);
      digitalWrite(TIVA_RST_MSP430, 1);
      delayMicroseconds(100);
      digitalWrite(MSP430_TEST, 1);
      digitalWrite(MSP430_TEST, 0);
      digitalWrite(MSP430_TEST, 1);
      digitalWrite(TIVA_RST_MSP430, 0);
      delayMicroseconds(100);
      digitalWrite(MSP430_TEST, 0);
      pinMode(MSP430_TEST, INPUT);
      pinMode(TIVA_RST_MSP430, INPUT);
    }
  }
}

void copyBoardID() {
  uint32_t boardID_raw[2];
  ROM_EEPROMRead(boardID_raw, ASPSDAQ_BOARD_ID_ADDRESS, 8);
  memcpy(boardID, boardID_raw, 8);
  boardID[8] = NULL;
}

int doLoop(int argc, char **argv) {
  Serial.println("Doing infinite loop...");
  while(1);
}

// The board ID is 8 characters.
int setBoardID(int argc, char **argv) {
  // EEPROM wants a 32-bit aligned pointer. So allocate 8 bytes as 2 32-bit objects.
  uint32_t boardID_raw[2];
  // Byte pointer to above.
  char *pBoardID;
  int i;
  int err;
  char *newBoardStr;
  argc--;
  argv++;
  if (!argc) {
    Serial.println("setid needs an 8-character ID");
    return 0;
  }
  newBoardStr = *argv;
  // Get byte pointer to 32-bit storage space.
  pBoardID = (char *) boardID_raw;
  for (i=0;i<8;i++) {
    if (newBoardStr[i] == NULL) {
      Serial.println("setid needs an 8-character ID");
      return 0;
    }
    pBoardID[i] = newBoardStr[i];
  }
  if (err = ROM_EEPROMProgram(boardID_raw, ASPSDAQ_BOARD_ID_ADDRESS, 8)) {
    Serial.println("Board ID update failed!");
    Serial.print("Error ");
    Serial.println(err);
  } else {
    Serial.println("Board ID update OK.");
    copyBoardID();
  }
  return 0;
}

int doBsl(int argc, char **argv) {
  bool heater;
  bool bsl;
  
  argc--;
  argv++;
  if (argc < 2) {
    Serial.println("bsl [0-1] [0-1]");
    return 0;
  }
  heater = false;
  bsl = false;
  if (atoi(*argv)) heater = true;
  argv++;
  if (atoi(*argv)) bsl = true;
  bslReset(heater, bsl);
  return 0;
}

int doReboot(int argc, char **argv) {
  SysCtlReset();
}

int doIdentify(int argc, char **argv) {  
  Serial.print("ADAQ ");
  Serial.print(boardID);
  Serial.print(" ");
  Serial.println(VERSION);
  return 0;
}

int control(int argc, char **argv) {
  unsigned char ch;
  unsigned char st;
  
  argc--;
  argv++;
  if (argc < 2) {
    Serial.println("control [0-4] [0-1]");
    return 0;
  }
  ch = atoi(*argv);
  argv++;
  st = atoi(*argv);
  if (ch > 4) {
    Serial.println("control [0-4] [0-1]");
    return 0;
  }
  if (st) {
    pinMode(onPins[ch], INPUT);
    onState[ch] = 1;
  }  else {
    pinMode(onPins[ch], OUTPUT);
    onState[ch] = 0;
  }
  return 0;
}

int printStatus(int argc, char **argv) {
  for (unsigned int i=0;i<5;i++) {
    Serial.print("Current #");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(curSensors.current[i]);
    Serial.println(" mA");
  }
  Serial.print("On   : ");
  for (unsigned int i=0;i<5;i++) {
    if (onState[i]) {
      Serial.print(i);
      Serial.print(" ");
    } else {
      Serial.print("  ");
    }
  }
  Serial.println();
  Serial.print("Fault: ");
  for (unsigned int i=0;i<5;i++) {
    if (curSensors.fault & (1<<i)) Serial.print(i);
    else Serial.print(" ");
    if (i != 4) Serial.print(" ");
    else Serial.println();
  }
  for (unsigned int i=0;i<3;i++) {
    Serial.print("Temp #");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(curSensors.temps[i]);
    Serial.println(" C");
  }
  return 0;
}

// ip by itself prints IP address
// ip 128 146 32 24
// sets IP address to 128.146.32.24 *permanently*, but you need to reset.
// ip 255 255 255 255 goes back to DHCP mode
int showip(int argc, char **argv) {
  uint32_t ip_address_tmp;
  
  argc--;
  argv++;
  if (!argc) {
    if (eState == ETHERNET_NOT_STARTED) {
      Serial.println("No MAC address");
      return 0;
    }
    if (eState == ETHERNET_STARTED_WAIT_DHCP) {
      Serial.println("No IP address");
      return 0;
    }
    Serial.println(Ethernet.localIP());
  } else if (argc < 4) {
    unsigned int tmp;
    int i;
    ip_address_tmp = 0;
    for (i=0;i<4;i++) {
      tmp = strtoul(argv[i], NULL, 0);
      if (tmp < 256) {
        ip_address_tmp = ip_address_tmp << 8;
        ip_address_tmp = ip_address_tmp | tmp;
      } else {
        Serial.println("ip needs 4 octets less than 256");
        return 0;
      }
    }
    ROM_EEPROMProgram(&ip_address_tmp, ASPSDAQ_IP_ADDRESS, 4);
    Serial.print("IP address set: ");
    Serial.print((ip_address_tmp & 0xFF000000)>>24);
    Serial.print(".");
    Serial.print((ip_address_tmp & 0x00FF0000)>>16);
    Serial.print(".");
    Serial.print((ip_address_tmp & 0x0000FF00)>>8);
    Serial.print(".");
    Serial.println((ip_address_tmp & 0x000000FF));
    if (ip_address_tmp != 0xFFFFFFFF) {
      Serial.println("Non-DHCP mode selected! This is a warning!");
      Serial.println("Check it before you reset!");      
    }    
  }
  return 0;
}

int getMacAddress(int argc, char **argv) {
  unsigned char tmp;
  uint32_t user0, user1;
  // print out the Ethernet MAC
  ROM_FlashUserGet(&user0, &user1);  
  Serial.print("MAC ");
  tmp = (user0 >> 0) & 0xFF;
  if (tmp < 15) Serial.print("0");
  Serial.print(tmp, HEX);
  Serial.print(":");
  tmp = (user0 >> 8) & 0xFF;
  if (tmp < 15) Serial.print("0");
  Serial.print(tmp, HEX);
  Serial.print(":");
  tmp = (user0 >> 16) & 0xFF;
  if (tmp < 15) Serial.print("0");
  Serial.print(tmp, HEX);
  Serial.print(":");
  tmp = (user1 >> 0) & 0xFF;
  if (tmp < 15) Serial.print("0");
  Serial.print(tmp, HEX);
  Serial.print(":");
  tmp = (user1 >> 8) & 0xFF;
  if (tmp < 15) Serial.print("0");
  Serial.print(tmp, HEX);
  Serial.print(":");
  tmp = (user1 >> 16) & 0xFF;
  if (tmp < 15) Serial.print("0");
  Serial.println(tmp, HEX);
  return 0;
}

int setMacAddress(int argc, char **argv) {
  uint32_t user0;
  uint32_t user1;
  uint8_t tmp[6];
  argc--;
  argv++;
  if (argc < 6) {
    Serial.println("setmac needs a MAC address: 6 hex digits");
    return 0;
  }
  ROM_FlashUserGet(&user0, &user1);  
  if (user0 != 0xFFFFFFFF ||
      user1 != 0xFFFFFFFF) {
    Serial.println("MAC address is already programmed, can't overwrite.");
    return 0;
  }
  Serial.print("Updating MAC to ");
  for (unsigned int i=0;i<6;i++) {
    tmp[i] = strtoul(argv[i], NULL, 16);
    if (i) Serial.print(":");
    Serial.print(tmp[i], HEX);
  }
  Serial.println();
  user0 = tmp[0];
  user0 = user0 | (tmp[1] << 8);
  user0 = user0 | (tmp[2] << 16);
  user1 = tmp[3];
  user1 = user1 | (tmp[4] << 8);
  user1 = user1 | (tmp[5] << 16);
  ROM_FlashUserSet(user0, user1);
  return 0;
}

int saveMacAddress(int argc, char **argv) {
  ROM_FlashUserSave();
  Serial.println("--- MAC ADDRESS HAS BEEN COMMITTED ---");
  return 0;
}

#define PORT_BRIDGE_MAX 4
int serialBridge(int argc, char **argv) {
  HardwareSerial *thisSerial;
  unsigned int port;
  argc--;
  argv++;
  if (!argc) {
    Serial.println("bridge needs a port to bridge to");
    return 0;
  }
  port = strtoul(*argv, NULL, 0);
  if (port < PORT_BRIDGE_MAX) {
    unsigned char exit_match = 0;
    switch (port) {
      case 0: bridgeSerial = &ser1; break;
      case 1: bridgeSerial = &ser4; break;
      case 2: bridgeSerial = &ser5; break;
      case 3: bridgeSerial = &ser7; break;
    }
  } else {
    Serial.println("invalid port");
    return 0;
  }
  bridgeSerial->bridge(true);
  return 1;
}

void sensorLoop()
{
  int32_t temp;
  unsigned char faultBit;
  char tmp;
  
  switch (sState) {
    case SENSOR_STATE_ADC0:
    case SENSOR_STATE_ADC1:
    case SENSOR_STATE_ADC2:
    case SENSOR_STATE_ADC3:
    case SENSOR_STATE_ADC4:
      temp = analogRead(currentPins[(unsigned char) sState]);
      // IMON output is 53 uA/A. Then passed through 10k resistor, gives 0.53 V/A or 0.53 mV/mA
      // Max readout is 4096, so 3300 mV/4096 ADC = 0.8056 mV/ADC
      // So conversion is 0.8056 mV/ADC / 0.53 mV/mA = 1.52 mA / ADC.
      // Sticking to integers, this is times 389, downshift by 8.
      temp = (temp*389);
      temp = temp >> 8;
      curSensors.current[(unsigned char) sState] = temp;
      sState = (SensorState) ((unsigned char) sState + 1);
      break;
    case SENSOR_STATE_TEMPSENSOR:
      temp = analogRead(TEMPSENSOR);
      // Now we need (1475 - ((2475/4096)*adc)/10
      // or (295 - 0.1208*adc)/2
      // or (295 - (31*adc)>>8)>>1
      temp = (temp*31);
      temp = temp >> 8;
      temp = 295 - temp;
      temp = temp >> 1;
      curSensors.temps[0] = temp;
      sState = (SensorState) ((unsigned char) sState + 1);
      break;
    case SENSOR_STATE_FAULTS:
      curSensors.fault = 0;
      faultBit = 1;
      for (unsigned int i=0;i<5;i++) {
        if (!digitalRead(faultPins[i])) curSensors.fault |= faultBit;
        faultBit <<= 1;
      }
      sState = (SensorState) ((unsigned char) sState + 1);
      break;
   case SENSOR_STATE_I2C_0:
      Wire.beginTransmission(0x48);
      Wire.write(0x00);
      Wire.endTransmission_nonblock();
      sState = (SensorState) ((unsigned char) sState + 1);
      break;
   case SENSOR_STATE_I2C_0_READ:
      if (Wire.status() == I2C_MASTER_ERR_BUSY) return;
      if (Wire.status() != I2C_MASTER_ERR_NONE) {
        Serial.println("I2C> I2C error");
        sState = SENSOR_STATE_WAIT;
        sensorUpdateTime = millis() + SENSOR_UPDATE_PERIOD;
        break;
      }
      Wire.requestFrom_nonblock(0x48, 1);
      sState = (SensorState) ((unsigned char) sState + 1);
      break;
   case SENSOR_STATE_I2C_1:
      if (Wire.status() == I2C_MASTER_ERR_BUSY) return;
      if (Wire.status() != I2C_MASTER_ERR_NONE) {
        Serial.println("I2C> I2C error");
        sState = SENSOR_STATE_WAIT;
        sensorUpdateTime = millis() + SENSOR_UPDATE_PERIOD;
        break;
      }
      tmp = Wire.read();
      if (tmp & 0x80) curSensors.temps[1] = 0 - (256 - tmp);
      else curSensors.temps[1] = tmp;
      Wire.beginTransmission(0x48);
      Wire.write(0x01);
      Wire.endTransmission_nonblock();
      sState = (SensorState) ((unsigned char) sState + 1);
      break;
   case SENSOR_STATE_I2C_1_READ:
      if (Wire.status() == I2C_MASTER_ERR_BUSY) return;
      if (Wire.status() != I2C_MASTER_ERR_NONE) {
         Serial.println("I2C> I2C error");
         sState = SENSOR_STATE_WAIT;
         sensorUpdateTime = millis() + SENSOR_UPDATE_PERIOD;
         break;
      }
      Wire.requestFrom_nonblock(0x48, 1);
      sState = (SensorState) ((unsigned char) sState + 1);
      break;
  case SENSOR_STATE_I2C_1_COMPLETE:
      if (Wire.status() == I2C_MASTER_ERR_BUSY) return;
      if (Wire.status() == I2C_MASTER_ERR_NONE) {
        tmp = Wire.read();
        if (tmp & 0x80) curSensors.temps[2] = 0 - (256 - tmp);
        else curSensors.temps[2] = tmp;
      } else {
        Serial.println("I2C> I2C error");
      }
      sensorUpdateTime = millis() + SENSOR_UPDATE_PERIOD;
      sState = SENSOR_STATE_WAIT;
      break;
  case SENSOR_STATE_WAIT:
      if ((long) (millis() - sensorUpdateTime) > 0) {
        sState = SENSOR_STATE_ADC0;
        break;
      }
  } 
}

void loop() {
  char c;

  // Watchdog. There's no race condition here because we only set, and the ISR only clears.
  // The ISR is also slow enough that if we miss it the first time, we'll catch it the second.
  if (!feedWatchdog) feedWatchdog = true;

  // Sensor handling.
  sensorLoop();
  
  // Serial handling.
  if (bridgeSerial == NULL)
    cmdPoll();
  else {
    char c;
    while (Serial.available() && bridgeExitMatch < EXIT_LENGTH) {
      c = Serial.read();
      if (c == EXIT_CHAR) bridgeExitMatch++;
      else bridgeExitMatch = 0;
      bridgeSerial->write(c);
    }
    if (bridgeExitMatch >= EXIT_LENGTH) {
      bridgeSerial->bridge(false);
      bridgeSerial = NULL;
      bridgeExitMatch = 0;
      cmdPrompt();
    }
  }
  // Serial handling.
  ser1.handle();
  ser4.handle();
  ser5.handle();
  ser7.handle();

  if (eState == ETHERNET_STARTED_WAIT_DHCP) {
    if (Ethernet.ready()) {
      eState = ETHERNET_READY;
      if (bridgeSerial == NULL) {
        Serial.print("DHCP Complete: ");
        Serial.println(Ethernet.localIP());
      }
      beginEthernet();
    }
  }
  if (eState == ETHERNET_READY) {
    boot.handle();
    webServer.processConnection();
  }
}

void beginEthernet() {
  boot.begin();
  webServer.begin();
  webServer.addCommand("index.html", &defaultPage);
  webServer.addCommand("serial.html", &serialPage);
  webServer.addCommand("bsl.html", &bslPage);
  webServer.setDefaultCommand(&defaultPage);
  ser1.beginEthernet();
  ser4.beginEthernet();
  ser5.beginEthernet();
  ser7.beginEthernet();
}

void bslPage(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  char name[5];
  char value[5];
  URLPARAM_RESULT rc;
  bool reboot_heater = false;
  bool bsl_boot = false;
  const char *startPage = "<html>"
                          "<head>"
                          "<title>ASPS-DAQ BSL Helper Page</title>"
                          "</head>"
                          "<body>"
                          "<h1>ASPS-DAQ BSL Helper Page</h1>"
                          "<br>"
                          "<form action=\"bsl.html\" method=\"get\">"
                          "<input type=\"radio\" name=\"0\" value=\"0\" checked>ASPS-POWER"
                          "<input type=\"radio\" name=\"0\" value=\"1\">Heater"
                          "<br>"
                          "<input type=\"radio\" name=\"1\" value=\"0\" checked>Normal"
                          "<input type=\"radio\" name=\"1\" value=\"1\">BSL"
                          "<br>"
                          "<input type=\"submit\" value=\"Reboot Microcontroller\">"
                          "</form>"
                          "</body>"
                          "</html>";
  server.httpSuccess();

  if (strlen(url_tail)) {
    while (strlen(url_tail)) {
      rc = server.nextURLparam(&url_tail, name, 5, value, 5);
      if (rc != URLPARAM_EOS) {
        if (atoi(name) == 0) if (atoi(value)) reboot_heater = true;
        if (atoi(name) == 1) if (atoi(value)) bsl_boot = true;
      }
    }
    bslReset(reboot_heater, bsl_boot);
  }

  
  server.print(startPage);
}  

void serialPage(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  char name[5];
  char value[5];
  
  const char *startPage = "<html>"
                          "<head>"
                          "<title>ASPS-DAQ Serial Status Page</title>"
                          "</head>"
                          "<body>"
                          "<h1>ASPS-DAQ Serial Status Page</h1>"
                          "<br>";
  const char *endPage = "<form action=\"serial.html\" method=\"get\">"
                        "<input type=\"checkbox\" name=\"dis\" value=\"0\">Port 23<br>"
                        "<input type=\"checkbox\" name=\"dis\" value=\"1\">Port 24<br>"
                        "<input type=\"checkbox\" name=\"dis\" value=\"2\">Port 25<br>"
                        "<input type=\"checkbox\" name=\"dis\" value=\"3\">Port 26<br>"
                        "<input type=\"submit\" value=\"Disconnect\"></form></body></html>";  
  URLPARAM_RESULT rc;

  server.httpSuccess();
  
  if (strlen(url_tail)) {
    while (strlen(url_tail)) {
      rc = server.nextURLparam(&url_tail, name, 5, value, 5);
      if (rc != URLPARAM_EOS) {
        if (!strcmp("dis", name)) {
          switch(atoi(value)) {
            case 0: if (ser1.connected()) ser1.disconnect(); break;
            case 1: if (ser4.connected()) ser4.disconnect(); break;
            case 2: if (ser5.connected()) ser5.disconnect(); break;
            case 3: if (ser7.connected()) ser7.disconnect(); break;
            default: break;
          }
        }
      }
    }
  }
    
  server.print(startPage);
  server.print("<p>Port 23 (Heater BSL): ");
  if (!ser1.connected()) server.print("NOT ");
  server.print("CONNECTED</p>");

  server.print("<p>Port 24 (ASPS-POWER): ");
  if (!ser4.connected()) server.print("NOT ");
  server.print("CONNECTED</p>");

  server.print("<p>Port 25 (SBC): ");
  if (!ser5.connected()) server.print("NOT ");
  server.print("CONNECTED</p>");

  server.print("<p>Port 26 (Heater): ");
  if (!ser7.connected()) server.print("NOT ");
  server.print("CONNECTED</p>");
  
  server.print(endPage);
}

const char *output_labels[5] = {
  "AUX  ",
  "SBC  ",
  "AUX2 ",
  "ARAFE",
  "FIBER"   
};

void defaultPage(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  char name[5];
  char value[5];
  URLPARAM_RESULT rc;
  unsigned int param_number;
  
  const char *startPage = "<html>"
                            "<head>"
                            "<title>ASPS-DAQ Main Page</title>"
                            "</head>"
                            "<body>"
                            "<h1>ASPS-DAQ Main Page</h1>"
                            "<br>";
  const char *idStart   =   "<p>Board ID: ";
  const char *idEnd     =   "</p>"
                            "<br>"
                            "<h2>Sensors</h2>"
                            "<form action=\"index.html\" method=\"get\">"
                            "<p>";
  const char *tempStart   = "<br>Note: turning off Fiber doesn't actually work here (on purpose).<br><input type=\"submit\" value=\"Change Outputs\"></form></p><p>"
                            "Temperature:"; 
  const char *endPage     = "</p></body></html>";
  
  unsigned int i;
  server.httpSuccess();
  if (strlen(url_tail)) {
    while (strlen(url_tail)) {
      rc = server.nextURLparam(&url_tail, name, 5, value, 5);
      if (rc != URLPARAM_EOS) {
        param_number = atoi(name);
        if (param_number < 4) {
          if (atoi(value)) {
             pinMode(onPins[param_number], INPUT);
             onState[param_number] = 1;
          } else {
             pinMode(onPins[param_number], OUTPUT);
             onState[param_number] = 0;
          }
        }
      }
    }
  }
  server.print(startPage);
  server.print(idStart);
  server.print(boardID);
  server.print(" " VERSION);
  server.print(idEnd);
  for (i=0;i<5;i++) {
    server.print("<h3>");
    server.print(output_labels[i]);
    server.print("</h3>");
    server.print("<input type=\"radio\" name=\"");
    server.print(i);
    server.print("\" value=\"1\"");
    if (onState[i]) server.print(" checked");
    server.print(">On<input type=\"radio\" name=\"");
    server.print(i);
    server.print("\" value=\"0\"");
    if (!onState[i]) server.print(" checked");
    server.print(">Off<br>");
    server.print("Current: ");
    server.print(curSensors.current[i]);
    server.print(" mA<br>");    
  }
  server.print(tempStart);
  for (i=0;i<3;i++) {
    server.print(" ");
    server.print(curSensors.temps[i]);
    server.print(" C");
  }
  server.print(endPage);
}

void WatchdogIntHandler(void) {
  if (!feedWatchdog) return;
  feedWatchdog = false;
  WatchdogIntClear(WATCHDOG0_BASE);  
}
