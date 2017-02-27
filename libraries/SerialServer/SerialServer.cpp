#include "SerialServer.h"


// IAC DO LINEMODE
// IAC SB LINEMODE MODE 0
// IAC SE
// IAC WILL SUPPRESS-GOAHEAD
// IAC DO SUPPRESS-GOAHEAD
// IAC DO SUPPRESS-LOCAL-ECHO
// IAC WILL BINARY
// IAC DO BINARY
static const unsigned char telnet_raw_mode[] = { 255, 253, 34, 255, 250, 34, 1, 0, 255, 240, 255, 253, 3, 255, 251, 3, 255, 253, 45, 255, 253, 0, 255, 251, 0 };

SerialServer::SerialServer(uint16_t tcpPort, unsigned long uart) : _server(tcpPort),_uart(uart), _client(NULL), _bridging(false), _state(state_NO_ETHERNET) {
  switch (uart) {
  case 0: serial = &Serial; break;
  case 1: serial = &Serial1; break;
  case 2: serial = &Serial2; break;
  case 3: serial = &Serial3; break;
  case 4: serial = &Serial4; break;
  case 5: serial = &Serial5; break;
  case 6: serial = &Serial6; break;
  case 7: serial = &Serial7; break;
  default: serial = &Serial; break;
  }
}

void SerialServer::beginEthernet() {
  _state = state_NO_CONNECTION;
  _server.begin();
}

void SerialServer::beginSerial(unsigned long baud) {
  serial->begin(baud);
}

void SerialServer::handle() {
  handleEthernet();
  handleSerial();
}

void SerialServer::handleEthernet() {
  unsigned int nb;
  unsigned int space;
  unsigned char inb;
  EthernetClient temp;
  
  if (_state == state_NO_ETHERNET) return;

  if (_state == state_NO_CONNECTION) {
    _client = _server.available();
    if (_client) {
      Serial.println("Adding client.");
      _client.write(telnet_raw_mode, sizeof(telnet_raw_mode));
      _client.print("Serial Server for UART");
      _client.println(_uart);
      _state = state_IDLE;
    } else return;
  }
  if (!_client) {
    if (!_client.available()) {
      Serial.println("Closing client.");
      _client.stop();
      _state = state_NO_CONNECTION;
    }
  }
  
  nb = _client.available();
  if (!nb) return;
  space = serial->tx_available();
  while (nb-- && space) {
    inb = _client.read();
    switch (_state) {
    default:
      _state = state_IDLE;
    case state_IDLE:
      if (inb == TELNET_IAC) _state = state_IAC;
      serial->write(inb);
      space--;
      break;
    case state_IAC:
      if (inb == TELNET_IAC) {
	serial->write(inb);
	space--;
	break;
      }
      if (inb == TELNET_SB){
	_state = state_IAC_SB;
	break;
      }
      if (inb == TELNET_WILL || inb == TELNET_WONT || inb == TELNET_DO ||
	  inb == TELNET_DONT) {
	_state = state_IAC_PARAM;
	break;
      }
      if (inb == TELNET_AYT) {
	_client.println("[Yes]");
      }
      _state = state_IDLE;
      break;
    case state_IAC_PARAM:
      // whatever, I don't care
      _state = state_IDLE;
      break;
    case state_IAC_SB:
      if (inb == TELNET_IAC) {
	_state = state_IAC_SB_IAC;
	break;
      }
      break;
    case state_IAC_SB_IAC:
      if (inb == TELNET_SE) {
	_state = state_IDLE;
	break;
      }
      _state = state_IAC_SB;
      break;
    }
  }
}

void SerialServer::handleSerial() {
  unsigned int nb;

  nb = serial->available();
  if (nb) {
    if (_state > state_NO_CONNECTION) {
      char c;
      for (unsigned int i=0;i<nb;i++) {
	c = serial->read();
	_client.write(c);
	if (_bridging) Serial.write(c);
      }
    } else {
      if (_bridging) 
	for (unsigned int i=0;i<nb;i++) Serial.write(serial->read());
      else
	for (unsigned int i=0;i<nb;i++) serial->read();
    }
  }
}

void SerialServer::write(const char c) {
  serial->write(c);
}

void SerialServer::bridge(bool yesno) {
  _bridging = yesno;
}

bool SerialServer::connected() {
  return (_state > state_NO_CONNECTION);
}

void SerialServer::disconnect() {
  _client.stop();
  _state = state_NO_CONNECTION;  
}
