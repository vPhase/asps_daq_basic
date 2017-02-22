#include <Ethernet.h>
#include <HardwareSerial.h>

// Basis for a Serial-to-Ethernet server.
// Note that this will only service a single client.


class SerialServer {
 public:
  SerialServer(uint16_t tcpPort, unsigned long uart);
  void beginEthernet();
  void beginSerial(unsigned long baud);
  void handle();
  void write(const char c);
  void bridge(bool yesno);
 private:
  void handleEthernet();
  void handleSerial();

  HardwareSerial *serial;
  unsigned long _uart;
  EthernetClient _client;
  EthernetServer _server;
  bool _bridging;
  
  const unsigned char TELNET_SE = 240;
  const unsigned char TELNET_AYT = 246;
  const unsigned char TELNET_SB = 250;
  const unsigned char TELNET_WILL = 251;
  const unsigned char TELNET_WONT = 252;
  const unsigned char TELNET_DO = 253;
  const unsigned char TELNET_DONT = 254;
  const unsigned char TELNET_IAC = 255;  
  
  typedef enum telnet_state {
    state_NO_ETHERNET = 0,
    state_NO_CONNECTION = 1,
    state_IDLE = 2,
    state_IAC = 3,        // found an IAC, wait for next char
    state_IAC_PARAM = 4,  // found an IAC WILL/DO/WONT/DONT, wait for next char
    state_IAC_SB = 5,     // found an IAC SB, wait for IAC
    state_IAC_SB_IAC = 6  // found an IAC SB IAC, check for SE.
  } telnet_state_t;
  telnet_state_t _state;
};
  
