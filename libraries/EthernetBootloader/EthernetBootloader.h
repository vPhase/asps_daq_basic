#include <Ethernet.h>
#include <EthernetUdp.h>

// Ethernet bootloader server.
class EthernetBootloader {
 public:
  EthernetBootloader() {}
  void begin();
  void handle();
 private:

  const int magicPacketSize = 30;
  const int magicMarker = 0xAA;
  const int magicPort = 9;
  
  EthernetUDP _server;
};
  
