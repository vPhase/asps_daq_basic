#ifndef TivaTwoWire_h
#define TivaTwoWire_h

#include <inttypes.h>
#include <Stream.h>

#define BUFFER_LENGTH 8

#define I2C_MASTER_ERR_BUSY 1

class TivaTwoWire : public Stream {
 private:
  static uint8_t thisAddress;
  static size_t thisCount;
  
  void forceStop();

 public:
  TivaTwoWire(void);
  void begin();
  void beginTransmission(uint8_t);
  void beginTransmission(int);
  uint8_t endTransmission(void);
  void endTransmission_nonblock(void);
  uint8_t requestFrom(uint8_t, uint8_t);
  void requestFrom_nonblock(uint8_t, uint8_t);
  unsigned long status();
  
  virtual size_t write(const uint8_t *, size_t);
  virtual size_t write(uint8_t);
  virtual int available(void);
  virtual int read(void);
  virtual int peek(void);
  virtual void flush(void);
  
  inline size_t write(unsigned long n) { return write((uint8_t)n); }
  inline size_t write(long n) { return write((uint8_t)n); }
  inline size_t write(unsigned int n) { return write((uint8_t)n); }
  inline size_t write(int n) { return write((uint8_t) n); }
  using Print::write;
};

extern TivaTwoWire Wire;

#endif
