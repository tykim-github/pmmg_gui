//
//    FILE: MS5607.cpp
//  AUTHOR: Seongbin An, based on the MS5611-master by Rob Tillaart
// PURPOSE: MS5607 Temperature & Atmospheric Pressure library for Arduino
//     URL:
//


#ifndef MS5607_h
#define MS5607_h

#if ARDUINO < 100
#error "VERSION NOT SUPPPORTED"
#else
#include <Arduino.h>
#endif

#define MS5607_LIB_VERSION (F("0.1.0"))

#define MS5607_READ_OK  0

class MS5607
{
public:
  explicit        MS5607(uint8_t CSpin, bool InvertCSSignal);

  void            init();
  int             read(uint8_t bits = 8);
  inline int32_t  getTemperature()        const { return _temperature; };
  inline uint32_t getPressure()           const { return _pressure; };
  inline int      getLastResult()         const { return _result; };
  inline uint16_t getPromValue(uint8_t p) const { return C[p]; };

private:
  void     reset();
  void     convert(const uint8_t addr, uint8_t bits);
  uint32_t readADC();
  uint16_t readProm(uint8_t reg);
  void     command(const uint8_t command);

  uint8_t  _cspin;
  int32_t  _temperature;
  uint32_t _pressure;
  int      _result;
  uint16_t C[8];

  bool     _invertCS;
  bool     CS_ON;
  bool     CS_OFF;
};
#endif

// END OF FILE
