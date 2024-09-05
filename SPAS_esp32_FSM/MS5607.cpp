//
//    FILE: MS5607.cpp
//  AUTHOR: Seongbin An, based on the MS5611-master by Rob Tillaart
// PURPOSE: MS5607 Temperature & Atmospheric Pressure library for Arduino
//     URL:
//

#include "MS5607.h"
#include <SPI.h>
//SPISettings settingsA(1000000, MSBFIRST, SPI_MODE0);  // define SPI settings; limit SPI communication to 40MHz
SPISettings settingsA(10000000, MSBFIRST, SPI_MODE0);  // define SPI settings; limit SPI communication to 40MHz

#ifdef ESP8266
#define min _min
#define max _max
#endif

#ifdef ESP32
#define min _min
#define max _max
#endif

/////////////////////////////////////////////////////
//
// PUBLIC
//
MS5607::MS5607(uint8_t CSn, bool InvertCS)
{
  _cspin = CSn;
  pinMode(_cspin, OUTPUT);
  _invertCS = InvertCS;
  if (InvertCS)
  {
    CS_OFF = LOW;
    CS_ON = HIGH;
  }
  else
  {
    CS_OFF = HIGH;
    CS_ON = LOW;
  }
  //digitalWrite(_cspin, HIGH);
  digitalWrite(_cspin, CS_OFF);
  _temperature = -999;
  _pressure = -999;
  init();
}

void MS5607::init()
{
  reset();

  // Default values (C1 t/m C6 taken from example column in datasheet to test calculations):
  C[0] = 1;
  C[1] = 40127;
  C[2] = 36924;
  C[3] = 23317;
  C[4] = 23282;
  C[5] = 33464;
  C[6] = 28312;
  C[7] = 0xF0F0;
 
  // Read all factory parameters C0 to C7 from PROM
  SPI.beginTransaction(settingsA);
  for (uint8_t reg = 0; reg < 8; reg++)
  {
    C[reg] = readProm(reg);
  }
  SPI.endTransaction();
}

int MS5607::read(uint8_t bits)
{
  // VARIABLES NAMES BASED ON DATASHEET  <- Nice!
  convert(0x40, bits);
  uint32_t D1 = readADC();

  convert(0x50, bits);
  uint32_t D2 = readADC();

  // TODO the multiplications of these constants can be done in init()
  // but first they need to be verified.

  // TEMP & PRESS MATH - PAGE 7/20 of the datasheet
  //  - avoiding float type to make running on Tiny's not-impossible
  //  - running into issues with uint64_t so using uint32_t with adjustments
  //      (at the cost of reduced resolution for temperature).
  uint32_t Tref, dT;
  uint32_t dTC6;
  int32_t  TEMP;
  Tref = C[5] * 256UL;
  if (D2 < Tref ) {  // (to avoid signed int so we can bit-shift for divisioning)
    dT   = Tref - D2;
    dTC6  = ((uint64_t)dT * (uint64_t)C[6]) >> 23;
    TEMP = 2000 - dTC6;
  } else {
    dT   = D2 - Tref;
    dTC6  = ((uint64_t)dT * (uint64_t)C[6]) >> 23;
    TEMP = 2000 + dTC6;
  }

  // OFF = offT1 + TCO * dT = C2 * 2^16 + (C4 * dT ) / 2^7 - MS5611
  // OFF = offT1 + TCO * dT = C2 * 2^17 + (C4 * dT ) / 2^6 - MS5607
  uint64_t offT1  =  (uint64_t)C[2] << 17;
  uint64_t TCOdT  = ((uint64_t)C[4] * (uint64_t)dT) >> 6;
  int64_t  OFF;
  if (D2 < Tref ) {
    OFF = offT1 - TCOdT;
  } else {
    OFF = offT1 + TCOdT;
  }

  // SENSE = sensT1 + TCS * dT = C1 * 2^15 + (C3 * dT ) / 2^8 - MS5611
  // SENSE = sensT1 + TCS * dT = C1 * 2^16 + (C3 * dT ) / 2^7 - MS5607
  uint64_t sensT1 =  (uint64_t)C[1] << 16;
  uint64_t TCSdT  = ((uint64_t)C[3] * (uint64_t)dT) >> 7;
  int64_t  SENS;
  if (D2 < Tref ) {
    SENS   = sensT1 - TCSdT;
  } else {
    SENS   = sensT1 + TCSdT;
  }


  // SECOND ORDER COMPENSATION - PAGE 8/20 of the datasheet
  // COMMENT OUT < 2000 CORRECTION IF NOT NEEDED
  // NOTE TEMPERATURE IS IN 0.01 C
  uint32_t T2    = 0;
  uint32_t OFF2  = 0;
  uint32_t SENS2 = 0;
  if (TEMP < 2000)
  {
    uint64_t tSQ;
    T2     = ((uint64_t)dT * (uint64_t)dT) >> 31;
    tSQ    = (int32_t)TEMP - 2000L;
    tSQ   *= tSQ;
    OFF2   = (61ULL * (uint64_t)tSQ) >> 4;
    SENS2  = (2ULL * (uint64_t)tSQ) >> 2;
    // COMMENT OUT < -1500 CORRECTION IF NOT NEEDED
    if (TEMP < -1500)
    {
      tSQ    = (int32_t)TEMP - 2000L;
      tSQ   *= tSQ;
      OFF2  +=   15ULL * (uint64_t)tSQ;
      SENS2 += (8ULL * (uint64_t)tSQ);
    }
  }

  TEMP -= T2;
  OFF  -= OFF2;
  SENS -= SENS2;
  //
  // END SECOND ORDER COMPENSATION
  //


  // P = D1 * SENS - OFF = (D1 * SENS / 2 21 - OFF) / 2 15
  int64_t P  = (int64_t)D1 * SENS;
  P /= 2097152LL;
  P -= OFF;
  P /= 32768LL;

  _temperature = TEMP; 
  _pressure = (uint32_t)P; 

  return 0;
}


/////////////////////////////////////////////////////
//
// PRIVATE
//
void MS5607::reset()
{
  SPI.beginTransaction(settingsA);                      // start SPI transaction
  //digitalWrite(_cspin, LOW);                            // pull CS line low
  digitalWrite(_cspin, CS_ON);
  SPI.transfer(0x1E);                                   // send reset command
  delay(4);
  //digitalWrite(_cspin,HIGH);                            // pull CS line high
  digitalWrite(_cspin, CS_OFF);
  SPI.endTransaction();                                 // end SPI transaction
}

void MS5607::convert(const uint8_t addr, uint8_t bits)
{
  uint8_t del[5] = {1, 2, 3, 5, 10};                    // array of MS5607 conversion time (in ms)
  bits = constrain(bits, 8, 12);
  uint8_t offset = (bits - 8) * 2;
  SPI.beginTransaction(settingsA);                      // start SPI transaction
  //digitalWrite(_cspin, LOW);                            // pull CS line low
  digitalWrite(_cspin, CS_ON);
  SPI.transfer(addr + offset);                          // send command
  //delay(del[offset/2]);                                 // MS5607 needs some time for conversion; wait for this...
  delay(1);
  //digitalWrite(_cspin,HIGH);                            // pull CS line high
  digitalWrite(_cspin, CS_OFF);
  SPI.endTransaction();                                 // end SPI transaction
}

uint16_t MS5607::readProm(uint8_t reg) {
  // read two bytes from SPI and return accumulated value
  reg = min(reg, 7);
  uint8_t offset = reg * 2;
  uint16_t val = 0;
  //digitalWrite(_cspin, LOW);                            // pull CS line low
  digitalWrite(_cspin, CS_ON);
  SPI.transfer(0xA0 + offset);                          // send command
  val  = SPI.transfer(0x00) * 256;                      // read 8 bits of data (MSB)
  val += SPI.transfer(0x00);                            // read 8 bits of data (LSB)
  //digitalWrite(_cspin,HIGH);                            // pull CS line high
  digitalWrite(_cspin, CS_OFF);
  return val;
}

uint32_t MS5607::readADC() {
  // read three bytes from SPI and return accumulated value
  uint32_t val = 0UL;
  SPI.beginTransaction(settingsA);                      // start SPI transaction
  //digitalWrite(_cspin, LOW);                            // pull CS line low
  digitalWrite(_cspin, CS_ON);
  SPI.transfer(0x00);                                   // send command
  val  = SPI.transfer(0x00) * 65536UL;                  // read 8 bits of data (MSB)
  val += SPI.transfer(0x00) * 256UL;                    // read 8 bits of data
  val += SPI.transfer(0x00);                            // read 8 bits of data (LSB)
  //digitalWrite(_cspin,HIGH);                            // pull CS line high
  digitalWrite(_cspin, CS_OFF);
  SPI.endTransaction();                                 // end SPI transaction
  return val;
}

// END OF FILE
