#include "MechaQMC5883.h"

#include <Wire.h>

void MechaQMC5883::setAddress(uint8_t addr){
  address = addr;
}

void MechaQMC5883::WriteReg(byte Reg,byte val){
  I2CBus->beginTransmission(address); //start talking
  I2CBus->write(Reg); // Tell the HMC5883 to Continuously Measure
  I2CBus->write(val); // Set the Register
  I2CBus->endTransmission();
}

void MechaQMC5883::init(){
  WriteReg(0x0B,0x01);
  //Define Set/Reset period
  setMode(Mode_Continuous,ODR_200Hz,RNG_8G,OSR_512);
  /*
  Define
  OSR = 512
  Full Scale Range = 8G(Gauss)
  ODR = 200HZ
  set continuous measurement mode
  */
}

void MechaQMC5883::setMode(uint16_t mode,uint16_t odr,uint16_t rng,uint16_t osr){
  WriteReg(0x09,mode|odr|rng|osr);
}


void MechaQMC5883::softReset(){
  WriteReg(0x0A,0x80);
}

/**
 * read values from device
 * @return status value:
 *  - 0:success
 *  - 1:data too long to fit in transmit buffer
 *  - 2:received NACK on transmit of address
 *  - 3:received NACK on transmit of data
 *  - 4:other error
 *  - 8:overflow (magnetic field too strong)
 */
int MechaQMC5883::read(int* x,int* y,int* z){
  I2CBus->beginTransmission(address);
  I2CBus->write(0x00);
  int err = I2CBus->endTransmission();
  if (err) {return err;}
  I2CBus->requestFrom(address, static_cast<uint8_t>(7));
  *x = (int)(int16_t)(I2CBus->read() | I2CBus->read() << 8);
  *y = (int)(int16_t)(I2CBus->read() | I2CBus->read() << 8);
  *z = (int)(int16_t)(I2CBus->read() | I2CBus->read() << 8);
  byte overflow = I2CBus->read() & 0x02;
  return overflow << 2;
}

int MechaQMC5883::read(int* x,int* y,int* z,int* a){
  int err = read(x,y,z);
  *a = azimuth(y,x);
  return err;
}

int MechaQMC5883::read(int* x,int* y,int* z,float* a){
  int err = read(x,y,z);
  *a = azimuth(y,x);
  return err;
}


float MechaQMC5883::azimuth(int *a, int *b){
  float azimuth = atan2((int)*a,(int)*b) * 180.0/PI;
  return azimuth < 0?360 + azimuth:azimuth;
}
