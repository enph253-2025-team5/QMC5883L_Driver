#ifndef Mecha_QMC5883
#define Mecha_QMC5883

#include "Arduino.h"
#include "Wire.h"

#define QMC5883_ADDR 0x0D


//REG CONTROL

//0x09

#define Mode_Standby    0b00000000
#define Mode_Continuous 0b00000001

#define ODR_10Hz        0b00000000
#define ODR_50Hz        0b00000100
#define ODR_100Hz       0b00001000
#define ODR_200Hz       0b00001100

#define RNG_2G          0b00000000
#define RNG_8G          0b00010000

#define OSR_512         0b00000000
#define OSR_256         0b01000000
#define OSR_128         0b10000000
#define OSR_64          0b11000000


class MechaQMC5883{
public:

/// @brief Construct a MechaQMC5883 object
/// @param I2CBus The I2C bus to use. Default: `Wire` (equivalent to TwoWire(0)).
MechaQMC5883(TwoWire* I2CBus = &Wire) : I2CBus(I2CBus) {};

void setAddress(uint8_t addr);

void init(); //init qmc5883

void setMode(uint16_t mode,uint16_t odr,uint16_t rng,uint16_t osr); // setting

void softReset(); //soft RESET

int read(int* x,int* y,int* z); //reading
int read(int* x,int* y,int* z,int* a);
int read(int* x,int* y,int* z,float* a);

float azimuth(int* a,int* b);

private:

void WriteReg(uint8_t Reg,uint8_t val);

TwoWire* I2CBus;

uint8_t address = QMC5883_ADDR;

};



#endif
