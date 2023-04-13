#ifndef Mecha_QMC5883
#define Mecha_QMC5883

#include "Arduino.h"
#include "Wire.h"

#define QMC5883_ADDR 0x0D

#ifndef DEFINITIONS_H
// Convert degrees to radians
#define DEG(x) ((x) *180.0f / (float) PI)

// Limit angle to 0 to 360 degrees
#define LIM_ANGLE(angle) (angle > 0 ? fmod(angle, 360) : fmod(angle, 360) + 360)
#endif

// REG CONTROL

// 0x09

#define Mode_Standby    0b00000000
#define Mode_Continuous 0b00000001

#define ODR_10Hz  0b00000000
#define ODR_50Hz  0b00000100
#define ODR_100Hz 0b00001000
#define ODR_200Hz 0b00001100

#define RNG_2G 0b00000000
#define RNG_8G 0b00010000

#define OSR_512 0b00000000
#define OSR_256 0b01000000
#define OSR_128 0b10000000
#define OSR_64  0b11000000

class MechaQMC5883 {
    public:
    MechaQMC5883(TwoWire &wire, float x, float y, float e, float angle);
    void setAddress(uint8_t addr);

    void init(); // init qmc5883

    void setMode(uint16_t mode, uint16_t odr, uint16_t rng, uint16_t osr); // setting

    void softReset();                  // soft RESET
    int  read(int *x, int *y, int *z); // reading
    int  read(int *x, int *y, int *z, int *a);
    int  read(int *x, int *y, int *z, float *a);

    void  tare();
    float readAngle();
    float readRawAngle();
    void  printRaw();
    float azimuth(int *a, int *b);

    private:
    void     _WriteReg(uint8_t Reg, uint8_t val);
    TwoWire &_wire;
    float    _zeroError = 0;
    float    _calibration[4];
    uint8_t  _address = QMC5883_ADDR;
    void     _correctReadings(int *x, int *y);
};

#endif
