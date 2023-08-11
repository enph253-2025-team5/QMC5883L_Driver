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


#include <Wire.h>

MechaQMC5883::MechaQMC5883(TwoWire &wire, float x, float y, float e, float angle) :
    _wire(wire), _calibration{x, y, e, angle} {
}

void MechaQMC5883::setAddress(uint8_t addr) {
    _address = addr;
}

void MechaQMC5883::_WriteReg(byte Reg, byte val) {
    _wire.beginTransmission(_address); // start talking
    _wire.write(Reg);                 // Tell the HMC5883 to Continuously Measure
    _wire.write(val);                 // Set the Register
    _wire.endTransmission();
}

void MechaQMC5883::init() {
    _WriteReg(0x0B, 0x01);
    // Define Set/Reset period
    setMode(Mode_Continuous, ODR_200Hz, RNG_8G, OSR_512);
    /*
    Define
    OSR = 512
    Full Scale Range = 8G(Gauss)
    ODR = 200HZ
    set continuous measurement mode
    */
}

void MechaQMC5883::setMode(uint16_t mode, uint16_t odr, uint16_t rng, uint16_t osr) {
    _WriteReg(0x09, mode | odr | rng | osr);
}

void MechaQMC5883::softReset() {
    _WriteReg(0x0A, 0x80);
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
int MechaQMC5883::read(int *x, int *y, int *z) {
    _wire.beginTransmission(_address);
    _wire.write(0x00);
    int err = _wire.endTransmission();
    if (err) {
        return err;
    }
    _wire.requestFrom(_address, 7);
    *x = (int) (int16_t) (_wire.read() | _wire.read() << 8);
    *y = (int) (int16_t) (_wire.read() | _wire.read() << 8);
    *z = (int) (int16_t) (_wire.read() | _wire.read() << 8);

    _correctReadings(x, y);

    byte overflow = _wire.read() & 0x02;
    return overflow << 2;
}

int MechaQMC5883::read(int *x, int *y, int *z, int *a) {
    int err = read(x, y, z);
    *a      = azimuth(y, x);
    return err;
}

int MechaQMC5883::read(int *x, int *y, int *z, float *a) {
    int err = read(x, y, z);
    *a      = azimuth(y, x);
    return err;
}

float MechaQMC5883::azimuth(int *a, int *b) {
    float azimuth = atan2((int) *a, (int) *b) * 180.0 / PI;
    return azimuth < 0 ? 360 + azimuth : azimuth;
}

void MechaQMC5883::tare() {
    int err, x, y, z;
    for (int i = 0; i < 200; i++) { // discard first 200 values
        err = read(&x, &y, &z) * -1;
    }
    err        = read(&x, &y, &z) * -1;
    _zeroError = 360 - LIM_ANGLE(DEG(atan2(x, y)));
}

float MechaQMC5883::readAngle() {
    int err, x, y, z;
    err = read(&x, &y, &z) * -1;
    if (err) {
        Serial.println("Error occured reading from IMU.");
        init();
    }
    return LIM_ANGLE((360 - LIM_ANGLE(DEG(atan2(x, y)))) - _zeroError);
}

float MechaQMC5883::readRawAngle() {
    int err, x, y, z;
    err = read(&x, &y, &z) * -1;
    return 360 - LIM_ANGLE(DEG(atan2(x, y)));
}

void MechaQMC5883::printRaw() {
    int err, x, y, z;
    err = read(&x, &y, &z) * -1;
    if (err) {
        Serial.println("Error occured reading from IMU.");
        init();
    } else {
        Serial.print('(');
        Serial.print(x);
        Serial.print(",");
        Serial.print(y);
        Serial.print(')');
        Serial.println();
    }
}

void MechaQMC5883::_correctReadings(int *x, int *y) {
    // _calibration[4] = {xOffset, yOffset, a/b in desmos, angle}
    if (_calibration[0] != 0 and _calibration[1] != 0) {
        // Shift the center of the ellipse to the origin

        *x -= _calibration[0];
        *y -= _calibration[1];

        float _x = *x;
        float _y = *y;

        float sine   = sinf(_calibration[3] / 180 * PI);
        float cosine = cosf(_calibration[3] / 180 * PI);
        // Rotate the ellipse to the axis and compress the x axis into a circle
        *x = _x * cosine + _y * sine / _calibration[2];
        *y = -_x * sine + _y * cosine;

        _x = *x;
        _y = *y;

        *x = _x * cosine - _y * sine;
        *y = _x * sine + _y * cosine;
    }
}
