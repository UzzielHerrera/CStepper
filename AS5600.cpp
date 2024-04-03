#include "AS5600.h"


void AS5600::begin(uint8_t directionPin) {
    _directionPin = directionPin;
    directModeOutput(_directionPin);
    directWriteLow(_directionPin);
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SDL, 800000);
}

bool AS5600::isConnected(void) {
    Wire.beginTransmission(AS5600_ADDRESS);
    return(Wire.endTransmission() == 0);
}

uint16_t AS5600::rawAngle(void) {
    uint16_t value = readReg2Bytes(AS5600_RAW_ANGLE) & 0x0FFF;
    return value;
}

uint16_t AS5600::readAngle(void) {
    uint16_t value = readReg2Bytes(AS5600_ANGLE) & 0x0FFF;
    return value;
}

float AS5600::readRawAngle(void) {
    uint16_t value = readReg2Bytes(AS5600_RAW_ANGLE) & 0x0FFF;
    return (value * 360.0) / 4096.0;
}

uint8_t AS5600::readStatus(void) {
    uint8_t value = readReg(AS5600_STATUS);
    return value;
}

bool AS5600::magnetDetect() {
    return (readStatus() & AS5600_MAGNET_DETECT) > 1;
}

bool AS5600::magnetTooStrong() {
    return (readStatus() & AS5600_MAGNET_HIGH) > 1;
}


bool AS5600::magnetTooWeak() {
    return (readStatus() & AS5600_MAGNET_LOW) > 1;
}

uint8_t AS5600::readReg(uint8_t reg) {
    Wire.beginTransmission(AS5600_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(AS5600_ADDRESS, (uint8_t)1);
    uint8_t _data = Wire.read();
    return _data;
}


uint16_t AS5600::readReg2Bytes(uint8_t reg) {
    Wire.beginTransmission(AS5600_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(AS5600_ADDRESS, (uint8_t)2);
    uint16_t _data = Wire.read();
    _data <<= 8;
    _data += Wire.read();
    return _data;
}

