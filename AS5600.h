#ifndef AS5600_H
#define AS5600_H

#include <Arduino.h>
#include <Wire.h>
#include "Board.h"
#include "DirectIO.h"

#define AS5600_ADDRESS 0x36

#define AS5600_RAW_ANGLE 0x0C
#define AS5600_ANGLE 0x0E
#define AS5600_STATUS 0x0B

#define AS5600_MAGNET_HIGH 0x08
#define AS5600_MAGNET_LOW 0X10
#define AS5600_MAGNET_DETECT 0x20

class AS5600 {
    public:
        void begin(uint8_t directionPin);
        bool isConnected(void);
        uint16_t rawAngle(void);
        float readRawAngle(void);
        uint16_t readAngle(void);

        bool magnetTooStrong(void);
        bool magnetTooWeak(void);
        bool magnetDetect(void);

    private:
        uint8_t _directionPin;
        uint8_t readStatus(void);
        uint8_t readReg(uint8_t reg);
        uint16_t readReg2Bytes(uint8_t reg);
};

#endif