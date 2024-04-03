#ifndef AS5047D_H
#define AS5047D_H

#include <Arduino.h>
#include "SysLog.h"
#include <SPI.h>
#include "DirectIO.h"

#define AS5047D_DEGREES_PER_BIT (360.0/(float)(0x3FFF))

#define AS5047D_CMD_NOP     0x0000
#define AS5047D_CMD_ERRFL   0x0001
#define AS5047D_CMD_PROG    0X0003
#define AS5047D_CMD_DIAAGC  0x3FFC
#define AS5047D_CMD_MAG     0x3FFD
#define AS5047D_CMD_ANGLEUNC    0x3FFE
#define AS5047D_CMD_ANGLECOM    0x3FFF

class AS5047D {
    public:
        void begin(int csPin);
        int readEncoder(void);
        void readEncoderDiagnostics(void);
        float readAngle(void);

    private:
        int chipSelectionPin;
        int16_t readAddress(uint16_t addr);
};  

#endif