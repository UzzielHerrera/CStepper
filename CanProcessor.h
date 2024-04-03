#ifndef CANPROCESSOR_H
#define CANPROCESSOR_H

#include <Arduino.h>
#include "Board.h"
#include "Parameters.h"
#include "CloseLoop.h"
#include "ESP32-TWAI-CAN.hpp"
#include "SysLog.h"
#include "SerialManager.h"
#include "StepperController.h"
#include "NonVolatile.h"

#define NOP_CMD 0x00

#define SMOOTH_MOVE_CMD 0xAF
#define REQ_CURRENT_POSITION_CMD 0xAE

#define SETPOINT_CMD 0xA0
#define REQ_SETPOINT_CMD 0xA1

#define ENABLE_CMD 0xA2
#define REQ_ENABLE_CMD 0xA3

#define MODE_CMD 0xA4
#define REQ_MODE_CMD 0xA5

#define DIR_CMD 0xB0
#define REQ_DIR_CMD 0xB1
#define STEP_CMD 0xB2

#ifdef DEBUG_SMOOTH
    extern CanFrame savedAngles[DEBUG_MAX];
#endif

class CanProcessor {
    public:
        void begin(void);
        void process(void);
        void commandCheck(void);

    private:
        CanFrame bufferFrame;
        int32_t dataBuffer;
        float floatBuffer;
        CanFrame message;
        bool isMessageComplete;
};

#endif