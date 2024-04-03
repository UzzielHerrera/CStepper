#ifndef SERIALMANAGER_H
#define SERIALMANAGER_H

#include <Arduino.h>
#include "Board.h"
#include "NonVolatile.h"
#include "SysLog.h"
#include "StepperController.h"

class SerialManager {
    public:
        void begin(void);
        void showMenu(void);
        void process(void);
        void printCurrentAngle(void);
        void sineGeneretor(void);
        void editParameters(void);
        void editPID(void);
        void editPositionPID(void);
        void editVelocityPID(void);
        void editOtherPID(void);
        void parameterQuery(void);
        void stepResponse(void);
        void calibrate(void);
        void antiCoggingCal(void);
        void calibrateZeroStart(void);
        void calibrateAngleLimits(void);
        
    private:
        volatile float angle = 0.0;
        volatile float time = 0.0;
};

extern SerialManager serialComm;

#endif