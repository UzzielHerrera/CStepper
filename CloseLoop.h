#ifndef CLOSELOOP_H
#define CLOSELOOP_H

#include <Arduino.h>
#include "Board.h"
#include "NonVolatile.h"


class CloseLoop {
    public:
        // Methods
        void begin(void);
        void resetValue(void);
        void loadSavedData(void);

        // State Variables
        volatile float setpoint;
        volatile float currentPosition;
        volatile float lastPosition;
        volatile float currentPositionWrapped;
        volatile float lastPositionWrapped;

        volatile float velocityFilter;
        // volatile float p;
        // volatile float i;

        volatile float effort;
        volatile float lastEffort;
        volatile int absoluteEffort;
        volatile float error;
        volatile float lastError;
        volatile int counter;

        volatile int wrapCount;
        volatile int stepCount;
        int stepNumber;
        bool dir;

        volatile float ITerm;
        volatile float DTerm;

        volatile bool inAngleCalibration;

        char currentMode;
        bool print_yw;
};

extern CloseLoop controlLoop;

#ifdef DEBUG_SMOOTH
    extern volatile float can_r[DEBUG_MAX];
    extern volatile float smooth_r[DEBUG_MAX * 2];
    extern volatile uint32_t smooth_time[DEBUG_MAX * 2];
    extern volatile uint16_t smooth_r_point;
#endif

#endif
