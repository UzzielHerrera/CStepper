#include "CloseLoop.h"

void CloseLoop::begin(void) {
    resetValue();
    loadSavedData();
}

void CloseLoop::resetValue(void) {
    // zeroOffset = 0.0;
    // setpointOffset = 0.0;
    setpoint = 0.0;
    currentPosition = 0.0;
    lastPosition = 0.0;
    currentPositionWrapped = 0.0;
    lastPositionWrapped = 0.0;

    velocityFilter = 0.0;
    // p = 0.0;
    // i = 0.0;

    effort = 0.0;
    lastEffort = 0.0;
    absoluteEffort = 0;
    error = 0.0;
    lastError = 0.0;

    counter = 0;
    wrapCount = 0;

    stepCount = 0;
    stepNumber = 0;
    dir = false;

    ITerm = 0.0;
    DTerm = 0.0;

    currentMode = 'n';

    inAngleCalibration = false;

    print_yw = false;
}

void CloseLoop::loadSavedData(void) {
    lastPositionWrapped = nonVolatile.setpointOffset;
    setpoint = nonVolatile.setpointOffset;
    currentMode = nonVolatile.initialMode;
}

#ifdef DEBUG_SMOOTH
    volatile float can_r[DEBUG_MAX];
    volatile float smooth_r[DEBUG_MAX * 2];
    volatile uint32_t smooth_time[DEBUG_MAX * 2];
    volatile uint16_t smooth_r_point;
#endif
