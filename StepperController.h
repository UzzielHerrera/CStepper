#ifndef STEPPERCONTROLLER_H
#define STEPPERCONTROLLER_H

#include <Arduino.h>
#include "Board.h"
#include "Parameters.h"
#include "CloseLoop.h"
#include "CalibrationTable.h"
#include "SineTable.h"
#include "AS5047D.h"
#include "A4954.h"
#include "AS5600.h"

extern volatile bool controllerEnabled;
extern AS5600 zeroEncoder;
extern AS5047D encoder;
extern A4954 driver;

extern volatile bool executingSmoothMove;
extern float finalAngle;
extern float left;
extern float delta_angle;
extern uint64_t microseconds;
extern uint32_t sections;
extern uint32_t currentSection;
extern uint32_t micros_per_section;
extern uint64_t lastSectionMoveTime;

#ifndef CSTEPPER_CONTROLLER_ON_INTERRUPT
    void controllerInterruptHandler(void);
    extern volatile uint64_t lastControlMicros;
#endif

void initializeController(void);
void oneStep(void);
void setupControllerInterrupt(void);
void enableControllerInterrupt(void);
void disableControllerInterrupt(void);
void hybridControl(void);
void smoothMove(void);
void processSmoothMove(void);
void zeroStart(void);

#endif
