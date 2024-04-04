#ifndef SAVEDATA_H
#define SAVEDATA_H

#include <Arduino.h>
#include "Board.h"
#include <Preferences.h>
#include "SysLog.h"

#if IDENTIFIER == 0xC1
    #define DEFAULT_PARAMS_ZERO 35.95
    #define DEFAULT_PARAMS_SETPOINT 97.80
    #define DEFAULT_PARAMS_MINIMUM_ANGLE -408.65
    #define DEFAULT_PARAMS_MAXIMUM_ANGLE 148.26
#elif IDENTIFIER == 0xC2
    #define DEFAULT_PARAMS_ZERO 24.17
    #define DEFAULT_PARAMS_SETPOINT 58.25
    #define DEFAULT_PARAMS_MINIMUM_ANGLE -445.80
    #define DEFAULT_PARAMS_MAXIMUM_ANGLE 103.21
#elif IDENTIFIER == 0xC3
    #define DEFAULT_PARAMS_ZERO 12.57
    #define DEFAULT_PARAMS_SETPOINT 62.85
    #define DEFAULT_PARAMS_MINIMUM_ANGLE -432.70
    #define DEFAULT_PARAMS_MAXIMUM_ANGLE 121.17
#else 
    #define DEFAULT_PARAMS_ZERO 0.0
    #define DEFAULT_PARAMS_SETPOINT 0.0
    #define DEFAULT_PARAMS_MINIMUM_ANGLE 0.0
#define DEFAULT_PARAMS_MAXIMUM_ANGLE 0.0
#endif

#define DEFUALT_PARAMS_USE_ANGLE_LIMITS true
#define DEFAULT_PARAMS_SAFE_ANGLE 10.0
#define DEFAULT_PARAMS_ZERO_CURRENT 0.8
#define DEFAULT_PARAMS_CURRENT 1.4
#define DEFAULT_PARAMS_MODE 'x'
#define DEFAULT_PARAMS_CLOSE_LOOP_SAMPLE_RATE 6000.0

// Position pid default values
#define DEFAULT_PID_POS_PROPORTIONAL 15.00000
#define DEFAULT_PID_POS_INTEGRAL 0.00003
#define DEFAULT_PID_POS_DERIVATIVE 230.00000
#define DEFAULT_PID_POS_FRECUENCY 30.00000

// Velocity pid default values
#define DEFAULT_PID_VEL_PROPORTIONAL 0.00100
#define DEFAULT_PID_VEL_INTEGRAL 0.00100
#define DEFAULT_PID_VEL_DERIVATIVE 0.00000
#define DEFAULT_PID_VEL_FRECUENCY 100.00000

// NSVN -> NON VOLATILE STORAGE NAME
#define NVSN_STEPPER_PARAMETERS "SParameters"
#define NVSN_PID_POSITION "PIDPos"
#define NVSN_PID_VELOCITY "PIDVel"

// NVSK -> NON VOLATILE STORAGE KEY
// parameters keys
#define NVSK_MAX_CURRENT "maxcurrent"
#define NVSK_MODE "mode"
#define NVSK_SAMPLE_RATE "closeloophz"
#define NVSK_ZERO "zero"
#define NVSK_SETPOINT "setpoint"
#define NVSK_ZERO_CURRENT "zerocurrent"
#define NVSK_MAX_ANGLE "maxangle"
#define NVSK_MIN_ANGLE "minangle"
#define NVSK_SAFE_ANGLE "safeangle"
#define NVSK_USE_ANGLE_LIMIT "uselimits"

// pid keys
#define NVSK_PROPORTIONAL "proportional"
#define NVSK_INTEGRAL "integral"
#define NVSK_DERIVATIVE "derivative"
#define NVSK_FRECUENCY "freq"

// checksum keys
#define NVSK_CHECKSUM "checksum"

struct PID_t {
    float proportional;
    float integral;
    float derivative;
    float frecuency;
};

class NonVolatile{
    public:
        void begin(void);

        void loadStepperParameters(void);
        void saveStepperParameters(void);
        void saveDefaultParameters(void);

        void loadPIDPosition(void);
        void savePIDPosition(void);
        void saveDefaultPIDPosition(void);

        void loadPIDVelocity(void);
        void savePIDVelocity(void);
        void saveDefaultPIDVelocity(void);
        
        float maxStepperCurrent;
        float zeroStartCurrent;
        float zeroStartOffset;
        float setpointOffset;
        float closeLoopHz;
        char initialMode;

        bool usingAngleLimits;
        float highAngleLimit;
        float lowAngleLimit;
        float safeAngleLimit;

        PID_t position;
        PID_t velocity;

    private:
        Preferences storage;
        bool savedValues;
};

extern NonVolatile nonVolatile;

#endif