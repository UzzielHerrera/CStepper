#include "NonVolatile.h"

void NonVolatile::begin(void) {
    savedValues = false;
    loadStepperParameters();
    loadPIDPosition();
    loadPIDVelocity();
}

void NonVolatile::loadStepperParameters(void) {
    // Load stepper parameters
    storage.begin(NVSN_STEPPER_PARAMETERS, false);
    savedValues = storage.getBool(NVSK_CHECKSUM, false);
    if(!savedValues){
        storage.end();
        WARNING("[NVS] Loading stepper parameters default values to NVS");
        saveDefaultParameters();
        storage.begin(NVSN_STEPPER_PARAMETERS, false);
    }

    closeLoopHz = storage.getFloat(NVSK_SAMPLE_RATE, DEFAULT_PARAMS_CLOSE_LOOP_SAMPLE_RATE);
    lowAngleLimit = storage.getFloat(NVSK_MIN_ANGLE, DEFAULT_PARAMS_MINIMUM_ANGLE);
    highAngleLimit = storage.getFloat(NVSK_MAX_ANGLE, DEFAULT_PARAMS_MAXIMUM_ANGLE);
    safeAngleLimit = storage.getFloat(NVSK_SAFE_ANGLE, DEFAULT_PARAMS_SAFE_ANGLE);
    maxStepperCurrent = storage.getFloat(NVSK_MAX_CURRENT, DEFAULT_PARAMS_CURRENT);
    zeroStartCurrent = storage.getFloat(NVSK_ZERO_CURRENT, DEFAULT_PARAMS_ZERO_CURRENT);
    zeroStartOffset = storage.getFloat(NVSK_ZERO,DEFAULT_PARAMS_ZERO);
    setpointOffset = storage.getFloat(NVSK_SETPOINT, DEFAULT_PARAMS_SETPOINT);
    usingAngleLimits = storage.getBool(NVSK_USE_ANGLE_LIMIT, DEFUALT_PARAMS_USE_ANGLE_LIMITS);
    initialMode = storage.getChar(NVSK_MODE, 'e');
    if(initialMode == 'e') {
        ERROR("[NVS] Using default parameters");
        initialMode = DEFAULT_PARAMS_MODE;
    }
    storage.end();

    #ifdef CSTEPPER_SHOW_PARAMETERS_AT_STARTUP
        INFO("[NVS] closeLoopHz: %0.2f", closeLoopHz);
        INFO("[NVS] lowAngleLimit: %0.2f", lowAngleLimit);
        INFO("[NVS] highAngleLimit: %0.2f", highAngleLimit);
        INFO("[NVS] safeAngleLimit: %0.2f", safeAngleLimit);
        INFO("[NVS] maxStepperCurrent: %0.2f", maxStepperCurrent);
        INFO("[NVS] zeroStartCurrent: %0.2f", zeroStartCurrent);
        INFO("[NVS] zeroStartOffset: %0.2f", zeroStartOffset);
        INFO("[NVS] setpointOffset: %0.2f", setpointOffset);
        INFO("[NVS] usingAngleLimits: %s", usingAngleLimits ? "true" : "false");
        INFO("[NVS] initialMode: %c", initialMode);
    #endif
}

void NonVolatile::saveDefaultParameters(void) {
    storage.begin(NVSN_STEPPER_PARAMETERS, false);

    storage.putFloat(NVSK_SAMPLE_RATE, (float_t)DEFAULT_PARAMS_CLOSE_LOOP_SAMPLE_RATE);
    storage.putFloat(NVSK_MIN_ANGLE, (float_t)DEFAULT_PARAMS_MINIMUM_ANGLE);
    storage.putFloat(NVSK_MAX_ANGLE, (float_t)DEFAULT_PARAMS_MAXIMUM_ANGLE);
    storage.putFloat(NVSK_SAFE_ANGLE, (float_t)DEFAULT_PARAMS_SAFE_ANGLE);
    storage.putFloat(NVSK_MAX_CURRENT, (float_t)DEFAULT_PARAMS_CURRENT);
    storage.putFloat(NVSK_ZERO_CURRENT, (float_t)DEFAULT_PARAMS_ZERO_CURRENT);
    storage.putFloat(NVSK_ZERO, (float_t)DEFAULT_PARAMS_ZERO);
    storage.putFloat(NVSK_SETPOINT, (float_t)DEFAULT_PARAMS_SETPOINT);
    storage.putBool(NVSK_USE_ANGLE_LIMIT, (bool)DEFUALT_PARAMS_USE_ANGLE_LIMITS);
    storage.putChar(NVSK_MODE, (float_t)DEFAULT_PARAMS_MODE);

    storage.putBool(NVSK_CHECKSUM, true);
    storage.end();
    WARNING("[NVS] Default stepper parameter values loaded to NVS");
}

void NonVolatile::saveStepperParameters(void) {
    storage.begin(NVSN_STEPPER_PARAMETERS, false);

    storage.putFloat(NVSK_SAMPLE_RATE, closeLoopHz);
    storage.putFloat(NVSK_MIN_ANGLE, lowAngleLimit);
    storage.putFloat(NVSK_MAX_ANGLE, highAngleLimit);
    storage.putFloat(NVSK_SAFE_ANGLE, safeAngleLimit);
    storage.putFloat(NVSK_MAX_CURRENT, maxStepperCurrent);
    storage.putFloat(NVSK_ZERO_CURRENT, zeroStartCurrent);
    storage.putFloat(NVSK_ZERO, zeroStartOffset);
    storage.putFloat(NVSK_SETPOINT, setpointOffset);
    storage.putBool(NVSK_USE_ANGLE_LIMIT, usingAngleLimits);
    storage.putChar(NVSK_MODE, initialMode);

    storage.putBool(NVSK_CHECKSUM, true);
    storage.end();
    WARNING("[NVS] New stepper parameter values loaded to NVS");
}

void NonVolatile::loadPIDPosition(void) {
    storage.begin(NVSN_PID_POSITION, false);
    savedValues = storage.getBool(NVSK_CHECKSUM, false);
    
    if(!savedValues) {
        storage.end();
        WARNING("[NVS] Loading position PID default values to NVS");
        saveDefaultPIDPosition();
        storage.begin(NVSN_PID_POSITION, false);
    }

    position.proportional = storage.getFloat(NVSK_PROPORTIONAL, DEFAULT_PID_POS_PROPORTIONAL);
    position.integral = storage.getFloat(NVSK_INTEGRAL, DEFAULT_PID_POS_INTEGRAL);
    position.derivative = storage.getFloat(NVSK_DERIVATIVE, DEFAULT_PID_POS_DERIVATIVE);
    position.frecuency = storage.getFloat(NVSK_FRECUENCY, DEFAULT_PID_POS_FRECUENCY);

    storage.end();
    #ifdef CSTEPPER_SHOW_PARAMETERS_AT_STARTUP
        INFO("[NVS] position.proportional: %0.4f", position.proportional);
        INFO("[NVS] position.integral: %0.4f", position.integral);
        INFO("[NVS] position.derivative: %0.4f", position.derivative);
        INFO("[NVS] position.frecuency: %0.4f", position.frecuency);
    #endif
}

void NonVolatile::saveDefaultPIDPosition(void) {
    storage.begin(NVSN_PID_POSITION, false);

    storage.putFloat(NVSK_PROPORTIONAL, (float_t)DEFAULT_PID_POS_PROPORTIONAL);
    storage.putFloat(NVSK_INTEGRAL, (float_t)DEFAULT_PID_POS_INTEGRAL);
    storage.putFloat(NVSK_DERIVATIVE, (float_t)DEFAULT_PID_POS_DERIVATIVE);
    storage.putFloat(NVSK_FRECUENCY, (float_t)DEFAULT_PID_POS_FRECUENCY);

    storage.putBool(NVSK_CHECKSUM, true);
    storage.end();
    WARNING("[NVS] Default position PID values loaded to NVS");
}

void NonVolatile::savePIDPosition(void) {
    storage.begin(NVSN_PID_POSITION, false);

    storage.putFloat(NVSK_PROPORTIONAL, position.proportional);
    storage.putFloat(NVSK_INTEGRAL, position.integral);
    storage.putFloat(NVSK_DERIVATIVE, position.derivative);
    storage.putFloat(NVSK_FRECUENCY, position.frecuency);

    storage.putBool(NVSK_CHECKSUM, true);
    storage.end();
    WARNING("[NVS] New position PID values loaded to NVS");
}

void NonVolatile::loadPIDVelocity(void) {
    storage.begin(NVSN_PID_VELOCITY, false);
    savedValues = storage.getBool(NVSK_CHECKSUM, false);

    if(!savedValues) { 
        storage.end();
        WARNING("[NVS] Loading velocity PID default values to NVS");
        saveDefaultPIDVelocity();
        storage.begin(NVSN_PID_VELOCITY, false);
    }
    
    velocity.proportional = storage.getFloat(NVSK_PROPORTIONAL, DEFAULT_PID_VEL_PROPORTIONAL);
    velocity.integral = storage.getFloat(NVSK_INTEGRAL, DEFAULT_PID_VEL_INTEGRAL);
    velocity.derivative = storage.getFloat(NVSK_DERIVATIVE, DEFAULT_PID_VEL_DERIVATIVE);
    velocity.frecuency = storage.getFloat(NVSK_FRECUENCY, DEFAULT_PID_VEL_FRECUENCY);

    storage.end();
    #ifdef CSTEPPER_SHOW_PARAMETERS_AT_STARTUP
        INFO("[NVS] velocity.proportional: %0.4f", velocity.proportional);
        INFO("[NVS] velocity.integral: %0.4f", velocity.integral);
        INFO("[NVS] velocity.derivative: %0.4f", velocity.derivative);
        INFO("[NVS] velocity.frecuency: %0.4f", velocity.frecuency);
    #endif
}

void NonVolatile::saveDefaultPIDVelocity(void) {
    storage.begin(NVSN_PID_VELOCITY, false);

    storage.putFloat(NVSK_PROPORTIONAL, (float_t)DEFAULT_PID_VEL_PROPORTIONAL);
    storage.putFloat(NVSK_INTEGRAL, (float_t)DEFAULT_PID_VEL_INTEGRAL);
    storage.putFloat(NVSK_DERIVATIVE, (float_t)DEFAULT_PID_VEL_DERIVATIVE);
    storage.putFloat(NVSK_FRECUENCY, (float_t)DEFAULT_PID_VEL_FRECUENCY);

    storage.putBool(NVSK_CHECKSUM, true);
    storage.end();
    WARNING("[NVS] Default velocity PID values loaded to NVS");
}

void NonVolatile::savePIDVelocity(void) {
    storage.begin(NVSN_PID_VELOCITY, false);

    storage.putFloat(NVSK_PROPORTIONAL, velocity.proportional);
    storage.putFloat(NVSK_INTEGRAL, velocity.integral);
    storage.putFloat(NVSK_DERIVATIVE, velocity.derivative);
    storage.putFloat(NVSK_FRECUENCY, velocity.frecuency);

    storage.putBool(NVSK_CHECKSUM, true);
    storage.end();
    WARNING("[NVS] New velocity PID values loaded to NVS");
}