#include "StepperController.h"

volatile bool controllerEnabled;
AS5600 zeroEncoder;
AS5047D encoder;
A4954 driver;
hw_timer_t *stepperControllerTimer = NULL;

volatile bool executingSmoothMove;
float finalAngle;
float left;
float delta_angle;
uint64_t microseconds;
uint32_t sections;
uint32_t currentSection;
uint32_t micros_per_section;
uint64_t lastSectionMoveTime;
int encoderValue = 0;


#ifdef CSTEPPER_CONTROLLER_ON_INTERRUPT
void IRAM_ATTR controllerInterruptHandler(void) {
#else
volatile uint64_t lastControlMicros = micros();
void controllerInterruptHandler(void) {
    if(micros() - lastControlMicros <= (uint16_t)(USECTOSEC / stepperParameters.Fs))
        return;
    lastControlMicros = micros();
#endif

    static int print_counter = 0;               //this is used by step response

    #if defined(CSTEPPER_CONTROL_LOOP_ON_GREEN_LED) && !defined(CSTEPPER_INDICATOR_ON_GREEN_LED)
        directWriteHigh(PIN_GREEN_LED);
    #endif
    
    
    controlLoop.currentPosition = calibrationTable[encoder.readEncoder()];                    //read encoder and lookup corrected angle in calibration lookup table
    
    if ((controlLoop.currentPosition - controlLoop.lastPosition) < -180.0) controlLoop.wrapCount += 1;      //Check if we've rotated more than a full revolution (have we "wrapped" around from 359 degrees to 0 or ffrom 0 to 359?)
    else if ((controlLoop.currentPosition - controlLoop.lastPosition) > 180.0) controlLoop.wrapCount -= 1;
    controlLoop.currentPositionWrapped = (controlLoop.currentPosition + (360.0 * controlLoop.wrapCount));              //yw is the wrapped angle (can exceed one revolution)

    if (controlLoop.currentMode == 'h') {                            //choose control algorithm based on mode
    	hybridControl();                            // hybrid control is still under development...
    } else {
        switch (controlLoop.currentMode) {
            case 'x':         // position control                        
                controlLoop.error = (controlLoop.setpoint - controlLoop.currentPositionWrapped);
                controlLoop.ITerm += (stepperParameters.pKi *controlLoop.error);                             //Integral wind up limit
                if (controlLoop.ITerm > 150.0) controlLoop.ITerm = 150.0;
                else if (controlLoop.ITerm < -150.0) controlLoop.ITerm = -150.0;          
                controlLoop.DTerm = stepperParameters.pLPFa * controlLoop.DTerm -  stepperParameters.pLPFb * stepperParameters.pKd*(controlLoop.currentPositionWrapped - controlLoop.lastPositionWrapped);
                controlLoop.effort = (stepperParameters.pKp * controlLoop.error) + controlLoop.ITerm + controlLoop.DTerm;
                break;
            case 'v':         // velocity controlr
                controlLoop.velocityFilter = stepperParameters.vLPFa * controlLoop.velocityFilter +  stepperParameters.vLPFb * (controlLoop.currentPositionWrapped - controlLoop.lastPositionWrapped);     //filtered velocity called "DTerm" because it is similar to derivative action in position loop
                controlLoop.error = (controlLoop.setpoint - controlLoop.velocityFilter);   //error in degrees per rpm (sample frequency in Hz * (60 seconds/min) / (360 degrees/rev) )
                controlLoop.ITerm += (stepperParameters.vKi * controlLoop.error);                 //Integral wind up limit
                if (controlLoop.ITerm > 200) controlLoop.ITerm = 200;
                else if (controlLoop.ITerm < -200) controlLoop.ITerm = -200;
                controlLoop.effort = ((stepperParameters.vKp * controlLoop.error) + controlLoop.ITerm - (stepperParameters.vKd * (controlLoop.error - controlLoop.lastError)));
                //Serial.println(e);
                break;
            case 't':         // torque control
                controlLoop.effort = 1.0 * controlLoop.setpoint ;
                break;
            default:
                controlLoop.effort = 0;
                break;
        }

        controlLoop.lastPosition = controlLoop.currentPosition;  //copy current value of y to previous value (y_1) for next control cycle before PA angle added
        if (controlLoop.effort > 0) {         //Depending on direction we want to apply torque, add or subtract a phase angle of PA for max effective torque.  PA should be equal to one full step angle: if the excitation angle is the same as the current position, we would not move!  
            //You can experiment with "Phase Advance" by increasing PA when operating at high speeds
            controlLoop.currentPosition += stepperParameters.PA;          //update phase excitation angle
            if (controlLoop.effort > stepperParameters.uMAX)     // limit control effort
            controlLoop.effort = stepperParameters.uMAX;       //saturation limits max current command
        } else {
            controlLoop.currentPosition -= stepperParameters.PA;          //update phase excitation angle
            if (controlLoop.effort < -stepperParameters.uMAX)    // limit control effort
                controlLoop.effort = -stepperParameters.uMAX;      //saturation limits max current command
        }

        controlLoop.absoluteEffort = abs(controlLoop.effort);       //
        #if defined(CSTEPPER_CONTROL_ERROR_ON_RED_LED) && !defined(CSTEPPER_BUSY_ON_RED_LED)
            if (abs(controlLoop.error) < 0.1) directWriteHigh(PIN_RED_LED);    // turn on LED if error is less than 0.1
            else directWriteLow(PIN_RED_LED);                  //digitalWrite(ledPin, LOW);
        #endif

        if(!controlLoop.inAngleCalibration)
            driver.output(-controlLoop.currentPosition, round(controlLoop.absoluteEffort));    // update phase currents
    }
     
    controlLoop.lastError = controlLoop.error;
    controlLoop.lastEffort = controlLoop.effort;
    controlLoop.lastPositionWrapped = controlLoop.currentPositionWrapped;

    if (controlLoop.print_yw && !controlLoop.inAngleCalibration){       //for step resonse... still under development
        print_counter += 1;  
        if (print_counter >= 5){    // print position every 5th loop (every time is too much data for plotter and may slow down control loop
            Serial.println(int(controlLoop.currentPositionWrapped * 1024));    //*1024 allows us to print ints instead of floats... may be faster
            print_counter = 0;
        }
    }

    if(controlLoop.inAngleCalibration && controlLoop.print_yw){       //for step resonse... still under development
        print_counter += 1;  
        if (print_counter >= 30){    // print position every 5th loop (every time is too much data for plotter and may slow down control loop
            Serial.println(controlLoop.currentPositionWrapped - nonVolatile.setpointOffset);    //*1024 allows us to print ints instead of floats... may be faster
            print_counter = 0;
        }
    }

    #if defined(CSTEPPER_CONTROL_LOOP_ON_GREEN_LED) && !defined(CSTEPPER_INDICATOR_ON_GREEN_LED)
        directWriteLow(PIN_GREEN_LED);
    #endif
}

#ifdef CSTEPPER_CONTROLLER_ON_INTERRUPT
void setupControllerInterrupt(void) { 
    stepperControllerTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(stepperControllerTimer, &controllerInterruptHandler, false);
    timerAlarmWrite(stepperControllerTimer, (uint64_t)(USECTOSEC / stepperParameters.Fs), true);
}
#endif

void enableControllerInterrupt(void) {
    #ifdef CSTEPPER_CONTROLLER_ON_INTERRUPT
        timerAlarmEnable(stepperControllerTimer);
    #endif
    controllerEnabled = true;
}

void disableControllerInterrupt(void) {
    #ifdef CSTEPPER_CONTROLLER_ON_INTERRUPT
        timerAlarmDisable(stepperControllerTimer);
    #endif
    controllerEnabled = false;
}

void initializeController(void) {
    encoder.begin(PIN_AS5047D_CS);
    zeroEncoder.begin(PIN_AS5600_DIR);
    driver.begin();
    #ifdef CSTEPPER_CONTROLLER_ON_INTERRUPT
        setupControllerInterrupt();
    #endif
}

void oneStep(void) {
    controlLoop.stepNumber += !controlLoop.dir ? +1 : -1;
    //output(1.8 * stepNumber, 64); //updata 1.8 to aps..., second number is control effort
    driver.output(stepperParameters.aps * controlLoop.stepNumber, (int)(0.33 * stepperParameters.uMAX));
    delay(10);
}

void zeroStartStep(void) { 
    controlLoop.stepNumber += !controlLoop.dir ? + 1 : -1;
    driver.output(stepperParameters.aps * controlLoop.stepNumber, (int)(stepperParameters.zeroMaxCurrent * stepperParameters.uMAX));
    delay(10);
}

void hybridControl() {        //still under development
	static int missed_steps = 0;
	static float iLevel = 0.6;  //hybrid stepping current level.  In this mode, this current is continuous (unlike closed loop mode). Be very careful raising this value as you risk overheating the A4954 driver!
	static float rSense = 0.15;

	if (controlLoop.currentPositionWrapped < controlLoop.setpoint - stepperParameters.aps) {
		missed_steps -= 1;
	} else if (controlLoop.currentPositionWrapped > controlLoop.setpoint + stepperParameters.aps) {
		missed_steps += 1;
	}

	driver.output(0.1125 * (-(controlLoop.setpoint - missed_steps)), (255 / 3.3) * (iLevel * 10 * rSense));
}

void processSmoothMove(void) {
    if(executingSmoothMove) {
        if(micros() - lastSectionMoveTime > micros_per_section){

            disableControllerInterrupt();

            controlLoop.setpoint += delta_angle;
            currentSection++;
            lastSectionMoveTime = micros();
            if (currentSection >= sections) {
                executingSmoothMove = false;
                if(controlLoop.setpoint != finalAngle) { controlLoop.setpoint = finalAngle; }
                #if defined(CSTEPPER_BUSY_ON_RED_LED) && !defined(CSTEPPER_CONTROL_ERROR_ON_RED_LED)
                    directWriteLow(PIN_RED_LED);
                #endif
            }

            enableControllerInterrupt();

        } else {
            return;
        }
    }
}

void smoothMove(void) {
    // check if destination angle is the current setpoint
    if (controlLoop.setpoint == finalAngle) return;
    // ALERT("finalAngle: %0.2f", controlLoop.setpoint);
  
    // Actualize parameters for movements
    left = finalAngle - controlLoop.setpoint;

    // delta_angle = delta_angle_init_value;
    delta_angle = 0.45;

    sections = (uint32_t)(abs(left / delta_angle));
    micros_per_section = (uint32_t)(microseconds / sections);
    currentSection = 0;

    // Recalculate delta angle for speed or smoothness
    // Smoothness will use short delta_angle to complete movements in larger periods of time
    while(micros_per_section >= 800 && delta_angle > 0.05625) {
        delta_angle = delta_angle / 2;
        sections = (uint32_t)(abs(left / delta_angle));
        micros_per_section = (uint32_t)(microseconds / sections);
        currentSection = 0; 
    }
    // Speed will use large delta_angle to complete movements in shorter periods of time
    while(micros_per_section <= 50 && delta_angle < 1.8) {
        delta_angle = delta_angle * 2;
        sections = (uint32_t)(abs(left / delta_angle));
        micros_per_section = (uint32_t)(microseconds / sections);
        currentSection = 0;
    }

    // Change delta sign according to direction
    delta_angle = left >= 0 ? +delta_angle : -delta_angle;
    
    // Init time, for debug purposes
    lastSectionMoveTime = micros();

    // Initialize movement process flag
    executingSmoothMove = true;

    #ifdef DEBUG_SMOOTH
    Serial.println(finalAngle);
    smooth_r[smooth_r_point] = finalAngle;
    smooth_r[smooth_r_point + DEBUG_MAX] = left;
    smooth_time[smooth_r_point] = sections;
    smooth_time[smooth_r_point + DEBUG_MAX] = micros_per_section;
    smooth_r_point++;
    if(smooth_r_point >= DEBUG_MAX) {smooth_r_point = 0; }
    #endif

    #if defined(CSTEPPER_BUSY_ON_RED_LED) && !defined(CSTEPPER_CONTROL_ERROR_ON_RED_LED)
        directWriteHigh(PIN_RED_LED);
    #endif

}

void zeroStart(void) {
    disableControllerInterrupt();
    float currentPosition = zeroEncoder.readRawAngle();
    uint16_t steps = 0;
    float middle = 180 + nonVolatile.zeroStartOffset;
    if(middle > 360.0) middle -= 360.0;
    float totalDistance = 0.0;
    
    if(nonVolatile.zeroStartOffset <= 180.0) {
        if(currentPosition <= middle && currentPosition >= nonVolatile.zeroStartOffset) {
            controlLoop.dir = false;
            // steps = (currentPosition - zeroOffset) / (aps / TRANSMISION_RELATION);
            totalDistance = currentPosition - nonVolatile.zeroStartOffset;
        } else {
            controlLoop.dir = true;
            if(currentPosition >= 180.0)
                totalDistance = (360.0 - currentPosition) + nonVolatile.zeroStartOffset;
            else
                totalDistance = nonVolatile.zeroStartOffset - currentPosition;
            // if(currentPosition >= 180.0)
            //     steps = ((360.0 - currentPosition) + zeroOffset) / (aps / TRANSMISION_RELATION);
            // else
            //     steps = (zeroOffset - currentPosition) / (aps / TRANSMISION_RELATION);
        }
    } else if (nonVolatile.zeroStartOffset >= 180.0) {
        if(currentPosition >= middle && currentPosition <= nonVolatile.zeroStartOffset) {
            controlLoop.dir = true;
            totalDistance = nonVolatile.zeroStartOffset - currentPosition;
            // steps = (zeroOffset - currentPosition) / (aps / TRANSMISION_RELATION);
        } else {
            controlLoop.dir = false;
            if(currentPosition <= 180.0)
                totalDistance = (360.0 - nonVolatile.zeroStartOffset) + currentPosition;
            else
                totalDistance = currentPosition - nonVolatile.zeroStartOffset;
            // if(currentPosition <= 180.0)
            //     steps = ((360.0 - zeroOffset) + currentPosition) / (aps / TRANSMISION_RELATION);
            // else
            //     steps = (currentPosition - zeroOffset) / (aps / TRANSMISION_RELATION);
        }
    }

    // controlLoop.dir = !controlLoop.dir;
    INFO("[ZEROSTART] Initialize homming");
    while(currentPosition <= nonVolatile.zeroStartOffset - 1.0 || currentPosition >= nonVolatile.zeroStartOffset + 1.0){
        zeroStartStep();
        currentPosition = zeroEncoder.readRawAngle();
        // LOG("[ZEROSTART] currentPosition: %0.2f", currentPosition);
    }
    INFO("[ZEROSTART] Finalize homming");
    controlLoop.stepNumber = 0;
}
