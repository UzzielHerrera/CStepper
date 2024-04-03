#include "CalibrationTable.h"
#include "NonVolatile.h"
#include "SysLog.h"
#include "Board.h"
#include "Parameters.h"
#include "CloseLoop.h"
#include "StepperController.h"
#include "SerialManager.h"
#include "CanProcessor.h"
#include "StateLed.h"

NonVolatile nonVolatile;
Parameters stepperParameters;
CloseLoop controlLoop;
SerialManager serialComm;
CanProcessor canComm;
StateLed stateLed;
bool initWithClosedLoop = false;

void setup() {
	delay(1000);
	serialComm.begin();

	NOTICE("[Start] Initialize SaveData module");
	nonVolatile.begin();

	NOTICE("[Start] Initialize Parameters module");
	stepperParameters.begin();

	NOTICE("[Start] Initialize CloseLoop module");
	controlLoop.begin();

	NOTICE("[Start] Setting up board pins");
	boardSetupPins();
	
	float initVoltage = GetMotorVoltage();
	if (initVoltage < 5.0) {
		ALERT("[ENERGY] No voltage, please energize");
		while (initVoltage < 5.0) {
			delay(1000);
			initVoltage = GetMotorVoltage();
		}
		delay(2000);
	} else {
		INFO("[ENERY] Voltage: %0.2f", initVoltage);
	}

	NOTICE("[Start] Initialize CAN module");
	canComm.begin();

	NOTICE("[Start] Initialize StepperController module");
	initializeController();

	if (calibrationTable[0] == 0 && calibrationTable[128] == 0 && calibrationTable[1024] == 0)
		ALERT("[CALIBRATION] calibrationTable is empty, run calibration");
	else
		NOTICE("[Start] Initialize CalibrationTable module");

  #if defined(CSTEPPER_RUN_ZERO_START)
	NOTICE("[Start] Initialize zero position");
	zeroStart();
  #endif

	NOTICE("[Start] Initialize StateLed module");
	stateLed.begin(PIN_GREEN_LED);
	stateLed.setup(3, 1500, 50);

	initWithClosedLoop = true;
}

void loop() {
	#ifndef CSTEPPER_CONTROLLER_ON_INTERRUPT
		if(controllerEnabled)
			controllerInterruptHandler();
	#endif

	if(initWithClosedLoop) {
		NOTICE("[Start] Initialize Controller interrupt");
		enableControllerInterrupt();
		initWithClosedLoop = false;
	}

	canComm.process();
	serialComm.process();
	stateLed.process();

	processSmoothMove();
}
