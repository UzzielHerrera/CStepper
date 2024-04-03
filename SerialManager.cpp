#include "SerialManager.h"
#ifdef DEBUG_SMOOTH
#include "CanProcessor.h"
#endif

void SerialManager::begin(void) {
    #ifndef SERIAL_ALTERNATIVE
        Serial.begin(SERIAL_BAUDRATE);
    #else
        Serial.begin(SERIAL_BAUDRATE, SERIAL_8N1, PIN_RXD, PIN_TXD);
    #endif
    SysLogInit(&Serial, LOG_DEBUG);
    NOTICE("[Start] Initialize SerialManager module");
    NOTICE("[Start] Initialize SysLog module.");
    INFO("[Start] Identifier: '%02X'", IDENTIFIER);
    INFO("[Start] Running %s", VERSION);
}

void SerialManager::showMenu(void) { 
    disableControllerInterrupt();
    LOG("");
    NOTICE("----- Closed Stepper -----");
    LOG("%s", VERSION);
    LOG("Identifier: %02X", IDENTIFIER);
    LOG("Main menu");
    LOG("");
    LOG(" s  -  step");
    LOG(" d  -  dir");
    LOG(" p  -  print angle");
    LOG(" ");
    LOG(" i  -  angle limits calibration");
    LOG(" z  -  zero start calibration");
    LOG(" c  -  encoder calibration");
    LOG(" a  -  anticoggin calibration");
    LOG(" e  -  check encoder diagnositics");
    LOG(" q  -  parameter query");
    LOG("");
    LOG(" x  -  position mode");
    LOG(" v  -  velocity mode");
    LOG(" t  -  torque mode");
    LOG(" h  -  hybrid mode");
    LOG("");
    LOG(" y  -  enable control loop");
    LOG(" n  -  disable control loop");
    LOG(" r  -  enter new setpoint");
    LOG("");
    LOG(" j  -  step response");
    LOG(" k  -  edit controller gains -- note, these edits are stored in volatile memory and will be reset if power is cycled");
    LOG(" g  -  generate sine commutation table");
    LOG(" m  -  print main menu");
    LOG("");
    LOG(" u  -  edit stepper parameters");
    LOG(" o  -  reset controller");
    LOG(" R  -  print current setpoint");
    LOG(" F  -  smooth move");
    LOG(" b  -  debug smooth move(if enable in 'board.h')");
    enableControllerInterrupt();
}

void SerialManager::printCurrentAngle(void) {
    LOG("stepNumber: %d, angle: %0.2f, raw: %d", controlLoop.stepNumber, encoder.readAngle(), encoder.readEncoder());
}

void SerialManager::sineGeneretor(void) {
    int temp;
    LOG("");
    LOG("The sineGen() function in Utils.cpp generates a sinusoidal commutation table.");
    LOG("The below table should be copied into sineTable in SineTable.cpp.");
    LOG("");
    LOG("Printing sine look up table:...");
    LOG("");
    for (int x = 0; x <= 3600; x++)
    {
        // temp = round(1024.0 * sin((3.14159265358979 * ((x * 0.1 / 180.0) + 0.25))));
        temp = round(1024.0 * sin((3.14159265358979 * ((x * 0.1 / 180.0) + 0.0))));
        Serial.print(temp);
        Serial.print(", ");
    }
}

void SerialManager::process(void) { // Monitors serial for commands.  Must be called in routinely in loop for serial interface to work.

    if (Serial.available())
    {

        char inChar = (char)Serial.read();
        switch (inChar) {
            #ifdef DEBUG_SMOOTH
            case 'b':
                disableControllerInterrupt();
                driver.setDAC(0, 0);
                for(uint16_t  i = 0; i < DEBUG_MAX; i++){
                    Serial.printf("%4u, %10.2f, %10.2f, %10.2f, %10u, %10u\r\n", i, can_r[i], smooth_r[i], smooth_r[i + DEBUG_MAX],smooth_time[i], smooth_time[i + DEBUG_MAX]);
                }
                for(uint16_t  i = 0; i < DEBUG_MAX; i++){
                    Serial.printf("%4u  ", i);
                    for(uint16_t j = 0; j < savedAngles[i].data_length_code; j++) {
                        Serial.printf("%02X  ", savedAngles[i].data[j]);
                    }
                    Serial.println();
                }
                Serial.printf("smooth_r_point: %u\r\n", smooth_r_point);
                enableControllerInterrupt();
                break;
            #endif

            case 'o':
                ESP.restart();
                break;

            case 'i':
                calibrateAngleLimits();
                break;

            case 'p': // print
                printCurrentAngle();
                break;

            case 's': // step
                oneStep();
                printCurrentAngle();
                break;

            case 'd': // dir
                controlLoop.dir = controlLoop.dir ? false : true;
                break;

            case 'z':
                calibrateZeroStart();
                break;

            case 'c':
                calibrate(); // cal routine
                break;

            case 'e':
                disableControllerInterrupt();
                encoder.readEncoderDiagnostics(); // encoder error?
                enableControllerInterrupt();
                break;

            case 'y':
                controlLoop.setpoint = (encoder.readAngle() + (360.0 * controlLoop.wrapCount)); // hold the current position
                NOTICE("[CLOSELOOP] Close loop controller enabled");
                INFO("New setpoint: %0.2f", controlLoop.setpoint - nonVolatile.setpointOffset);
                enableControllerInterrupt(); // enable closed loop
                break;

            case 'n':
                disableControllerInterrupt(); // disable closed loop
                driver.setDAC(0, 0);
                directWriteHigh(PIN_RED_LED);
                NOTICE("[CLOSELOOP] Close loop controller disabled");
                break;

            case 'r': // new setpoint
                INFO("[SETPOINT] Setpoint:");
                while (Serial.available() == 0);
                angle = Serial.parseFloat() + nonVolatile.setpointOffset;
                if(nonVolatile.usingAngleLimits) {
                    if(angle >= (nonVolatile.highAngleLimit - nonVolatile.safeAngleLimit)) {
                        controlLoop.setpoint = nonVolatile.highAngleLimit - nonVolatile.safeAngleLimit;
                    } else if(angle <= (nonVolatile.lowAngleLimit + nonVolatile.safeAngleLimit)) {
                        controlLoop.setpoint = nonVolatile.lowAngleLimit + nonVolatile.safeAngleLimit;
                    } else {
                        controlLoop.setpoint = angle;
                    }
                } else {
                    controlLoop.setpoint = angle;
                }
                disableControllerInterrupt();
                NOTICE("[SETPOINT] %0.2f",controlLoop.setpoint - nonVolatile.setpointOffset);
                enableControllerInterrupt();
                break;

            case 'x':
                controlLoop.currentMode = 'x'; // position loop
                break;

            case 'v':
                controlLoop.currentMode = 'v'; // velocity loop
                break;

            case 't':
                controlLoop.currentMode = 't'; // torque loop
                break;

            case 'h': // hybrid mode
                controlLoop.currentMode = 'h';
                break;

            case 'q':
                parameterQuery(); // prints copy-able parameters
                break;

            case 'a': // anticogging
                antiCoggingCal();
                break;

            case 'k':
                editPID();
                break;

            case 'g':
                sineGeneretor();
                break;

            case 'm':
                showMenu();
                break;

            case 'j':
                stepResponse();
                break;

            case 'R':
                INFO("r = %0.2f", controlLoop.setpoint - nonVolatile.setpointOffset);
                break;
            
            case 'D':
                INFO("dir =%d", controlLoop.dir);
                break;
                
            case 'F':
                NOTICE("Angle:");
                while (Serial.available() == 0);
                finalAngle = Serial.parseFloat() + nonVolatile.setpointOffset;
                NOTICE("Time:");
                while(Serial.available() == 0);
                microseconds = Serial.parseFloat();
                NOTICE("a: %0.2f; t: %0.2f", finalAngle - nonVolatile.setpointOffset, microseconds);
                smoothMove();
                break;

            case 'u':
                editParameters();
                break;

            default:
                break;
        }
    }
}

void SerialManager::editParameters() {
    disableControllerInterrupt();
    bool quit = false;
    while(!quit) {
        
        NOTICE("Edit stepper parameters");
        INFO("f - frecuency");
        INFO("i - max stepper current");
        INFO("m - new init setpoint");
        INFO("a - safe angle limit");
        INFO("p - print saved data");
        INFO("s - save data");
        INFO("q - quit");

        while(Serial.available() == 0);
        char inChar0 = (char) Serial.read();

        switch(inChar0) {
            case 'f':
                INFO("Close loop frecuency: ");
                while(Serial.available() == 0);
                nonVolatile.closeLoopHz = Serial.parseFloat();
                NOTICE("closeLoopHz: %0.2f", nonVolatile.closeLoopHz);
                break;

            case 'i':
                INFO("maxStepperCurrent: ");
                while(Serial.available() == 0);
                nonVolatile.maxStepperCurrent = Serial.parseFloat();
                NOTICE("maxStepperCurrent: %0.2f", nonVolatile.maxStepperCurrent);
                break;

            case 'm':
                INFO("initialMode: ");
                while(Serial.available() == 0);
                nonVolatile.initialMode = (char)Serial.read();
                NOTICE("initialMode: %c", nonVolatile.initialMode);
                break;
            
            case 'z':
                INFO("zeroStartCurrent: ");
                while(Serial.available() == 0);
                nonVolatile.zeroStartCurrent = Serial.parseFloat();
                NOTICE("zeroStartCurrent: %0.2f", nonVolatile.zeroStartCurrent);
                break;

            case 'a':
                INFO("safeAngleLimit : ");
                while(Serial.available() == 0);
                nonVolatile.safeAngleLimit = Serial.parseFloat();
                NOTICE("safeAngleLimit: %0.2f", nonVolatile.safeAngleLimit);
                break;
            
            case 'p':
                WARNING("closeLoopHz: %0.2f", nonVolatile.closeLoopHz);
                WARNING("maxStepperCurrent: %0.2f", nonVolatile.maxStepperCurrent);
                WARNING("zeroStartCurrent: %0.2f", nonVolatile.zeroStartCurrent);
                WARNING("safeAngleLimit: %0.2f", nonVolatile.safeAngleLimit);
                WARNING("lowAngleLimit: %0.2f", nonVolatile.lowAngleLimit);
                WARNING("highAngleLimit: %0.2f", nonVolatile.highAngleLimit);
                WARNING("setpointOffset: %0.2f", nonVolatile.setpointOffset);
                WARNING("zeroStartOffset: %0.2f", nonVolatile.zeroStartOffset);
                break;

            case 's':
                INFO("Save? (y/n)");
                while(Serial.available() == 0);
                if(Serial.read() == 'y') {
                    nonVolatile.saveStepperParameters();
                    stepperParameters.loadSavedData();
                    controlLoop.loadSavedData();
                    NOTICE("new parameters saved!");
                }
                break;

            case 'q':
                quit = true;
                NOTICE("Done");
                break;
        }
    }
    enableControllerInterrupt();
}

void SerialManager::editPID() {
    NOTICE("Edit parameters:");
    INFO("p ----- position loop");
    INFO("v ----- velocity loop");
    INFO("o ----- other");

    while (Serial.available() == 0);
    char inChar2 = (char)Serial.read();

    switch (inChar2) {
        case 'p':
            editPositionPID();
            break;
        case 'v':
            editVelocityPID();
            break;
        case 'o':
            editOtherPID();
            break;
        default:
            break;
    }
}

void SerialManager::editPositionPID() {
    disableControllerInterrupt();
    bool quit = false;
    while (!quit) {
        NOTICE("Edit position loop gains:");
        INFO("p ----- pKp = %0.4f", stepperParameters.pKp);
        INFO("i ----- pKi = %0.4f", stepperParameters.pKi);
        INFO("d ----- pKd = %0.4f", stepperParameters.pKd);
        INFO("l ----- LPF = %0.4f", stepperParameters.pLPF);
        INFO("s ----- save");
        INFO("q ----- quit");

        while (Serial.available() == 0);
        char inChar3 = (char)Serial.read();

        switch (inChar3){
            case 'p':
                INFO("pKp = ?");
                while (Serial.available() == 0);
                stepperParameters.pKp = Serial.parseFloat();
                NOTICE("new pKp = %0.4f", stepperParameters.pKp);
                nonVolatile.position.proportional = stepperParameters.pKp;
                break;

            case 'i':
                INFO("pKi = ?");
                while (Serial.available() == 0);
                stepperParameters.pKi = Serial.parseFloat();
                NOTICE("new pKi = %0.4f", stepperParameters.pKi);
                nonVolatile.position.integral = stepperParameters.pKi;
                break;

            case 'd':
                INFO("pKd = ?");
                while (Serial.available() == 0);
                stepperParameters.pKd = Serial.parseFloat();
                NOTICE("new pKd = %0.4f", stepperParameters.pKd);
                nonVolatile.position.derivative = stepperParameters.pKd;
                break;

            case 'l':
                INFO("pLPF = ?");
                while (Serial.available() == 0);
                stepperParameters.pLPF = Serial.parseFloat();
                stepperParameters.pLPFa = exp(stepperParameters.pLPF * -2 * 3.14159 / stepperParameters.Fs);
                stepperParameters.pLPFb = (1.0 - stepperParameters.pLPFa);
                NOTICE("new pLPF = %0.4f", stepperParameters.pLPF);
                nonVolatile.position.frecuency = stepperParameters.pLPF;
                break;

            case 's':
                INFO("Save? (y/n)");
                while(Serial.available() == 0);
                if(Serial.read() == 'y')
                    nonVolatile.savePIDPosition();
                break;

            case 'q':
                quit = true;
                NOTICE("Done");
                break;

            default:
                break;
        }
    }
    enableControllerInterrupt();
}

void SerialManager::editVelocityPID() {
    disableControllerInterrupt();
    bool quit = false;
    while (!quit) {
        INFO("Edit velocity loop gains:");
        INFO("p ----- vKp = %0.4f", stepperParameters.vKp);
        INFO("i ----- vKi = %0.4f", stepperParameters.vKi);
        INFO("d ----- vKd = %0.4f", stepperParameters.vKd);
        INFO("l ----- LPF = %0.4f", stepperParameters.vLPF);
        INFO("q ----- quit");

        while (Serial.available() == 0);
        char inChar4 = (char)Serial.read();

        switch (inChar4) {
            case 'p':
                INFO("vKp = ?");
                while (Serial.available() == 0);
                stepperParameters.vKp = Serial.parseFloat();
                NOTICE("new vKp = %0.4f", stepperParameters.vKp);
                nonVolatile.velocity.proportional = stepperParameters.vKp;
                break;

            case 'i':
                INFO("vKi = ?");
                while (Serial.available() == 0);
                stepperParameters.vKi = Serial.parseFloat();
                NOTICE("new vKi = %0.4f", stepperParameters.vKi);
                nonVolatile.velocity.integral = stepperParameters.vKi;
                break;

            case 'd':
                INFO("vKd = ?");
                while (Serial.available() == 0);
                stepperParameters.vKd = Serial.parseFloat();
                NOTICE("new vKd = %0.4f", stepperParameters.vKd);
                nonVolatile.velocity.derivative = stepperParameters.vKd;
                break;

            case 'l':
                INFO("vLPF = ?");
                while (Serial.available() == 0);
                stepperParameters.vLPF = Serial.parseFloat();
                stepperParameters.vLPFa = (exp(stepperParameters.vLPF * -2 * 3.14159 / stepperParameters.Fs));
                stepperParameters.vLPFb = (1.0 - stepperParameters.vLPFa) * stepperParameters.Fs * 0.16666667;
                NOTICE("new vLPF = %0.4f", stepperParameters.vLPF);
                nonVolatile.velocity.frecuency = stepperParameters.vLPF;
                break;

            case 's':
                INFO("Save? (y/n)");
                while(Serial.available() == 0);
                if(Serial.read() == 'y')
                    nonVolatile.savePIDVelocity();
                break;
            
            case 'q':
                quit = true;
                NOTICE("done");
                break;

            default:
                break;
        }
    }
    enableControllerInterrupt();
}

void SerialManager::editOtherPID() {
    INFO("Edit other parameters:");
    INFO("p ----- PA = %0.4f", stepperParameters.PA);
    while (Serial.available() == 0);
    char inChar3 = (char)Serial.read();

    switch (inChar3) {
        case 'p':
            INFO("PA = ?");
            while (Serial.available() == 0);
            stepperParameters.PA = Serial.parseFloat();
            NOTICE("new PA = %0.4f", stepperParameters.PA);
            break;
        default:
            break;
    }
}

void SerialManager::parameterQuery() { // print current parameters in a format that can be copied directly in to Parameters.cpp
    Serial.println("----Current Parameters-----");
    Serial.println(' ');

    Serial.print("volatile float Fs = ");
    Serial.print(stepperParameters.Fs, DEC);
    Serial.println(";  //Sample frequency in Hz");
    Serial.println(' ');

    Serial.print("volatile float pKp = ");
    Serial.print(stepperParameters.pKp, DEC);
    Serial.println(";      //position mode PID vallues.");

    Serial.print("volatile float pKi = ");
    Serial.print(stepperParameters.pKi, DEC);
    Serial.println(";");

    Serial.print("volatile float pKd = ");
    Serial.print(stepperParameters.pKd, DEC);
    Serial.println(";");

    Serial.print("volatile float pLPF = ");
    Serial.print(stepperParameters.pLPF, DEC);
    Serial.println(";");

    Serial.println(' ');

    Serial.print("volatile float vKp = ");
    Serial.print(stepperParameters.vKp, DEC);
    Serial.println(";      //velocity mode PID vallues.");

    Serial.print("volatile float vKi = ");
    Serial.print(stepperParameters.vKi, DEC);
    Serial.println(";");
    // Serial.println(vKi * Fs, DEC);
    // Serial.println(" / Fs;");

    Serial.print("volatile float vKd = ");
    Serial.print(stepperParameters.vKd, DEC);
    Serial.println(";");
    // Serial.print(vKd / Fs);
    // Serial.println(" * FS;");
    Serial.print("volatile float vLPF = ");
    Serial.print(stepperParameters.vLPF, DEC);
    Serial.println(";");
}

void SerialManager::stepResponse() {                                // not done yet...
    enableControllerInterrupt(); // start in closed loop mode
    controlLoop.currentMode = 'x';
    controlLoop.setpoint = 0;

    unsigned long timing = micros();
    while (micros() - timing < 1000000);

    controlLoop.dir = true;

    timing = micros();
    while (micros() - timing < 100000);

    controlLoop.setpoint = 97.65; /// choose step size as you like, 97.65 gives a nice plot since 97.65*1024 = 10,000

    timing = micros();
    while (micros() - timing < 400000);

    controlLoop.print_yw = false;
    controlLoop.setpoint = 0;

    timing = micros();
    while (micros() - timing < 500000);

    disableControllerInterrupt();
}

void SerialManager::calibrate() { /// this is the calibration routine

    int encoderReading = 0; // or float?  not sure if we can average for more res?
    int currentencoderReading = 0;
    int lastencoderReading = 0;
    int avg = 10; // how many readings to average

    int iStart = 0; // encoder zero position index
    int jStart = 0;
    int stepNo = 0;

    int fullStepReadings[stepperParameters.spr];

    int fullStep = 0;
    int ticks = 0;
    float lookupAngle = 0.0;
    NOTICE("[CALIBRATION] Initialize encoder calibration");

    encoderReading = encoder.readEncoder();
    controlLoop.dir = true;
    oneStep();
    delay(500);

    if ((encoder.readEncoder() - encoderReading) < 0)
    {                                      // check which way motor moves when dir = true
        ERROR("[CALIBRATION] Wired backwards"); // rewiring either phase should fix this.  You may get a false message if you happen to be near the point where the encoder rolls over...
        return;
    }

    while (controlLoop.stepNumber != 0)
    { // go to step zero
        controlLoop.dir = controlLoop.stepNumber > 0 ? true : false;
        oneStep();
        delay(100);
    }

    controlLoop.dir = true;
    for (int x = 0; x < stepperParameters.spr; x++)
    { // step through all full step positions, recording their encoder readings

        encoderReading = 0;
        delay(20); // moving too fast may not give accurate readings.  Motor needs time to settle after each step.
        lastencoderReading = encoder.readEncoder();

        for (int reading = 0; reading < avg; reading++)
        { // average multple readings at each step
            currentencoderReading = encoder.readEncoder();
            if ((currentencoderReading - lastencoderReading) < (-(stepperParameters.cpr / 2)))
            {
                currentencoderReading += stepperParameters.cpr;
            }
            else if ((currentencoderReading - lastencoderReading) > ((stepperParameters.cpr / 2)))
            {
                currentencoderReading -= stepperParameters.cpr;
            }
            encoderReading += currentencoderReading;
            delay(10);
            lastencoderReading = currentencoderReading;
        }
        encoderReading = encoderReading / avg;

        if (encoderReading > stepperParameters.cpr)
        {
            encoderReading -= stepperParameters.cpr;
        }
        else if (encoderReading < 0)
        {
            encoderReading += stepperParameters.cpr;
        }

        fullStepReadings[x] = encoderReading;
        if (x % 20 == 0)
        {
            Serial.println();
            Serial.print(100 * x / stepperParameters.spr);
            Serial.print("% ");
        }
        else
        {
            Serial.print('.');
        }
        oneStep();
    }
    Serial.println();

    // Serial.println(" ");
    // Serial.println("ticks:");                        //"ticks" represents the number of encoder counts between successive steps... these should be around 82 for a 1.8 degree stepper
    // Serial.println(" ");
    for (int i = 0; i < stepperParameters.spr; i++)
    {
        ticks = fullStepReadings[mod((i + 1), stepperParameters.spr)] - fullStepReadings[mod((i), stepperParameters.spr)];
        if (ticks < -15000)
            ticks += stepperParameters.cpr;
        else if (ticks > 15000)
            ticks -= stepperParameters.cpr;

        if (ticks > 1)
        { // note starting point with iStart,jStart
            for (int j = 0; j < ticks; j++)
            {
                stepNo = (mod(fullStepReadings[i] + j, stepperParameters.cpr));
                // Serial.println(stepNo);
                if (stepNo == 0)
                {
                    iStart = i;
                    jStart = j;
                }
            }
        }
        if (ticks < 1)
        { // note starting point with iStart,jStart
            for (int j = -ticks; j > 0; j--)
            {
                stepNo = (mod(fullStepReadings[stepperParameters.spr - 1 - i] + j, stepperParameters.cpr));
                // Serial.println(stepNo);
                if (stepNo == 0)
                {
                    iStart = i;
                    jStart = j;
                }
            }
        }
    }

    NOTICE("[CALIBRATION] New encoder calibration table:"); // The lookup table is too big to store in volatile memory, so we must generate and print it on the fly

    for (int i = iStart; i < (iStart + stepperParameters.spr + 1); i++)
    {
        ticks = fullStepReadings[mod((i + 1), stepperParameters.spr)] - fullStepReadings[mod((i), stepperParameters.spr)];

        if (ticks < -15000) // check if current interval wraps over encoder's zero positon
            ticks += stepperParameters.cpr;
        else if (ticks > 15000)
            ticks -= stepperParameters.cpr;
        // Here we print an interpolated angle corresponding to each encoder count (in order)
        if (ticks > 1)
        { // if encoder counts were increasing during cal routine...
            if (i == iStart)
            { // this is an edge case
                for (int j = jStart; j < ticks; j++)
                {
                    // store_lookup(0.001 * mod(1000 * ((aps * i) + ((aps * j ) / float(ticks))), 360000.0));
                    lookupAngle = 0.001 * mod(1000 * ((stepperParameters.aps * i) + ((stepperParameters.aps * j) / float(ticks))), 360000.0);
                    Serial.print(lookupAngle);
                    Serial.print(" , ");
                }
            }
            else if (i == (iStart + stepperParameters.spr))
            { // this is an edge case
                for (int j = 0; j < jStart; j++)
                {
                    // store_lookup(0.001 * mod(1000 * ((aps * i) + ((aps * j ) / float(ticks))), 360000.0));
                    lookupAngle = 0.001 * mod(1000 * ((stepperParameters.aps * i) + ((stepperParameters.aps * j) / float(ticks))), 360000.0);
                    Serial.print(lookupAngle);
                    Serial.print(" , ");
                }
            }
            else
            { // this is the general case
                for (int j = 0; j < ticks; j++)
                {
                    // store_lookup(0.001 * mod(1000 * ((aps * i) + ((aps * j ) / float(ticks))), 360000.0));
                    lookupAngle = 0.001 * mod(1000 * ((stepperParameters.aps * i) + ((stepperParameters.aps * j) / float(ticks))), 360000.0);
                    Serial.print(lookupAngle);
                    Serial.print(" , ");
                }
            }
        }
        else if (ticks < 1)
        { // similar to above... for case when encoder counts were decreasing during cal routine
            if (i == iStart)
            {
                for (int j = -ticks; j > (jStart); j--)
                {
                    // store_lookup(0.001 * mod(1000 * (aps * (i) + (aps * ((ticks + j)) / float(ticks))), 360000.0));
                    lookupAngle = 0.001 * mod(1000 * (stepperParameters.aps * (i) + (stepperParameters.aps * ((ticks + j)) / float(ticks))), 360000.0);
                    Serial.print(lookupAngle);
                    Serial.print(" , ");
                }
            }
            else if (i == iStart + stepperParameters.spr)
            {
                for (int j = jStart; j > 0; j--)
                {
                    // store_lookup(0.001 * mod(1000 * (aps * (i) + (aps * ((ticks + j)) / float(ticks))), 360000.0));
                    lookupAngle = 0.001 * mod(1000 * (stepperParameters.aps * (i) + (stepperParameters.aps * ((ticks + j)) / float(ticks))), 360000.0);
                    Serial.print(lookupAngle);
                    Serial.print(" , ");
                }
            }
            else
            {
                for (int j = -ticks; j > 0; j--)
                {
                    // store_lookup(0.001 * mod(1000 * (aps * (i) + (aps * ((ticks + j)) / float(ticks))), 360000.0));
                    lookupAngle = 0.001 * mod(1000 * (stepperParameters.aps * (i) + (stepperParameters.aps * ((ticks + j)) / float(ticks))), 360000.0);
                    Serial.print(lookupAngle);
                    Serial.print(" , ");
                }
            }
        }
    }

    Serial.println("");
    NOTICE("[CALIBRATION] Finalize encoder calibration");
}

void SerialManager::antiCoggingCal() { // This is still under development...  The idea is that we can calibrate out the stepper motor's detent torque by measuring the torque required to hold all possible positions.
    Serial.println(" -----------------BEGIN ANTICOGGING CALIBRATION!----------------");
    controlLoop.currentMode = 'x';
    controlLoop.setpoint = calibrationTable[1];
    enableControllerInterrupt();
    for (uint32_t i = 0; i <= 1000000; i++)
    {
        delayMicroseconds(1);
    }
    for (int i = 1; i < 657; i++)
    {
        controlLoop.setpoint = calibrationTable[i];
        Serial.print(controlLoop.setpoint, DEC);
        Serial.print(" , ");
        for (uint32_t i = 0; i <= 100000; i++)
        {
            delayMicroseconds(1);
        }
        Serial.println(controlLoop.effort, DEC);
    }
    Serial.println(" -----------------REVERSE!----------------");
    for (int i = 656; i > 0; i--)
    {
        controlLoop.setpoint = calibrationTable[i];
        Serial.print(controlLoop.setpoint, DEC);
        Serial.print(" , ");
        for (uint32_t i = 0; i <= 100000; i++)
        {
            delayMicroseconds(1);
        }
        Serial.println(controlLoop.effort, DEC);
    }
    Serial.println(" -----------------DONE!----------------");
    disableControllerInterrupt();
}

void SerialManager::calibrateZeroStart(void) {
    disableControllerInterrupt();
    ALERT("[CALIBRATION] Initialize calibration of software ZeroStart");
    INFO("[CALIBRATION] Calibration of zeroOffset");

    uint8_t dataSet0 = 100;
    uint64_t offsetData0 = 0;
    for (uint8_t i = 0; i < dataSet0; i++){
        offsetData0 += zeroEncoder.rawAngle();
        delay(10);
    }
    nonVolatile.zeroStartOffset = ((offsetData0 / dataSet0) * 360.0) / 4096.0;

    NOTICE("[CALIBRATION] nonVolatile.zeroStartOffset = %0.2f;\r\n", nonVolatile.zeroStartOffset);
    INFO("[CALIBRATION] Calibration of setpointOffset");

    uint8_t dataSet1 = 100;
    float offsetData1 = 0;
    for(uint8_t i = 0; i < dataSet1; i++){
        offsetData1 += encoder.readAngle();
        delay(10);
    }
    nonVolatile.setpointOffset = offsetData1 / dataSet1;

    NOTICE("[CALIBRATION] nonVolatile.setpointOffset = %0.2f;\r\n", nonVolatile.setpointOffset);
    ALERT("[CALIBRATION] Finalize calibration of software ZeroStart");

    nonVolatile.saveStepperParameters();
    controlLoop.loadSavedData();
}

void SerialManager::calibrateAngleLimits(void) {
    disableControllerInterrupt();
    uint64_t startTime = millis();
    int calibrationWaitTime = 4000;
    NOTICE("[CALIBRATION] Initialize calibration of software stop angle limits");
    INFO("[CALIBRATION] Initialize calibration of minimumAngle");
    
    controlLoop.inAngleCalibration = true;
    enableControllerInterrupt();

    startTime = millis();
    while(millis() - startTime <= calibrationWaitTime);

    disableControllerInterrupt();
    driver.setDAC(0, 0);
    controlLoop.inAngleCalibration = false;
    nonVolatile.lowAngleLimit = controlLoop.currentPositionWrapped;
    NOTICE("[CALIBRATION] nonVolatile.minimumAngle = %0.2f", nonVolatile.lowAngleLimit);


    INFO("[CALIBRATION] Going home position");
    enableControllerInterrupt();
    startTime = millis();
    while(millis() - startTime <= 1000);

    disableControllerInterrupt();
    driver.setDAC(0, 0);
    INFO("[CALIBRATION] Initialize calibration of maximumAngle");
    controlLoop.inAngleCalibration = true;
    enableControllerInterrupt();

    startTime = millis();
    while(millis() - startTime <= calibrationWaitTime);

    disableControllerInterrupt();
    controlLoop.inAngleCalibration = false;
    nonVolatile.highAngleLimit = controlLoop.currentPositionWrapped;
    NOTICE("[CALIBRATION] nonVolatile.maximumAngle = %0.2f", nonVolatile.highAngleLimit);

    NOTICE("[CALIBRATION] Finalize calibration of software stop angle limits");
    nonVolatile.saveStepperParameters();
    controlLoop.loadSavedData();
    enableControllerInterrupt();
}