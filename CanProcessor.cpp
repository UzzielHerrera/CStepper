#include "CanProcessor.h"

#ifdef DEBUG_SMOOTH
CanFrame savedAngles[DEBUG_MAX];
#endif

void CanProcessor::begin(void) {
    ESP32Can.setPins(PIN_CAN_TX, PIN_CAN_RX);
    ESP32Can.setRxQueueSize(20);
	ESP32Can.setTxQueueSize(20);
    ESP32Can.setSpeed(ESP32Can.convertSpeed(500));
    isMessageComplete = false;
    if(ESP32Can.begin()) {
        INFO("[CAN] Canbus started correctly");
    } else {
        ERROR("[CAN] Canbus started failed");
    }
    dataBuffer = 0;
    floatBuffer = 0.0;
}

void CanProcessor::process(void) {
    // check if message is in can bus
    if(ESP32Can.readFrame(message, 0)) {
        if(message.data_length_code > 1) {
            // check that message is for this module
            if(message.data[0] == IDENTIFIER)
                commandCheck();
            return;
        } else {
            isMessageComplete = false;
            return;
        }
    }
}

// MASTER SHORT LENGHT DATA[0] DATA[1] DATA[2] ... DATA[N]
void CanProcessor::commandCheck(void){
    if(message.data_length_code >= 2) { 
        switch(message.data[1]){
            // Make smoothMove
            case SMOOTH_MOVE_CMD:
                if(message.data_length_code == 8) {
                    disableControllerInterrupt();

                    finalAngle = (message.data[3] << 8) + message.data[4] + (message.data[5] / 100.0);
                    finalAngle = message.data[2] == 0x00 ? -finalAngle : finalAngle;
                    finalAngle += nonVolatile.setpointOffset;
                    microseconds = (((message.data[6] << 8) + message.data[7]) / 100.0) * 1000000.0;

                    if(nonVolatile.usingAngleLimits) {
                        if(finalAngle > nonVolatile.highAngleLimit - nonVolatile.safeAngleLimit) {
                            finalAngle = nonVolatile.highAngleLimit - nonVolatile.safeAngleLimit;
                        }
                        if(finalAngle < nonVolatile.lowAngleLimit + nonVolatile.safeAngleLimit) {
                            finalAngle = nonVolatile.lowAngleLimit + nonVolatile.safeAngleLimit;
                        }
                    }

                    bufferFrame.identifier = IDENTIFIER;
                    bufferFrame.extd = 0;
                    bufferFrame.data_length_code = 3;
                    bufferFrame.data[0] = message.identifier;
                    bufferFrame.data[1] = SMOOTH_MOVE_CMD;
                    bufferFrame.data[2] = 0x01;
                    ESP32Can.writeFrame(bufferFrame, 0);
                    bufferFrame = { 0 };
                    message = { 0 };
                    #ifdef DEBUG_SMOOTH
                        can_r[smooth_r_point] = finalAngle;
                    #endif
                    smoothMove();
                    
                    enableControllerInterrupt();
                }
                break;

            case REQ_CURRENT_POSITION_CMD:
                bufferFrame.identifier =  IDENTIFIER;
                bufferFrame.extd = 0;
                bufferFrame.data_length_code = 6;
                bufferFrame.data[0] = message.identifier;
                bufferFrame.data[1] = REQ_CURRENT_POSITION_CMD;
                bufferFrame.data[2] = controlLoop.currentPositionWrapped >= 0.0 ? 0x01 : 0x00;
                dataBuffer = abs(controlLoop.currentPositionWrapped) * 100.0;
                bufferFrame.data[3] = ((dataBuffer / 100) >> 8) & 0xFF;
                bufferFrame.data[4] = (dataBuffer / 100) & 0xFF;
                bufferFrame.data[5] = (dataBuffer % 100) & 0xFF;
                ESP32Can.writeFrame(bufferFrame, 0);
                bufferFrame = {0};
                break;

            // Change setpoint 'r' in closed loop
            case SETPOINT_CMD:
                disableControllerInterrupt();
                if(message.data_length_code == 6) {
                    // data[3] angle integer msb, data[4] angle integer lsb, data[5] angle decimals as integer.
                    float value = (message.data[3] << 8)  + message.data[4] + (message.data[5] / 100.0);
                    // data[2] direction : 1 for positive values, 0 for negative values.
                    controlLoop.setpoint = message.data[2] == 1 ? + value: - value;
                    controlLoop.setpoint += nonVolatile.setpointOffset;
                    // Serial.printf("%0.2f\r\n", r);
                } else {
                    // reset to home if no angle in package
                    controlLoop.setpoint = 0.0 + nonVolatile.setpointOffset;
                    // Serial.printf("%0.2f", r);
                }
                enableControllerInterrupt();
                break;
                        // Return current setpoint
            case REQ_SETPOINT_CMD:
                bufferFrame.identifier = IDENTIFIER;
                bufferFrame.extd = 0;
                bufferFrame.data_length_code = 6;
                bufferFrame.data[0] = message.identifier; // return command to requested device
                bufferFrame.data[1] = REQ_SETPOINT_CMD;   // current command return
                floatBuffer = controlLoop.setpoint - nonVolatile.setpointOffset;
                bufferFrame.data[2] = floatBuffer >= 0 ? 0x01 : 0x00; // direction of setpoint
                dataBuffer = abs(floatBuffer) * 100.0;
                bufferFrame.data[3] = ((dataBuffer / 100) >> 8) & 0xFF; // angle integer msb
                bufferFrame.data[4] = (dataBuffer / 100) & 0xFF; // angle integer lsb
                bufferFrame.data[5] = (dataBuffer % 100) & 0xFF; // angle decimals as integer
                ESP32Can.writeFrame(bufferFrame, 0);
                bufferFrame = {0};
                // Serial.println("wRq");
                break;

            // Change closed loop state
            // Master_id Short 0x03 IDENTIFIER 0xA1 0x01 :: enable closed loop
            // Master_id Short 0x03 IDENTIFIER 0xA1 0x00 :: disable closed loop
            case ENABLE_CMD:
                if(message.data_length_code == 3) {
                    if(message.data[2] == 0x00) {
                        disableControllerInterrupt();      //disable closed loop
                        driver.setDAC(0, 0);
                        directWriteHigh(PIN_RED_LED);
                    } else if(message.data[2] == 0x01) {
                        controlLoop.setpoint = (encoder.readAngle() + (360.0 * controlLoop.wrapCount));          // hold the current position
                        // Serial.print("New setpoint ");
                        // Serial.println(r, 2);
                        NOTICE("New setpoint: %0.2f", controlLoop.setpoint);
                        enableControllerInterrupt();      //enable closed loop
                    }
                } else {
                    disableControllerInterrupt();
                    driver.setDAC(0, 0);
                }
                break;
            
            // Return close loop enable state.
            case REQ_ENABLE_CMD:
                bufferFrame.identifier = IDENTIFIER;
                bufferFrame.extd = 0;
                bufferFrame.data_length_code = 3;
                bufferFrame.data[0] = message.identifier;
                bufferFrame.data[1] = message.data[1];
                // 0x00 control disabled, 0x01 control enabled.
                bufferFrame.data[2] = controllerEnabled ? 0x01 : 0x00;
                ESP32Can.writeFrame(bufferFrame, 0);
                bufferFrame = {0};
                break;

            // Change current controller mode
            // 0x00 for position, 0x01 for velocity, 0x02 for torque, 0x03 for hybrid
            case MODE_CMD:
                if(message.data_length_code == 3) {
                    switch(message.data[2]) {
                        case 0x00: controlLoop.currentMode = 'x'; //position loop
                            break;
                        case 0x01: controlLoop.currentMode = 'v'; //velocity loop
                            break;
                        case 0x02: controlLoop.currentMode = 't'; //torque loop
                            break;
                        case 0x03: controlLoop.currentMode = 'h'; //hybrid loop
                            break;
                    }
                }
                break;

            // Return controller mode
            // 0x00 for position, 0x01 for velocity, 0x02 for torque, 0x03 for hybrid
            case REQ_MODE_CMD:
                bufferFrame.identifier = IDENTIFIER;
                bufferFrame.extd = 0;
                bufferFrame.data_length_code = 3;
                bufferFrame.data[0] = message.identifier;
                bufferFrame.data[1] = message.data[1];
                switch(controlLoop.currentMode) {
                    case 'x': bufferFrame.data[2] = 0x00; //position loop
                        break;
                    case 'v': bufferFrame.data[2] = 0x01; //velocity loop
                        break;
                    case 't': bufferFrame.data[2] = 0x02; //torque loop
                        break;
                    case 'h': bufferFrame.data[2] = 0x03; //hybrid loop
                        break;
                    default: bufferFrame.data[2] = 0x00;
                        break;
                }
                ESP32Can.writeFrame(bufferFrame, 0);
                bufferFrame = { 0};
                break;

            // Change direction in open loop
            case DIR_CMD:
                controlLoop.dir = message.data[3] == 0x01 ? true : false;
                break;
            
            case REQ_DIR_CMD:
                bufferFrame.identifier = IDENTIFIER;
                bufferFrame.extd = 0;
                bufferFrame.data_length_code = 3;
                bufferFrame.data[0] = message.identifier;
                bufferFrame.data[1] = message.data[1];
                bufferFrame.data[2] = controlLoop.dir ? 0x01 : 0x00;
                ESP32Can.writeFrame(bufferFrame, 0);
                bufferFrame = {0};
                break;

            // Give one step in dir of open loop
            case STEP_CMD:
                oneStep();
                break;
            
            // Print current Angle
            case NOP_CMD:
                serialComm.printCurrentAngle();
                break;
            
            default:
                break;
        }
    }
    isMessageComplete = false;
}
