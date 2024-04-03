#include "A4954.h"

void A4954::setupDAC(void) {
    directModeOutput(PIN_A4954_VREF12);
    directModeOutput(PIN_A4954_VREF34);

    ledcSetup(A4954_CHANNEL_VREF12, A4954_DRIVER_FREQ, A4954_DRIVER_BITS);
    ledcAttachPin(PIN_A4954_VREF12, A4954_CHANNEL_VREF12);
    ledcWrite(A4954_CHANNEL_VREF12, A4954_DRIVER_INIT_PERCENT * stepperParameters.uMAX);

    ledcSetup(A4954_CHANNEL_VREF34, A4954_DRIVER_FREQ, A4954_DRIVER_BITS);
    ledcAttachPin(PIN_A4954_VREF34, A4954_CHANNEL_VREF34);
    ledcWrite(A4954_CHANNEL_VREF34, A4954_DRIVER_INIT_PERCENT * stepperParameters.uMAX);
}

void A4954::setDAC(int voltage12, int voltage34) {
    ledcWrite(A4954_CHANNEL_VREF12, voltage12);
    ledcWrite(A4954_CHANNEL_VREF34, voltage34);
}

void A4954::begin(void) {
    INFO("[Start] Initialize A4954 driver module");
    // LOG("PIN_A4954_IN1: %d", PIN_A4954_IN1);
    directModeOutput(PIN_A4954_IN1);

    // LOG("PIN_A4954_IN2: %d", PIN_A4954_IN2);
    directModeOutput(PIN_A4954_IN2);

    // LOG("PIN_A4954_IN3: %d", PIN_A4954_IN2);
    directModeOutput(PIN_A4954_IN3);

    // LOG("PIN_A4954_IN4: %d", PIN_A4954_IN3);
    directModeOutput(PIN_A4954_IN4);

    setupDAC();

    directWriteLow(PIN_A4954_IN1);
    directWriteHigh(PIN_A4954_IN2);
    directWriteLow(PIN_A4954_IN3);
    directWriteHigh(PIN_A4954_IN4);

}

void A4954::output(float theta, int effort) {
    int angle_12;
    int angle_34;
    int v_coil_12;
    int v_coil_34;

    int sin_coil_12;
    int sin_coil_34;
    int phase_multiplier = 10 * stepperParameters.spr / 4;

    angle_12 = mod((phase_multiplier *  theta), 3600);
    angle_34 = mod((phase_multiplier * theta) + 900, 3600);

    #ifdef CSTEPPER_FAST_SINE_TABLE
        sin_coil_12 = sineTable[angle_12];
        sin_coil_34 = sineTable[angle_34];
    #else 
        sin_coil_12 = sin(angle_12);
        sin_coil_34 = sin(angle_34);
    #endif

    v_coil_12 = ((effort * sin_coil_12) / 1024);
    v_coil_34 = ((effort * sin_coil_34) / 1024);

    setDAC(abs(v_coil_12), abs(v_coil_34));

    if(v_coil_12 >= 0) {
        directWriteHigh(PIN_A4954_IN2);
        directWriteLow(PIN_A4954_IN1);
    } else {
        directWriteLow(PIN_A4954_IN2);
        directWriteHigh(PIN_A4954_IN1);
    }

    if (v_coil_34 >= 0) {
        directWriteHigh(PIN_A4954_IN4);
        directWriteLow(PIN_A4954_IN3);
    } else {
        directWriteLow(PIN_A4954_IN4);
        directWriteHigh(PIN_A4954_IN3);
    }
}

