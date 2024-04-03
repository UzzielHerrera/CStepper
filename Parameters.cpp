#include "Parameters.h"
#include "math.h"

void Parameters::begin(void) {
    resetValues();
    loadSavedData();
}

void Parameters::resetValues(void) {
    Fs = 6500.0;

    pKp = 15.00000;
    pKi = 0.00001;
    pKd = 230.00000;
    pLPF = 30.00000;
    pLPFa = exp(pLPF*-2*3.14159/Fs);
    pLPFb = (1.0-pLPFa);

    vKp = 0.00100;
    vKi = 0.00100;
    vKd = 0.00000;
    vLPF = 100.00000;
    vLPFa = exp(vLPF*-2*3.14159/Fs);
    vLPFb = (1.0-vLPFa)* Fs * 0.16666667;

    spr = 200;
    aps = 360.0 / spr;
    cpr = 16384;
    stepangle = aps / 32.0;
    PA = aps;

    iMAX = 0.75;
    rSense = 0.100;
    uMAX = (1023 / 3.3) * (iMAX * 10 * rSense);

    zeroMaxCurrent = 0.33;
}

void Parameters::loadSavedData(void) {
    Fs = nonVolatile.closeLoopHz;

    pKp = nonVolatile.position.proportional;
    pKi = nonVolatile.position.integral;
    pKd = nonVolatile.position.derivative;
    pLPF = nonVolatile.position.frecuency;
    pLPFa = exp(pLPF*-2*3.14159/Fs); // z = e^st pole mapping
    pLPFb = (1.0-pLPFa);

    vKp = nonVolatile.velocity.proportional;
    vKi = nonVolatile.velocity.integral;
    vKd = nonVolatile.velocity.derivative;
    vLPF = nonVolatile.velocity.frecuency;
    vLPFa = exp(vLPF*-2*3.14159/Fs); // z = e^st pole mapping
    vLPFb = (1.0-vLPFa)* Fs * 0.16666667;

    iMAX = nonVolatile.maxStepperCurrent;
    uMAX = (1023 / 3.3) * (iMAX * 10 * rSense);
    INFO("[PARAMETERS] MaxStepperCurrent: %0.2f", iMAX);

    zeroMaxCurrent = nonVolatile.zeroStartCurrent;
    INFO("[PARAMETERS] ZeroMaxCurrent: %0.2f", zeroMaxCurrent);
}

