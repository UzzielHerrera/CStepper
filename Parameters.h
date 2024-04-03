//Contains the Mechaduino parameter declarations

#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "DirectIO.h"
#include "NonVolatile.h"

class Parameters {
    public:
        // Methods
        void begin(void);
        void resetValues(void);
        void loadSavedData(void);

        // Parameter variables
        volatile float Fs;
        volatile float pKp;
        volatile float pKi;
        volatile float pKd;
        volatile float pLPF;

        volatile float vKp;
        volatile float vKi;
        volatile float vKd;
        volatile float vLPF;

        volatile float pLPFa;
        volatile float pLPFb;
        volatile float vLPFa;
        volatile float vLPFb;

        int spr;
        float aps;
        int cpr;
        float stepangle;

        float zeroMaxCurrent;

        volatile float PA;
        float iMAX;
        float rSense;
        volatile int uMAX;
};

extern Parameters stepperParameters;

#endif
