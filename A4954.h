#ifndef A4954_H
#define A4954_H

#include <Arduino.h>
#include "SysLog.h"
#include "DirectIO.h"
#include "Board.h"
#include "SineTable.h"
#include "Parameters.h"

class A4954{
    public:
        void begin(void);
        void output(float theta, int effort);
        void setupDAC(void);
        void setDAC(int voltage12, int voltage34);
};

#endif