#ifndef SINETABLE_H
#define SINETABLE_H

#include "Board.h"


#define SINE_STEPS 1024L
#define SINE_MAX ((int32_t)(32768L))

#ifdef CSTEPPER_FAST_SINE_TABLE
    extern const int sineTable[];
#endif

#endif
