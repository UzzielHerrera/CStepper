#ifndef STATELED_H
#define STATELED_H

#include <Arduino.h>
#include "DirectIO.h"
#include "Board.h"

class StateLed {
    public:
        void begin(uint8_t ledPin);
        void setup(uint8_t blinks, uint16_t blinkCycleTime, uint16_t blinksPeriod);
        void process(void);
    
    private:
        int _ledPin;
        int _blinks;
        uint16_t _blinkCycleTime;
        uint16_t _blinkPeriod;
        bool _canBlink;
        uint32_t _lastBlinkCycleTime;
        uint32_t _lastBlinkTime;
        bool _lastState;
        int _currentBlink;
};

#endif