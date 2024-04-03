#include "StateLed.h"
#include "SysLog.h"

void StateLed::begin(uint8_t ledPin) {
    _ledPin = ledPin;
    _blinks = 1;
    _blinkCycleTime = 1000;
    _blinkPeriod = 200;
    _canBlink = true;
    _lastBlinkCycleTime = millis();
    _lastBlinkTime = millis();
    _lastState = false;
    _currentBlink = 0;
    #if defined(CSTEPPER_INDICATOR_ON_GREEN_LED) && !defined(CSTEPPER_CONTROL_LOOP_ON_GREEN_LED)
        directModeOutput(_ledPin);
        directWriteLow(_ledPin);
    #endif
    INFO("[STATELED] Blink started on pin %d", _ledPin);
}

void StateLed::setup(uint8_t blinks, uint16_t blinkCycleTime, uint16_t blinksPeriod) {
    _blinks = blinks;
    _blinkCycleTime = blinkCycleTime;
    _blinkPeriod = blinksPeriod;
}

void StateLed::process(void) {
    #if defined(CSTEPPER_INDICATOR_ON_GREEN_LED) && !defined(CSTEPPER_CONTROL_LOOP_ON_GREEN_LED)
        if(_blinks != 0) {
            if(!_canBlink && millis() - _lastBlinkCycleTime >= _blinkCycleTime) {
                _lastBlinkCycleTime = millis();
                _canBlink = true;
            }

            if(_canBlink && millis() - _lastBlinkTime >= _blinkPeriod / 2) {
                _lastBlinkTime = millis();
                if(_currentBlink < _blinks) {
                    if(_lastState) {
                        directWriteLow(_ledPin);
                        _currentBlink++;
                    } else {
                        directWriteHigh(_ledPin);
                    }
                    _lastState = !_lastState;
                } else {
                    _currentBlink = 0;
                    _canBlink = false;
                }
            }
        }
    #endif
}