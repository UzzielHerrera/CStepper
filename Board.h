#ifndef BOARD_H
#define BOARD_H

#include <Arduino.h>
#include "DirectIO.h"

// Firmware Version
#define VERSION "FW: 1.7.1"			//firmware version
#define IDENTIFIER 0xC2             // change this to help keep track

// Use to make close loop on interrupt
#define CSTEPPER_CONTROLLER_ON_INTERRUPT

// Use for faster sine table
#define CSTEPPER_FAST_SINE_TABLE

// Use to show close loop on green led 
// #define CSTEPPER_CONTROL_LOOP_ON_GREEN_LED

// Use to show stateled indicator on green led
#define CSTEPPER_INDICATOR_ON_GREEN_LED

// Use to show close loop error on red led
// #define CSTEPPER_CONTROL_ERROR_ON_RED_LED

// Use to show smooth move busy on red led
#define CSTEPPER_BUSY_ON_RED_LED

// Use to start with zero position start
#define CSTEPPER_RUN_ZERO_START

// Use to show parameters of stepper at startup
// #define CSTEPPER_SHOW_PARAMETERS_AT_STARTUP

// Define transmision relation of belt system
#define TRANSMISION_RELATION 3.0

// Use to debug smooth movement control
// #define DEBUG_SMOOTH
// #define DEBUG_MAX 500

// Serial alternative usage
// #define SERIAL_ALTERNATIVE

// Serial baudrate speed
#define SERIAL_BAUDRATE 115200

// Use serial alternative pins
#ifdef SERIAL_ALTERNATIVE
#define PIN_TXD 43
#define PIN_RXD 44
#endif

// Define AS5600 encoder pins
#define PIN_I2C_SDA 18
#define PIN_I2C_SDL 17
#define PIN_AS5600_DIR 16

// Define AS5047D encoder pins
#define PIN_AS5047D_CS 10
#define PIN_HSPI_MOSI 11
#define PIN_HSPI_SCK 12
#define PIN_HSPI_MISO 13

// Define switch pins
#define PIN_SW1 40
#define PIN_SW2 41

// Define can boardcast pins
#define PIN_CAN_RX 5
#define PIN_CAN_TX 4

// Define LED pins
#define PIN_RED_LED 1
#define PIN_GREEN_LED 2

// Define A4954 driver pins
#define PIN_A4954_IN1 35
#define PIN_A4954_IN2 45
#define PIN_A4954_IN3 47
#define PIN_A4954_IN4 48
#define PIN_A4954_VREF12 36
#define PIN_A4954_VREF34 21

#define A4954_CHANNEL_VREF12 0
#define A4954_CHANNEL_VREF34 1
#define A4954_DRIVER_FREQ 32768
#define A4954_DRIVER_BITS 10
#define A4954_DRIVER_INIT_PERCENT 0.0

#define A4954_CHANNEL_IN1 2
#define A4954_CHANNEL_IN2 3
#define A4954_CHANNEL_IN3 4
#define A4954_CHANNEL_IN4 5

// Define voltage pin
#define PIN_VMOTOR 6

// Time definitions
#define USECTOSEC 1000000.0

static void boardSetupPins(void){
	//setup switch pins
	pinMode(PIN_SW1, INPUT_PULLUP);
	pinMode(PIN_SW2, INPUT_PULLUP);

    directModeOutput(PIN_AS5047D_CS);
	directWriteHigh(PIN_AS5047D_CS);

	// setup as5047d
    directModeOutput(PIN_HSPI_MOSI);
	directWriteLow(PIN_HSPI_MOSI);
	directModeOutput(PIN_HSPI_SCK);
	directWriteLow(PIN_HSPI_SCK);
	pinMode(PIN_HSPI_MISO, INPUT);

    // setup the A4954 pins
	directModeOutput(PIN_A4954_IN1);
	directWriteLow(PIN_A4954_IN1);
	directModeOutput(PIN_A4954_IN2);
	directWriteLow(PIN_A4954_IN2);
	directModeOutput(PIN_A4954_IN3);
	directWriteLow(PIN_A4954_IN3);
	directModeOutput(PIN_A4954_IN4);
	directWriteLow(PIN_A4954_IN4);

    // setup the PWM for current on the A4954, set for low current
	directModeOutput(PIN_A4954_VREF12);
	directModeOutput(PIN_A4954_VREF34);
	directWriteLow(PIN_A4954_VREF12);
	directWriteLow(PIN_A4954_VREF34);

	// setup vmotor pin
	pinMode(PIN_VMOTOR, INPUT);

	// setup red led
	#if defined(CSTEPPER_BUSY_ON_RED_LED) || defined(CSTEPPER_ERROR_ON_RED_LED)
    	directModeOutput(PIN_RED_LED);
    	directWriteLow(PIN_RED_LED);
	#endif

	// setup green led
	#if defined(CSTEPPER_INDICATOR_ON_GREEN_LED) || defined(CSTEPPER_CONTROL_LOOP_ON_GREEN_LED)
		directModeOutput(PIN_GREEN_LED);
		directWriteLow(PIN_GREEN_LED);
	#endif
}

static float GetMotorVoltage(void) {
	float f;
	f = analogReadMilliVolts(PIN_VMOTOR) * 11.0;
	return f;
}

inline int mod(int xMod, int mMod) {
  return (xMod % mMod + mMod) % mMod;
}

#endif