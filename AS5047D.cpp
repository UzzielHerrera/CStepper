#include "AS5047D.h"
#include "CalibrationTable.h"

static inline int getBit(int16_t data, int bit) {
    return (data >> bit) & 0x01;
}

static int getParity(uint16_t data) {
    int i, bits;
    data = data & 0x7FFF;
    bits = 0;
    for (i = 0; i < 16; i++){
        if(0 != (data & ((0x0001) << i))) {
            bits++;
        }
    }
    return (bits & 0x01); 
}

void AS5047D::begin(int csPin) {
    chipSelectionPin = csPin;

    directModeOutput(chipSelectionPin);
    directWriteHigh(chipSelectionPin);

    // SPISettings settingAS5047D(10000000, MSBFIRST, SPI_MODE1);
    SPISettings settingAS5047D(1000000, MSBFIRST, SPI_MODE1);
    SPI.begin();
    INFO("[Start] Initialize AS5047D magnetic encoder module");
    delay(100);
    SPI.beginTransaction(settingAS5047D);

    readEncoder();
    readEncoder();
    readEncoder();
    readEncoder();
    readEncoder();
}

int AS5047D::readEncoder(void) {
    long bufferAngle;
    directWriteLow(chipSelectionPin);

    byte msb = SPI.transfer(0xFF);
    byte lsb = SPI.transfer(0xFF);

    bufferAngle = (((msb << 8) | lsb) & 0b0011111111111111);

    directWriteHigh(chipSelectionPin);
    return bufferAngle;
}


int16_t AS5047D::readAddress(uint16_t addr) {
	uint16_t data;
	//make sure it is a read by setting bit 14
	addr = addr | 0x4000;

	//add the parity to the command
	if (1 == getParity(addr)) {
		addr = (addr & 0x7FFF) | 0x8000; 
	}

	directWriteLow(chipSelectionPin);
	//clock out the address to read
	SPI.transfer16(addr);

	directWriteHigh(chipSelectionPin);
	delayMicroseconds(1);
	directWriteLow(chipSelectionPin);

	//clock out zeros to read in the data from address
	data = SPI.transfer16(0x0000);
	directWriteHigh(chipSelectionPin);
	delayMicroseconds(1);

	//mask off the error and parity bits
	data = data & 0x3FFF; 
	return data;
}

void AS5047D::readEncoderDiagnostics(void) { 
    int16_t data;
	int m, d;

    NOTICE("[AS5047D] Start checking AS5047D diagnostics");
	data = readAddress(AS5047D_CMD_DIAAGC);
    INFO("[AS5047D] DIAAGC: 0x%04X", data);
    INFO("[AS5047D] MAGL: %d", getBit(data, 11));
    INFO("[AS5047D] MAGH: %d", getBit(data, 10));
    INFO("[AS5047D] COF: %d", getBit(data, 9));
    INFO("[AS5047D] LFGL: %d", getBit(data, 8));
    INFO("[AS5047D] AGC: %d", data & 0x0FF);

    data = readAddress(AS5047D_CMD_MAG);
    INFO("[AS5047D] CMAG: 0x%04X(%d)", data, data);

    data = readAddress(AS5047D_CMD_ANGLEUNC);
    m = (int)((float)data * AS5047D_DEGREES_PER_BIT);
    d = (int)((float)data * AS5047D_DEGREES_PER_BIT * 100 - m * 100);
    INFO("[AS5047D] CORDICANG: 0x%04X(%d) %d.%02d deg(est)", data, data, m, d);

    data = readAddress(AS5047D_CMD_ANGLECOM);
    m = (int)((float)data * AS5047D_DEGREES_PER_BIT);
    d = (int)((float)data * AS5047D_DEGREES_PER_BIT * 100 - m * 100);
    INFO("[AS5047D] DAECANG: 0x%04X(%d) %d.%02d deg(est)", data, data, m, d);
}

float AS5047D::readAngle(void) {
    // disableControllerInterrupt();
    const int avg = 10;
    int encoderReading = 0;

    for(int reading = 0; reading < avg; reading++) { 
        encoderReading += readEncoder();
        delayMicroseconds(10);
    }

    return calibrationTable[encoderReading / avg];
}