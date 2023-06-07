#ifndef I2C_DEF
#define I2C_DEF
	#include "defines.h"
	void i2cInit();
	uint8_t i2cStart();
	void i2cStop();
	void i2cSend(uint8_t data);
	uint8_t i2cSendAddress(uint8_t address);
	uint8_t i2cSendData(uint8_t data);
	uint8_t i2cReceive(uint8_t last, uint8_t * data);
	uint8_t i2cReadRegister(uint8_t reg, uint8_t * data);
	uint8_t i2cReadRegisters(uint8_t reg, uint8_t * data, uint8_t size);
	uint8_t i2cWriteRegister(uint8_t reg, uint8_t data);
#endif
