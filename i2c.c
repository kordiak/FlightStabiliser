#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdio.h>

#include "i2c.h"

	const unsigned char DIR_WRITE = 0;
	const unsigned char DIR_READ  = 1;
	const unsigned char TWSR_STATUS_MASK = 0xf8;
	const unsigned char slave_addr = 0b11010110;




void i2cInit()
{
	TWCR = (1<<TWEN);
	TWSR &=~(1<<TWPS0) | ~(1<<TWPS1);
	TWBR = 2;//(uint8_t)((F_CPU/F_SCL-16) /2);
}
uint8_t i2cStart(){
	TWCR = (1<<TWEN) | (1<<TWSTA) | (1<<TWINT);
	while(!(TWCR & (1<<TWINT)));
	uint8_t status = TWSR & TWSR_STATUS_MASK;
	if( (status & I2_START_SET) || status & I2_START_REPEATED_SET){
		return I2C_OK;
	} else return status;
}
void i2cStop(){
	TWCR = (1<<TWEN) | (1<<TWSTO) | (1<<TWINT);
	while( TWCR & (1<<TWSTO));
}


void i2cSend(uint8_t data)
{
	TWDR =data;
	TWCR = (1<<TWEN) | (1<<TWINT);
	while(!(TWCR & (1<<TWINT)));
}

uint8_t i2cSendAddress(uint8_t address){
	i2cSend(address);
	uint8_t status = TWSR & TWSR_STATUS_MASK;
	if(address & 0b00000001){
		return (status == I2C_ADDR_READ_ACK? I2C_OK: status);
	} else 
	{
		return(status == I2C_ADDR_WRITE_ACK? I2C_OK: status);
	}

}

uint8_t i2cSendData(uint8_t data){
	i2cSend(data);
	uint8_t status = TWSR & TWSR_STATUS_MASK;
	return (status == I2C_DATA_SENT_ACK) ? I2C_OK : status;
}


uint8_t i2cReceive(uint8_t last, uint8_t * data){
	TWCR = last ? _BV(TWEN) | _BV(TWINT) : _BV(TWEN) | _BV(TWINT) | _BV(TWEA); 
	while (!(TWCR & _BV(TWINT)));
	*data = TWDR;
	uint8_t status = TWSR & TWSR_STATUS_MASK;
	if (last) {
		return status == I2C_DATA_RECEIVED_NACK ? I2C_OK : status; } 
	else {
		return status == I2C_DATA_RECEIVED_ACK ? I2C_OK : status; 
	}
}


uint8_t i2cReadRegister(uint8_t reg, uint8_t * data){

	uint8_t status = i2cStart();
	if(status==I2C_OK) status = i2cSendAddress(slave_addr & ~(1));
	if(status==I2C_OK) status = i2cSendData(reg);
	if(status==I2C_OK) status = i2cStart();
	if(status==I2C_OK) status = i2cSendAddress(slave_addr | 1);
	status = i2cReceive(1,data);
	i2cStop();
	return status;
}

uint8_t i2cReadRegisters(uint8_t reg, uint8_t * data, uint8_t size){
 uint8_t status = i2cStart();
 if(status==I2C_OK) status = i2cSendAddress(slave_addr & ~(1));
 if(status==I2C_OK) status = i2cSendData(reg|(1<<7));
 if(status==I2C_OK) status = i2cStart();
 if(status==I2C_OK) status = i2cSendAddress(slave_addr | 1);
 	
 for(int i=0; i<size && I2C_OK==status; i++)
 {
	status = i2cReceive(i==(size-1),data++);
 }
 i2cStop();
 return status;
}



uint8_t i2cWriteRegister(uint8_t reg, uint8_t data) {
uint8_t status=  i2cStart();
 if(status == I2C_OK) status = i2cSendAddress(slave_addr & ~1);
 if(status == I2C_OK) status = i2cSendData(reg);
 if(status == I2C_OK) status = i2cSendData(data);
 i2cStop();

return status;
}
