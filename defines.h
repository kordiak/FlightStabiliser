
#ifndef BASIC_DEFINES
#define BASIC_DEFINES

	#define F_SCL 1000
	#define I2_START_SET 0x08
	#define I2_START_REPEATED_SET 0x10

	#define I2C_DATA_SENT_ACK 0x28
	#define I2C_ADDR_READ_ACK 0x40
	#define I2C_ADDR_WRITE_ACK 0x18

	#define I2C_DATA_RECEIVED_NACK 0x58
	#define I2C_DATA_RECEIVED_ACK  0x50


	#define GYRO_CTRL1 0x20
	#define GYRO_CTRL4 0x23
	#define GYRO_CTRL5 0x24
	#define FIFO_CTRL 0x2E
	#define WHO_AM_I 0x0F
	#define OUT_Y_L 0x2A
	#define OUT_Y_H 0x2B
	#define I2C_OK 0


	extern const unsigned char DIR_WRITE;
	extern const unsigned char DIR_READ;
	extern const unsigned char TWSR_STATUS_MASK;
	extern const unsigned char slave_addr;


#endif
