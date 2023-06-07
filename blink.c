#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdio.h>
volatile long startTime=0;
volatile long CurrentTime=0;
volatile long pulses =0;

volatile unsigned long long int ticks = 0;
volatile unsigned long timer1_millis=0;


const unsigned char DIR_WRITE = 0;
const unsigned char DIR_READ  = 1;
const unsigned char TWSR_STATUS_MASK = 0xf8;
const unsigned char slave_addr = 0b11010110;

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


// I set UBRR as 12 from net table 
//#define USART_BAUDRATE 9600 
//#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1) 



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


void init_millis(unsigned long f_cpu)
{

  unsigned long ctc_match_overflow;

  ctc_match_overflow = ((f_cpu / 10000)) -1; //when timer1 is this value 1/10 of milisecond passed
					     //because 1000 000(FPU) / 10 000 = 100 => 0.0000 01(time of one cycle)/100 = 0.0001s
  // (Set timer to clear when matching ctc_match_overflow) | (no plescaler)
  TCCR1B |= (1 << WGM12) | (1 << CS10);

  // high byte first, then low byte 
  OCR1AH = (ctc_match_overflow >> 8); 
  OCR1AL = ctc_match_overflow;

  // Enable the compare match interrupt
  TIMSK |= (1 << OCIE1A);

  //REMEMBER TO ENABLE GLOBAL INTERRUPTS AFTER THIS WITH sei(); !!!

}


unsigned long oneTenthOfMilli (void)
{
  unsigned long millis_return;

  // Ensure this cannot be disrupted
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    millis_return = timer1_millis;
  }
  return millis_return;
}


void initListening(){
	DDRD &= ~(1<<PIND2); //INT0 as input
	//PORTD |= (1<<PIND2);
	GICR |= (1<<INT0); //turn on interrupt 	
	MCUCR |=  (1<< ISC00); // on every change
}

void initOutput(){
	DDRC  |= 15;
	PORTC=0xff;
}


ISR(INT0_vect){		
	
	long temp = oneTenthOfMilli();
	if((PIND & (1<<PIND2))>>PIND2 )
	{
		startTime = temp;
	} else {	
		pulses = temp - startTime;
	}

	/*
	CurrentTime = oneTenthOfMilli();;
	if(CurrentTime > startTime ){
		pulses= CurrentTime - startTime; 
		startTime=CurrentTime;
	}*/
}


void usart_init(void)        //funkcja inicjalizująca usart 
{ 
    UBRRH = 0;
    UBRRL = 12;
	//(BAUD_PRESCALE >> 8);        //wpisanie starszego bajtu 
   // UBRRL =  BAUD_PRESCALE;             //wpisanie mlodszego bajtu 
 
    //UCSRA bez zmian - 0x00 
    //UCSRB = (1<<TXEN); 
    //zmiana trybu działania tylko dla D1            
//UCSRC = (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);         //praca synchroniczna, brak kontroli parzystości, 1 bit stopu, 8 bitów danych 
//UCSRC = (1<<URSEL)|(1<<USBS)|(1<<UCSZ0); 
UCSRB = (1<<RXEN) | (1<<TXEN);
UCSRC = (1<<URSEL) | (3<<UCSZ0);
} 
 
void timer0_init(void) 
{ 
    //praca w przerwaniu od przepelnienia, preskaler 256, wysylanie danych co 3,5ms 
    TCCR0 |= (1<<CS02) | (1<<CS00); 
    TIMSK |= (1<<TOIE0); 
} 
 

volatile int previous = 0;

uint8_t calibrateGyro(int32_t* gyroDiff)
{
	uint8_t returnStatus = I2C_OK;
	*gyroDiff =0;	
	const int numOfCalibrationMeasurment = 1000;
	for(int i=0; i<numOfCalibrationMeasurment; ++i)
	{
		_delay_ms(10);
		if(returnStatus == I2C_OK){
	
//			uint8_t yValueTab[2];
//			yValueTab[0]=0;
//			yValueTab[1]=0;

			int yValue = 0;
			returnStatus = i2cReadRegisters(OUT_Y_L,(uint8_t*)&yValue,2);
			//yValue = (yValueTab[1]<<8) | yValueTab[0];
			*gyroDiff+=yValue;
		}

	}
	*gyroDiff /=numOfCalibrationMeasurment;

return returnStatus;
}

const char numbers[10] = {//0              //1       //2         //3        //4i        //5         //6        //7        //8         //9
			0b11000000, 0b11111001,0b10100100, 0b10110000,0b10011001, 0b10010010, 0b10000010,0b11111000,0b10000000, 0b10010000
		}; 
void seg7Init(){
	DDRB |=(1<<PORTB0)|(1<<PORTB1)|(1<<PORTB2)|(1<<PORTB3)|(1<<PORTB4)|(1<<PORTB5)|(1<<PORTB6);
	DDRC |=(1<<PORTC0)|(1<<PORTC1)|(1<<PORTC2)|(1<<PORTC3);
}

void seg7Display(int numberToDisplay)
{
	static const int powOfTen[4] = {1,10,100,1000};
	static const uint8_t   seg7TurnOffMask = (1<<PORTC0)|(1<<PORTC1)|(1<<PORTC2)|(1<<PORTC3);

	for(int i=3; i>=0;--i)
	{
		int currentPowOfTen = powOfTen[i];
		int digit = (int)numberToDisplay/currentPowOfTen;	
		numberToDisplay-= currentPowOfTen*digit;
		PORTB = numbers[digit];	
		PORTC |=seg7TurnOffMask;
		PORTC  &=~(1<<i);

	}	
}




void  main(void){	
/*
	initListening();
	initOutput();
*/
	init_millis(F_CPU);

	PORTD = 0x02;        //pullup na TXC 
    	DDRB  |= (1<<PORTB1) | (1<<PORTB2);
	PORTB = 0xFF;   

	usart_init();
	while ( !( UCSRA & (1<<UDRE)) );
	UDR = 's';
	_delay_ms(2000);
	while ( !( UCSRA & (1<<UDRE)) );
	UDR = '2';

	seg7Init();	

	
	//_delay_ms(1000)
	
	
//	timer0_init();	
	sei();
	i2cInit();
	uint8_t data = 0;
	uint8_t  status=i2cReadRegister(GYRO_CTRL1,&data);
	if(status==I2C_OK) {
		data=data|0xF;
		status= i2cWriteRegister(GYRO_CTRL1,data); //turn on gyro
		
		i2cReadRegister(GYRO_CTRL4,&data);
		data=data|0x80;
		status= i2cWriteRegister(GYRO_CTRL4,data); //read both only

	}
		
	
/* // probably turn on fifo
		data=0x40;
		status= i2cWriteRegister(GYRO_CTRL5,data); 
		

		data=0x40;
		status = i2cWriteRegister(FIFO_CTRL,data);
//*/

PORTC=0xff;
int32_t calDiff =0;
status = calibrateGyro(&calDiff);

while ( !( UCSRA & (1<<UDRE)) );
    UDR = 'c';


	while(1){

		unsigned long howLongTakes1register = oneTenthOfMilli();;
#define TEST_TWI

#ifdef TEST_PWM_RCV

	if(pulses>10  && pulses <= 18){

	if(pulses>16){
		PORTC=~15;
	} else if(pulses>14){
		PORTC=~7;
	} else if(pulses>12){
		PORTC=~3;
	} else {
		PORTC=~1;
	}
	pulses=0;
	}
#endif
#ifdef TEST_TWI
	data =0;


	
	if(status == I2C_OK){
		


		int yValue = 0;
		status = i2cReadRegisters(OUT_Y_L,(uint8_t*)&yValue,2);

	/*	
		float yValueF = yValue/5.88;
		yValueF /8.75;
		yValueF /= 36;          
		if(yValueF!=0){
			previous = previous+ yValueF;
	*/			
		if(status==I2C_OK){


		
//
			yValue = (yValue-calDiff)*0.001203007518797;

//			yValue = (yValue-calDiff)/41/8.75;

			previous+=yValue;


			/*
				if(yValue>0)
					previous=yValue-sum;
				else
					previous=yValue+sum;
			//*/
		//		PORTC=~(yValue>>7>>4);
		//		PORTC=~(previous);
		//				_delay_ms(1); 
		}		
	}				
	

	

//		previous = oneTenthOfMilli() - howLongTakes1register;
		
//		PORTC = ~((oneTenthOfMilli() -  howLongTakes1register)>>4); 

		unsigned long now = oneTenthOfMilli();
	/*

		if(howLongTakes1register> now){
				const unsigned long temp = now;
				now = howLongTakes1register;
				howLongTakes1register = temp;
		}
	*/	
		while(now - howLongTakes1register<115)
		{
					now=oneTenthOfMilli();
					seg7Display(previous/8);
		}
#endif
	}

/*
unsigned long howLongTakes1register = oneTenthOfMilli();;
char tempBuf[7];
tempBuf[0]=' ';
tempBuf[1]=' ';
tempBuf[2]=' ';
tempBuf[3]=' ';
tempBuf[4]=' ';
tempBuf[5]=' ';
tempBuf[6]=' ';
sprintf(tempBuf,"%i ",previous/3.4);
for(int i=0; i<6; i++){
while ( !( UCSRA & (1<<UDRE)) );
    UDR = tempBuf[i];;
}

*/


}
ISR(TIMER1_COMPA_vect)
{
//  TCNT1=0;
//  TIFR |=~(1<<TOV1) | ~(1<<OCF1A) | ~(1<<OCF1B);	
  timer1_millis++;
//  PORTC= ~PORTC;
}

ISR(TIMER0_OVF_vect) 
{ 
	
char tempBuf[7];
tempBuf[0]=' ';
tempBuf[1]=' ';
tempBuf[2]=' ';
tempBuf[3]=' ';
tempBuf[4]=' ';
tempBuf[5]=' ';
tempBuf[6]=' ';
sprintf(tempBuf,"%i ",previous);
for(int i=0; i<6; i++){
while ( !( UCSRA & (1<<UDRE)) ); 
    UDR = tempBuf[i];; 
}

}
