CC = avr-gcc
OC = avr-objcopy
F_CPU = 1000000UL

blink.hex: blink.elf
	${OC} -j.text -j.data -O ihex blink.elf blink.hex

blink.elf: blink.o
	${CC} -g -mmcu=atmega8 -o blink.elf i2c.o blink.o

i2c.o: i2c.c
	${CC} -g -Os -mmcu=atmega8 -c ./i2c.c -D F_CPU=${F_CPU}

blink.o: blink.c 
	${CC} -g -Os -mmcu=atmega8 -c ./blink.c -D F_CPU=${F_CPU}

install:
	avrdude -patmega8 -cusbasp -Uflash:w:blink.hex:i -P /dev/ttys000

clean:
	rm blink.hex blink.o blink.elf
