PROJECT=main
SOURCES=main.c
CC=avr-gcc
OBJCOPY=avr-objcopy
#MMCU=atmega88
MMCU=atmega168
PROGRAMMER=avrdude
F_CPU=20000000

CFLAGS=-mmcu=$(MMCU) -Wall -Os

$(PROJECT).hex: $(PROJECT).out 
	$(OBJCOPY) -j .text -O ihex $(PROJECT).out $(PROJECT).hex

$(PROJECT).out: $(SOURCES) Makefile
	$(CC) $(CFLAGS) -I./ -o $(PROJECT).out $(SOURCES) -DF_CPU=$(F_CPU)UL

program: $(PROJECT).hex
	$(PROGRAMMER) -c usbtiny  -p$(MMCU) -U flash:w:$(PROJECT).hex

clean:
	rm -f $(PROJECT).out
	rm -f $(PROJECT).hex

	#sudo avrdude -p $(MMCU) -c $(PROGRAMMER) -U flash:w:$(PROJECT).hex
