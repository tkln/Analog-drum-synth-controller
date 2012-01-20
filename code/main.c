#include <avr/io.h>
#include <stdint.h>
#include <avr/delay.h>

#define spi_send(data) SPDR = data; while(!(SPSR & (1<<SPIF)));

void spi_master_init(void);
void spi_transmit_byte(uint8_t byte);
void spi_trigger_latch(void);
void spi_transmit_word(uint16_t word);

void main(void){
	uint16_t i;
	spi_master_init();
	
	i = 0;

	while(1){
			
		spi_transmit_word(i);
		_delay_ms(30);
		i++;
	}
}

void spi_master_init(void){
	DDRB = (1<<PB2)|(1<<PB3)|(1<<PB5);
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
}

void spi_trigger_latch(void){
	/*Triggering output latch on 74595*/
	PORTB |= (1<<PB2);
	PORTB &= ~(1<<PB2);
}

void spi_transmit_byte(uint8_t byte){
	spi_send(byte);	
	spi_trigger_latch();
}

void spi_transmit_word(uint16_t word){	
	spi_send((uint8_t)(word>>8));
	spi_send((uint8_t)word);
	spi_trigger_latch();
}
