/* copyright Aapo Vienamo 2012 */
/* Analong drum module midi interface firmware*/
/* Made for atmega168 similar ones like atmega88, 8, 328 etc. would probably work also.*/
/* System diagram: */
/*
           +-----------------+ 
+------+   |                 |   +---------+
|      |   |                 |	 |         |--> lines out 
| MIDI |-->| usart       spi |-->|  74595  |
| opto |   |                 |   |shift reg|--\ serial trough for daisy chaining
+------+   |                 |   +---------+  |
           |                 | /--------------/
           |                 | | +---------+
           |       AVR       | | |         |--> lines out
           |    atmega168    | \>|  74595  |
+--------+ |                 |   |         |
|  DIP-  | |                 |   +---------+ amplifying from 0-5V to 0-15V
|switches|>|                 |   +------------+   +---------+   +--------------+
|chn sel | |       timer pwm |-->|rc lp filter|-->|  tl072  |-->|              |
+--------+ |                 |   +------------+   |  opamp  |   |      4067    |
           |                 |                    +---------+   | analog demux |
           |                 |                                  |              |
           |              io |--------------------------------->| pin adr      |--> lines out
 					 +-----------------+                                  +--------------+
 */
/* 	Design operational description */
/* Midi messages are read in the interrupt where all but note on */
/* events on correct channel (channel is set with DIP-switches on */
/* the back of the device.) are ignored. When a valid midi event */
/* arrives: */
/*	1 	velocity of the note on is set to dac */
/* 	2. 	analog demux updates that voltage to appropriate pin */
/*	   	pin is determined by note to which the drum voice is set */
/*  3. 	an spi package is crafted and sent to shift registers to */
/*		  trigger the drum voice.  */
/* Outputs of shift registers aro connected to trigger pins of the drum modules. */
/* Analog demux lines are connected to accent pins. */


#include <avr/io.h>
#include <stdint.h>
#include <avr/delay.h>
#include <avr/interrupt.h>

#define spi_send(data) SPDR = data; while(!(SPSR & (1<<SPIF)));

#define OC0A PD6
#define SS PB2
#define MOSI PB3
#define SCK PB5

#define BAUD 31250
#define BAUD_PRESCALE (((F_CPU / (31250 * 8UL))) - 1)

void spi_master_init(void);
void spi_transmit_byte(uint8_t byte);
void spi_trigger_latch(void);
void spi_transmit_word(uint16_t word);

void pwm_dac_init(void);

void usart_midi_init(uint16_t ubrr);

typedef enum midi_event_stage_t{
	event_type,
	note,
	velocity
} midi_event_stage_t;

/*Midi channels range from 1 to 16 but with 4 bits you can numbers expres from 0 to 15.*/
volatile uint8_t midi_listen_channel;
volatile uint8_t midi_trigger_event;
volatile uint32_t midi_frame_data;
volatile midi_event_stage_t midi_event_stage;

void main(void){
	spi_master_init();
	pwm_dac_init();
	usart_midi_init(15);

	sei();

	while(2){	
		/*spi_transmit_word(i);*/
		/*OCR0A = (uint8_t)i ^ (~i<<3) ^ (i<<7) ;*/
		/*OCR0A = (uint8_t)i;*/
		/*_delay_ms(30);*/
		/*i++;*/
		/*for(i = 0; i < 16; i++){*/
		/*spi_transmit_word(1<<i);*/
		_delay_ms(10);
		/*}*/
		spi_transmit_word(0);
	}
}

void usart_midi_init(uint16_t ubrr){
	UBRR0H = (uint8_t) (ubrr>>8);
	UBRR0L = (uint8_t) ubrr;
	
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);/*transmit for debugging*/
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);/*1 stop bit 8 data bits*/	

}

void usart_transmit(uint8_t data){
	while(!(UCSR0A & (1<<UDRE0)))
		;
	UDR0 = data;
}

ISR(USART_RX_vect){
	uint8_t data = UDR0;
	switch(midi_event_stage){
		case event_type: 
			if((data&0x0f) == midi_listen_channel){ 
				spi_transmit_word(0xffff);
				midi_event_stage++;
			}
			break;
		default:
			break;
	}

}

void pwm_dac_init(void){
	DDRD |= (1<<OC0A);
	TCCR0A = (1<<COM0A1)|(1<<WGM01)|(1<<WGM00); /* clear on compare match, fast pwm*/
	TCCR0B = (1<<CS00); /*no clock prescaling f=fcpu*/
	/*OCR0A = 0xfa; */
}

void spi_master_init(void){
	/*SS, MOSI and SCK pins set as output*/
	DDRB |= (1<<SS)|(1<<MOSI)|(1<<SCK);
	/*spi enabled, master mode, no clock bits set so it will be fosc/4*/
	SPCR = (1<<SPE)|(1<<MSTR);
}

void spi_trigger_latch(void){
	/*triggering output latch on 74595*/
	PORTB |= (1<<SS);
	PORTB &= ~(1<<SS);
}

void spi_transmit_byte(uint8_t byte){
	spi_send(byte);	
	spi_trigger_latch();
	spi_trigger_latch();
}

void spi_transmit_word(uint16_t word){	
	spi_send((uint8_t)(word>>8));
	spi_send((uint8_t)word);
	spi_trigger_latch();
}
