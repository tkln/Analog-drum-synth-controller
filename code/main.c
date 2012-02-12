/* 

	Copyright (C) 2011 Aapo Vienamo

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.


	Analong drum module midi interface firmware
  Made for atmega168 similar ones like atmega88, 8, 328 etc. would probably work also.
 	System diagram: 

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

	Design operational description
	Midi messages are read in the interrupt where all but note on 
	events on correct channel (channel is set with DIP-switches on 
	the back of the device.) are ignored. When a valid midi event
	arrives:
 	1 	velocity of the note on is set to dac
	2. 	analog demux updates that voltage to appropriate pin
	   	pin is determined by note to which the drum voice is set
  3. 	an spi package is crafted and sent to shift registers to
		  trigger the drum voice.
 	Outputs of shift registers aro connected to trigger pins of the drum modules.
 	Analog demux lines are connected to accent pins. 
*/


#include <avr/io.h>
#include <stdint.h>
#include <avr/delay.h>
#include <avr/interrupt.h>

#define spi_send(data) SPDR = data; while(!(SPSR & (1<<SPIF)));

#define OC0A PD6
#define SS PB2
#define MOSI PB3
#define SCK PB5
#define DAC_VAL OCR0A

#define BAUD 31250
#define BAUD_PRESCALE (((F_CPU / (31250 * 16UL))) - 1)

static inline void spi_master_init(void);
static inline void spi_transmit_byte(uint8_t byte);
static inline void spi_trigger_latch(void);
static inline void spi_transmit_word(uint16_t word);

static inline void pwm_dac_init(void);

static inline void usart_midi_init(uint16_t ubrr);
/* Channels are counted from 0 not 1 */
static inline void midi_set_listen_channel(uint8_t* data, uint8_t chnl);
static inline void midi_set_listen_event(uint8_t* data, uint8_t event);

typedef enum midi_event_stage_t{
	event_match,
	note,
	velocity
} midi_event_stage_t;

typedef struct midi_event_data_t{
	uint8_t event_type, 
					note, 
					velocity;
} midi_event_data_t;

/*Midi channels range from 1 to 16 but with 4 bits you can numbers expres from 0 to 15.*/
volatile uint8_t midi_listen_event; /* Status event to for trigger. Contains note on and channel. */
volatile uint8_t midi_trigger_event;
volatile uint32_t midi_frame_data;
volatile uint8_t note_offset=50;
volatile midi_event_stage_t midi_event_stage;
volatile midi_event_data_t midi_event_data;

void main(void){
	spi_master_init();
	pwm_dac_init();
	usart_midi_init(BAUD_PRESCALE);
	
	//midi_listen_event = 0b10010000; /*note on on channel 1 */
	midi_event_stage = event_match;
	midi_set_listen_event(&midi_listen_event, 0b1001);
	midi_set_listen_channel(&midi_listen_event, 8);

	sei();

	while(2){	
		_delay_ms(10);
		spi_transmit_word(0x0); /* To clean it. This is just for debugging! Wont be in final releas. */
	}
}
static inline void midi_set_listen_event(uint8_t* data, uint8_t event){
	*data = (*data & 0b00001111) + (event << 4);
}

static inline void midi_set_listen_channel(uint8_t* data, uint8_t chnl){
	*data = (*data & 0b11110000) + chnl;
}

static inline void usart_midi_init(uint16_t ubrr){
	UBRR0H = (uint8_t) (ubrr>>8);
	UBRR0L = (uint8_t) ubrr;
	
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);/*transmit for debugging*/
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);/*1 stop bit 8 data bits*/	

}

static inline void usart_transmit(uint8_t data){
	while(!(UCSR0A & (1<<UDRE0)))
		;
	UDR0 = data;
}

ISR(USART_RX_vect){
	uint8_t data = UDR0;
	switch(midi_event_stage){ /* Or how about using that struct as a list :p */
		case event_match: 
			if(data == midi_listen_event){
				spi_transmit_word(0xffff);
				midi_event_data.event_type = data;
				midi_event_stage++;
			}
			break;
		case note:
			midi_event_data.note = data;
			midi_event_stage++;
			break;
		case velocity:
			midi_event_data.velocity = data;
			DAC_VAL = (data<<1);			
			midi_event_stage = event_match;
		default:
			break;
	}

}

static inline void pwm_dac_init(void){
	DDRD |= (1<<OC0A);
	TCCR0A = (1<<COM0A1)|(1<<WGM01)|(1<<WGM00); /* clear on compare match, fast pwm*/
	TCCR0B = (1<<CS00); /*no clock prescaling f=fcpu*/
	/*OCR0A = 0xfa; */
}

static inline void spi_master_init(void){
	/*SS, MOSI and SCK pins set as output*/
	DDRB |= (1<<SS)|(1<<MOSI)|(1<<SCK);
	/*spi enabled, master mode, no clock bits set so it will be fosc/4*/
	SPCR = (1<<SPE)|(1<<MSTR);
}

static inline void spi_trigger_latch(void){
	/*triggering output latch on 74595*/
	PORTB |= (1<<SS);
	PORTB &= ~(1<<SS);
}

static inline void spi_transmit_byte(uint8_t byte){
	spi_send(byte);	
	//spi_send(0);
	spi_trigger_latch();
	spi_trigger_latch();
}

static inline void spi_transmit_word(uint16_t word){	
	spi_send((uint8_t)(word>>8));
	spi_send((uint8_t)word);
	spi_trigger_latch();
}
