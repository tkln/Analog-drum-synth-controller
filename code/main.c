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


	Analong drum module MIDI interface firmware
    Made for atmega168 similar ones like atmega88, 8, 328 etc. would probably
    work also.
 	System diagram: 

	           +-----------------+ 
	+------+   |                 |   +---------+
	|      |   |                 |	 |         |--> lines out 
	| MIDI |-->| usart       spi |-->|  74595  |
	| opto |   |                 |   |shift reg|--\ serial daisy chaining
	+------+   |                 |   +---------+  |
	           |                 | /--------------/
	           |                 | | +---------+
	           |       AVR       | | |         |--> lines out
	           |    atmega168    | \>|  74595  |
	+--------+ |                 |   |         |
	|  DIP-  | |                 |   +---------+ amplifying from 0-5V to 0-15V
	|switches|>|                 |   +------------+   +---------+
	|chn sel | |       timer pwm |-->|rc lp filter|-->|  tl072  |---\
	+--------+ |                 |   +------------+   |  opamp  |   |
	           |                 |                    +---------+   |
	           |                 |            /---------------------/
  	           |              io |------\     | +--------------+    
               +-----------------+      |     \>|              |   
                                        |       |     4067     |
                                        |       | analog demux |-->lines out
                                        |       |              |
                                        \------>| adr bus      |
                                                +--------------+



	Design operational description  
	Midi messages are read in the interrupt where all but note on events on 
    correct channel (channel is set with DIP-switches on the back of the 
    device.) are ignored. When a valid midi even arrives:
 	1. velocity of the note on is set to dac
	2. analog demux updates that voltage to appropriate pin
	   pin is determined by note to which the drum voice is set
    3. an spi package is crafted and sent to shift registers to
		  trigger the drum voice.
 	Outputs of shift registers aro connected to trigger pins of the drum 
    modules. Analog demux lines are connected to accent pins. 
 */


#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define spi_send(data) SPDR = data; while(!(SPSR & (1<<SPIF)));

#define OC0A PD6
#define SS PB2
#define MOSI PB3
#define SCK PB5
#define DAC_VAL OCR0A

#define MUX_PORT PORTC 
/* mux enable line is PC4 */

#define BAUD 31250
#define BAUD_PRESCALE (((F_CPU / (31250 * 16UL))) - 1)
#define MAX_VOICES 16

static inline void spi_master_init(void);
static inline void spi_transmit_byte(uint8_t byte);
static inline void spi_trigger_latch(void);
static inline void spi_transmit_word(uint16_t word);

static inline void pwm_dac_init(void);

static inline void usart_midi_init(uint16_t ubrr);
/* Channels are counted from 0 not 1 */
static inline void midi_set_listen_channel(uint8_t* data, uint8_t chnl);
static inline void midi_set_listen_event(uint8_t* data, uint8_t event);

//static inline void mux_toggle_enable();

static inline void mux_init_io(void);
static inline void mux_dac_set(uint8_t adr, char val);

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

/* Midi channels range from 1 to 16 but the actual data reads form 0 to 15. */

/* Status event to for trigger. Contains note on and channel. */
volatile uint8_t midi_listen_event;
volatile uint8_t midi_trigger_event;
volatile uint32_t midi_frame_data;
volatile uint8_t note_offset;
volatile midi_event_stage_t midi_event_stage;
volatile midi_event_data_t midi_event_data;
volatile uint8_t note_vel_list[MAX_VOICES];

void main(void){
	volatile uint8_t i;
	spi_master_init();
	pwm_dac_init();
	mux_init_io();
	usart_midi_init(BAUD_PRESCALE);
	
	midi_event_stage = event_match;
	midi_set_listen_event((uint8_t*)&midi_listen_event, 0b1001);
	midi_set_listen_channel((uint8_t*)&midi_listen_event, 8);
	note_offset = 24;
	sei();
	DAC_VAL = 0;
	
	while(1){
		for(i = 0; i < MAX_VOICES; i++){	
			mux_dac_set(i, note_vel_list[i]);
			_delay_ms(5);
		}
        /* To clean it. This is just for debugging! Wont be in final releas. */
		spi_transmit_word(0x0); 
	}
}

ISR(USART_RX_vect){
	uint8_t data = UDR0;
	switch(midi_event_stage){ /* Or how about using that struct as a list :p */
		case event_match: 
			if(data == midi_listen_event){
				midi_event_data.event_type = data;
				midi_event_stage++;
			}
			break;
		case note:
			midi_event_data.note =  (uint16_t)(data - note_offset) ;
			spi_transmit_word(1<< midi_event_data.note);
			midi_event_stage++;
			break;
		case velocity:
			midi_event_data.velocity = data;
			midi_event_stage = event_match;
			note_vel_list[midi_event_data.note] = data;
			break;
		default:
			break;
	}
}

static inline void mux_init_io(void){
	DDRC = 0xff; /* whole port as output */
	MUX_PORT = 0|(1<<4); /* adress lines to 0 enable (active low) to high */
}

static inline void mux_dac_set(uint8_t adr, char val){
	DAC_VAL = val;
	MUX_PORT = adr&0b1111;
	MUX_PORT &= ~(1<<4);
//	_delay_ms(10);
	//MUX_PORT |= (1<<4);
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
	
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);/* transmit for debugging */
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);/* 1 stop bit 8 data bits */	

}

static inline void usart_transmit(uint8_t data){
	while(!(UCSR0A & (1<<UDRE0)))
		;
	UDR0 = data;
}

static inline void pwm_dac_init(void){
	DDRD |= (1<<OC0A);
    /* clear on compare match, fast pwm */
	TCCR0A = (1<<COM0A1)|(1<<WGM01)|(1<<WGM00); 
	TCCR0B = (1<<CS00); /* no clock prescaling f=fcpu */
	/* OCR0A = 0xfa; */
}

static inline void spi_master_init(void){
	/* SS, MOSI and SCK pins set as output */
	DDRB |= (1<<SS)|(1<<MOSI)|(1<<SCK);
	/* SPI enabled, master mode, no clock bits set so it will be fosc/4 */
	SPCR = (1<<SPE)|(1<<MSTR);
}

static inline void spi_trigger_latch(void){
	/* triggering output latch on 74595 */
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
