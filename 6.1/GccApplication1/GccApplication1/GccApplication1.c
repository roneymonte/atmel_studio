/*
 * GccApplication1.c
 *
 * Created: 11/11/2013 08:50:04
 *  Author: Roney
 */ 


#include <avr/io.h>
#include <util/delay.h> 

#define F_CPU 20000000


int main(void)
{
	DDRC = 0xFF;
	DDRB = 0xFF;
	
	PORTB = 0b0001;
	
    while(1)
    {
		PORTC |= (1<<0);
		//PAUSE 250 miliseconds
		PORTB = 0b0000;
		_delay_ms(250);
		
		//turns C0 LOW
		PORTC &= ~(1 << 0);
		//PAUSE 250 miliseconds
		PORTB = 0b0001;
		_delay_ms(250);
		
		PORTC ^= _BV(PC0);
		//_delay_ms(2500);

    }
}

