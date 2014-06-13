#include <avr/io.h>
#include <util/delay.h>

int main(void)
{

	DDRB = (1 << PORTB0);

    while(1)
    {

	PORTB = 0b0;
	_delay_ms(1000);
	PORTB = 0b1;
	_delay_ms(1000);
    }
}
