/*
 * _04_UART_e_LCD.c
 *
 * Created: 23/07/2014 09:38:25
 *  Author: roney
 */ 


#include <avr/io.h>
#include "libs_aux/lcd.h"
#include "libs_aux/USART.h"

int main(void)
{
	char serialCharacter;
	
	lcd_init(LCD_DISP_ON);
	lcd_gotoxy(2,0); lcd_puts("Serial");
	lcd_gotoxy(3,1); lcd_puts("LCD");
	
	DDRD |= (1<<PD1);	// habilita o pino PD1 como TXD (output)
	DDRB |= (1<<PB0);	// habilita o LED no pino PB0
	
	PORTB|=(1<<PB0);
	
	initUSART();
	printString("Serial ok:");
	
	PORTB&=~(1<<PB0);
	
    while(1)
    {	PORTB|=(1<<PB0);
        serialCharacter = receiveByte();
		PORTB&=~(1<<PB0);
		lcd_putc(serialCharacter);
    }
}