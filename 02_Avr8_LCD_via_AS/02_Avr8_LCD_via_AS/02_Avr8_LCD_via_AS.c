/*
 * _02_Avr8_LCD_via_AS.c
 *
 * Created: 30/06/2014 15:44:03
 *  Author: roney
 */ 

#define  F_CPU 1000000UL 

#include <avr/io.h>
#include "lcd.h"
#include <util/delay.h>

int main(void)
{
	
	short int a;
	
	DDRB |= (1<<PORTB0);
	lcd_init(LCD_DISP_ON_CURSOR_BLINK);
	
    while(1)
    {
		PORTB = 1;		// liga apenas o primeiro bit de B
		_delay_ms(1000);	
		
		lcd_clrscr();
		PORTB = 0;		// desliga o primeiro bit;
		_delay_ms(3000);
				
		lcd_puts("AVRm168P\n");
		_delay_ms(250);
		lcd_puts("AtmelS62");
		_delay_ms(250);
		
		for (a=0;a<8;a++)
		{
		lcd_command(LCD_MOVE_DISP_LEFT);
		_delay_ms(250);
		}
		
		for (a=0;a<8;a++)
		{
			lcd_command(LCD_MOVE_DISP_RIGHT);
			_delay_ms(250);
		}
		_delay_ms(500);
		for (a=0;a<8;a++)
		{
			lcd_command(LCD_MOVE_DISP_RIGHT);
			_delay_ms(250);
		}
		_delay_ms(500);
		
		for (a=0;a<8;a++)
		{
			lcd_command(LCD_MOVE_DISP_LEFT);
			_delay_ms(250);
		}
		
		_delay_ms(1000);
		
			
		
		
        //TODO:: Please write your application code 
    }
}