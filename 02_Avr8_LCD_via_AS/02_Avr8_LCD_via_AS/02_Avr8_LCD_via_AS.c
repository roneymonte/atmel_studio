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
    while(1)
    {
		lcd_clrscr();
		
		lcd_puts("AVR 168P\n");
		lcd_puts("Atmel 62");
		
		_delay_ms(1000);
		
			
		
		
        //TODO:: Please write your application code 
    }
}