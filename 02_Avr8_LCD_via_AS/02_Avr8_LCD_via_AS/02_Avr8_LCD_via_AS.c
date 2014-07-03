/*
 * _02_Avr8_LCD_via_AS.c
 *
 * Created: 30/06/2014 15:44:03
 *  Author: roney
 
 *  Atualizado em 03/Jul/2014 - Atmel Studio 5.1.208
 
 */ 

#define  F_CPU 1000000UL 
#define DELAY_MOVER 50

#include <avr/io.h>
#include "lcd.h"
#include <util/delay.h>
#include <stdlib.h>

void direita8 (void);
void esquerda8 (void);

int main(void)
{
	DDRB |= (1<<PORTB0);
	lcd_init(LCD_DISP_ON);
	lcd_gotoxy(3,0); lcd_puts("by");
	lcd_gotoxy(1,1); lcd_puts("Roney");
	
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
		
	//	lcd_command(LCD_DISP_ON_BLINK);
	//	lcd_command(LCD_DISP_ON);
		
		esquerda8();
		direita8();
		direita8();
		esquerda8();
		
		_delay_ms(1000);
		
		lcd_clrscr();
			
		lcd_gotoxy(1,1);
		lcd_puts("Final");
		
		lcd_gotoxy(1,0);
		lcd_puts("Demo");
		
		unsigned short int b;
		char buf[2];
		
		for (b=21;b>0;b--)
		{
			lcd_gotoxy(6,0);
			itoa(b-1,buf,10);
			
			lcd_puts(buf);
			lcd_puts(" ");
			_delay_ms(1000);
		}
		
		_delay_ms(5000);
		
        //TODO:: Please write your application code 
    }
}

void esquerda8 (void)
{	unsigned short int a;
	for (a=0;a<8;a++)
	{
		lcd_command(LCD_MOVE_DISP_LEFT);
		_delay_ms(DELAY_MOVER);
	}
}

void direita8 (void)
{
	unsigned short int a;
	for (a=0;a<8;a++)
	{
		lcd_command(LCD_MOVE_DISP_RIGHT);
		_delay_ms(DELAY_MOVER);
	}
}