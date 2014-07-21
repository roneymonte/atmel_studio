/*
 * _02_Avr8_LCD_via_AS.c
 *
 * Created: 30/06/2014 15:44:03 (criado no Atmel Studio 6.2)
 *  Author: roney
 
 *  Atualizado em 03/Jul/2014 - Atmel Studio 5.1.208
	Atualizado em 21/Jul/2014 - Atmel Studio 6.2
		  adicionado PWM na porta PB6 com LED adicional a conexao
		  de control de LCD (2 funcionalidades no mesmo pino PB6)
 */ 

#define F_CPU 1000000UL 
#define DELAY_MOVER 20
#define ATRASO 20
	//  ATRASO eh o intervalo em ms entre a mudanca de tempo do PWM

#include <avr/io.h>
#include "lcd.h"
#include <util/delay.h>
#include <stdlib.h>
#include <avr/sfr_defs.h> // contem a definicao do _BV
/* sfr_defs.h - macros for accessing AVR special function registers */

void direita8 (void);
void esquerda8 (void);
void pwm1(uint16_t);
void pwmoff(void);
void pwmBump(void);

int main(void)
{
	

	DDRB |= (1<<PORTB0);	// configura a porta B0 como output com led
	
	pwmBump();
	
	lcd_init(LCD_DISP_ON);
	lcd_gotoxy(3,0); lcd_puts("by");
	lcd_gotoxy(1,1); lcd_puts("Roney");
	
	pwmBump();
	
	/*
		OBS: fatalmente o LED em PB6 ira piscar fora da rotina do PWM
		pois a mesma porta eh compartilhada com o controle do LCD
	*/

	
    while(1)
    {
		PORTB = 1;		// liga apenas o primeiro bit de B
		lcd_clrscr();
		PORTB = 0;		// desliga o primeiro bit;
				
		lcd_puts("AVRm168P\n");
		_delay_ms(250);
		lcd_puts("AtmelS62");
		_delay_ms(250);
		
		pwmBump();
		
	//	lcd_command(LCD_DISP_ON_BLINK);
	//	lcd_command(LCD_DISP_ON);
		
		esquerda8();
		direita8();
		direita8();
		esquerda8();
		
		pwmBump();
		
		lcd_clrscr();
			
		lcd_gotoxy(1,1);
		lcd_puts("Final");
		
		lcd_gotoxy(1,0);
		lcd_puts("Demo");
		
		unsigned short int b;
		char buf[2];
		
		for (b=16;b>0;b--)		// 16 passos simbolizam 4 segundos
		{
			lcd_gotoxy(6,0);
			itoa(b-1,buf,10);
			
			lcd_puts(buf);
			lcd_puts(" ");
			_delay_ms(250);		//delay com 1/4 de segundo
		} 
		
		pwmBump();
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

void pwm1 (uint16_t valor)
{

	DDRD |= (1 << DDD6);
	// PD6 is now an output

	OCR0A = valor; // era 128
	// set PWM for 50% duty cycle

	TCCR0A |= (1 << COM0A1);
	// set none-inverting mode

	TCCR0A |= (1 << WGM01) | (1 << WGM00);
	// set fast PWM Mode

	TCCR0B |= (1 << CS01);
	// set prescaler to 8 and starts PWM

}

void pwmoff (void)
{
	// ate este ponto o PWM ainda esta EXPERIMENTAL
	
	// usando o pino D6, que eh chamado OC0A
	DDRD |= (1 << DDD6);
	// PD6 is now an output

	OCR0A = 0; // era 128
	// chamado de "variable TOP" 
	// set PWM for 50% duty cycle


	//TCCR0A |= (1 << COM0A1);
	// set none-inverting mode
	// COM0A1 e COM0A0 determina o modo de funcionamento da
	// porta OC0A (PD6)
	//TCCR0A &= ~(1 << COM0A1); // colocara 0 no TCCR0A
	TCCR0A &= _BV(COM0A1);

	//TCCR0A |= (1 << WGM01) | (1 << WGM00);
	// set fast PWM Mode
	//TCCR0A &= ~(1 << WGM01);
	//TCCR0A &= ~(1 << WGM00);
	TCCR0A &= _BV(WGM00) | _BV(WGM01);
	// exemplo TCCR0A = _BV (COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00);
	

	//TCCR0B |= (1 << CS01);
	// set prescaler to 8 and starts PWM
	//TCCR0B &= ~ (1 << CS01);
	TCCR0B &= _BV(CS01);
	
}

void pwmBump (void)
{
		uint8_t variavel;
		
		for(variavel=0;variavel<240;variavel += 8)
		{
			pwm1(variavel);
			_delay_ms(ATRASO);
		}
		for(variavel=248;variavel>=8;variavel -= 8)
		{
			pwm1(variavel);
			_delay_ms(ATRASO);
		}
		pwmoff();
	
}