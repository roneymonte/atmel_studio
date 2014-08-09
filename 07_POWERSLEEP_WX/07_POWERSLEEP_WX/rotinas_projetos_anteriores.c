/*
 * rotinas_projetos_anteriores.c
 *
 * Created: 09/08/2014 17:21:45
 *  Author: roney
 */ 

#define F_CPU 1000000L

#include <avr/io.h>
#include <util/delay.h>


void iniciaPORTAS (void)
{
	//DDRB |= (1<<PB0) | (1<<PB5);		// leds vermelho e amarelo (conjunto)
	//DDRD |= (1<<PD6) | (1<<PD1);		// led grande branco PD6 do PWM, e saida da UART TX PD1
	
	initUSART();						// por default essa biblioteca usa 9600 bps
};


uint16_t  coletarADC (char multiplexador)
{
	//ADCSRA &= ~(1<<ADEN);
	//ADMUX	= 0b00000000;
	ADMUX	= _BV(REFS0)|_BV(REFS1) ;	// inicia ADMUX
	// e configura voltagem referencia para 1.1v
	
	ADMUX	|=  multiplexador ;			// coleta analogica no pino multiplexador
	ADCSRA |= (1<<ADEN);				// habilita o AD
	_delay_ms(20);						// pausa para estabilizacao da voltagem no AD
	ADCSRA	|= (1<<ADSC);				// AD Start Conversion
	
	loop_until_bit_is_clear(ADCSRA, ADSC);	// espera num loop ate que o valor esteja disponivel
	
	//ADCSRA &= ~(1<<ADEN);				// Seria aconselhavel DESLIGAR o ADC depois da operacao
	return ADC;
}

uint16_t getLuz (void)
{
	uint16_t valorADC;		// coleta do ADC de 0 a 1023
	valorADC = coletarADC( (1<<MUX1)|(1<<MUX0) );	// coleta o pino C3
	//luz = ( (valorADC * 1.00	) / 1023 ) * 100;
	//printString( dtostrf( ( ( (valorADC * 1.00	) / 1023 ) * 100 ) , 3 , 0 , buf) );
	//printString("%;");
	return (valorADC);
}

uint16_t getVolt (void)
{
	uint16_t valorADC;		// coleta do ADC de 0 a 1023
	valorADC = coletarADC ( (1<<MUX1) );	// coleta o pino C2
	//printString( dtostrf(((valorADC * 4.56) / 1023),4,2,buf) );
	//printString("v;");
	return (valorADC);
}

uint16_t getTempAVR (void)
{
	uint16_t valorADC;		// coleta do ADC de 0 a 1023
	valorADC = coletarADC( _BV(MUX3) );	// coleta o ADC8 (virtual)
	//tempChip = (valorADC - 125) * 1.075 / 10;		// exemplo de Peter Knight
	return(valorADC);
}