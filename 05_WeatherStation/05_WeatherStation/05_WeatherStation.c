/*
 * _05_WeatherStation.c
 *
 * Created: 31/07/2014 18:54:04
 *  Author: roney
 * 
 * Escopo: Sistema para Weather Station remota, com transferencias de dados
 * via radio IEEE 802.15.4;
 *
 * Os sensores sao conectados diretamente nas portas analogicas, ou via I2C.
 *
 * O sistema eh dotado de Bootloader, o que possibilita a atualizacao do
 * firmware remotamente. 
 *
 * O circuito eh alimentado por uma bateria LiPo de 3.7v e 2.5A, carregado
 * atraves de uma placa solar de 6v e 1watt, com controlador de carregamento.
 *
 * A conectividade se da atraves de Radio IEEE 802.15.4 (XBee) ligado na porta
 * UART nos pinos RX e TX. Ainda existe o pino RST conectado ao DIO3 do radio,
 * para possibilitar reset remoto, e ainda ativacao do bootloader/update no firmware.
 *
 * _ versao 1.0c,d,e - 04ago2014:
 *		- experimental com valores para VOLT ref 1.1v, luz e temperatura chip
 *
 * _ versao 1.0f - 05ago2014:
 *		- acerto do ADC com valores calibrados para bateria e luz
 */ 


#define  F_CPU 1000000L

#define LED01	PORTB0	//	led vermelho, resistor de 330R
#define LED02	PORTB5	//	led amarelo, resistor de 330R (o mesmo usado no bootloader)
#define LED03	PORTD6	//	led branco grande de PWM, resistor de 330R
#define LDR		0b011	//	conectado LCD com VCC, e resistor de 10K com GND (ADC3/PC3)
#define VOLT	0b010 	//	ligado diretamente no barramento VCC (ADC2/PC2)



#include <avr/io.h>
#include <avr/sfr_defs.h>	// contem a definicao do _BV
							/* sfr_defs.h - macros for accessing AVR special function registers */
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <float.h>
#include "USART.h"
#include "pinDefines.h"

void dormirADC (void);
void hello (void);
uint16_t coletarADC (char multiplexador);

EMPTY_INTERRUPT(ADC_vect);

int main(void)
{
	uint16_t valorADC;
	//unsigned char valor;
	float vcc;
	float luz;
	//uint16_t vcc;
	//uint16_t luz;
	uint16_t tempChip;
	uint8_t contador;
	char serialCharacter;
	char buf[16];
	
	DDRB |= (1<<PB0) | (1<<PB5);		// leds vermelho e amarelo (conjunto)
	DDRD |= (1<<PD6);					// led grande branco do PWM
	DDRD |= (1<<PD1);					// saida da UART TX
	DDRD |= ~(1<<PD0);					// entrada da UART RX
	
	PORTC &= ~(1<<PC2);
	PORTC &= ~(1<<PC3);
	DDRC &= ~(1<<PC2);		// entradas analogicas do ADC (Volts)
	DDRC &= ~(1<<PC3);		// entradas analogicas do ADC (LDR)
	//PORTC &= ~(1<<PC2);
	//PORTC &= ~(1<<PC3);
	
	PORTB &= ~(1<<LED01) | ~(1<<LED02);		// inicia com leds apagados
	PORTD &= ~(1<<LED03);					// inicia led PWM apagado
	
	/* =================================*/
	ADMUX	= 0b00000000;
	ADMUX	|= (1<<REFS0) | (1<<REFS1);	// configura voltagem referencia por voltagem interna de 1.1v
	ADCSRA	|= (1<<ADPS1) | (1<<ADPS0);	// prescaler do ADC como /8 do clock (1/8 de 1mhz = 125 khz)
	_delay_ms(500);
	ADCSRA |= (1<<ADEN);					// habilita o ADC (AD ENABLE)
	_delay_ms(500);
	/* =================================*/
	
	initUSART();						// por default essa biblioteca usa 9600 bps
	
	hello();
	
    while(1)
    {
		serialCharacter = receiveByte();
		
		
		if (serialCharacter == '*')
		{	
			printString("ADC ");
			
			valorADC = coletarADC( (1<<MUX1)|(1<<MUX0) );
			luz = ( (valorADC * 1.1) / 1023 ) * 100; // 1.1v * 100 = 1100 para dar 100%
			
			PORTB |= (1<<LED01);		// iniciando com led aceso
			PORTB &= ~(1<<LED02);		// iniciando com led apagado
			
			_delay_ms(200);
			PORTB ^= _BV(LED01);
			PORTB ^= _BV(LED02);
			
			printString("luz: [");
				itoa(valorADC, buf, 10); printString(buf);
				printString("] ");
				
			//itoa(luz, buf, 10);
			//sprintf(buf, "%3.1f", luz);
			dtostrf(luz,3,0,buf);
			
			printString(buf);
			
			_delay_ms(200);
			PORTB ^= _BV(LED01);
			PORTB ^= _BV(LED02);
			
			
			
			
			
			/* ==============VOLTAGEM===========*/
			/* =================================*/
			
			valorADC = coletarADC ( (1<<MUX1) );
			vcc = (valorADC * 4.4) / 1023;

			_delay_ms(200);
			PORTB ^= _BV(LED01);
			PORTB ^= _BV(LED02);
						
			printString("%, volt: [");
				itoa(valorADC, buf, 10); printString(buf);
				printString("] ");
				
			dtostrf(vcc,4,2,buf);
			printString(buf);
			printString("v; ");
			
			
			
			
			/* 
		
				int raw_temp;
				while( ( ( raw_temp = raw() ) < 0 ) );
				for( int i = 0; i < TEMPERATURE_SAMPLES; i++ ) {
					readings[i] = raw_temp;
				}
			}		
			*/

			
			
			/* ======Temperatura=do=CHIP========*/
			/* =================================*/
			valorADC = coletarADC(_BV(MUX3));
			
			printString(" temp_chip: [");
			itoa(valorADC, buf, 10); printString(buf);
			printString("] ");
			
				tempChip = (valorADC * 1024) /1024;
				
			sprintf(buf, "%d", tempChip);
			printString(buf);
			printString(". \r\n");
			
			PORTB &= ~(1<<LED01) & ~(1<<LED02);	// no final da rotina, apaga os leds
			/* =================================*/
			
		}
		else
		if (serialCharacter == '!')
		{
			PORTB |= (1<<LED01);		// iniciando com led aceso
			PORTB &= ~(1<<LED02);		// iniciando com led apagado
			
			printString("\r\nContador: ");
			for(contador=0;contador<60;contador++)
			{
				
				_delay_ms(500);
				printString(itoa(contador,buf,10));
				printString(",");
				_delay_ms(500);		// pisca-leds por 60 segundos	
				
				PORTB ^= _BV(LED01);
				PORTB ^= _BV(LED02);
				
			}	
			printString(", final.\r\n");
			PORTB &= ~(1<<LED01) & ~(1<<LED02);	// no final da rotina, apaga os leds
		}
		else
		if (serialCharacter == '0')
		{
			printString("LEDs [Verm, Amar], Brco: ");
			printHexByte((PINB | LED01) & 0b1); printString(","); // B0
			printHexByte((PINB | LED02) >>5 ); printString(","); // B5
			printHexByte((PIND | LED03) >>6 & 0b1 ); printString("\r\n"); // D6
		}
		else
		if (serialCharacter == '1')
		{
			PORTD ^= _BV(LED03);
		}
		else
		if (serialCharacter == '2')
		{
			PORTB ^= _BV(LED01);
		}
		else
		if (serialCharacter == '3')
		{
			PORTB ^= _BV(LED02);
		}
		else
		if (serialCharacter == '?')
		hello();
		else
		if (serialCharacter == 13)
		printWord(010);
		else
		{
			transmitByte(serialCharacter);
			PORTB &= ~(1<<LED01) & ~(1<<LED02);	// no final da rotina, apaga os leds
		}
    }
	
	return(0);
}

void dormirADC (void)
{
	set_sleep_mode(SLEEP_MODE_ADC);
	ADCSRA |= (1<<ADIE);
	sei();
}

void hello (void)
{
	
	printString("\r\nWeather Station v1.0f by RM @ RJ - 05ago2014.\r\n");
	printString("pressione:\r\n* coleta de dados\r\n! piscaled 1 min\r\n0 status leds\r\n1 led branco\r\n");
	printString("2 led vermelho\r\n3 led amarelo\r\n");
	
}

uint16_t  coletarADC (char multiplexador)
{
	/* ===========LUMINOSIDADE==========*/
	/* =================================*/

	ADCSRA &= ~(1<<ADEN);
	ADMUX	= 0b00000000;
	ADMUX	|= _BV(REFS0)|_BV(REFS1) ;	// configura voltagem referencia por 1.1v
				
	ADMUX	|=  multiplexador ;		// coleta analogica no pino C3 (LDR)
	
	ADCSRA |= (1<<ADEN);
	_delay_ms(1000);
				
	ADCSRA	|= (1<<ADSC);		// AD Start Conversion
	//dormirADC();
	loop_until_bit_is_clear(ADCSRA, ADSC);
	/* =================================*/
	return ADC;
	
};
