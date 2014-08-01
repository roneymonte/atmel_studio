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
 */ 


#define  F_CPU 1000000L
#define LED01	PORTB0	//	led vermelho, resistor de 330R
#define LED02	PORTB5	//	led amarelo, resistor de 330R (o mesmo usado no bootloader)
#define LED03	PORTD6	//	led branco grande de PWM, resistor de 330R
#define LDR		PORTC3	//	conectado LCD com VCC, e resistor de 10K com GND
#define VOLT	PORTC2	//	ligado diretamente no barramento VCC



#include <avr/io.h>
#include <avr/sfr_defs.h>	// contem a definicao do _BV
							/* sfr_defs.h - macros for accessing AVR special function registers */
#include <util/delay.h>
#include <stdlib.h>
#include "USART.h"


int main(void)
{
	uint16_t luz;
	uint16_t voltagem;
	uint8_t contador;
	char serialCharacter;
	char buf[4];
	
	DDRB |= (1<<PB0) | (1<<PB5);		// leds vermelho e amarelo (conjunto)
	DDRD |= (1<<PD6);					// led grande branco do PWM
	DDRD |= (1<<PD1);					// saida da UART TX
	DDRD |= ~(1<<PD0);					// entrada da UART RX
	DDRC &= ~(1<<PC3) & ~(1<<PC2);		// entradas analogicas do ADC
	
	PORTB |= ~(1<<PB0) & ~(1<<PB5);		// inicia com leds apagados
	PORTD |= ~(1<<PD6);					// inicia led PWM apagado
	
	ADMUX |= (1<<REFS0);				// configura voltagem referencia por AVCC
	ADCSRA |= (1<<ADPS1) | (1<<ADPS0);	// prescaler do ADC como /8 do clock	
	ADCSRA = (1<<ADEN);					// habilita o ADC (AD ENABLE)
	
	initUSART();						// por default essa biblioteca usa 9600 bps
	printString("Weather Station v1.0b by RM.\r\n");
	printString("pressione:\r\n* coleta de dados\r\n! piscaled 1 min\r\n0 status leds\r\n1 led branco\r\n");
	printString("2 led vermelho\r\n3 led amarelo\r\n");
	

	
    while(1)
    {
		serialCharacter = receiveByte();
		transmitByte(serialCharacter);
		
		if (serialCharacter == '*')
		{	
			printString("\r\nColetando...");

			
			ADMUX = ADMUX & 0b11110000;
			ADMUX = ADMUX | LDR;		// coleta analogica no pino C3 (LDR)
			ADCSRA |= (1<<ADSC);
			loop_until_bit_is_clear(ADCSRA, ADSC);
			luz = ADC;
			
			PORTB |= (1<<LED01);		// iniciando com led aceso
			PORTB &= ~(1<<LED02);		// iniciando com led apagado
			
			_delay_ms(200);
			PORTB ^= _BV(LED01);
			PORTB ^= _BV(LED02);
			
			printString(" luz: ");
			itoa(luz, buf, 10);
			printString(buf);
			
			_delay_ms(200);
			PORTB ^= _BV(LED01);
			PORTB ^= _BV(LED02);
			
			ADMUX = ADMUX | VOLT;		// coleta analogica no pino C2 (voltagem)
			ADCSRA |= (1<<ADSC);
			loop_until_bit_is_clear(ADCSRA, ADSC);
			voltagem = ADC;
						
			_delay_ms(200);
			PORTB ^= _BV(LED01);
			PORTB ^= _BV(LED02);
						
			printString(", voltagem: ");
			itoa(voltagem, buf, 10);
			printString(buf);
			printString("\r\n");
			PORTB &= ~(1<<LED01) & ~(1<<LED02);	// no final da rotina, apaga os leds
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
			printString("\r\nLEDs Verm, Amar, Brco: ");
			printHexByte(PORTB0); printString(",");
			printHexByte(PORTB5); printString(",");
			printHexByte(PORTD6); printString("\r\n");
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
        PORTB &= ~(1<<LED01) & ~(1<<LED02);	// no final da rotina, apaga os leds
    }
	
	return(0);
}