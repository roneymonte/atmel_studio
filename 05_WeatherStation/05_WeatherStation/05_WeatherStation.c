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
 * _ versao 1.0g - 05ago2014:
 *		- acerto do ADC com valores calibrados para bateria e luz
 */ 


#define  F_CPU 1000000L

#define LED01	PORTB0	//	led vermelho, resistor de 330R
#define LED02	PORTB5	//	led amarelo, resistor de 330R (o mesmo usado no bootloader)
#define LED03	PORTD6	//	led branco grande de PWM, resistor de 330R
//#define LDR		0b011	//	conectado LCD com VCC, e resistor de 10K com GND (ADC3/PC3)
//#define VOLT	0b010 	//	ligado diretamente no barramento VCC (ADC2/PC2)



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


void dormirADC (void);		// nao utilizado
void hello (void);			// mensagem inicial
uint16_t coletarADC (char multiplexador);	// leitura do ADC do MUX
void flipLed (void);		// inverte LEDs 1 e 2
void valorRecebido (uint16_t valor);

EMPTY_INTERRUPT(ADC_vect);

int main(void)
{
	uint16_t valorADC;		// coleta do ADC de 0 a 1023

	//float vcc;				// valor calculado para voltagem da bateria
	float luz;				// valor calculado para luminosidade

	uint16_t tempChip;		// temperatura interna do chip
	uint8_t contador;		// contador para pisca-led
	char serialCharacter;	// caractere recebido na console serial
	char buf[7];			// buffer de string com ate 6 caracteres
	
	DDRB |= (1<<PB0) | (1<<PB5);		// leds vermelho e amarelo (conjunto)
	DDRD |= (1<<PD6);					// led grande branco do PWM
	DDRD |= (1<<PD1);					// saida da UART TX
	DDRD |= ~(1<<PD0);					// entrada da UART RX
	
	// As seguintes configuracoes abaixo sao DESNECESSARIAS, pois o AVR ja tem
	// as portas inicialmente com DDR = 0 (input)
	//PORTC &= ~(1<<PC2);	// zerando a porta antes de configurar a direcao DDR
	//PORTC &= ~(1<<PC3);	// zerando a porta antes de configurar a direcao DDR
	//DDRC &= ~(1<<PC2);		// entradas analogicas do ADC (Volts)
	//DDRC &= ~(1<<PC3);		// entradas analogicas do ADC (LDR)

	
	PORTB &= ~(1<<LED01) | ~(1<<LED02);		// inicia com leds apagados
	PORTD &= ~(1<<LED03);					// inicia led PWM apagado
	
	initUSART();						// por default essa biblioteca usa 9600 bps
	//hello();							// imprime mensagem inicial na console
	printString("\r\n? para help\r\n");
	
    while(1)
    {
		serialCharacter = receiveByte();
		
		if (serialCharacter == '*')
		{	
			printString("\rADC Luz:");
			
			valorADC = coletarADC( (1<<MUX1)|(1<<MUX0) );	// coleta o pino C3
			luz = ( (valorADC * 1.00	) / 1023 ) * 100;	// Resistor 4.7k down, e 10k antes do LDR com positivo
			// OBS: eh necessario usar o numero 1 como 1.00 para que o FLOAT seja estipulado.
			// caso contrario o numero calculado se comportara como inteiro.
						
			PORTB |= (1<<LED01);		// iniciando com led aceso
			PORTB &= ~(1<<LED02);		// iniciando com led apagado
			
			flipLed();
			
			valorRecebido(valorADC);
				
			dtostrf(luz,3,0,buf);
			printString(buf);
			
			flipLed();
			
			
			
			/* ==============VOLTAGEM===========*/
			/* =================================*/
			
			valorADC = coletarADC ( (1<<MUX1) );	// coleta o pino C2
			//vcc = (valorADC * 4.56) / 1023;	// 4.56 volts foi o valor maximo medido no multimetro
											// quando o ADC estava em 1023, com sol pleno na placa solar

			flipLed();
			printString("% , tensao:");
			valorRecebido(valorADC);
				
			dtostrf(((valorADC * 4.56) / 1023),4,2,buf);
			printString(buf);
			printString("v , temp_AVR:");
			
			
			/* ======Temperatura=do=CHIP========*/
			/* =================================*/
			valorADC = coletarADC( _BV(MUX3) );	// coleta o ADC8 (virtual)
			
			valorRecebido(valorADC);
			
				//tempChip = (valorADC * 1024) /1024;			// por Roney
				tempChip = (valorADC - 125) * 1.075 / 10;		// exemplo de Peter Knight
				
			sprintf(buf, "%d", tempChip );
			printString(buf);
			printString(" graus.\r\n");
			
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
				
				flipLed();
				
			}	
			printString(" fim.\r\n");
			PORTB &= ~(1<<LED01) & ~(1<<LED02);	// no final da rotina, apaga os leds
		}
		else
		if (serialCharacter == '0')
		{
			printString("\rLEDs [Verm, Amar], Brco: ");
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
			transmitByte(012);	// Line-Feed (control-J) depois do Carriage-Return (^M)
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
	
	printString("\r\nWeather Station v1.0h by RM @ RJ - 05ago2014.\r\n");
	printString("pressione:\r\n* coleta de dados\r\n! piscaled 1 min\r\n0 status leds\r\n1 led branco\r\n");
	printString("2 led vermelho\r\n3 led amarelo\r\n");
	
}

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
	return ADC;
};

void flipLed (void)
{
	PORTB ^= _BV(LED01);
	PORTB ^= _BV(LED02);
}

void valorRecebido (uint16_t valor)
{
	char buffer[5];
	
	printString(" [");		// imprime entre cochetes o valor do ADC recebido
	itoa(valor, buffer, 10);
	printString(buffer);
	printString("] ");
}
