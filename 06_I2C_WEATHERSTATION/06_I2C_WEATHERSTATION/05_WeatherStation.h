/*
 * _05_WeatherStation.h
 *
 * Created: 07/08/2014 09:16:23
 *  Author: roney
 */ 


//#ifndef 05_WEATHERSTATION_H_
//#define 05_WEATHERSTATION_H_
//#endif /* 05_WEATHERSTATION_H_ */

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


void main_wx(void);
void iniciaPORTAS (void);
void getLuz (void);
void getVolt (void);
void getTempAVR (void);
//void dormirADC (void);		// nao utilizado
void hello (void);			// mensagem inicial
uint16_t coletarADC (char multiplexador);	// leitura do ADC do MUX
void flipLed (void);		// inverte LEDs 1 e 2
//void valorRecebido (uint16_t valor);

//EMPTY_INTERRUPT(ADC_vect);