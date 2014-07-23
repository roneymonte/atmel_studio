/*
 * _03_UART.c
 *
 * Created: 20/07/2014 19:29:01
 *  Author: roney
 
 Exemplo do Capitulo 5 do livro "Make: Avr Programming"
 Consiste em fazer uma loopback na serial do MCU, devolvendo
 ao terminal serial os caracteres digitados, e esperando os
 bits na PORTB.
 
 */ 

#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include "pinDefines.h"
#include "USART.h"

int main(void)
{
	DDRD = (1<<1); // habilita output no pino PD1 (TXD)
	
	char serialCharacter;
	
	LED_DDR=0xff;
	initUSART();
	printString("Ola Mundo\r\n");
	
    while(1)
    {
		serialCharacter = receiveByte();
		transmitByte(serialCharacter);
		LED_PORT=serialCharacter;
	}
	
	return(0);
}