/*
 * _03_UART.c
 *
 * Created: 20/07/2014 19:29:01
 *  Author: roney
 */ 

#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include "pinDefines.h"
#include "USART.h"





int main(void)
{
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