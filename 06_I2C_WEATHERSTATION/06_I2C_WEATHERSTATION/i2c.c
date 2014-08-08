/*
 * i2c.c
 *
 * Created: 07/08/2014 10:30:29
 *  Author: roney
 */ 

#include "i2c.h"
#define F_CPU 1000000L

void iniciaI2C(void)
{
	/*
	the SCL frequency equation is not correct as shown it should 
	be SCL Frequency = CPU CLOCK FREQUENCY/(16 + 2*TWBR*4^TWPS) 
	you have 
	
	SCL Frequency = CPU CLOCK FREQUENCY/(16 + 2*TWBR*prescaler)
	SCL = 1.000.000 / (16 + 2 * TWBR * prescaler)
	
	
	8MHz AVR TWBR = 0.5(8MHz/100kHz - 16) = 32
	4MHz AVR TWBR = 0.5(4MHz/100kHz - 16) = 12
	1mhz avr twbr = 0,5(1mhz/100hz - 16) = 0,5(10000 - 16)= 0,5(
	1MHz AVR TWBR = 0.5(1MHz/50kHz - 16) = 2 (not recommended)
	1MHz AVR TWBR = 0.5(1MHz/25kHz - 16) = 12

	In theory a 16MHz AVR should do 400kHz:
	16MHz AVR TWBR = 0.5(16MHz/400kHz - 16) = 12
	
	*/
	
	// Init
	TWBR = 12;				// Bit Rate = ~25 khz com clock de 1mhz
	TWCR |= (1<<TWEN);	// habilita o I2C
	
}

void finalizaI2C(void)
{
	TWCR &= ~(1<<TWEN);
}

void i2cStart(void)
{
	// Start
	TWCR = ( _BV(TWINT) | _BV(TWEN) | _BV(TWSTA) );	
	// TWI InterruptFlag + TWI Enable + TWI Start
	loop_until_bit_is_set (TWCR,TWINT);		//  TWI Interrupt Flag 
}

void i2cRestart(void)
{
	// Re-Start
	TWCR = ( _BV(TWINT) | _BV(TWEN) | _BV(TWSTO) | _BV(TWSTA) );	
	// TWI InterruptFlag + TWI Enable + TWI stop/Start
	loop_until_bit_is_set (TWCR,TWINT);		//  TWI Interrupt Flag
}
void i2cAck(void)
{
	TWCR = ( _BV(TWINT) | _BV(TWEN) | _BV(TWEA) ); // TW EA = Ack
	loop_until_bit_is_set (TWCR,TWINT);		//  TWI Interrupt Flag	
}

void i2cNotAck(void)
{
	TWCR = ( _BV(TWINT) | _BV(TWEN)  );
	TWCR &= ~_BV(TWEA); // Not TWEA = Not Ack
	loop_until_bit_is_set (TWCR,TWINT);		//  TWI Interrupt Flag
}

void i2cStop(void)
{
	// Stop
	TWCR = ( _BV(TWINT) | _BV(TWEN) | _BV(TWSTO) ); // TWI InterruptFlag + TWI Enable + TWI Stop
}

uint8_t i2cReadAck(void)
{
	// Ler ACK
	TWCR = ( _BV(TWINT) | _BV(TWEN) | _BV(TWEA) );
	loop_until_bit_is_set(TWCR,TWINT);
	return (TWDR);
}

uint8_t i2cReadNoAck(void)
{
	// Ler NoACK
	TWCR = ( _BV(TWINT) | _BV(TWEN) );
	loop_until_bit_is_set(TWCR,TWINT);
	return (TWDR);
}

void i2cSend(uint8_t data)
{
	// Envia um byte ao I2C
	TWDR = data;
	TWCR = ( _BV(TWINT) | _BV(TWEN) );
	loop_until_bit_is_set(TWCR,TWINT);
}