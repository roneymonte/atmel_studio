/*
 * mcp23008_leds.c
 *
 * Created: 12/08/2014 12:24:58
 *  Author: roney
 */ 

#define F_CPU 1000000L
#define DELAY 60

#include "mcp23008_leds.h"

void liga_mcp23008 (void)
{	
	i2c_start_wait(MCP23008_ID);
	
	i2c_write(IODIR);
	i2c_write(0x00);
	i2c_stop();
	
	i2c_rep_start(MCP23008_ID);
	i2c_write(GPIO);
	i2c_write(0b1);	// liga o primeiro LED GP0
	i2c_stop();
	_delay_ms(DELAY);
	/*
	i2c_rep_start(MCP23008_ID);
	i2c_write(GPIO);
	i2c_write(0x00);
	i2c_stop();
	*/
}

void seqLed_mcp23008 (void)
{
	uint8_t contador=0b1;
	
	//i2c_start(MCP23008_ID << 1);	// de 0x40 fica <<1 = 0x80
	
	while (contador != 0b10000000)
	{	
		contador=contador<<1;
		i2c_start(MCP23008_ID);
		i2c_write(GPIO);
		i2c_write(contador);
		i2c_stop();
		_delay_ms(DELAY);
	}
	
	while (contador!=0b0)
	{
		contador=contador>>1;
		i2c_start(MCP23008_ID);
		i2c_write(GPIO);
		i2c_write(contador);
		i2c_stop();
		_delay_ms(DELAY);
	}
	
}