/*
 * mcp23008_leds.h
 *
 * Created: 12/08/2014 12:25:22
 *  Author: roney
 */ 



#ifndef MCP23008_LEDS_H_
#define MCP23008_LEDS_H_





#endif /* MCP23008_LEDS_H_ */



#define MCP23008_ID    0x40  // MCP23008 Device Identifier
//#define MCP23008_ADDR  0x0E  // MCP23008 Device Address

#define IODIR 0x00           // MCP23008 I/O Direction Register
#define GPIO  0x09           // MCP23008 General Purpose I/O Register

#include <avr/io.h>
#include <util/delay.h>

void seqLed_mcp23008 (void);