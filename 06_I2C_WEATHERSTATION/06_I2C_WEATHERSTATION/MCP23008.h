/*
 * MCP23008.h
 *
 * Created: 07/08/2014 12:55:07
 *  Author: roney
 */ 


//#ifndef MCP23008_H_
//#define MCP23008_H_
//#endif /* MCP23008_H_ */

//#include "global.h"
#include <inttypes.h>

#define MCP23008_BASE_ADDRESS	0x40
#define MCP23008_IODIR		0x00
#define MCP23008_IOPOL		0x01
#define MCP23008_GPINTEN	0x02
#define MCP23008_DEFVAL		0x03
#define MCP23008_INTCON		0x04
#define MCP23008_IOCON		0x05
#define MCP23008_GPPU		0x06
#define MCP23008_INTF		0x07
#define MCP23008_INTCAP		0x08
#define MCP23008_GPIO		0x09
#define MCP23008_OLAT		0x0A

#define u08 unsigned char

typedef struct {
	u08 address;
	u08 data;
} MCP23008;

void mcp23008_init(MCP23008 *obj, u08 address);
void mcp23008_write(MCP23008 *object);
void mcp23008_write_register( MCP23008 *obj, u08 reg, u08 data);
u08 mcp23008_read_register( MCP23008 *obj, u08 reg);