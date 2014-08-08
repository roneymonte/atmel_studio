/*
 * i2c.h
 *
 * Created: 07/08/2014 10:31:07
 *  Author: roney
 */ 


//#ifndef I2C_H_
//#define I2C_H_
//#endif /* I2C_H_ */

#include <util/twi.h>
//#include "../../../../../../../Program Files (x86)/Atmel/Atmel Toolchain/AVR8 GCC/Native/3.4.1056/avr8-gnu-toolchain/avr/include/avr/iom168p.h"


void iniciaI2C(void);
void i2cStart(void);
void i2cStop(void);
uint8_t i2cReadAck(void);
uint8_t i2cReadNoAck(void);
void i2cSend(uint8_t data);
void finalizaI2C(void);
void i2cRestart(void);
void i2cAck(void);
void i2cNotAck(void);