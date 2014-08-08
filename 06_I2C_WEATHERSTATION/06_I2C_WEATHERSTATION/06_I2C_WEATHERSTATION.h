/*
 * _06_I2C_WEATHERSTATION.h
 *
 * Created: 07/08/2014 10:23:00
 *  Author: roney
 */ 


//#ifndef 06_I2C_WEATHERSTATION_H_
//#define 06_I2C_WEATHERSTATION_H_
//#endif /* 06_I2C_WEATHERSTATION_H_ */

#include <avr/io.h>
#include "05_WeatherStation.h"

#define 	BMP180_R 		0xEF
#define		BMP180_W 		0xEE		// endereco do CHIP BMP180 I2C

#define		BMP180_control_reg			0xF4	// Controle
#define		BMP180_ler_temp_cmd			0x2E	// Ler Temperatura
#define		BMP180_ler_pressao_cmd		0x34
//#define		BMP180_ler_pressao_cmd		0x74

#define		BMP180_temperature_data		0xF6	// Dado a ser retornado
#define		BMP180_pressure_data		0xF6

#define		BMP180_calibration_AC2		0xAC	// Calibracao AC2

#define BMP085_ULTRALOWPOWER 0
#define BMP085_STANDARD      1
#define BMP085_HIGHRES       2
#define BMP085_ULTRAHIGHRES  3

#define BMP085_CAL_AC1           0xAA  // R   Calibration data (16 bits)
#define BMP085_CAL_AC2           0xAC  // R   Calibration data (16 bits)
#define BMP085_CAL_AC3           0xAE  // R   Calibration data (16 bits)
#define BMP085_CAL_AC4           0xB0  // R   Calibration data (16 bits)
#define BMP085_CAL_AC5           0xB2  // R   Calibration data (16 bits)
#define BMP085_CAL_AC6           0xB4  // R   Calibration data (16 bits)
#define BMP085_CAL_B1            0xB6  // R   Calibration data (16 bits)
#define BMP085_CAL_B2            0xB8  // R   Calibration data (16 bits)
#define BMP085_CAL_MB            0xBA  // R   Calibration data (16 bits)
#define BMP085_CAL_MC            0xBC  // R   Calibration data (16 bits)
#define BMP085_CAL_MD            0xBE  // R   Calibration data (16 bits)

#define BMP085_CONTROL           0xF4
#define BMP085_TEMPDATA          0xF6
#define BMP085_PRESSUREDATA      0xF6
#define BMP085_READTEMPCMD       0x2E
#define BMP085_READPRESSURECMD   0x34

int main(void);
long lerBMP180temperatura (void);
long lerBMP180pressao (void);
void BMP180conversor (long *temp, long *pressao);
void BMP180calibragem (void);
long lerBMP180 (unsigned char endereco);
void leitura_rapida_bosch (void);
void modoBosch (void);
long BMP180_lerRapido (unsigned char endereco);