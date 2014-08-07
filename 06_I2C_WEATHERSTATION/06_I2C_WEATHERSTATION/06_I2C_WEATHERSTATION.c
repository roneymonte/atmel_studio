/*
 * _06_I2C_WEATHERSTATION.c
 *
 * Created: 07/08/2014 09:10:20
 *  Author: roney
 */ 


#include "06_I2C_WEATHERSTATION.h"
#include <avr/power.h>
#include "i2c.h"

#define F_CPU 1000000L

//#define BMP085_ADDRESS 0xEE      // I2C address of BMP085 (ou BMP180) que eh
// o mesmo do modulo 085 ou 180; ja eh um ende-
// reco de 8 bits, nao precisa << 1
#define P_CORRECTION   1.5       // in mBars - factor to adjust for elevation to match local weather station pressure
// this value for 14' above sea level (in Boca Raton, Florida)
#define OSS 1       // BMP180 Over Sampling Settings

// variaveis da BOSCH para calibragem do medidor de pressao e temperatura
short ac1;  // pressao
short ac2;  // pressao
short ac3;  // pressao
short b1;
short b2;
short mb;
short mc;           // temperatura
short md;           // temperatura
unsigned short ac4; // pressao
unsigned short ac5; // temperatura
unsigned short ac6; // temperatura

int main(void)
{
	clock_prescale_set(clock_div_8);
	
	//DDRC |= (1<<PORTC5) | (1<<PORTC4); // configura portas do I2C
	
	
	uint8_t	tempHighByte, tempLowByte;
	uint8_t	pressaoHigh, pressaoLow;

	
	iniciaPORTAS();		
	printString("\r\nIniciando UART.");
	
	_delay_ms(2000);
	
			
	printString("\r\nIniciando I2C.");
	iniciaI2C();
	
	_delay_ms(1000);
	
	printString("\r\nIniciando Calibragem Bosch.");
	
	BMP180calibragem();
	
	
    while(1)
    {
        //TODO:: Please write your application code 
		main_wx();
		
		printString("\r\nBMP180:");
		
		i2cStart();
		i2cSend(BMP180_W);					// Escreve o proximo byte que eh:
		i2cSend(BMP180_control_reg);		// Controle de Registro
		i2cSend(BMP180_ler_temp_cmd);		// Dizendo que quer ler a temperatura
		_delay_ms(5);
		i2cSend(BMP180_temperature_data);	// Envia o Dado que quer ser lido
		i2cStart();
		i2cSend(BMP180_R);					// Envia o comando para efetuar a leitura
		tempHighByte = i2cReadAck();
		tempLowByte = i2cReadNoAck();
		i2cStop();
		
		printString("_Temp_");
		printByte(tempHighByte);
		if (tempLowByte & (1<<7) )
		{
			printString(".5 ___ ");
		}
		else
		printString(".0 ___ ");
		
		/*==================================*/
		
		
			i2cStart();
			i2cSend(BMP180_W);					// Escreve o proximo byte, que eh:
			i2cSend(BMP180_calibration_AC2);	// Envia o Dado que quer ser lido
			i2cStart();
			i2cSend(BMP180_R);					// Envia o comando para efetuar a leitura
			pressaoHigh =  i2cReadAck();
			pressaoLow = i2cReadNoAck();
			i2cStop();
			
			printString("Pressao_");
			printByte(pressaoHigh);
			printString(" + ");
			printByte(pressaoLow);
			printString(".");
		
    }
}


long lerBMP180 (unsigned char endereco)
{
	unsigned short msb, lsb, dado;
	char msg[40];

		i2cStart();
			i2cSend( BMP180_W );  // modo default de escrita
			//i2cSend(BMP180_control_reg);
		i2cSend(endereco);
	// nao eh realmente necessario Stop antes de um Restart ? Nao,
	// o proprio Restart ja gera um Stop implicitamente

	//i2cStop(); _delay_ms(5);
	i2cStart();
	
		i2cSend( BMP180_R);   // modo de leitura
	
	msb = i2cReadAck();    // out_msb (0xF6)
	//AckI2C();

	lsb = i2cReadNoAck();    // out_lsb (0xF7)
	//NotAckI2C();
	
	i2cStop();

	dado = msb;
	dado *= 256;
	dado |= lsb;
	sprintf(msg,"[0x%x,0x%x=0x%X]", msb, lsb, dado);
	printString(msg);

	return dado;
}



long lerBMP180temperatura (void)
{
	printString("[");
    i2cStart();
		printString(".");
        i2cSend( BMP180_W );
		printString(".");
        i2cSend(BMP180_control_reg); // campo de pedido
                        // ctrl_meas, 2bits_oss / sco / 4bits_controle
        //AckI2C();
		printString(".");
        i2cSend(BMP180_ler_temp_cmd); // pedindo as variaveis de temperatura
    
	//i2cStop();

        _delay_ms(5);  // espere pelo menos 4.5 ms
                        // (conversion time pressure max) para OSS 0 (1 sample)
                        // obs: PDF da Bosch somente indica delay para pressao

		printString(".");
		i2cSend(BMP180_R);
		printString("]");
		
        return ( (i2cReadAck()<<8 | i2cReadNoAck() ) ); // ira ler o campo de resultado 0xF6 (out_msb)
                                // sendo long, mesclara com 0xF7 (out_lsb)
}

long lerBMP180pressao (void)
{
	printString("[");
    i2cStart();
		printString(".");
        i2cSend(BMP180_W );
		printString(".");
        i2cSend(BMP180_control_reg); // campo de pedido
                        // ctrl_meas, 2bits_oss / sco / 4bits_controle
        //AckI2C();
		printString(".");
        i2cSend(0x34); // pedindo as variaveis de pressao descompensada
                        // 0x34 + (oss<<6)
    //i2cStop();

    _delay_ms(5);  // espere pelo menos 4.5 ms
                    // (conversion time pressure max) para OSS 0 (1 sample)
	printString(".");
	i2cSend(0xF6);
	printString("]");
    return ( (i2cReadAck()<<8|i2cReadNoAck()) ); // ira ler o campo de resultado 0xF6 (out_msb)
                                // sendo long, mesclara com 0xF7 (out_lsb)
}

void BMP180conversor (long *temp, long *pressao)
{
    char msg[40];
    /* pagina 15 do manual da Bosch
     * BST-BMP180-DS000-09 (revisao 2.5 - 5 April 2013 )
     */

    long ut;
	long up;
	long x1, x2, b5, b6, x3, b3, p;
	unsigned long b4, b7;

	// ler a temperatura antes da pressao
	printString("_temp_");
	ut = lerBMP180temperatura();
	printString("_pres_");
	up = lerBMP180pressao();
	printString("_CALCULOS_\r\n");

	x1 = ((long) ut - ac6) * ac5 >> 15;
	x2 = ((long) mc * 1 << 11) / (x1 + md);
	b5 = x1 + x2;

	*temp = (b5 + 8) >> 4;


	b6 = b5 - 4000;
	x1 = (b2 * (b6 * b6 >> 12)) >> 11;
	x2 = ac2 * b6 >> 11;
	x3 = x1 + x2;

        sprintf(msg,"    ( x1=%d x2=%d x3=%d )\r\n", x1, x2, x3);
        printString(msg);
        /////////////////////////////////////////////////////

	//b3 = (( ((long) ac1 * 4 + x3) + 2) << 2);
        b3 = ( ( ((long) ac1 * 4 + x3) << OSS) +2 ) >> 2; // (mesmo que / 4)

        sprintf(msg,"    ( b3=%d e ac1=%d )\r\n", b3, ac1);
        printString(msg);
        /////////////////////////////////////////////////////

	x1 = ac3 * b6 >> 13;
	x2 = (b1 * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;

        sprintf(msg,"    ( x1=%d x2=%d x3=%d )\r\n", x1, x2, x3);
        printString(msg);
        /////////////////////////////////////////////////////
		printString("\r\n");

	b4 = (ac4 * (unsigned long) (x3 + 32768)) >> 15;
	b7 = ((unsigned long) up - b3) * (50000 >> OSS);

        sprintf(msg,"    ( b4=%d b7=%d up=%d )\r\n", b4, b7, up);
        printString(msg);
        /////////////////////////////////////////////////////

	p = ( b7 < 0x80000000 ) ? ( (b7 * 2) / b4 ) : ( (b7 / b4) * 2) ;

        sprintf(msg,"    ( p=%d b7=%d b4=%d )\r\n", p, b7,b4 );
        printString(msg);
        /////////////////////////////////////////////////////

	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;

        sprintf(msg,"    ( x1=%d x2=%d x3=%d )\r\n", x1, x2, x3);
        printString(msg);
        /////////////////////////////////////////////////////


	*pressao = ( p + ((x1 + x2 + 3791) ))  >> 4;

	_delay_ms(10);
}

void BMP180calibragem (void)
{
	printString("\r\nCalibragem...\r\n");
	char msg[40];

	ac1 = lerBMP180(0xAA);
	ac2 = lerBMP180(0xAC);
	ac3 = lerBMP180(0xAE);
	ac4 = lerBMP180(0xB0);
	ac5 = lerBMP180(0xB2);
	ac6 = lerBMP180(0xB4);
	b1  = lerBMP180(0xB6);
	b2  = lerBMP180(0xB8);
	mb  = lerBMP180(0xBA);
	mc  = lerBMP180(0xBC);
	md  = lerBMP180(0xBE);

	sprintf(msg,"\r\n\tAC1 = %d\r\n", ac1); printString(msg);
	sprintf(msg,"\tAC2 = %d\r\n", ac2); printString(msg);
	sprintf(msg,"\tAC3 = %d\r\n", ac3); printString(msg);
	sprintf(msg,"\tAC4 = %d\r\n", ac4); printString(msg);
	sprintf(msg,"\tAC5 = %d\r\n", ac5); printString(msg);
	sprintf(msg,"\tAC6 = %d\r\n", ac6); printString(msg);
	sprintf(msg,"\tB1 = %d\r\n", b1); printString(msg);
	sprintf(msg,"\tB2 = %d\r\n", b2); printString(msg);
	sprintf(msg,"\tMB = %d\r\n", mb); printString(msg);
	sprintf(msg,"\tMC = %d\r\n", mc); printString(msg);
	sprintf(msg,"\tMD = %d\r\n", md); printString(msg);
	sprintf(msg,"------------------------\r\n"); printString(msg);
	printString("Fim Calibragem.\r\n");
}

void modoBosch (void)
{
	long temperatura=0, pressaoL=0;
	char msg[40];
	
	
	
	printString("_conversor_bosch_");
	
	BMP180conversor( &temperatura, &pressaoL );
	
	sprintf(msg,"_Temp %2.1dc, Pres %dPa\r\n", (temperatura/10), pressaoL);
	printString(msg);
	printString("Fim Conversor\r\n");
}