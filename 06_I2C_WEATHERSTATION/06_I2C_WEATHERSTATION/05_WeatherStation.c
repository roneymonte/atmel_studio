/*
 * _05_WeatherStation.c
 *
 * Created: 31/07/2014 18:54:04
 *  Author: roney
 * 
 * Escopo: Sistema para Weather Station remota, com transferencias de dados
 * via radio IEEE 802.15.4;
 *
 * Os sensores sao conectados diretamente nas portas analogicas, ou via I2C.
 *
 * O sistema eh dotado de Bootloader, o que possibilita a atualizacao do
 * firmware remotamente. 
 *
 * O circuito eh alimentado por uma bateria LiPo de 3.7v e 2.5A, carregado
 * atraves de uma placa solar de 6v e 1watt, com controlador de carregamento.
 *
 * A conectividade se da atraves de Radio IEEE 802.15.4 (XBee) ligado na porta
 * UART nos pinos RX e TX. Ainda existe o pino RST conectado ao DIO3 do radio,
 * para possibilitar reset remoto, e ainda ativacao do bootloader/update no firmware.
 *
 * _ versao 1.0c,d,e - 04ago2014:
 *		- experimental com valores para VOLT ref 1.1v, luz e temperatura chip
 *
 * _ versao 1.0g - 05ago2014:
 *		- acerto do ADC com valores calibrados para bateria e luz
 * _ versao 1.0i - 07ago2014:
 *		- finalizacao e encerramento do projeto 05, versao final.
 *
 * _ versao 1.1 - 08ago2014:
 *		- implementando I2C, problemas com o tamanho do codigo acima de 9K
 *		  no carregamento via bootloader e Xbee
 *
 */ 


#include "05_WeatherStation.h"
#include "06_I2C_WEATHERSTATION.h"

void main_wx(void)
{
	

	//float vcc;				// valor calculado para voltagem da bateria
	//float luz;				// valor calculado para luminosidade

	
	//uint8_t contador;		// contador para pisca-led
	char serialCharacter;	// caractere recebido na console serial
	//char buf[7];			// buffer de string com ate 6 caracteres
		printString("\r\n>");
		serialCharacter = receiveByte();
		
		if (serialCharacter == '*')
		{	
			//printString("\r\nLuzVoltTempAVR:\r\n");
			/* ==============Luz=LDR============*/
			getLuz();
			/* =================================*/
			
			/* ==============VOLTAGEM===========*/
			getVolt();
			/* =================================*/
			
			/* ======Temperatura=do=CHIP========*/
			getTempAVR();
			/* =================================*/
			
		}
		else
		/*
		if (serialCharacter == '!')
		{
			PORTB |= (1<<LED01);		// iniciando com led aceso
			PORTB &= ~(1<<LED02);		// iniciando com led apagado
			
			printString("\r\nC:");
			for(contador=0;contador<60;contador++)
			{
				
				_delay_ms(500);
				printString(itoa(contador,buf,10));
				_delay_ms(500);		// pisca-leds por 60 segundos	
				
				flipLed();
				
			}	
			PORTB &= ~(1<<LED01) & ~(1<<LED02);	// no final da rotina, apaga os leds
		}
		else
		*/
		/*
		if (serialCharacter == '0')
		{
			printString("\rVM,AM,BC:");
			printHexByte((PINB | LED01) & 0b1); //printString(","); // B0
			printHexByte((PINB | LED02) >>5 ); //printString(","); // B5
			printHexByte((PIND | LED03) >>6 & 0b1 ); //printString("\r\n"); // D6
		}
		else
		
		if (serialCharacter == '1')
		{
			PORTD ^= _BV(LED03);
		}
		else
		if (serialCharacter == '2')
		{
			PORTB ^= _BV(LED01);
		}
		else
		if (serialCharacter == '3')
		{
			PORTB ^= _BV(LED02);
		}
		else
		*/
		if (serialCharacter == '?')
			hello();
		else
		//if (serialCharacter == 13)
		//	transmitByte(012);	// Line-Feed (control-J) depois do Carriage-Return (^M)
		//else
		if (serialCharacter == '_')
			modoBosch();
		else
		if (serialCharacter == '-')
			leitura_rapida_bosch();
		else
		{
			transmitByte(serialCharacter);
			PORTB &= ~(1<<LED01) & ~(1<<LED02);	// no final da rotina, apaga os leds
		}
    
	//return(0);
}

void iniciaPORTAS (void)
{
		DDRB |= (1<<PB0) | (1<<PB5);		// leds vermelho e amarelo (conjunto)
		DDRD |= (1<<PD6) | (1<<PD1);		// led grande branco PD6 do PWM, e saida da UART TX PD1
		//DDRD |= ~(1<<PD0);					// entrada da UART RX
				
		//PORTB &= ~(1<<LED01) | ~(1<<LED02);		// inicia com leds apagados
		//PORTD &= ~(1<<LED03);					// inicia led PWM apagado
		
		initUSART();						// por default essa biblioteca usa 9600 bps
		//hello();							// imprime mensagem inicial na console
		printString("\r\n? help\r\n");
};

void getLuz (void)
{
	uint16_t valorADC;		// coleta do ADC de 0 a 1023
	char buf[7];
				
	printString("L:");
				
	valorADC = coletarADC( (1<<MUX1)|(1<<MUX0) );	// coleta o pino C3
	//flipLed();
	//valorRecebido(valorADC);
	//luz = ( (valorADC * 1.00	) / 1023 ) * 100;
	//dtostrf(luz,3,0,buf);
	
	printString( dtostrf( ( ( (valorADC * 1.00	) / 1023 ) * 100 ) , 3 , 0 , buf) );
	
	printString("%;");
	flipLed();
}

void getVolt (void)
{
	uint16_t valorADC;		// coleta do ADC de 0 a 1023
	char buf[7];
	
	printString("T:");
	
	valorADC = coletarADC ( (1<<MUX1) );	// coleta o pino C2
	//flipLed();
	
	//valorRecebido(valorADC);

	printString( dtostrf(((valorADC * 4.56) / 1023),4,2,buf) );
	printString("v;");
	flipLed();
}

void getTempAVR (void)
{
	uint16_t valorADC;		// coleta do ADC de 0 a 1023
	char buf[7];
	//uint16_t tempChip;		// temperatura interna do chip
	
	printString("AVR:");
	valorADC = coletarADC( _BV(MUX3) );	// coleta o ADC8 (virtual)
	//flipLed();
				
	//valorRecebido(valorADC);
				
	//tempChip = (valorADC * 1024) /1024;			// por Roney
	//tempChip = (valorADC - 125) * 1.075 / 10;		// exemplo de Peter Knight
				
	//printString( sprintf(NULL, "%d", tempChip ) );
	printString( dtostrf( ((valorADC - 125) * 1.075 / 10), 2,1,buf ) );
	printString("oC;");
	//flipLed();
				
	//PORTB &= ~(1<<LED01) & ~(1<<LED02);	// no final da rotina, apaga os leds	
}

/*void dormirADC (void)
{
	set_sleep_mode(SLEEP_MODE_ADC);
	ADCSRA |= (1<<ADIE);
	sei();
}*/

void hello (void)
{
	//printString("\r\nWX1.1 RM08ago14\r\n");
	//printString("* coleta\r\n! pisca 1m\r\n0 status leds\r\n1/2/3 leds\r\n");
	//printString("-/_ RoneyBMP/Bosch\r\n");
	printString("\r*!0123-_\r\n");
}

uint16_t  coletarADC (char multiplexador)
{
	//ADCSRA &= ~(1<<ADEN);
	//ADMUX	= 0b00000000;
	ADMUX	= _BV(REFS0)|_BV(REFS1) ;	// inicia ADMUX 
										// e configura voltagem referencia para 1.1v
	ADMUX	|=  multiplexador ;			// coleta analogica no pino multiplexador
	ADCSRA |= (1<<ADEN);				// habilita o AD
	_delay_ms(20);						// pausa para estabilizacao da voltagem no AD
	ADCSRA	|= (1<<ADSC);				// AD Start Conversion
	
	loop_until_bit_is_clear(ADCSRA, ADSC);	// espera num loop ate que o valor esteja disponivel
	return ADC;
};

void flipLed (void)
{
	PORTB ^= _BV(LED01);
	PORTB ^= _BV(LED02);
}

/*void valorRecebido (uint16_t valor)
{
	char buffer[5];
	
	printString("[");		// imprime entre cochetes o valor do ADC recebido
	itoa(valor, buffer, 10);
	printString(buffer);
	printString("]");
}*/
