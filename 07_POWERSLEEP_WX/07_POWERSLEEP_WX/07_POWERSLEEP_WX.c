/*
 * _07_POWERSLEEP_WX.c
 *
 * Created: 09/08/2014 16:50:49
 *  Author: roney
 *
 * Projeto adicionado complementarmente ao projeto da Weather Station, para testar
 * preservacao de energia, ja que a mesma eh alimentada por bateria e placa solar.
 * 
 * Durante periodos prolongados sem luz solar, a bateria estava perdendo muita carga.
 * Varios dispositivos (ponte de resistores, LDR, I2C, etc estavam consumindo energia.
 * Uma das solucoes foi controla-los atraves de um MOSFET que desliga-os quando nao necessario.
 *
 * Este program tem o intuito de testar o SLEEP MODE pelas seguintes formas:
 *	0)	Dormir indefinidamente ate que a USART RX gere uma Interrupcao
 *	1)	Dormir por 1 hora, acordando a cada 5 minutos (checagem em 8 x 8 seg) com WDT de 8 seg
 *	2)	Dormir indefinidamente, acordando a cada 5 minutos (checagem em 8 x 8 seg) com WDT de 8 seg
 *
 * v1.0 - utilizada biblioteca I2C para DS1307 de http://davidegironi.blogspot.com.br/
 *
 * v1.1 - 12/08/2014 - adicionado MOSFET N-Channel para ligar/desligar os perifericos (I2C, ADC, etc)
 *		- Este mosfet tem o gate acionado na porta PD7.
 *		- Implementado o MCP23008 da placa de desenvolvimento PicKit2 Serial I2C DemoBoard
 *
 */ 

#define F_CPU 1000000L

#include <avr/io.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/wdt.h>

#include "davidegironi/ds1307.h"
#include "rotinas_projetos_anteriores.h"
#include "mcp23008_leds.h"

uint16_t agora (void);
void habilitarWDT (void);
void getEnv (void);

char MODO = '0';	// variavel global de definicao de modo de operacao (0, 1 ou 2)

int main(void)
{
	char buf[7]; // antes apenas 7 era o suficiente
	uint8_t contador;
	uint16_t horaMinuto;
	
	uint16_t	horaDesperto;
	uint8_t		ciclosWDTSono;

	iniciaPORTAS();
	
	PORTD |= (1<<PD7);					// liga MOSFET
	_delay_ms(20);						// espera 20ms para energizar os circuitos
	
	ds1307_init();
	
	// MCUSR – MCU Status Register
	// The MCU Status Register provides information on which reset source caused an MCU reset.
	
	printString("MCU Status Register:");
	printHexByte(  ((MCUSR | WDRF)  &0b1000 >> 3) );
	printString(",");
	printHexByte(  ((MCUSR | BORF)  &0b0100 >> 2) );
	printString(",");
	printHexByte(  ((MCUSR | EXTRF) &0b0010 >> 1) );
	printString(",");
	printHexByte(  ((MCUSR | PORF)  &0b0001     ) );
	printString("\r\nProj 07 v1.1 PowerSleepWDT_");
	agora();
	

	//WDTCSR – Watchdog Timer Control Register
	
	/*
	When the Brown-out Detector (BOD) is enabled by BODLEVEL fuses, Table 28-6 on page 293,
	the BOD is actively monitoring the power supply voltage during a sleep period. To save power, it
	is possible to disable the BOD by software for some of the sleep modes, see Table 10-1 on page
	40. The sleep mode power consumption will then be at the same level as when BOD is globally
	disabled by fuses. If BOD is disabled in software, the BOD function is turned off immediately
	after entering the sleep mode. Upon wake-up from sleep, BOD is automatically enabled again.
	This ensures safe operation in case the VCC level has dropped during the sleep period.
	
	When the BOD has been disabled,the wake-up time from sleep mode will be approximately 60
	µs to ensure that the BOD is working correctly before the MCU continues executing code.
	BOD disable is controlled by bit 6, BODS (BOD Sleep) in the control register MCUCR, see
	”MCUCR – MCU Control Register” on page 45. Writing this bit to one turns off the BOD in relevant 
	sleep modes, while a zero in this bit keeps BOD active. Default setting keeps BOD active,
	i.e. BODS set to zero.
	*/
		
	//sleep_bod_disable();
	/* faz o mesmo que abaixo */
	//MCUCR |= (1<<BODS) | (1<<BODSE);		// desabilita o BOD para o sleep mode
	//MCUCR |= (1<<BODS);						// desabilita o BOD
	//MCUCR &= ~(1<<BODSE);					//
	
	//SMCR |= (1<<SE);						// Sleep Enable
	//SMCR |= (1<<SM2) | (1<<SM1) | (1<<SM0);	// IDLE
	
	//power_tx_modulator_enable();
	
	// disable ADC
	//ADCSRA = 0;		//	With that there the power consumption drops a large amount, down from 335 µA to 0.355 µA! (that is, 355 nA)
	
	
			/*
		20.7.3 Receive Compete Flag and Interrupt
		The USART Receiver has one flag that indicates the Receiver state.
		The Receive Complete (RXCn) Flag indicates if there are unread data present in the receive 
		buffer. This flag is one when unread data exist in the receive buffer, and zero when the receive
		buffer is empty (i.e., does not contain any unread data). If the Receiver is disabled (RXENn = 0),
		the receive buffer will be flushed and consequently the RXCn bit will become zero.
		
		When the Receive Complete Interrupt Enable (RXCIEn) in UCSRnB is set, the USART Receive
		Complete interrupt will be executed as long as the RXCn Flag is set (provided thatglobal interrupts 
		are enabled). When interrupt-driven data reception is used, the receive complete routine
		must read the received data from UDRn in order to clear the RXCn Flag, otherwise a new interrupt 
		will occur once the interrupt routine terminates.
		*/
		UCSR0B |= (1<<RXCIE0);				// Habilita a Interrupcao de RX no controle da USART
		sei();
		_delay_ms(10000);
		
		
	printString("\r\nLoop Sleep Indefinido - RXINT.\r\n");
    while(1)
    {
		cli();
		for (contador=0;contador<3;contador++)
		{
			getEnv();
			_delay_ms(3333);
		}
		horaMinuto=agora();		
		
		
		if(MODO=='0')		// o MODO 0 caracteriza-se por dormir indefinidamente
							// ate que uma interrupcao na USART acorde o MCU
		{
			set_sleep_mode(SLEEP_MODE_IDLE);	// configura o MODO de sleep
			sei();								// habilita todos interrupts
			
			liga_mcp23008();
			seqLed_mcp23008();
		
			//printString("Habilitando Sleep.\r\n");
			sleep_enable();						// habilita a dormirda
		
			power_adc_disable();
			power_spi_disable();
			power_timer0_disable();
			power_timer1_disable();
			power_timer2_disable();
			power_twi_disable();
		
			sleep_bod_disable();				// desliga o comparador de voltagem do BOD
		
			PORTD &= ~(1<<PD7);					// desliga MOSFET
		
			printString("dormindo...");
			sleep_mode();						// realmente coloca para dormir
			/*--------------------------------------------------------------*/
			printString("...Acordou!\r\n");
			sleep_disable();
			
			PORTD |= (1<<PD7);					// liga MOSFET
			_delay_ms(20);						// espera 20ms para energizar os circuitos
			
			power_all_enable();
			agora();
		}
		else
		{
			printString("\r\nLoop Sono de 1h/5m (64s/64s).\r\n_");
			
			// Marca ZERO ciclos de Sleep com WDT
			ciclosWDTSono=0;	
			
			while (MODO=='1' | MODO=='2')	// o Modo 1 eh o Sleep com WDT de no maximo 1h
											// o Modo 2 eh o Sleep com WDT sem hora para realmente acordar
			{
				//////////////////////////////////////////////////////
				// OBS: nao foi feita restricao de integridade para //
				// horarios que rodem a meia-noite, pois isso iria  //
				// causar uma subtracao negativa no calculo de tempo//
				// decorrido.                                       //
				//////////////////////////////////////////////////////

				//UCSR0B &= ~(1<<RXCIE0);	// Deabilita a Interrupcao de RX no controle da USART
				sei();					// Habilita interrupcoes, por causa do WDT
				
				PORTD &= ~(1<<PD7);					// desliga MOSFET
				
				habilitarWDT();		// coloca a CPU para dormir em SLEEP_MODE_PWR_DOWN
									// sendo acordada 8 segundos depois pelo WDT
									
				ciclosWDTSono++;	// computa mais um ciclo de WDT de 8 segundos
				
				if (ciclosWDTSono >= 8)	// que os ciclos de sleep+WDT forem maiores que 8 (64 segundos)
				{
					ciclosWDTSono=0;	// zera o contador de ciclos a cada "minuto" (ou mais segundos) de sono
					
					PORTD |= (1<<PD7);					// liga MOSFET
					_delay_ms(20);						// espera 20ms para energizar os circuitos
					
					liga_mcp23008();
					seqLed_mcp23008();
					
					horaDesperto = agora();	// registra o horario (HORA*MIN) da acordada do WDT
					
					//printString("\r\nDormiu=");
					//printString( itoa(horaMinuto,buf,10) );
					//printString(", ");
					
					////////////////////////////////////////////////////				
					if( ( (horaDesperto-horaMinuto) / 5.0 ) == round((horaDesperto-horaMinuto)/5) )		
						// testa se ja fazem mais de 5 minutos que dormi
					{
						//
						getEnv();
						//printString("Rapida acordada=");
						//printString( itoa(horaDesperto,buf,10) );
						printString(" ZZzzzz dif=");
						
						printString( itoa(( horaDesperto - horaMinuto) ,buf,10) );
						printString("min.\r\n");
						_delay_ms(2000);
						//
					}
					////////////////////////////////////////////////////				
					if ( ((horaDesperto-horaMinuto)/60.0) == round((horaDesperto-horaMinuto)/60) )	
						// testa se ja faz mais de 1 hora que dormi
					{
						//
						if (MODO=='1')
						{
							MODO='0';	// forcar para sair do MODO 1 (WDT) e voltar para o MODO 0 (USART RX INT)
							printString("Saindo do Modo Sono de 1 hora.\r\n_");
						}
						else
						{
							horaMinuto=agora();	// fala que o inicio da dormida de 1h passa a ser agora, novamente
							printString("Dormindo por mais 1 horinha...\r\n_");
						}
						//
					}
					////////////////////////////////////////////////////
					_delay_ms(2000);	
				}
			}
		}
		
    }
}

ISR(USART_RX_vect)
{
	char BYTESERIAL;
	
	BYTESERIAL = receiveByte();
	transmitByte(BYTESERIAL);
	
	if(BYTESERIAL=='0') MODO='0';		// MODO 0 sleep ate que receba byte na USART
	else if(BYTESERIAL=='1') MODO='1';	// MODO 1 aciona sleep 64s/64s/contador de 5m/5m ate 1h
	else if(BYTESERIAL=='2') MODO='2';	// MODO 2 aciona sleep 64s/64s/contador de 5m/5m sem termino
	else if(BYTESERIAL=='L')	{		// 'L' gera sequencial de LEDs no MCP23008
									liga_mcp23008(); 
									seqLed_mcp23008(); 
								}	// L gera sequencia de LEDs no MCP23008
}

ISR(WDT_vect)
{
	wdt_disable();
}


void getEnv (void)
{
	char buf[7];
	
			printString(	dtostrf(	(	(getVolt()*4.56) / 1023	)	,	4	,	2	,		buf)	);
			printString("v / ");
			printString(	dtostrf(	(	((getLuz() * 1.00)	 / 1023 ) * 100 ) , 3 , 0 ,		buf)	);
			printString("% / ");
			printString(	dtostrf(	(	(getTempAVR() - 125) * 1.075 / 10), 2 , 1 ,			buf)	);
			printString(" oC\r\n");
}

/*
SLEEP_MODE_IDLE: 15 mA	= unico modo que monitora a USART
SLEEP_MODE_ADC: 6.5 mA
SLEEP_MODE_PWR_SAVE: 1.62 mA
SLEEP_MODE_EXT_STANDBY: 1.62 mA
SLEEP_MODE_STANDBY : 0.84 mA
SLEEP_MODE_PWR_DOWN : 0.36 mA
*/

/*
Power Reduction Register (PRR)

The next thing to experiment with is the Power Reduction Register (PRR). This lets you "turn off" various things inside the processor.
The various bits in this register turn off internal devices, as follows:

Bit 7 - PRTWI: Power Reduction TWI
Bit 6 - PRTIM2: Power Reduction Timer/Counter2
Bit 5 - PRTIM0: Power Reduction Timer/Counter0
Bit 4 - Res: Reserved bit
Bit 3 - PRTIM1: Power Reduction Timer/Counter1
Bit 2 - PRSPI: Power Reduction Serial Peripheral Interface
Bit 1 - PRUSART0: Power Reduction USART0
Bit 0 - PRADC: Power Reduction ADC

power_adc_disable;
power_usartd0_disable;
power_twi_disable;
power_spi_disable;
power_timer0_disable;
PRR |= (1<<power_twi_disable);

*/

uint16_t agora (void)
{
	char buf[20];
	uint8_t year = 0;
	uint8_t month = 0;
	uint8_t day = 0;
	uint8_t hour = 0;
	uint8_t minute = 0;
	uint8_t second = 0;
	
	ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
	
	if (MODO=='0')
	{
		sprintf(buf, "%d/%d/%d %d:%d:%d\r\n", day, month, year, hour, minute, second);;
		printString(buf);
	}
		
	return ((hour*60)+minute);	// faz um calculo para um unico inteiro representando hora e minuto
}

void habilitarWDT (void)
{
		MCUSR = 0;							// Limpa o Status Register de inicio da MCU
		
		WDTCSR = _BV (WDCE) | _BV (WDE);	// Bit 4 – WDCE: Watchdog Change Enable
											// Bit 3 – WDE: Watchdog System Reset Enable
											
		WDTCSR = _BV (WDIE) | _BV (WDP3) | _BV (WDP0);    // set WDIE, and 8 seconds delay
											/*
											Bit 6 – WDIE: Watchdog Interrupt Enable
											WDP[3:0]: Watchdog Timer Prescaler 3, 2, 1 and 0
											0 0 0 0 2K		(2048)		cycles	16 ms
											0 0 0 1 4K		(4096)		cycles	32 ms
											0 0 1 0 8K		(8192)		cycles	64 ms
											0 0 1 1 16K		(16384)		cycles	0.125 s
											0 1 0 0 32K		(32768)		cycles	0.25 s
											0 1 0 1 64K		(65536)		cycles	0.5 s
											0 1 1 0 128K	(131072)	cycles	1.0 s
											0 1 1 1 256K	(262144)	cycles	2.0 s
											1 0 0 0 512K	(524288)	cycles	4.0 s
											1 0 0 1 1024K	(1048576)	cycles	8.0
											*/
		wdt_reset();		// Limpa o Status do WDT
		ADCSRA = 0;			// Desabilita o ADC

		set_sleep_mode (SLEEP_MODE_PWR_DOWN);	// Modo de Sleep como Power Down 
		sleep_enable();							// Habilita o Sleep

		// turn off brown-out enable in software
		//MCUCR = _BV (BODS) | _BV (BODSE);
		//MCUCR = _BV (BODS);
		sleep_bod_disable(); // Faz o mesmo que as intrucoes acima
		
		sleep_cpu ();		// Coloca para dormir por 8 segundos
		sleep_disable();	// Na volta ou ACORDADA, desabilita o sleep
}
