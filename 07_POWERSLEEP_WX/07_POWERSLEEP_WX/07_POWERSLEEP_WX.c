/*
 * _07_POWERSLEEP_WX.c
 *
 * Created: 09/08/2014 16:50:49
 *  Author: roney
 */ 

#define F_CPU 1000000L


#include <avr/io.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <util/delay.h>

#include "davidegironi/ds1307.h"

#include "rotinas_projetos_anteriores.h"

void agora (void);

int main(void)
{
	char buf[7]; // antes apenas 7 era o suficiente
	uint8_t contador;

	ds1307_init();
	iniciaPORTAS();
	
	printString("\r\nProj 07 Power Sleep. (10 seg RXTX delay)_");
	agora();
	

	
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
		
		
	printString("\r\nLoop em Sleep Mode - Serial RX para Acordar.\r\n");
    while(1)
    {
		cli();
		for (contador=0;contador<3;contador++)
		{
		printString(	dtostrf(	(	(getVolt()*4.56) / 1023	)	,	4	,	2	,		buf)	);
		printString("v / ");
		printString(	dtostrf(	(	((getLuz() * 1.00)	 / 1023 ) * 100 ) , 3 , 0 ,		buf)	);
		printString("% / ");
		printString(	dtostrf(	(	(getTempAVR() - 125) * 1.075 / 10), 2 , 1 ,			buf)	);
		printString(" oC\r\n");
		_delay_ms(3333);
		}
		agora();		
		
		//ADCSRA = 0;
		
										// limpa todos interrupts, desabilitando-os
		
		set_sleep_mode(SLEEP_MODE_IDLE);	// configura o MODO de sleep
		

		
		sei();								// habilita todos interrupts
		
        
		//printString("Habilitando Sleep.\r\n");
		sleep_enable();						// poe para dormir
		
		power_adc_disable();
		power_spi_disable();
		power_timer0_disable();
		power_timer1_disable();
		power_timer2_disable();
		power_twi_disable();
		
		//printString("Desabilitando BOD.\r\n");
		sleep_bod_disable();
		
		printString("dormindo...");
		sleep_mode();
		
		printString("...Acordou!\r\n");
		sleep_disable();
		power_all_enable();
		agora();
		
    }
}

ISR(USART_RX_vect)
{
	char BYTESERIAL;
	
	BYTESERIAL = receiveByte();
	transmitByte(BYTESERIAL);
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

void agora (void)
{
	char buf[20];
	uint8_t year = 0;
	uint8_t month = 0;
	uint8_t day = 0;
	uint8_t hour = 0;
	uint8_t minute = 0;
	uint8_t second = 0;
	ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
	sprintf(buf, "%d/%d/%d %d:%d:%d\r\n", day, month, year, hour, minute, second);;
	printString(buf);
}
