
/*
  Quick and dirty functions that make serial communications work.

  Note that receiveByte() blocks -- it sits and waits _forever_ for
   a byte to come in.  If you're doing anything that's more interesting,
   you'll want to implement this with interrupts.

   initUSART requires BAUDRATE to be defined in order to calculate
     the bit-rate multiplier.  9600 is a reasonable default.

  May not work with some of the older chips:
    Tiny2313, Mega8, Mega16, Mega32 have different pin macros
    If you're using these chips, see (e.g.) iom8.h for how it's done.
    These old chips don't specify UDR0 vs UDR1.
    Correspondingly, the macros will just be defined as UDR.
*/

#include <avr/io.h>
#include "USART.h"
#include "setbaud.h"

/* *** exemplo apresentado no datasheet da Atmel para inicializacao da serial
void USART_Init( unsigned int ubrr)
{
	//Set baud rate
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	
	//Enable receiver and transmitter
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	
	// Set frame format: 8data, 2stop bit
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
	}
	
	// UBRR0H USART Baud Rate Register High
	// UBRR0L USART Baud Rate Register Low
*/

void initUSART(void) {                                /* requires BAUD */
  UBRR0H = UBRRH_VALUE;                        /* defined in setbaud.h */
  UBRR0L = UBRRL_VALUE;
  
#if USE_2X
/*
	Modo de envio e recepcao prontos (buffer zerado)
	Completamento de Envio e Recepcao
*/
  UCSR0A |= (1 << U2X0);
#else
  UCSR0A &= ~(1 << U2X0);
#endif


  /* Enable USART transmitter/receiver */
  // habilitacoes de envio e recebimento, interrupcoes
  UCSR0B = (1 << TXEN0)		| (1 << RXEN0);
  
  // Configuracao do Modo da Serial
  UCSR0C = (1 << UCSZ01)	| (1 << UCSZ00);   /* 8 data bits, 1 stop bit */
}
/*
Address	Name	Bit 7	Bit 6	Bit 5	Bit 4	Bit 3	Bit 2			Bit 1			Bit 0
(0xC2)	UCSR0C	UMSEL01	UMSEL00	UPM01	UPM00	USBS0	UCSZ01/UDORD0	UCSZ00/UCPHA0	UCPOL0
(0xC1)	UCSR0B	RXCIE0	TXCIE0	UDRIE0	RXEN0	TXEN0	UCSZ02			RXB80			TXB80
(0xC0)	UCSR0A	RXC0	TXC0	UDRE0	FE0		DOR0	UPE0			U2X0			MPCM0

UCSRnC – USART MSPIM Control and Status Register n C

Bit 7:6 – UMSELn1:0: USART Mode Select
These bits select the mode of operation of the USART as shown in Table 21-4. See ”UCSRnC –
USART Control and Status Register n C” on page 195for full description of the normal USART
operation. The MSPIM is enabled when both UMSELn bits are set to one. The UDORDn,
UCPHAn, and UCPOLn can be set in the same write operation where the MSPIM is enabled

Bit 5:3 – Reserved Bits in MSPI mode
When in MSPI mode, these bits are reserved for future use. For compatibility with future devices,
these bits must be written tozero when UCSRnC is written.

Bit 2 – UDORDn: Data Order
When set to one the LSB of the data word is transmitted first. When set to zero the MSB of the
data word is transmitted first. Refer to the Frame Formats section page 4 for details.

Bit 1 – UCPHAn: Clock Phase
The UCPHAn bit setting determine if data is sampled on the leasing edge (first) or tailing (last)
edge of XCKn. Refer to the SPI Data Modes and Timing section page 4 for details.

Bit 0 – UCPOLn: Clock Polarity
The UCPOLn bit sets the polarity of the XCKn clock. The combination of the UCPOLn and
UCPHAn bit settings determine the timing of the data transfer. Refer to the SPI Data Modes and
Timing section page 4 for details.



UCSRnB – USART MSPIM Control and Status Register n B

Bit 7 – RXCIEn: RX Complete Interrupt Enable
Writing this bit to one enables interrupt on the RXCn Flag. A USART Receive Complete interrupt
will be generated only if the RXCIEn bit is written to one, the Global Interrupt Flag in SREG is
written to one and the RXCn bit in UCSRnA is set

Bit 6 – TXCIEn: TX Complete Interrupt Enable
Writing this bit to one enables interrupt on the TXCn Flag. A USART Transmit Completeinterrupt
will be generated only if the TXCIEn bit is written to one, the Global Interrupt Flag in SREG is
written to one and the TXCn bit in UCSRnA is set.

Bit 5 – UDRIE: USART Data Register Empty Interrupt Enable
Writing this bit to one enables interrupt on the UDREn Flag. A Data RegisterEmpty interrupt will
be generated only ifthe UDRIE bit is written to one, the Global Interrupt Flag in SREG is written
to one and the UDREn bit in UCSRnA is set.

Bit 4 – RXENn: Receiver Enable
Writing this bit to one enables the USART Receiver in MSPIM mode. The Receiver will override
normal port operation for the RxDn pin when enabled. Disabling the Receiver will flush the
receive buffer. Only enabling the receiver in MSPI mode (i.e. setting RXENn=1 and TXENn=0)
has no meaning since it is the transmitter that controls the transfer clock and since only master
mode is supported.

Bit 3 – TXENn: Transmitter Enable
Writing this bit to one enables the USART Transmitter. The Transmitter will override normal port
operation for the TxDn pin when enabled. The disabling of the Transmitter (writing TXENn to
zero) will not become effective until ongoing and pending transmissions are completed, i.e.,
when the Transmit Shift Register and Transmit Buffer Register do not contain data to be transmitted.
When disabled, the Transmitter will no longer override the TxDn port.



UCSRnA – USART MSPIM Control and Status Register n A

Bit 7 – RXCn: USART Receive Complete
This flag bit is set when there are unread data in the receive buffer and cleared when the receive
buffer is empty (i.e., does not contain any unread data). If the Receiver is disabled, the receive
buffer will be flushed and consequently the RXCn bit will become zero. The RXCn Flag can be
used to generate a Receive Complete interrupt (see description of the RXCIEn bit)

Bit 6 – TXCn: USART Transmit Complete
This flag bit is set when the entire frame in the Transmit Shift Register has been shifted out and
there are no new data currently present in the transmit buffer (UDRn). The TXCn Flag bit is automatically
cleared when a transmit complete interrupt is executed, or it can be cleared by writing
a one to its bit location. The TXCn Flag can generate a Transmit Complete interrupt (see
description of the TXCIEn bit).

Bit 5 – UDREn: USART Data Register Empty
The UDREn Flag indicates if the transmit buffer (UDRn) is ready to receive new data. If UDREn
is one, the buffer is empty, and therefore ready to be written. The UDREn Flag can generate a
Data Register Empty interrupt (see description of the UDRIE bit). UDREn is set after a reset to
indicate that the Transmitter is ready
*/


void transmitByte(uint8_t data) {
                                     /* Wait for empty transmit buffer */
/*
Bit 5 – UDREn: USART Data Register Empty 
(bit do registrador UCSRnA – USART Control and Status Register n A)

The UDREn Flag indicates if the transmit buffer (UDRn) is ready to receive new data. If UDREn
is one, the buffer is empty, and therefore ready to be written. The UDREn Flag can generate a
Data Register Empty interrupt (see description of the UDRIEn bit). UDREn is set after a reset to
indicate that the Transmitter is ready.
*/									 
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = data;                                            /* send data */
}

/*
UDRn – USART I/O Data Register n

The USART Transmit Data Buffer Register and USART Receive Data Buffer Registers share the
same I/O address referred to as USART Data Register orUDRn. The Transmit Data Buffer Register (TXB) will be
the destination for data written to the UDRn Registerlocation. Reading the
UDRn Register location will return the contents of the Receive Data Buffer Register (RXB).
For 5-, 6-, or 7-bit characters the upper unused bits will be ignored by the Transmitter and set to
zero by the Receiver.

The transmit buffer can only be written when the UDREn Flag in the UCSRnA Register is set.
Data written to UDRn when the UDREn Flag is not set, will beignored by the USART Transmitter. When data is 
written to the transmit buffer, and the Transmitter is enabled, the Transmitter
will load the data into the Transmit Shift Register when the Shift Register is empty. Then the
data will be serially transmitted on the TxDn pin.
The receive buffer consists of a two level FIFO. The FIFO will change its state whenever the
receive buffer is accessed. Due to this behavior of the receive buffer, do not use Read-ModifyWrite instructions 
(SBI and CBI) on this location. Be careful when using bit test instructions
(SBIC and SBIS), since these also will change the state of the FIFO
*/

uint8_t receiveByte(void) {
  loop_until_bit_is_set(UCSR0A, RXC0);       /* Wait for incoming data */
  return UDR0;                                /* return register value */
}


/* Here are a bunch of useful printing commands */

void printString(const char myString[]) {
  uint8_t i = 0;
  while (myString[i]) {
    transmitByte(myString[i]);
    i++;
  }
}

void readString(char myString[], uint8_t maxLength) {
  char response;
  uint8_t i;
  i = 0;
  while (i < (maxLength - 1)) {                   /* prevent over-runs */
    response = receiveByte();
    transmitByte(response);                                    /* echo */
    if (response == '\r') {                     /* enter marks the end */
      break;
    }
    else {
      myString[i] = response;                       /* add in a letter */
      i++;
    }
  }
  myString[i] = 0;                          /* terminal NULL character */
}

void printByte(uint8_t byte) {
              /* Converts a byte to a string of decimal text, sends it */
  transmitByte('0' + (byte / 100));                        /* Hundreds */
  transmitByte('0' + ((byte / 10) % 10));                      /* Tens */
  transmitByte('0' + (byte % 10));                             /* Ones */
}

void printWord(uint16_t word) {
  transmitByte('0' + (word / 10000));                 /* Ten-thousands */
  transmitByte('0' + ((word / 1000) % 10));               /* Thousands */
  transmitByte('0' + ((word / 100) % 10));                 /* Hundreds */
  transmitByte('0' + ((word / 10) % 10));                      /* Tens */
  transmitByte('0' + (word % 10));                             /* Ones */
}

void printBinaryByte(uint8_t byte) {
                       /* Prints out a byte as a series of 1's and 0's */
  uint8_t bit;
  for (bit = 7; bit < 255; bit--) {
    if (bit_is_set(byte, bit))
      transmitByte('1');
    else
      transmitByte('0');
  }
}

char nibbleToHexCharacter(uint8_t nibble) {
                                   /* Converts 4 bits into hexadecimal */
  if (nibble < 10) {
    return ('0' + nibble);
  }
  else {
    return ('A' + nibble - 10);
  }
}

void printHexByte(uint8_t byte) {
                        /* Prints a byte as its hexadecimal equivalent */
  uint8_t nibble;
  nibble = (byte & 0b11110000) >> 4;
  transmitByte(nibbleToHexCharacter(nibble));
  nibble = byte & 0b00001111;
  transmitByte(nibbleToHexCharacter(nibble));
}

uint8_t getNumber(void) {
  // Gets a numerical 0-255 from the serial port.
  // Converts from string to number.
  char hundreds = '0';
  char tens = '0';
  char ones = '0';
  char thisChar = '0';
  do {                                                   /* shift over */
    hundreds = tens;
    tens = ones;
    ones = thisChar;
    thisChar = receiveByte();                   /* get a new character */
    transmitByte(thisChar);                                    /* echo */
  } while (thisChar != '\r');                     /* until type return */
  return (100 * (hundreds - '0') + 10 * (tens - '0') + ones - '0');
}
