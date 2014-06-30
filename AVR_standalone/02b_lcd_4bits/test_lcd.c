/*************************************************************************
Title:    testing output to a HD44780 based LCD display.
Author:   Peter Fleury  <pfleury@gmx.ch>  http://jump.to/fleury
File:     $Id: test_lcd.c,v 1.6 2004/12/10 13:53:59 peter Exp $
Software: AVR-GCC 3.3
Hardware: HD44780 compatible LCD text display
          ATS90S8515/ATmega if memory-mapped LCD interface is used
          any AVR with 7 free I/O pins if 4-bit IO port mode is used
**************************************************************************/
#include <stdlib.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "lcd.h"
#include <util/delay.h>

#define F_CPU 8000000UL
/*
** constant definitions
*/
static const PROGMEM unsigned char copyRightChar[] =
{
	0x07, 0x08, 0x13, 0x14, 0x14, 0x13, 0x08, 0x07,
	0x00, 0x10, 0x08, 0x08, 0x08, 0x08, 0x10, 0x00
	/* Este caractere representa o (R)
	de Marca Registrada */
};


/*
** function prototypes
*/ 

int main(void)
{
    char buffer[7];
    int  num=134;
    unsigned char i;
    
    DDRD &=~ (1 << PD2);        /* Pin PD2 input              */
    PORTD |= (1 << PD2);        /* Pin PD2 pull-up enabled    */
    DDRB |= (1 << PB0);		/* Led na porta PB0 para sinalizacao */

    //-----------------
    DDRD |= (1 << PD5);		// liga a porta D pino 5
    DDRD |= (1 << PD6);
    DDRD |= (1 << PD7);
    PORTD &= ~((1<<PD5)|(1<<PD6)|(1<<PD7));
    //-----------------
    DDRC |= (1 << PC0);
    DDRC |= (1 << PC1);
    DDRC |= (1 << PC2);
    DDRC |= (1 << PC3);

	PORTB=0b1; _delay_ms(1000);
	PORTB=0b0; _delay_ms(1000);
	PORTB=0b1; _delay_ms(1000);
	PORTB=0b0; _delay_ms(1000);
	PORTB=0b1; _delay_ms(1000);
	PORTB=0b0; _delay_ms(1000);


    /* initialize display, cursor off */
    lcd_init(LCD_DISP_ON);
	PORTB=0b1; _delay_ms(1000);
	PORTB=0b0; _delay_ms(1000);

    for (;;) {                           /* loop forever */
        /* 
         * Test 1:  write text to display
         */

        /* clear display and home cursor */
        lcd_clrscr();
        
        /* put string to display (line 1) with linefeed */
        lcd_puts("LCD Test Line 1\n");
	_delay_ms(1000);

        /* cursor is now on second line, write second line */
        lcd_puts("Line 2");
	_delay_ms(1000);
        
        /* move cursor to position 8 on line 2 */
        lcd_gotoxy(7,1);  
        
        /* write single char to display */
        lcd_putc(':');
        
        /* wait until push button PD2 (INT0) is pressed */
        
        
        /*
         * Test 2: use lcd_command() to turn on cursor
         */
        
        /* turn on cursor */
        lcd_command(LCD_DISP_ON_CURSOR);

        /* put string */
        lcd_puts( "CurOn");
	_delay_ms(1000);
        
        /* wait until push button PD2 (INT0) is pressed */


        /*
         * Test 3: display shift
         */
        
        lcd_clrscr();     /* clear display home cursor */

        /* put string from program memory to display */
        lcd_puts_P( "Line 1 longer than 14 characters\n" );
	_delay_ms(1000);
        lcd_puts_P( "Line 2 longer than 14 characters" );
	_delay_ms(1000);
        
        /* move BOTH lines one position to the left */
        lcd_command(LCD_MOVE_DISP_LEFT);
	_delay_ms(1000);
        
        /* wait until push button PD2 (INT0) is pressed */

        /* turn off cursor */
        lcd_command(LCD_DISP_ON);
        
        
        /*
         *   Test: Display integer values
         */
        
        lcd_clrscr();   /* clear display home cursor */
	_delay_ms(1000);
        
        /* convert interger into string */
        itoa( num , buffer, 10);
        
        /* put converted string to display */
        lcd_puts(buffer);
        
        /* wait until push button PD2 (INT0) is pressed */
        
        
        /*
         *  Test: Display userdefined characters
         */

       lcd_clrscr();   /* clear display home cursor */
       
       lcd_puts("Copyright: ");
       
       /*
        * load two userdefined characters from program memory
        * into LCD controller CG RAM location 0 and 1
        */
       lcd_command(_BV(LCD_CGRAM));  /* set CG RAM start address 0 */
       for(i=0; i<16; i++)
       {
           lcd_data(pgm_read_byte_near(&copyRightChar[i]));
       }
	_delay_ms(1000);
       
       /* move cursor to position 0 on line 2 */
       /* Note: this switched back to DD RAM adresses */
       lcd_gotoxy(0,1);
       
       /* display user defined (c), built using two user defined chars */
       lcd_putc(0);
       lcd_putc(1);
	_delay_ms(1000);
       

       /* wait until push button PD2 (INT0) is pressed */
              
    }
}
