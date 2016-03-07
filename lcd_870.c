///////////////////////////////////////////////////////////////////////////
////                             LCDD.C                                ////
////                 Driver for common LCD modules                     ////
////                                                                   ////
////  lcd_init()   Must be called before any other function.           ////
////                                                                   ////
////  lcd_putc(c)  Will display c on the next position of the LCD.     ////
////                     The following have special meaning:           ////
////                      \f  Clear display                            ////
////                      \n  Go to start of second line               ////
////                      \b  Move back one position                   ////
////                                                                   ////
////  lcd_gotoxy(x,y) Set write position on LCD (upper left is 1,1)    ////
////                                                                   ////
////  lcd_getc(x,y)   Returns character at position x,y on LCD         ////
////                                                                   ////
///////////////////////////////////////////////////////////////////////////
////        (C) Copyright 1996,2003 Custom Computer Services           ////
//// This source code may only be used by licensed users of the CCS C  ////
//// compiler.  This source code may only be distributed to other      ////
//// licensed users of the CCS C compiler.  No other use, reproduction ////
//// or distribution is permitted without written permission.          ////
//// Derivative programs created using this software in object code    ////
//// form are not restricted in any way.                               ////
///////////////////////////////////////////////////////////////////////////
#if defined(__XC)
#include <xc.h>         /* XC8 General Include File */
#include <stdint.h>         /* For uint8_t definition */
#include <stdbool.h>        /* For true/false definition */
#endif

#include "lcd_870.h"
#include "system.h"


/*
hardware:
RA0 - D4 LCD
RA1 - D5 LCD
RA2 - D6 LCD
RA3 - D7 LCD
RC4 - EN LCD
RC5 - RS LCD
 */

// As defined in the following structure the pin connection is as follows:
//     D0  enable
//     D1  rs
//     D2  rw
//     D4  D4
//     D5  D5
//     D6  D6
//     D7  D7
//
//   LCD pins D0-D3 are not used and PIC D3 is not used.

// Un-comment the following define to use port B
// #define use_portb_lcd TRUE


#if defined (_PIC16F876A_H_)

struct lcd_pin_map { // This structure is overlayed
    unsigned data : 4; // on to an I/O port to gain
    unsigned enable : 1; // access to the LCD pins.
    unsigned rs : 1; // The bits are allocated from
    // low order up.  ENABLE will
    // be pin B0.
};

volatile struct lcd_pin_map lcd @ &PORTA;
volatile struct lcd_pin_map lcd_ctr @ &PORTC;

#elif defined(_PIC18F2550_H_)

struct lcd_pin_map { // This structure is overlayed
    unsigned data : 4; // on to an I/O port to gain
    unsigned enable : 1; // access to the LCD pins.
    unsigned rs : 1; // The bits are allocated from
    // low order up.  ENABLE will
    // be pin B0.
};

volatile struct lcd_pin_map lcd @ &LATA;
volatile struct lcd_pin_map lcd_ctr @ &LATB;


#else
#warning nenhum header escolhido
#endif

#define lcd_type 2           // 0=5x7, 1=5x10, 2=2 lines
#define lcd_line_two 0x40    // LCD RAM address for the second line


unsigned char const LCD_INIT_STRING[4] = {0x20 | (lcd_type << 2), 0xc, 1, 6};
// These bytes need to be sent to the LCD
// to start it up.


// The following are used for setting
// the I/O port direction register.

struct lcd_pin_map const LCD_WRITE = {0, 0, 0}; // For write mode all pins are out
struct lcd_pin_map const LCD_READ = {15, 0, 0}; // For read mode data pins are in

unsigned char lcdline;

//BYTE lcd_read_byte() {
//    BYTE low, high;
//    //      set_tris_lcd(LCD_READ);
//    //      lcd.rw = 1;
//    //      delay_cycles(1);
//    lcd_ctr.enable = 1;
//    __delay_us(1);
//    high = lcd.data;
//    lcd_ctr.enable = 0;
//    __delay_us(1);
//    lcd_ctr.enable = 1;
//    __delay_us(1);
//    low = lcd.data;
//    lcd_ctr.enable = 0;
//    return ( (high << 4) | low);
//}

void lcd_send_nibble(unsigned char n) {
    lcd.data = n;
    __delay_us(1);
    lcd_ctr.enable = 1;
    __delay_us(2);
    //		delay_cycles(10);
    lcd_ctr.enable = 0;
}

void lcd_send_byte(unsigned char address, unsigned char n) {
    //      set_tris_lcd(LCD_WRITE);
    lcd_ctr.rs = 0;
    __delay_us(48); //aguarda periodo de execução
    //      while ( bit_test(lcd_read_byte(),7) ) ;
    lcd_ctr.rs = address;
    //***      delay_cycles(1);
    //      lcd.rw = 0;
    //      delay_cycles(1);
    lcd_ctr.enable = 0;
    lcd_send_nibble(n >> 4); //byte superior
    lcd_send_nibble(n & 0xf); //byte inferior
}

void lcd_init() {
    unsigned char i;
#ifdef debug
    return;
#endif

    //    set_tris_lcd(LCD_WRITE);
    lcd_ctr.rs = 0;
    //    lcd.rw = 0;
    lcd_ctr.enable = 0;
    CLRWDT();
    __delaywdt_ms(10);
    //    __delaywdt_ms(10);//15
    for (i = 1; i <= 3; ++i) {
        lcd_send_nibble(3);
        CLRWDT();
        __delaywdt_ms(5);
        //        __delaywdt_ms(5);
    }
    lcd_send_nibble(2);
    for (i = 0; i <= 3; ++i)
        lcd_send_byte(0, LCD_INIT_STRING[i]);
}

void lcd_gotoxy(unsigned char x, unsigned char y) {
    unsigned char address;

#ifdef debug
    return;
#endif

    switch (y) {
            //seleciona o endereço de coluna0 para cada linha
        case 1: address = 0x80;
            break;
        case 2: address = 0xc0;
            break;
        case 3: address = 0x90;
            break;
        case 4: address = 0xd0;
            break;
    }
    lcdline = y;
    address += x - 1;
    lcd_send_byte(0, address);
}

//void lcd_gotoxy( BYTE x, BYTE y) {
//   BYTE address;
//
//   if(y!=1)
//     address=lcd_line_two;
//   else
//     address=0;
//   address+=x-1;
//   lcd_send_byte(0,0x80|address);
//}

void lcd_putc(char c) {
#define SHR_C 	0x1C 	// desloca caracter pra direita
#define SHL_C 	0x18 	// desloca caracter pra esquerda
    unsigned char i;

#ifdef debug
    return;
#endif

    switch (c) {
        case '\a':
            for (i = 1; i <= 16; ++i) {
                lcd_send_byte(0, SHL_C);
                __delay_us(48); //aguarda periodo de execução
            }
            break; //shift display
        case '\f':
            lcd_send_byte(0, 1);
            lcdline = 1;
            CLRWDT();
            __delaywdt_ms(2);
            //            __delaywdt_ms(2);
            break;
        case '\n':
            lcd_gotoxy(1, ++lcdline);
            break;
        case '\r':
            lcd_gotoxy(1, lcdline);
            break; //retorna home da linha
            //     case '\r'   : lcd_send_byte(0,0x02);    	   break; //retorna home
        case '\b':
            lcd_send_byte(0, 0x10);
            break; //backspace
        default:
            lcd_send_byte(1, c);
            break;
    }
}

//char lcd_getc( BYTE x, BYTE y) {
//   char value;
//
//    lcd_gotoxy(x,y);
////    while ( bit_test(lcd_read_byte(),7) ); // wait until busy flag is low
//    __delaywdt_ms(2);
//    lcd_ctr.rs=1;
//    value = lcd_read_byte();
//    lcd_ctr.rs=0;
//    return(value);
//}
