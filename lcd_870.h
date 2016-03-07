/* 
 * File:   lcd_870.h
 * Author: edu
 *
 * Created on 4 de Junho de 2015, 23:33
 */

#ifndef LCD_870_H
#define	LCD_870_H

#ifdef	__cplusplus
extern "C" {
#endif
#include <xc.h>

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
    void lcd_init();
    void lcd_gotoxy(unsigned char x, unsigned char y);
    void lcd_putc(char c);
//    void lcd_send_nibble(BYTE n);
    void lcd_send_byte(unsigned char address, unsigned char n);

#ifdef	__cplusplus
}
#endif

#endif	/* LCD_870_H */

