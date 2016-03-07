/*
 * interruptions
 *
 * Tested on:
 * Microstick
 *
 * File name: crc.h
 * Author:    Eduardo Lopes
 * Info:      elopes_74@hotmail.com
 *
 * Last modification: 03-11-2013
 */

#ifndef _INTERRUPTS_H
#define _INTERRUPTS_H

volatile char decimo; //tempo dereptição do teclado
volatile char tick;

void tratar_contagem18F(void);

#endif
