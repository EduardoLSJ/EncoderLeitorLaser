/*
 * modulo CRC
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

#ifndef _CRC_H
#define _CRC_H


/* polinomio de calculo do CRC */
#define CRC_16    0x8005      //bit pattern (1)1000 0000 0000 0101

/*---------------------------------------------------------------------
  Function name: generate_16bit_crc
  Description: calcula o CRC da stream no Arg1
  Input parameters: -
  Output parameters: -
-----------------------------------------------------------------------*/
unsigned short generate_16bit_crc(char* data, unsigned char length, unsigned short pattern);

#endif
