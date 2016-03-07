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

#if defined(__XC)
#include <xc.h>         /* XC8 General Include File */
#endif

#include <stdio.h>
#include <string.h>
#include "crc.h"

/*---------------------------------------------------------------------
  Function name: generate_16bit_crc
  Description: Initialization of UART1 module
  Input parameters: -
  Output parameters: -
-----------------------------------------------------------------------*/

unsigned short generate_16bit_crc(char* data, unsigned char length, unsigned short pattern) {
    char *current_data;
    unsigned short crc_Dbyte = 0;
    unsigned short byte_counter = 0;
    unsigned char bit_counter = 0;

    char hexa[2] = "\0";
    unsigned char current = 0;

    //utiliza o valor ASCII da string, não convertendo para numérico
    current_data = data + 2;
    memcpy(hexa, data, sizeof (float));
    crc_Dbyte = (hexa[0] << 8) + hexa[1];

    for (byte_counter = 0; byte_counter < (length - 2); byte_counter++) {
        memcpy(hexa, current_data, sizeof (char));
        current = hexa[0];
        for (bit_counter = 0; bit_counter < 8; bit_counter++) {
            //aplicar polinômio, se MSB = 1
            if (!(crc_Dbyte & 0x8000)) {
                crc_Dbyte <<= 1;
                if (current & (1 << (7 - bit_counter)))
                    crc_Dbyte |= 0x0001;
                else
                    crc_Dbyte &= 0xFFFE;
                continue;
            }
            //acrescenta próximo bit do stream
            crc_Dbyte <<= 1;
            if (current & (1 << (7 - bit_counter)))
                crc_Dbyte |= 0x0001;
            else
                crc_Dbyte &= 0xFFFE;

            crc_Dbyte ^= pattern;
        }
        //seleciona proximo byte do stream
        current_data++;
    }
    /*coloca mais 16 bits para finalizar CRC*/
    for (bit_counter = 0; bit_counter < 16; bit_counter++) {
        //se MSB = 1, aplicar o polinômio
        if (!(crc_Dbyte & 0x8000)) {
            crc_Dbyte <<= 1;
            continue;
        }
        crc_Dbyte <<= 1;
        crc_Dbyte ^= pattern;
    }
    return crc_Dbyte;
}

