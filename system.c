/******************************************************************************/
/*Files to Include                                                            */
/******************************************************************************/

#if defined(__XC)
    #include <xc.h>         /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>        /* HiTech General Include File */
#elif defined(__18CXX)
    #include <p18cxxx.h>    /* C18 General Include File */
#endif

#if defined(__XC) || defined(HI_TECH_C)

#include <stdint.h>         /* For uint8_t definition */
#include <stdbool.h>        /* For true/false definition */

#endif

#include "system.h"

/* Refer to the device datasheet for information about available
oscillator configurations. */
void ConfigureOscillator(void)
{

    /* Typical actions in this function are to tweak the oscillator tuning
    register, select new clock sources, and to wait until new clock sources
    are stable before resuming execution of the main project. */

#ifdef _PIC18F2550_H_
    /*desliga o Watch Dog timer*/
    WDTCONbits.SWDTEN = 0;

    /*muda para o clock principal*/
    OSCCONbits.SCS = 0;


    /*aguarda a mudança para o PLL*/
    while (OSCCONbits.OSTS != 0b001);
    /*aguarda o PLL travar*/


    WDTCONbits.SWDTEN = 1;
#endif
}
