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
#include "interrupts.h"
#include "user.h"
#include "protocolo_taxi.h"
#include <conio.h>
#endif

/******************************************************************************/
/* Interrupt Routines                                                         */
/******************************************************************************/
/* Baseline devices don't have interrupts. Note that some PIC16's
 * are baseline devices.  Unfortunately the baseline detection macro is
 * _PIC12 */
#ifndef _PIC12
#if defined(_PIC18F2550_H_)
/* High-priority service */

#if defined(__XC) || defined(HI_TECH_C)
void interrupt high_isr(void)
#elif defined (__18CXX)
#pragma code high_isr=0x08
#pragma interrupt high_isr
void high_isr(void)
#else
#error "Invalid compiler selection for implemented ISR routines"
#endif
{
    /* ccp2if indica pulso de sincronismo/laser */
    /*armazenar o valor atual de tempo e pulsos do encoder*/
    if (CCP2IF) {
        /*armazenar valor de (DEC) coletado por Tmr0 */
        EncDecAbsol.lsb_16 = READTIMER0();
        EncoderAbsoluto = EncDecAbsol.dec;
        /*verifica se há overflow de tmr0 com tmr0iF
         corrigir MSB e deixar int corrigir valor total */
        if (TMR0IF) {
            if (!(0x8000 && EncDecAbsol.lsb_16)) {
                EncoderAbsoluto += 2^8; //0x00010000;
            }
        }
        /*armazena valor de (INC) coletado por tmr1*/
        EncIncAbsol.lsb = CCPR1;
        EncoderAbsoluto = EncIncAbsol.inc - EncoderAbsoluto;
        /*verifica se há overflow de tmr1 com tmr1if
         corrigir MSB e deixar int corrigir valor total */
        if (TMR1IF) {
            if (!(0x800 && EncIncAbsol.lsb)) {
                EncoderAbsoluto += 2^8; //0x00010000;
            }
        }

        /* SEMPRE armazena e incrementa resultado*/
        if (barreira_pont <= NUMERO_TRECHO) {
            barreira[barreira_pont].pulsos = EncoderAbsoluto;
            /*armazena valor de (tempo) coletado por tmr3*/
            barreira[barreira_pont].tempo = ((long) tempoAbsol.msb << 16) + CCPR2;
            //            barreira[barreira_pont].tempo = tempoAbsol.tempo32;


            /*verifica se há overflow de tmr3 com tmr3if
             corrigir MSB e deixar int corrigir valor total*/
            if (TMR3IF) {
                if (!(0x80 && CCPR2H)) {
                    barreira[barreira_pont].tempo += 2^8; //0x00010000;
                }
            }
            if (barreira_pont)
                trecho_cont++; /* incrementa quando obtiver um trecho */
            barreira_pont += 1;
        }
        flags.disparo = 1;
        CCP2IF = false;
    } else {
        /* Unhandled interrupts */
    }
}

/* Low-priority interrupt routine */
#if defined(__XC) || defined(HI_TECH_C)
void low_priority interrupt low_isr(void)
#elif defined (__18CXX)
#pragma code low_isr=0x18
#pragma interruptlow low_isr
void low_isr(void)
#else
#error "Invalid compiler selection for implemented ISR routines"
#endif
{
    if (TMR0IF) {
        /* overflow do contador, MSB é incrementado para completar 32 bits*/
        if (EncIncAbsol.msb) {
            EncIncAbsol.msb--;
        } else {
            EncDecAbsol.msb++;
        }
        TMR0IF = 0; /* Clear Interrupt Flag */
    } else if (TMR1IF) {
        /* overflow do contador, MSB é incrementado para completar 32 bits*/ if (EncDecAbsol.msb) {
            EncDecAbsol.msb--;
        } else {
            EncIncAbsol.msb++;
        }
        TMR1IF = 0; /* Clear Interrupt Flag */
    } else if (TMR2IF) {
        tratar_contagem18F();
        TMR2IF = 0; /* Clear Interrupt Flag */
    } else if (TMR3IF) {
        /* overflow do contador, MSB é incrementado para completar 32 bits*/
        tempoAbsol.msb++;
        TMR3IF = false;
    } else if (RCIF) {
        /* armazena o byte recebido no buffer circular */
        BuffRxPtr++;
        /* reinicia ponteiro, se ocorrer overflow (64bytes) */
        if (BuffRxPtr >= 64)
            BuffRxPtr = 0;
        buffer[BuffRxPtr] = ReadUSART();
    } else {
        /* Unhandled interrupts */
    }
}


#elif defined(_PIC16F876A_H_)

void interrupt isr(void) {
    /* Determine which flag generated the interrupt */
    if (T0IF) {
        tratar_sync();
        T0IF = 0; /* Clear Interrupt Flag 1 */
    } else if (TMR1IF) {
        tratar_inc();
        TMR1IF = 0; /* Clear Interrupt Flag 2 */
    } else if (TMR0IF) {
        tratar_dec();
        TMR0IF = 0; /* Clear Interrupt Flag 3 */
    } else if (TMR2IF) {
        tratar_contagem();
        TMR2IF = 0; /* Clear Interrupt Flag 4 */
    } else {
        /* Unhandled interrupts */
    }
}
#else
#warning nenhum Header escolhido
#endif
#endif


//void tratar_sync() {
//    /*************************
//    o pulso de sincronismo externo,
//    quando habilitado, inicia e termina
//    a contagem dos pulsos.
//     *************************/
//    /*armazenar o valor atual de tempo e de encoder*/
//    decremento.lsb_8 = READTMR0();
//    incremento.lsb = READTMR1();
//
//    if (!flags.disparo) {
//        //reinicia a contagem de pulsos, na abertura da janela
//        TMR0 = 0;
//        TMR1 = 1; //aguarda 1 falling edge para iniciar contagem
//        decremento.dec = 0;
//        incremento.inc = 0;
//        flags.disparo = 1;
//        TMR0IE = false;
//    } else {
//        EncoderAjuste = incremento.inc - decremento.dec;
//        flags.disparo = 0;
//        TMR0IE = false;
//    }
//    resultado[1].pulsos = EncoderAjuste;
//    resultado[1].tempo = 1;
//}
//
//void tratar_inc() {
//    /*************************
//    A cada overflow do contador, o
//    byte superior é incrementado,
//    para completar 32 bits
//     *************************/
//    if (decremento.msb) {
//        decremento.msb--;
//    } else {
//        incremento.msb++;
//    }
//}
//
//void tratar_dec() {
//    /*************************
//    A cada overflow do contador, o
//    byte superior é incrementado,
//    para completar 32 bits
//     *************************/
//    if (incremento.msb) {
//        incremento.msb--;
//    } else {
//        decremento.msb++;
//    }
//}
//
//void tratar_contagem() {
//    /*
//    na interrupção por tempo TIMER2, os contadores serão verificados, onde:
//    EncoderContador = EncoderINC - Encoder_DEC;
//     */
//    /*************************
//    Em intervalos regulares 1/10s
//    o valor calculado de:
//    distancia percorrida e
//    velocidade instantânea
//    são atualizados
//     *************************/
//
//    i++;
//    if (i >= 99) {
//        incremento.lsb = READTIMER1();
//        decremento.lsb_8 = READTIMER0();
//        dVdt = EncoderContador;
//        EncoderContador = incremento.inc - decremento.dec;
//        dVdt = EncoderContador - dVdt;
//        //armazena o maior valor de dVdT coletado
//        if (dVdtMax < dVdt)
//            dVdtMax = dVdt;
//        i = 0;
//        flags.atualiza = 1; //libera atualização do LCD
//        decimo++;
//        if (time_out)
//            time_out--;
//    }
//    time_count++;
//    tick++;
//}

//void tratar_sync18F() {
//    /*************************
//    o pulso de sincronismo externo,
//    quando habilitado, inicia e termina
//    a contagem dos pulsos.
////     *************************/
//    /*armazenar o valor atual de tempo e de encoder*/
//    decremento.lsb_16 = READTIMER0();
//    incremento.lsb = READTIMER1();
//
//    if (!flags.disparo) {
//        //reinicia a contagem de pulsos, na abertura da janela
//        WRITETIMER0(1);
//        WRITETIMER1(1); //aguarda 1 falling edge para iniciar contagem
//        decremento.dec = 0;
//        incremento.inc = 0;
//        flags.disparo = 1;
//        //        INT0IE=false; //TMR0IE = false;
//    } else {
//        EncoderAjuste = incremento.inc - decremento.dec;
//        if (resultado_pont < NUMERO_RESULTADOS) {
//            resultado[resultado_pont].pulsos = EncoderAjuste;
//            resultado[resultado_pont].tempo = tempo32.tempo32;
//            resultado_pont += 1;
//        }
//        flags.disparo = 0;
//        //        INT0IE = false; //TMR0IE = false;
//    }
//}

//void tratar_inc18F() {
//    /*************************
//    A cada overflow do contador, o
//    byte superior é incrementado,
//    para completar 32 bits
//     *************************/
//    if (decremento.msb) {
//        decremento.msb--;
//    } else {
//        incremento.msb++;
//    }
//}

/**
 * /*************************
    A cada overflow do contador, o
    byte superior é incrementado,
    para completar 32 bits
 ************************
// */
//void tratar_dec18F() {
//
//    if (incremento.msb) {
//        incremento.msb--;
//    } else {
//        decremento.msb++;
//    }
//}

/**
 *
    na interrupção por tempo TIMER2, os contadores serão verificados, onde:
    EncoderContador = EncoderINC - Encoder_DEC;

 *************************
    Em intervalos regulares 1/10s
    o valor calculado de:
    distancia percorrida e
    velocidade instantânea
    são atualizados
 ************************
 */
void tratar_contagem18F() {
    //    static signed long Encoder100m;
    static char tempo100ms = 0;

    /*incrementa contador de tempo em 1 ms*/
    tempo100ms++;
    if (tempo100ms >= 99) {
        EncInc100ms.lsb = READTIMER1();
        EncDec100ms.lsb_16 = READTIMER0();
        EncInc100ms.msb = EncIncAbsol.msb;
        EncDec100ms.msb = EncDecAbsol.msb;
        dVdt = Encoder100ms;
        Encoder100ms = EncInc100ms.inc - EncDec100ms.dec;
        dVdt = Encoder100ms - dVdt;
        //armazena o maior valor de dVdT coletado
        if (dVdtMax < dVdt)
            dVdtMax = dVdt;
        tempo100ms = 0;
        flags.atualiza = true; //libera atualização do LCD
        decimo++;
        if (time_out)
            time_out--;
    }
    time_count++;
    tick++;
    flags.tick1ms = true;
}

///**
// *  armazena o byte recebido no buffer circular
// * reinicia ponteiro, se ocorrer overflow (64bytes)
// */
//void tratar_rx_serial() {
//    if (DataRdyUSART())
//    {
//        pRecebido++;
//        /* reinicia ponteiro, se ocorrer overflow (64bytes) */
//        if (pRecebido >= 64)
//            pRecebido = 0;
//        buffer[pRecebido] = ReadUSART();
//    }
//}