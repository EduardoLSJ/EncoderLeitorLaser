/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/
#ifndef _USER_H
#define _USER_H

#include <xc.h>
#include <stddef.h>
#include <string.h>
#include "system.h"
#include <float.h>

/******************************************************************************/
/* User Function Prototypes                                                   */
/******************************************************************************/
//void tratar_sync(void);
//void tratar_inc(void);
//void tratar_dec(void);
//void tratar_contagem(void);
char aguarda_botao(void);
//void atualiza_LCD(void);
void ajusta_parametros(void);
void inicia_bluetooth(void);
long mult_with10(long num);
void get_string(char* s, unsigned char max, char dp);
long atol(char *s);
long get_long(char nchar, char dp);
void atualizaDistVel(void);
char lerBuffer(void);
void putch(char data);
void BT_putc(char data);
void vInitU1(void);
void (*putchFunc)(char c);
void eeprom_read_object(unsigned int ee_addr, void *obj_p, size_t obj_size);
void eeprom_write_object(unsigned int ee_addr, void *obj_p, size_t obj_size);
void reinicia_coleta(void);

/****************************
Define Hardware do equipamento
 ****************************/
#ifdef _PIC_H_
#define TRIS_A 	0b00010000
#define TRIS_B 	0b00110111
#define TRIS_C 	0b11000001

#define pulso 	PORTAbits.RA5
#define sync	PORTBbits.RB0
#define B_Zero	PORTBbits.RB1
#define B_Func	PORTBbits.RB2
#define BT_Key	PORTCbits.RC3
#define BT_Reset PORTCbits.RC2

#elif defined _PIC18_H

#define TRIS_A 	0b00010000
#define TRIS_B 	0b00000110
#define TRIS_C 	0b11000111

//#define pulso 	LATA5
#define sync	LATC1//B0
#define B_Zero	LATB1
#define B_Func	LATB2
//#define BT_Key	LATC3
#define BT_Reset LATA5

#else
#warning nenhum Header escolhido
#endif

//#byte kbd = PORTB                  // on to port B (at address 6)
#define kbd PORTB
#define Bot0 (1 << 1)
#define Bot1 (1 << 2)

#define ALL_BOTTONS (Bot0|Bot1)

#define fator_debounce	20	//20ms para eliminar o repique
#define	fator_repeat	5	//500ms para tecla de repetição

#define BT_LINVOR
//#define BT_HC05
//#define BT_BOLUTEK

#define NUMERO_TRECHO 25

/******************************************************************************/
/* User Global Variable Declaration                                           */
/******************************************************************************/

/*valor de pulsos de entrada com sentido horário ou incremento*/
union INCStruct {
    unsigned long inc; /*armazena o valor final para cálculo*/

    struct {
        unsigned int lsb; /*armazena leitura do timer1*/
        unsigned int msb; /*armazena overflow para completar 32bits*/
    };

    struct {
        char unused;
        unsigned char mid; /*registro intermediario para 16 bits*/
    };

};

/*valor de pulsos de entrada com sentido anti-horário ou decremento*/
union DECStruct {
    unsigned long dec; /*armazena o valor final para cálculo*/

    struct {
        unsigned int lsb_16; /*armazena leitura do timer0*/
        unsigned int msb; /*armazena overflow para completar 32 bits*/
    };

    struct {
        unsigned char lsb_8;
        unsigned char mid; /*registro intermediario para 16 bits*/
    };

};

/*contador de tempo*/
union tempoStruct {
    unsigned long tempo32;

    struct {
        unsigned int lsb; /*armazena leitura do timer2*/
        unsigned int msb; /*armazena overflow para completar 32 bits*/
    };
};

/*valores absolutos a gravar na matriz*/
typedef struct {
    signed long pulsos;
    unsigned long tempo;
} medida;

/*flags de alteração temporal*/
struct {
    unsigned atualiza : 1;
    unsigned disparo : 1;
    unsigned tick1ms : 1;
} volatile flags;
; /* ponteiro dos resultados parciais*/

/*Valores absolutos*/
/*valores instantâneos de cada passagem pela barreira*/
volatile medida barreira[NUMERO_TRECHO + 1];
volatile char barreira_pont; /* ponteiro dos resultados parciais*/
volatile char trecho_cont; /*ponteiro para o numero de trechos*/

/*Valores absolutos*/
volatile union INCStruct EncIncAbsol;
volatile union DECStruct EncDecAbsol;
volatile union tempoStruct tempoAbsol;
volatile signed long EncoderAbsoluto;

/*Valores para atualização de 100 ms no display*/
volatile union INCStruct EncInc100ms; //valor coletado a cada 100ms
volatile union DECStruct EncDec100ms; //valor coletado a cada 100ms
float distancia;
//float k;
/*TODO corrigir k para long */
unsigned long k;
float velocidade;
volatile signed long dVdt, dVdtMax,
Encoder100ms; //valor atualizado em 100ms do encoder absoluto

signed long EncoderAjusteK;

unsigned int time_out,
time_count; //tempo morto para redisparo do laser

enum {
    PRINCIPAL,
    MENU_SAIR,
    MENU_ZERA_DIST,
    MENU_LIMPA_COLETA,
    MENU_MEDE_DIST,
    MENU_APRES_K,
    MENU_AJUSTE_K,
    MENU_K_AUTO_MAN,
    AJUSTE_MAN,


    ZERA_DIST,
    LIMPA_COLETA,
    APRES_K,
    AJUSTE_K,
    APRES_COLETA,
    COLETA_ZERO,
    DIST_MAT,
    AJUSTE,
    MED_DIST,
    COLETA_DIST,
    COLETA_INI1,
    COLETA_TECLADO,
    COLETA_ACK,
    DELAY
};

/* Speed of the UART1 module */
#define BAUD_RATE_UART1	9600L
#define USART_BRG ((long) SYS_FREQ / (4 * (long)BAUD_RATE_UART1)) - 1

#define USART2_ADCON 0x0F /*a/d disabled*/
#define USART2_CMCON 0X07 /*disable all comparators*/

/*posição inicial da memoria eeprom*/
#define EEPROM_k 0x00
__EEPROM_DATA(0x79, 0x2A, 0x4A, 0, 0, 0, 0, 0);
//__EEPROM_DATA(0xAE, 0xE7, 0x41, 0, 0, 0, 0, 0);
#endif


char str_buf[20]; //buffer para escrita no LCD

enum mensagens_main_list {
    Forno,
    Forno1,
    Inicializando,
    Forno_SMD,
    FALHA,
    troque,
    Calibracao,
    ENT,
    DINST,
    Iniciar,
    Editar,
    Registrar,
    Calibrar,
    Terminado,
    press,
    DINST1
};

//const char* mensagens_main[] = {
//  "Forno SMD v1.0\n\r",
//  "Forno SMD v1.0",
//  "  Inicializando",
//  "\n\r\n\rForno SMD\n\r",
//  "FALHA DA EEPROM!",
//  "troque o 24c1024",
//  "Calibracao",
//  "[ENT] inicia",
//  "DINST   SMD-OVEN",
//  "->Iniciar Solda",
//  "->Editar Perfil",
//  "->Registrar",
//  "->Calibrar",
//  "Terminado",
//  "press [ENT]",
//  "DINST     %2uh%2um"
//};
