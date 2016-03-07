/* 
 * File:   protocolo_taxi.h
 * Author: edu
 *
 * Created on 4 de Junho de 2015, 22:32
 */

#ifndef PROTOCOLO_TAXI_H
#define	PROTOCOLO_TAXI_H

#ifdef	__cplusplus
extern "C" {
#endif

#define NUMERO_BYTES_MENSAGEM 16

    char buffer[64], //buffer circular de comunicação serial BT
    BuffLidoPtr, //ponteiro de Leitura do buffer circular BT
    BuffRxPtr; //ponteiro de Escrita do buffer circular BT

    enum tipo_mensagem {
        INSTRUCAO = 0xA2,
        RESPOSTA,
        ERRO = 0xA5
    };

    enum FORMAT_PAYLOAD_PROTOCOL {
        FORMAT_HEX,
        FORMAT_BCD
    };

    enum rx_comandos {
        NOP,
        RESET_SISTEMA = 0x80,
        INICIAR_SISTEMA,
        ZERAR_DISTANCIA,
        CANCELAR,
        AJUSTAR,
        MEDIR_DIST,
        MAIOR_VEL,
        ENVIA_DIST = 0x0A,
        FINALIZA_AJUSTE,
        ENVIA_LEITURA,
        ZERAR_COLETA = 0x40, //REINICIA CONTADOR DE COLETA DE PERCURSO
        QUANT, /*QUANTIDADE DE PERCURSOS COLETADOS (100 m CADA)*/
        LEITURA_100M /*RETORNA O VALOR (TEMPO, PULSOS) DO PERCURSO, i*/
    };

    //comando

    typedef struct {
        char cmd;
        char nBytes;
        char formato;

    } comando;

    //valor (hex) dos comandos, para comparação.
    comando lista_comandos_entrada[] = {
        {NOP, 0, 0},
        {RESET_SISTEMA, 0, 0},
        {INICIAR_SISTEMA, 6, 1},
        {ZERAR_DISTANCIA, 0, 0},
        {CANCELAR, 0, 0},
        {AJUSTAR, 0, 0},
        {MEDIR_DIST, 0, 0},
        {MAIOR_VEL, 0, 1},
        {ZERAR_COLETA, 0, 0},
        {QUANT, 0, 0},
        {LEITURA_100M, 1, 0}
    };
    comando lista_comandos_saida[] = {
        {ENVIA_DIST, 3, 1},
        {FINALIZA_AJUSTE, 3, 1},
        {ENVIA_LEITURA, 6, 1}
    };

    //lista de estados para protocolo_rx

    enum {
        RX_STX,
        RX_COMANDO,
        RX_FORMATO,
        RX_NUMERO,
        RX_DADOS,
        RX_CRC,
    };

    // reserved Registers

    typedef struct {
        char stx;
        char comando;
        char formato; // : 1;
        char numero;
        char dados[NUMERO_BYTES_MENSAGEM];
        int crc;
    } frame; //frame

    frame rx_frame;
    frame tx_frame;

    short int protocolo_rx(frame *rx_frame, frame *tx_frame);
    void protocolo_tx(frame *tx_frame);
    unsigned long getNumber(frame *rx_frame, char pos, char size);
    void setNumber(frame *tx_frame, long origem, char pos, char size);

#ifdef	__cplusplus
}
#endif

#endif	/* PROTOCOLO_TAXI_H */

