
/************************************************************************
 ****                      Protocolo Mototaximetro                    ****
 ****                                                                 ****
 ****	Protótipos, definições e defines usados para				 ****
 **** 	módulos de comunicação serial e Bluetooth					 ****
 *************************************************************************
 ****                                                                 ****
 **** 	Essa biblioteca apresenta as seguintes funções:				 ****
 ****	(para mais informações veja os comentários nos cabeçalhos	 ****
 ****	de cada função.)                                             ****
 ****                                                                 ****
 ****	short int protocolo_rx(rx_frame *frame)	recebe dados/		 ****
 ****		instruções pela serial									 ****
 ****	                                                             ****
 ****	void protocolo_tx(struct rx_frame *frame) responde comandos	 ****
 ****	  pela porta serial											 ****
 ****                                                                 ****
 ****                                                                 ****
 ************************************************************************/
/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#if defined(__XC)
#include <xc.h>         /* XC8 General Include File */
#endif

#if defined(__XC)
#include <stdint.h>         /* For uint8_t definition */
#include <stdbool.h>        /* For true/false definition */
#endif

#include "protocolo_taxi.h"
#include "user.h"
#include "crc.h"
#include <usart.h>

/**
 * Rotina para decodificar o protocolo
 *
 * recebe frame, verifica se recepção com
 * sucesso separa os dados e
 * preenche outro frame com a resposta
 *
 * @param rx_frame: recebe frame
 * @param tx_frame: preenche outro frame com a resposta
 * @return short int: válido (true) ou inválido(false)
 */
short int protocolo_rx(frame *rx_frame, frame *tx_frame) {
    
    int crc_calc;
    char stream[15],
            stream_indice, //indice do fluxo de dados
            quadro_indice, //indice do quadro recebido
            caracter,
            rx_estado,
            i;
    int time_out = 2; //200ms = 16x tempo do quadro (12*1/960)
    comando* lista_comandos;
    (*rx_frame).comando = 0;

    rx_estado = RX_STX; //inicia estado

    stream_indice = 0; //bytes recebidos

    while (true) {
        do {
            if (time_out == 0) //timer expired
            {
                (*tx_frame).stx = ERRO;
                (*tx_frame).comando = (*rx_frame).comando;
                (*tx_frame).formato = 0x00;
                (*tx_frame).numero = 0x02;
                (*tx_frame).dados[0] = 0x00;
                (*tx_frame).dados[1] = 0x03; //protocolo_tx(erro de atraso 0x03)
                return (false);
            }
        } while (BuffRxPtr == BuffLidoPtr);
        caracter = lerBuffer();

        stream[stream_indice] = caracter;

        switch (rx_estado) {
            case RX_STX:
                if (INSTRUCAO == caracter || RESPOSTA == caracter || ERRO == caracter) {
                    rx_estado = RX_COMANDO;
                    stream_indice++;
                    (*rx_frame).stx = caracter;
                } else {
                    (*tx_frame).stx = ERRO;
                    (*tx_frame).comando = (*rx_frame).comando;
                    (*tx_frame).formato = 0x00;
                    (*tx_frame).numero = 0x02;
                    (*tx_frame).dados[0] = 0x00;
                    (*tx_frame).dados[1] = 0x01; //protocolo_tx(erro de quadro 0x01)
                    return (false); //comando inválido
                }
                break;

            case RX_COMANDO:
                lista_comandos = 0;
                if (INSTRUCAO == (*rx_frame).stx) {
                    for (i = 0; i < sizeof (lista_comandos_entrada); i++) {
                        if (caracter == lista_comandos_entrada[i].cmd) {
                            lista_comandos = &lista_comandos_entrada[i];
                            break;
                        }
                    }
                } else {
                    for (i = 0; i < sizeof (lista_comandos_saida); i++) {
                        if (caracter == lista_comandos_saida[i].cmd) {
                            lista_comandos = &lista_comandos_saida[i];
                            break;
                        }
                    }
                }
                if (!lista_comandos) {
                    (*tx_frame).stx = ERRO;
                    (*tx_frame).comando = caracter;
                    (*tx_frame).formato = 0x00;
                    (*tx_frame).numero = 0x02;
                    (*tx_frame).dados[0] = 0x00;
                    (*tx_frame).dados[1] = 0x05; //protocolo_tx(erro comando inválido 0x05)
                    return (false); //comando inválido, não está na lista
                }
                stream_indice++;
                (*rx_frame).comando = caracter;
                rx_estado = RX_FORMATO;
                break;

            case RX_FORMATO:
                if (caracter > 1) {
                    (*tx_frame).stx = ERRO;
                    (*tx_frame).comando = (*rx_frame).comando;
                    (*tx_frame).formato = 0x00;
                    (*tx_frame).numero = 0x02;
                    (*tx_frame).dados[0] = 0x00;
                    (*tx_frame).dados[1] = 0x01; //protocolo_tx(erro de quadro, palavra com formato inválido)
                    return (false);
                } else {
                    stream_indice++;
                    (*rx_frame).formato = caracter;
                    rx_estado = RX_NUMERO;
                }
                break;

            case RX_NUMERO:
                if (caracter != (*lista_comandos).nBytes) {
                    (*tx_frame).stx = ERRO;
                    (*tx_frame).comando = (*rx_frame).comando;
                    (*tx_frame).formato = 0x00;
                    (*tx_frame).numero = 0x02;
                    (*tx_frame).dados[0] = 0x00;
                    (*tx_frame).dados[1] = 0x04; //protocolo_tx(erro de comprimento da mensagem)
                    return (false);
                }
                stream_indice++;
                quadro_indice = 0;
                (*rx_frame).numero = caracter;
                i = caracter; //contador das mensagens
                if (caracter)
                    rx_estado = RX_DADOS;
                else {
                    rx_estado = RX_CRC; //contador de bytes CRC
                    i = 2;
                }
                break;

            case RX_DADOS:
                i--;
                stream_indice++;
                (*rx_frame).dados[quadro_indice] = caracter;
                quadro_indice++;
                if (!i) {
                    rx_estado = RX_CRC;
                    i = 2; //contador de bytes CRC
                }
                break;

            case RX_CRC:
                i--;
                stream_indice++;
                if (!i) {
                    crc_calc = generate_16bit_crc(stream, stream_indice, CRC_16);

                    if (crc_calc) {
                        (*tx_frame).stx = ERRO;
                        (*tx_frame).comando = (*rx_frame).comando;
                        (*tx_frame).formato = 0x00;
                        (*tx_frame).numero = 0x02;
                        (*tx_frame).dados[0] = 0x00;
                        (*tx_frame).dados[1] = 0x02; //protocolo_tx(erro de CRC 0x02)
                        return (false);
                    } else {
                        //protocolo_tx(0xA3, comando, 0x00, crc); ok
                        (*tx_frame).stx = RESPOSTA;
                        (*tx_frame).comando = (*rx_frame).comando;
                        (*tx_frame).formato = (*rx_frame).formato;
                        (*tx_frame).numero = 0x00;
                        return (true);
                    }
                } else {
                    quadro_indice = caracter;
                }
                break;

            default:
                break;
        }
    }
    return (false);
}

void protocolo_tx(frame *tx_frame) {
    char stream[NUMERO_BYTES_MENSAGEM],
            stream_indice = 0; //indice do fluxo de dados;
    char i;

    comando* lista_comandos;
    stream[stream_indice] = (*tx_frame).stx;
    stream_indice++;
    stream[stream_indice] = (*tx_frame).comando;
    stream_indice++;

    if (RESPOSTA == (*tx_frame).stx) {
        for (i = 0; i < sizeof (lista_comandos_entrada); i++) {
            if ((*tx_frame).comando == lista_comandos_entrada[i].cmd) {
                lista_comandos = &lista_comandos_entrada[i];
                break;
            }
        }
    }
    stream[stream_indice] = (*tx_frame).formato;
    stream_indice++;

    stream[stream_indice] = (*tx_frame).numero;
    stream_indice++;
    for (i = 0; i < (*tx_frame).numero; i++) {
        stream[stream_indice] = (*tx_frame).dados[i];
        stream_indice++;
    }
    (*tx_frame).crc = generate_16bit_crc(&stream[0], stream_indice, CRC_16);
    stream[stream_indice] = (*tx_frame).crc >> 8; //MAKE8((*tx_frame).crc,1);
    stream_indice++;
    stream[stream_indice] = (*tx_frame).crc & 0xFF; //MAKE8((*tx_frame).crc,0);
    for (i = 0; i <= stream_indice; i++) {
        putchFunc = BT_putc;
        putch(stream[i]);
    }
}

/**
 * função para coletar número da mensagem recebida
 *
 * para números em hexa, rx_frame.formato = 0
 *  size = 1, dados[pos] -> Lê char
 *  size = 2, dados[pos] -> Lê int
 *  size = 4, dados[pos] -> Lê long
 * para número em BCD, rx_frame.formato = 1
 *  size <= 2, dados[pos] -> Lê char (255)
 *  size <= 3, dados[pos] -> Lê int (65535)
 *  size <= 5, dados[pos] -> Lê long (4294967295)
 *
 * @param rx_frame: mensagem de entrada
 * @param destino: onde o número será armazenado
 * @param pos: ponteiro para início do número no buffer da mensagem
 * @param size: número de bytes no buffer de entrada
 */
unsigned long getNumber(frame *rx_frame, char pos, char size) {

    union {
        unsigned long l;
        unsigned char c[4];

        struct {
            unsigned char BCD[5];
        };
    } in;
    in.l = 0;
    in.BCD[4] = 0;
    unsigned long out = 0;
    char mult, contador;

    /*converte de mensagem big-endian para PIC little-endian*/
    if (size == 5) {
        in.BCD[4] = (*rx_frame).dados[pos++];
        size--;
    }
    for (contador = size; contador != 0; --contador) {
        in.c[contador - 1] = (*rx_frame).dados[pos++];
    }
    if ((*rx_frame).formato) {
        /* buffer em BCD */
        for (contador = 10; contador != 0; contador--) {
            mult = (in.BCD[4] & 0xF0) >> 4;
            out = (out << 3)+(out << 1); /* mult10 3 vezes mais rápido*/
            out += mult;

            in.BCD[4] = (in.BCD[4] << 4) + (in.BCD[3] >> 4);
            in.l <<= 4;
        }
        return out;
    } else {
        /* buffer em HEXA */
        return in.l;
    }
}

/**
 *  função para converter número para a mensagem de saída
 *
 * para números em hexa, tx_frame.formato = 0
 *  size = 1, origem char -> dados[pos]
 *  size = 2, origem int -> dados[pos]
 *  size = 4, origem long -> dados[pos]
 * para número em BCD, tx_frame.formato = 1
 *  size = 1, origem char -> dados[pos] (255)
 *  size = 2, origem int -> dados[pos] (65535)
 *  size = 4, origem long -> dados[pos] (4294967295)
 *
 * @param tx_frame: mensagem de saida
 * @param origem: valor a transmitir (hex ou BCD) 4bytes
 * @param pos: ponteiro para início do número no buffer da mensagem
 * @param size: número de bytes no buffer de saída
 */
void setNumber(frame *tx_frame, long origem, char pos, char size) {

    union {
        unsigned long l;
        unsigned char c[4];

        struct {
            unsigned char BCD[5];
        };
    } out;

    out.l = origem;
    out.BCD[4] = 0;
    unsigned long in = 0;
    char contador;

    if ((*tx_frame).formato) {
        /* buffer em BCD */
        out.l = 0;
        for (contador = 10; origem != 0; contador--) {
            in = origem / 10;
            out.l >>= 4;
            out.BCD[3] += (out.BCD[4] & 0x0F) << 4;
            out.BCD[4] >>= 4;
            out.BCD[4] += (char) (origem - 10 * in) << 4;
            origem = in;
        }
        //preenche os bytes mais significativos com zero
        for (; contador != 0; contador--) {
            out.l >>= 4;
            out.BCD[3] += (out.BCD[4] & 0x0F) << 4;
            out.BCD[4] >>= 4;
        }
    }
    /*converte de mensagem big-endian para PIC little-endian*/
    if (size == 5) {
        (*tx_frame).dados[pos++] = out.BCD[4];
        size--;
    }
    /* buffer em HEXA */
    for (contador = size; contador != 0; --contador) {
        (*tx_frame).dados[pos++] = out.c[contador - 1];
    }
}
