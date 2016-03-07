/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/
#define USE_OR_MASKS

#if defined(__XC)
#include <xc.h>        /* XC8 General Include File */
#elif defined(HI_TECH_C)
#include <htc.h>       /* HiTech General Include File */
#elif defined(__18CXX)
#include <p18cxxx.h>   /* C18 General Include File */
#endif

#if defined(__XC) || defined(HI_TECH_C)

#include <stdint.h>        /* For uint8_t definition */
#include <stdbool.h>       /* For true/false definition */
#include <stdio.h>
#endif

#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp */
#include "protocolo_taxi.h"
#include "lcd_870.h"

#define BOT_MENU 4
#define BOT_PROX 2
#define BOT_SEL 1
#define BOT_DIRLONG 3
#define BOT_DIR 1
#define BOT_ESQ 2

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

/*****************************DICOF / INMETRO *******************************
 * INMETRO/DIMEL/SINST
 * Programa para:
 *          1)Calibrar Banco de Rolos para Cronotacógrafos
 *             	ler pulsos AB em quadratura do encoder
 *             	usar o pulso Z para iniciar/terminar uma leitura
 *             	converter a leitura de pulsos para geração de frequencias (velocidade)
 *				memoriza a contagem de pulsos.
 *
 *           Registro de temperatura e Umidade Relativa
 *           comunicação serial BT para envio de relatório
 *
 **************************************************************************
 Data: 28/6/2012
 Autor:
          Eduardo Lopes - eljunior@inmetro.gov.br
 **************************************************************************
Revisões:
28/06/2012
        inicio do projeto
17/09/2012
        adicionado modulo bletooth.
 * rotinas de atualiza display e aguarda botoes revista
        adicionado rotina para inicializacao do modulo BT
05/10/2012
        inicio do desenvolvimento do protocolo, segundo orientação do RTM mototaximetro

 **************************************************************************
    - Microcontrolador PIC 18F8720 -

    Compilador: CCS C Compiler Version
    IDE: MPLAB Version 8.60
    Gravador: PICkit3
 **************************************************************************
        condições iniciais:

        As entradas de pulsos do encoder são: INCremento e DECremento
        Os pulsos são tratados com contadores por hardware.
        Filtros analógicos e digitais foram aplicados para minimizar interferências.
        A contagem é realizada com timers internos ao microcontrolador
        O tratamento de interrupção por tempo realiza a leitura dos timers e calcula
          a velocidade instantânea.
        Em caso de interrupção do pulso Z, a contagem é reiniciada/interrompida

 *****************************************************************************
hardware:

RA0 - D4 LCD
RA1 - D5 LCD
RA2 - D6 LCD
RA3 - D7 LCD
RA4 - DEC
RA5 - Pulso
RB0 - Sync
RB4	- TX BT auxiliar
RB5 - RX BT auxiliar
RC0 - INC
RC2 - /RESET BLUETOOTH
RC3 - KEY BLUETOOTH
RC4 - EN LCD
RC5 - RS LCD
RC6 - TX BLUETOOTH
RC7 - RX BLUETOOTH


 *****************************************************************************/

void main(void) {
    // inicializa a variavel de controle de estado
    static char estado = PRINCIPAL;
    char key_char;
    char ponteiro;
    static bit rx_bt = false;
    static bit desenhar = false;
    long LTrecho;
    signed long pulsos;
    unsigned long tempo;

    /* Configure the oscillator for the device */
    ConfigureOscillator();

    /* Initialize I/O and Peripherals for application */
    ajusta_parametros();
    /*define saída principal das mensagens*/
    putchFunc = lcd_putc;

    /**************************************************************************
     *	1) Loop principal
     *		Apresentar no display
     *			Distancia percorrida em mm
     *                  numero de trechos de 100 m
     *			Velocidade em km/h
     *			Numero de pulsos coletados desde o último "Zero" (EncoderContador).
     *		Aguardar Teclado
     *			'1' - Mostra valor da coleta de trecho NN+
     *			'2' - Mostra valor da coleta de trecho NN-
     *                  '3' - Passa para tela de MENU <sair>
     *          Aguarda serial
     *                  A2400000 - Zera distância registrada
     *                  A2410000 - Responde com numero de trechos de 100 m
     *                  A24200xx - Responde com distancia+tempo do trecho solicitado
     *
     *  2) Menu <sair>
     *      Aguardar Teclado
     *          '1' - Menu <Zera Distância> ++
     *          '2' - Menu <Novo k: manual> --
     *          '3' - retorna para loop principal
     *  3) Menu <Zera Distância>
     *      Aguardar Teclado
     *          '1' - Menu <Limpa Coleta 100 m> ++
     *          '2' - Menu <sair> --
     *          '3' -
     *  4) Menu <Limpa Coleta 100 m>
     *      Aguardar Teclado
     *          '1' - Menu <apresenta k> ++
     *          '2' - Menu <Zera Distância> --
     *  5) Menu <apresenta k>
     *      Aguardar Teclado
     *          '1' - Menu <Novo k: laser> ++
     *          '2' - Menu <Limpa Coleta 100 m> --
     *  6) Menu <Novo k: laser>
     *      Aguardar Teclado
     *          '1' - Menu <Novo k: manual> ++
     *          '2' - Menu <apresenta k> --
     *  7) Menu <Novo k: manual>
     *      Aguardar Teclado
     *          '1' - Menu <sair> ++
     *          '2' - Menu <Novo k: laser> --
     *
     *	2) Durante receber Medida Materializada
     *		Exporta 'M' para indicar comando aceito
     *		Aguarda "P_________\r"  com o valor da medida materializada em mm
     *		Timeout-global de 5s
     *
     *	3) Durante Ajuste de "k"
     *		Apresenta no display "Aguarda disparo"
     *		Exporta "K_________\r\n"
     *		Aguarda pulso do detector de pista, teclado ou serial
     *			'2'	- Retorna para 1)
     *			Pulso - Inicia contagem quando detectado pulso
     *
     *	4) Contagem de pulsos da pista de Ajuste
     *		Apresentar no display
     *			"Ajuste..."
     *		Exportar
     *			Exporta "K_________\r\n"
     *			Número de pulsos coletados;
     *		Aguarda pulso do detector de pista, teclado ou serial
     *			'2' - Retorna para 1)
     *			pulso - termina contagem e armazena totalizador (EncoderContador)
     *
     *	5) Solicita valor da distancia de referencia, percorrida
     *		Apresentar no display e exportar
     *			total de pulsos coletados
     *			Exporta "D_________\r\n", para sinalizar fim da coleta
     *		Aguarda teclado ou serial
     *			Recebe "P_________\r" com valor da distancia materializada
     *			'1'	- OK recebido pelo teclado para informar distancia
     *		Solicita do usuario a distancia percorrida
     *			solicitar do usuário a distância percorrida, pelo teclado.
     *		Retorna para 1)
     *
     *	Durante ensaio apresentar no display
     *		Velocidade instantanea, a partir de pulsos por intervalo
     *		distância percorrida.
     *		ambos os valores dependem do valor de k determinado no ajuste
     *
     **************************************************************************/

    while (true) {
        CLRWDT();
        key_char = aguarda_botao();

        /*trata comando Serial*/
        if (BuffRxPtr != BuffLidoPtr) {
            rx_bt = protocolo_rx(&rx_frame, &tx_frame);
            /*se não estiver na tela principal, responde ocupado */
            if (estado != PRINCIPAL ||
                    rx_frame.stx != INSTRUCAO) {
                tx_frame.stx = ERRO;
                tx_frame.comando = rx_frame.comando;
                tx_frame.formato = 0x00;
                tx_frame.numero = 0x02;
                tx_frame.dados[0] = 0x00;
                tx_frame.dados[1] = 0x07; //protocolo_tx(erro: ocupado 0x07)
                rx_bt = false;
            }
            /* em caso de erro, responda de acordo */
            if (!rx_bt)
                protocolo_tx(&tx_frame);
            else
                key_char = 0; /* serial tem prioridade */
        }

        switch (estado) {

            case PRINCIPAL:
                /* calcula distancia em mm */
                /* calcula velocidade em km/h */
                /* apresenta distancia, n.secoes, velocidade e numero de pulsos*/
                if (flags.atualiza) {
                    atualizaDistVel();
                }
                /* Monitorar entrada Serial */
                if (rx_bt) {
                    rx_bt = false;
                    switch (rx_frame.comando) {
                        case ZERAR_COLETA:
                            /*reinicia contador de coleta de medidas */
                            reinicia_coleta();
                            ponteiro = 0;
                            break;
                        case QUANT:
                            /*informa o número atual de coletas*/
                            tx_frame.numero = 0x01;
                            tx_frame.dados[0] = trecho_cont;
                            break;
                        case LEITURA_100M:
                            /*se ultrapassar valor máximo, responde erro*/
                            ponteiro = getNumber(&rx_frame, 0, 1);
                            tx_frame.dados[0] = ponteiro;
                            if (ponteiro > trecho_cont ||
                                    !ponteiro) {
                                tx_frame.stx = ERRO;
                                tx_frame.formato = FORMAT_HEX;
                                tx_frame.numero = 0x02;
                                tx_frame.dados[0] = 0x00;
                                tx_frame.dados[1] = 0x06; //protocolo_tx(erro de valor inválido 0x06)
                                ponteiro = 0;
                            } else {
                                //exporta pelo BT, valor pedido
                                tx_frame.stx = RESPOSTA;
                                tx_frame.formato = FORMAT_HEX;
                                tx_frame.numero = 0x09;

                                pulsos = barreira[ponteiro].pulsos - barreira[ponteiro - 1].pulsos;
                                tempo = ((barreira[ponteiro].tempo - barreira[ponteiro - 1].tempo) / 10)* 8 / 1000;
                                setNumber(&tx_frame,
                                        pulsos,
                                        sizeof (ponteiro),
                                        sizeof (pulsos));
                                setNumber(&tx_frame,
                                        tempo,
                                        sizeof (ponteiro)
                                        + sizeof (pulsos),
                                        sizeof (tempo));
                                //                                setNumber(&tx_frame,
                                //                                        barreira[ponteiro].pulsos,
                                //                                        sizeof (ponteiro),
                                //                                        sizeof (barreira[ponteiro].pulsos));
                                //                                setNumber(&tx_frame,
                                //                                        barreira[ponteiro].tempo,
                                //                                        sizeof (ponteiro)
                                //                                        + sizeof (barreira[ponteiro].pulsos),
                                //                                        sizeof (barreira[ponteiro].tempo));
                            }
                            break;
                        default:
                            break;
                    }
                    protocolo_tx(&tx_frame);
                } else {
                    /* trata teclado */
                    if (key_char == BOT_ESQ || key_char == BOT_DIR) {
                        desenhar = true;
                        estado = APRES_COLETA;
                    } else if (key_char == BOT_MENU) {
                        /* Menu inicial */
                        estado = MENU_SAIR;
                        desenhar = true;
                    } else
                        estado = PRINCIPAL;
                }
                break;
            case APRES_COLETA:
                if (ponteiro > trecho_cont)
                    ponteiro = 0;
                /* mostra a coleta selecionada*/
                if (desenhar) {
                    if (!ponteiro) {
                        pulsos = 0;
                        tempo = 0;
                    } else {
                        pulsos = barreira[ponteiro].pulsos - barreira[ponteiro - 1].pulsos;
                        tempo = ((barreira[ponteiro].tempo - barreira[ponteiro - 1].tempo) / 10)* 8 / 1000;
                    }
                    putchFunc = lcd_putc;
                    /*TODO voltar para lcd_putc*/
                    //                putchFunc = BT_putc;
                    printf("\f%9ldms n\xDF%2d", tempo, ponteiro); //B2 =  DF = 
                    printf("\n%9ld pulsos", pulsos);
                    //                    printf("\fpulso:t(us) n\xDF%2d", ponteiro); //B2 =  DF = 
                    //                    printf("\n%7ld:%8ld", pulsos, tempo);
                    time_out = 200; //20 s
                    desenhar = false;
                } else
                    /* trata teclado */
                    if (key_char == BOT_DIR) {
                    /* incrementa valor para apresentar */
                    ponteiro++;
                    if (ponteiro > trecho_cont)
                        ponteiro = 0;
                    desenhar = true;
                } else if (key_char == BOT_ESQ) {
                    /* decrementa valor para apresentar */
                    if (!ponteiro)
                        ponteiro = trecho_cont;
                    else
                        ponteiro--;
                    desenhar = true;
                } else if (key_char == BOT_MENU ||
                        key_char == BOT_DIRLONG || !time_out) {
                    /* saida por timeout ou teclas longas*/
                    estado = PRINCIPAL;
                }
                break;
            case MENU_SAIR:
                /* Menu de entrada */
                /* desenha a tela do menu */
                if (desenhar) {
                    putchFunc = lcd_putc;
                    printf("\fMENU: SAIR");
                    printf("\n<prox>     <sel>");
                    desenhar = false;
                    time_out = 200; //20 s
                }
                /* atende teclado */
                if (key_char == BOT_PROX) {
                    estado = MENU_ZERA_DIST;
                    desenhar = true;
                } else if (!time_out || key_char == BOT_SEL) {
                    /* aguarda outra tecla para sair? */
                    estado = PRINCIPAL;
                }
                break;
            case MENU_ZERA_DIST:
                /* reinicia contador de distância */
                /* desenha a tela do menu */
                if (desenhar) {
                    putchFunc = lcd_putc;
                    printf("\fMENU:Reset Odom.");
                    printf("\n<prox>     <sel>");
                    desenhar = false;
                    time_out = 200; //30 s
                }
                /* atende teclado */
                if (key_char == BOT_PROX) {
                    estado = MENU_LIMPA_COLETA;
                    desenhar = true;
                } else if (key_char == BOT_SEL) {
                    di();
                    WRITETIMER0(0);
                    WRITETIMER1(0); //aguarda 1 falling edge para iniciar contagem
                    WRITETIMER3(0);
                    EncIncAbsol.inc = 0;
                    EncDecAbsol.dec = 0;
                    tempoAbsol.tempo32 = 0;
                    EncoderAbsoluto = 0;
                    EncInc100ms.inc = 0; //valor coletado a cada 100ms
                    EncDec100ms.dec = 0;
                    dVdtMax = 0; //reinicia armazenamento
                    TMR0IF = 0;
                    PIR1 = 0;
                    PIR2 = 0;
                    ei();
                    estado = PRINCIPAL;
                } else if (!time_out)
                    /* aguarda outra tecla para sair? */
                    estado = PRINCIPAL;
                break;
            case MENU_LIMPA_COLETA:
                /* limpa os ponteiros das coletas anteriores */
                /* desenha a tela do menu */
                if (desenhar) {
                    putchFunc = lcd_putc;
                    printf("\fMENU:LimpaColeta");
                    printf("\n<prox>     <sel>");
                    desenhar = false;
                    time_out = 200; //20 s
                }
                /* atende teclado */
                if (key_char == BOT_PROX) {
                    estado = MENU_MEDE_DIST;
                    desenhar = true;
                } else if (key_char == BOT_SEL) {
                    /*reinicia contador de coleta de medidas */
                    reinicia_coleta();
                    ponteiro = 0;
                    estado = PRINCIPAL;
                } else if (!time_out)
                    /* aguarda outra tecla para sair? */
                    estado = PRINCIPAL;
                break;
            case MENU_MEDE_DIST:
                /* mede a distância percorrida entre barreiras oticas */
                /* desenha a tela do menu */
                if (desenhar) {
                    putchFunc = lcd_putc;
                    printf("\fMENU:  Mede dist");
                    printf("\n<prox>     <sel>");
                    desenhar = false;
                    time_out = 200; //20 s
                }
                /* atende teclado */
                if (key_char == BOT_PROX) {
                    estado = MENU_APRES_K;
                    desenhar = true;
                } else if (key_char == BOT_SEL) {
                    //passa para medida de distância
                    barreira_pont = 0;
                    trecho_cont = 0;
                    flags.disparo = 0;
                    putchFunc = lcd_putc;
                    printf("\fAguarda disparo");
                    time_out = 3000; //5min
                    estado = MED_DIST;
                } else if (!time_out)
                    /* aguarda outra tecla para sair? */
                    estado = PRINCIPAL;
                break;
            case MENU_APRES_K:
                /* apresenta o valor de K ajustado */
                /* desenha a tela do menu */
                if (desenhar) {
                    putchFunc = lcd_putc;
                    printf("\fMENU:Apresenta K");
                    printf("\n<prox>     <sel>");
                    desenhar = false;
                    time_out = 200; //20 s
                }
                /* atende teclado */
                if (key_char == BOT_PROX) {
                    estado = MENU_AJUSTE_K;
                    desenhar = true;
                } else if (key_char == BOT_SEL) {
                    putchFunc = lcd_putc;
                    //                    printf("\fk=%12.2f", k);
                    printf("\fk=%9ld", k);
                    printf("\nPulsos/km");
                    time_out = 200; //20 s
                    estado = DELAY;
                } else if (!time_out)
                    /* aguarda outra tecla para sair? */
                    estado = PRINCIPAL;
                break;
            case MENU_AJUSTE_K:
                /* ajusta valor de K */
                /* desenha a tela do menu */
                if (desenhar) {
                    putchFunc = lcd_putc;
                    printf("\fMENU:  Ajuste K");
                    printf("\n<prox>     <sel>");
                    desenhar = false;
                    time_out = 200; //20 s
                }
                /* atende teclado */
                if (key_char == BOT_PROX) {
                    estado = MENU_SAIR;
                    desenhar = true;
                } else if (key_char == BOT_SEL) {
                    estado = MENU_K_AUTO_MAN;
                    desenhar = true;
                } else if (!time_out)
                    /* aguarda outra tecla para sair? */
                    estado = PRINCIPAL;
                break;
            case MENU_K_AUTO_MAN:
                /* Seleciona modo de ajuste de valor de K
                 * Manualmente ou com barreira otica */
                /* desenha a tela do menu */
                if (desenhar) {
                    putchFunc = lcd_putc;
                    printf("\fMetodo de Ajuste");
                    printf("\n<manual>  <auto>");
                    desenhar = false;
                    time_out = 200; //20 s
                }
                /* atende teclado */
                if (key_char == BOT_ESQ) {
                    estado = AJUSTE_MAN;
                    desenhar = true;
                } else if (key_char == BOT_DIR) {
                    /*reinicia contador de coleta de medidas */
                    reinicia_coleta();
                    //passa para calibração de distância
                    flags.disparo = 0;
                    putchFunc = lcd_putc;
                    printf("\fAguarda disparo");
                    estado = AJUSTE;
                    time_out = 3000; //5min
                    desenhar = true;
                } else if (!time_out)
                    /* aguarda outra tecla para sair? */
                    estado = PRINCIPAL;
                break;
            case MED_DIST:
                if (!time_out) // debounce timer expired
                {
                    estado = PRINCIPAL;
                }
                if (flags.disparo)//primeiro sensor de piso, inicio do percurso
                {
                    //apresentar no LCD
                    putchFunc = lcd_putc;
                    printf("\fMedindo...");
                    time_count = 0;
                    estado = COLETA_DIST;
                    time_out = 3000;
                }
                if (key_char == BOT_PROX)
                    estado = PRINCIPAL;
                break;
            case COLETA_DIST:
                if (key_char == BOT_PROX)
                    estado = PRINCIPAL;
                if (!time_out) // debounce timer expired
                {
                    estado = PRINCIPAL;
                }
                // tempo para 5m e 80km/h -> 225ms
                // tempo para 5cm e 1km/h -> 185ms
                if (flags.disparo) {
                    //aguarda 220 ms para liberar próxima coleta (debounce)
                    if (time_count >= 220) {
                        //sinaliza fim de coleta
                        LTrecho = barreira[1].pulsos -
                                barreira[0].pulsos;
                        distancia = (1E6 * (float) LTrecho) / k;
                        putchFunc = lcd_putc;
                        printf("\fValor medido:");
                        printf("\n%12.2f mm", distancia);
                        time_out = 3000;
                        //coleta pelo teclado
                        estado = DELAY;
                    } else {
                        barreira_pont = 1;
                    }
                    flags.disparo = false;
                }
                break;
            case AJUSTE_MAN:
                /* Ajuste de valor de K Manualmente */
                /* desenha a tela do menu */
                if (desenhar) {
                    putchFunc = lcd_putc;
                    printf("\fInforme k (p/km)\n");
                    //                    printf("\n0000000000 ");
                    desenhar = false;
                    time_out = 200; //20 s
                }
                k = get_long(11, 10);
                eeprom_write_object(EEPROM_k, &k, sizeof k);
                if (!time_out) // timer expired
                {
                    estado = PRINCIPAL;
                }
                estado = PRINCIPAL;
                break;
            case AJUSTE: // absorb data and place in a buffer
                if (!time_out) // debounce timer expired
                {
                    estado = PRINCIPAL;
                }
                if (flags.disparo)//primeiro sensor de piso, inicio do percurso
                {
                    //apresentar no LCD
                    putchFunc = lcd_putc;
                    printf("\fLendo pulsos...");
                    time_count = 0;
                    estado = COLETA_INI1;
                    time_out = 3000;
                }
                if (key_char == BOT_PROX)
                    estado = PRINCIPAL;
                break;
            case COLETA_INI1:
                if (key_char == BOT_PROX)
                    estado = PRINCIPAL;
                if (!time_out) // debounce timer expired
                {
                    estado = PRINCIPAL;
                }
                if (flags.atualiza) {
                    //rotina depende de Timer2 e acontece a cada 1/10s
                    EncoderAjusteK = Encoder100ms - barreira[0].pulsos + 1;
                    putchFunc = lcd_putc;
                    lcd_gotoxy(1, 2);
                    printf("Encoder%9ld", EncoderAjusteK); //apresenta numero de pulsos
                    flags.atualiza = 0;
                }
                if (time_count >= 220) {
                    if (flags.disparo) {
                        //segundo sensor de piso, termino da pista
                        //sinaliza fim de coleta
                        //                        putchFunc = lcd_putc;
                        //                        printf("\fColeta Encerrada\nPressione botao!");
                        time_out = 3000;
                        //coleta pelo teclado
                        estado = COLETA_TECLADO;
                    }
                } else {
                    //aguarda 220 ms para liberar próxima coleta (debounce)
                    barreira_pont = 1;
                    flags.disparo = false;
                }

                break;
            case COLETA_TECLADO:
                //coleta pelo teclado
                //solicita distância, digito a digito.
                EncoderAjusteK = barreira[1].pulsos - barreira[0].pulsos;
                putchFunc = lcd_putc;
                printf("\fInformeDistancia\n000.000m%7ldp", EncoderAjusteK);
                lcd_gotoxy(1, 2);
                LTrecho = get_long(7, 3); /*resultado em mm */
                estado = PRINCIPAL;
                /*TODO corrigido*/
                k = (long) (((float) EncoderAjusteK / (float) LTrecho)* 1E6);
                eeprom_write_object(EEPROM_k, &k, sizeof k);
                printf("\f");
                break;
            case DELAY:
                /* aguarda outra tecla para sair? */
                if (key_char || !time_out)
                    estado = PRINCIPAL;
                break;
            default:
                estado = PRINCIPAL;
                break;
        }
    }
}

