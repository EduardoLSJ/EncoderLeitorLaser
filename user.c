/******************************************************************************/
/* Files to Include                                                           */
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
#include <stdio.h>

#endif

#define USE_OR_MASKS

#include "user.h"
#include "protocolo_taxi.h"
#include "lcd_870.h"
#include "interrupts.h"
#include <timers.h>
#include <portb.h>
#include <usart.h>

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

/**
 * inicia o hardware do microcontrolador
 */
void ajusta_parametros() {
    di();

    PORTA = 0;
    PORTB = 0;
    PORTC = 0;

    /* define macros para uso com familia F18 */
#ifdef _PIC18_H
    LATA = 0;
    LATB = 0;
    LATC = 0;
#endif

    //desliga portas a/d
    ADCON1 = USART2_ADCON; //disable analog inputs, change it to ttl
    CMCON = USART2_CMCON; //configure comparators for digital input

    TRISA = TRIS_A;
    TRISB = TRIS_B;
    TRISC = TRIS_C;

#ifdef _PIC18_H
    /* CCP2+CCP1 - entrada de interrupção externa - Laser */

    /*timer0 - contador DEC*/
    //timer0 entrada externa, sem prescaler,interrupção habilitada
    //    config = 0b10101111;
    T0CON = T0_16BIT & T0_SOURCE_EXT & T0_EDGE_RISE & T0_PS_1_1;
    /*
        T0PS    :3; 111
        PSA     :1; 1 //T0_PS_1_1
        T0SE    :1; 0 //T0_EDGE_RISE
        T0CS    :1; 1 //T0_SOURCE_EXT
        T08BIT  :1; 0 // The 16 selection bit T0_16BIT
        TMR0ON  :1; 1 //Start Timer0
     */
    WRITETIMER0(0); // Reset Timer0 to 0x0000

    /*Timer1 - contador INC*/
    //timer1: entrada externa, sem prescaler, interrupção habilitada
    //    config = 0b10000011;
    T1CON = T1_16BIT_RW & T1_SOURCE_CCP & T1_PS_1_1 &
            T1_OSC1EN_OFF & T1_SYNC_EXT_ON;
    /*
       TMR1ON    :1; 1 // Start Timer1
       TMR1CS    :1; 1 //T1_SOURCE_EXT T13CKI pin (on the rising edge)
       nT1SYNC   :1; 0 //Synchronize external clock input
       T1OSCEN   :1; 0 //Timer1 oscillator is shut off
       T1CKPS    :2; 00 //1:1 Prescale value
       T1RUN     :1; 0 //Device clock is derived from another source
       RD16      :1; 1 // The 16 selection bit
     */
    WRITETIMER1(0); // Reset Timer1 to 0x0000

    /*timer2 - base de tempo*/
    //timer2: prescaler=4, contador=125,postscaler=10,
    //interrupção habilitada, quando medir velocidade.
    PR2 = 125;
    //    config = 0b01001001;
    T2CON = T2_POST_1_10 & T2_PS_1_4;
    /*
      T2CKPS0     :1; 1 //Prescaler is 4
      T2CKPS1     :1; 0
      TMR2ON      :1; 1 //Timer2 is on
      T2OUTPS0    :1; 1 //Postscale = 10
      T2OUTPS1    :1; 0
      T2OUTPS2    :1; 0
      T2OUTPS3    :1; 1
     */
    WriteTimer2(0); // Clear Timer2

    /*timer3 - base de tempo, para calculo de intervalo*/
    //timer3: prescaler=4, cada bit representa 800ns,
    // em 80 km/h, W=27913000 pulso/km, 1 bit = .496 pulso
    //interrupção habilitada, quando medir velocidade.
    //    config = 0b10101001;
    T3CON = T3_16BIT_RW & T1_CCP1_T3_CCP2 & T3_PS_1_4 &
            T3_SYNC_EXT_ON & T3_SOURCE_INT;
    /*
     TMR3ON    :1; 1 //Enables Timer3
     TMR3CS    :1; 0 //Internal clock (FOSC/4)
     nT3SYNC   :1; 0 //Synchronize external clock input
     T3CCP1    :1; 1 //Timer1 source for CCP1
     T3CKPS    :2; 10 //1:4 Prescale value
     T3CCP2    :1; 0 //Timer3 source for CCP2 ;
     RD16      :1; 1 //read/write of Timer3 in one 16-bit operation
     */
    WRITETIMER3(0);

    /*CCP2 captura da entrada da barreira otica*/
    /*captura da entrada RC1*/
    /*captura do timer 3 (tempo)*/
    /*captura na borda de subida*/
    //    config = 0b00000101;
    CCP2CON = CAP_EVERY_RISE_EDGE;

    /*CCP1 captura do tempo ao passar na barreira otica*/
    /*captura do timer1 (INC)*/
    /*captura da entrada RC2*/
    /*captura na borda de subida*/
    //    config = 0b00000101;
    CCP1CON = CAP_EVERY_RISE_EDGE;

    /* controle de interrupções */
    INTCON = 0b01100000;
    /*
      RBIF       :1; 0
      INT0IF     :1; 0 INTCONbits.INT0IF = 0;
      TMR0IF     :1; 0 INTCONbits.TMR0IF = 0; // Clear Timer0 overflow flag
      RBIE       :1; 0; //disable PORTB Interrupt on change
      INT0IE     :1; 0; //disable interrupt
      TMR0IE     :1; 1; // Enable Timer0 overflow interrupt
      PEIE_GIEL  :1; 1
      GIE_GIEH   :1; 0
     */
    //PORTB_PULLUPS_ON
    INTCON2 = 0b01110000;
    /*
      RBIP     :1; 0; //low priority rb change
               :1; 0
      TMR0IP   :1; 0; //prioridade baixa tmr0
               :1; 0
      INTEDG2  :1; 1
      INTEDG1  :1; 1
      INTEDG0  :1; 1; //select rising edge
      nRBPU    :1; 0; //enable pullups
     */
    INTCON3 = 0b00000000;
    /*
      INT1IF   :1; 0
      INT2IF   :1; 0
               :1; 0
      INT1IE   :1; 0
      INT2IE   :1; 0
               :1; 0
      INT1IP   :1; 0
      INT2IP   :1; 0
     */
    /* limpa flag de interrupções */
    PIR1 = 0b00000000;
    /*
      TMR1IF   :1; 0 // Clear Timer1 overflow flag
      TMR2IF   :1; 0 // Clear Timer2 overflow flag
      CCP1IF   :1; 0
      SSPIF    :1; 0
      TXIF     :1; 0
      RCIF     :1; 0
      ADIF     :1; 0
      SPPIF    :1; 0
     */
    PIR2 = 0b00000000;
    /*
      CCP2IF   :1; 0
      TMR3IF   :1; 0 // Clear Timer3 overflow flag
      HLVDIF   :1; 0
      BCLIF    :1; 0
      EEIF     :1; 0
      USBIF    :1; 0
      CMIF     :1; 0
      OSCFIF   :1; 0
     */

    /*prioridade de interrupções*/
    IPEN = 1;
    IPR1 = 0b00000000;
    /*
      TMR1IP  :1; 0 //prioridade baixa tmr1
      TMR2IP  :1; 0 //prioridade baixa tmr2
      CCP1IP  :1; 0 //prioridade baixa para CCP1
      SSPIP   :1; 0
      TXIP    :1; 0
      RCIP    :1; 0
      ADIP    :1; 0
     */
    IPR2 = 0b00000001;
    /*
      CCP2IP   :1; 1; //prioridade alta para CCP2
      TMR3IP   :1; 0; //prioridade baixa tmr3
      HLVDIP   :1; 0
      BCLIP    :1; 0
      EEIP     :1; 0
      USBIP    :1; 0
      CMIP     :1; 0
      OSCFIP   :1; 0
     */

    /* habilita interrupções */
    PIE1 = 0b00100011;
    /*
     TMR1IE   :1; 1; // Enable Timer1 overflow interrupt
     TMR2IE   :1; 1; // Enable Timer2 overflow interrupt
     CCP1IE   :1; 0;
     SSPIE    :1; 0
     TXIE     :1; 0
     RCIE     :1; 1 // Enables the EUSART receive interrupt
     ADIE     :1; 0
     SSPIE    :1; 0
     */
    PIE2 = 0b00000011;
    /*
      CCP2IE    :1; 1 //Enable CCP2 Interrupt
      TMR3IE    :1; 1 // Enable Timer3 overflow interrupt
      HLVDIE    :1; 0
      BCLIE     :1; 0
      EEIE      :1; 0
      USBIE     :1; 0
      CMIE      :1; 0
      OSCFIE    :1; 0
     */



#elif defined _PIC_H_
    /* INT0 - entrada de interrupção externa - PULSE_IN*/
    //timer0 entrada externa, sem prescaler,interrupção habilitada
    OPTION_REG = 0b01101111; /*RTCC_EXT_L_TO_H | RTCC_DIV_1*/

    INTCON2bits.NOT_RBPU = 0;
    INTCON2bits.INTEDG0 = 1;
    INTCONbits.INT0IE = 1;
    INTCONbits.TMR0IE = 1;
    INTCONbits.PEIE = 1;
    config = RISING_EDGE_INT | PORTB_PULLUPS_ON;

    /*timer0 - decremento*/
    //    OpenRB0INT(config);
    /*T0CON - 10101000
     * T0 on, 16bits, entrada T0CKI, rise inc, ps=1, */

    /*Timer1 - incremento*/
    //timer1: entrada externa, sem prescaler, interrupção habilitada
    /*desliga CCP1*/
    CCP1CON = 0x00;
    T1CON = 0b00001011; /*T1_EXTERNAL_SYNC | T1_DIV_BY_1*/

    /*timer2 - base de tempo*/
    //timer2: prescaler, contador=,postscaler=, interrupção habilitada, quando medir velocidade.
    CCP2CON = 0; /*CCP_OFF*/
    T2CON = 0b01001101; /*T2_DIV_BY_4, 125, 10*/
    PR2 = 125;

    TMR0 = 0;
    TMR1 = 0;
    TMR2 = 0;
#endif

    CLRWDT();

    lcd_init();
    vInitU1();
    inicia_bluetooth();

    /*reinicia contadores*/
    Encoder100ms = 0;
    EncInc100ms.inc = 0;
    EncDec100ms.dec = 0;
    EncoderAbsoluto = 0;
    EncIncAbsol.inc = 0;
    EncDecAbsol.dec = 0;
    barreira_pont = 0;
    trecho_cont = 0;

    BuffRxPtr = 0;
    BuffLidoPtr = 0;

    /*recuperar o valor do último ajuste*/
    eeprom_read_object(EEPROM_k, &k, sizeof k);

    ei();
}

/**
 * Rotina para selecionar a função de saída de comunicação
 *
 * @param c: função de saída (BT ou LCD)
 */
void putch(char c) {
    putchFunc(c); // call whatever function we've set the pointer to point to
}

/**
 * função de saída pela porta serial BT
 * é utilizado espera pelo esvaziamento do buffer de saída.
 *
 * @param data: carater a transmitir
 */
void BT_putc(char data) {
    while (!TXIF)
        continue;
    TXREG = data;
}

/**
 * inicializa a porta de comunicação serial BT
 */
void vInitU1(void) {
#ifdef _PIC18_H
    CloseUSART();

    TXSTA = 0b00100110; //brgh=1
    RCSTA = 0b00000000;
    PIE1bits.TXIE = 0;
    PIE1bits.RCIE = 1; // Enable RC interrupt

    //    TXSTAbits.TXEN = 1; // Enable transmitter
    //    RCSTAbits.SPEN = 1; // Enable receiver
    BAUDCON = 0b00001010;
    //    config = USART_TX_INT_OFF & USART_RX_INT_OFF & USART_BRGH_HIGH
    //            & USART_CONT_RX & USART_EIGHT_BIT & USART_ASYNCH_MODE & USART_ADDEN_OFF;
    //    config = 0b01111111 & 0b10111111 & 0b11111111
    //            & 0b11111111 & 0b11111101 & 0b11111110 & 0b11011111;
    //    OpenUSART(config, USART_BRG);



    //    TXSTA = 0b00100000;
    //    RCSTA = 0b10010000;
    //    BAUDCON = 0b00001000;
    //    config = BAUD_8_BIT_RATE | BAUD_AUTO_OFF;
    //    config = 0b11110111 & 0b11111110;
    //    baudUSART(config);

#elif defined _PIC_H_
    //Transmit Status and Control (TXSTA)
    TXSTA = 0b00100110;

    //Receive Status and Control (RCSTA)
    RCSTA = 0b00000000;

    //Baud Rate Control (BAUDCON)
    BAUDCON = 0b00001000;
#else
#warning Não definido procesador
#endif
    SPBRG = USART_BRG;
    SPBRGH = USART_BRG >> 8;

    RCSTAbits.SPEN = 1; // Enable UART1 module
    RCSTAbits.CREN = 1;
}

void inicia_bluetooth() {
    BT_Reset = 0;
    CLRWDT();
    __delaywdt_ms(5);
    //    __delaywdt_ms(5);
    BT_Reset = 1;
    CLRWDT();
    __delaywdt_ms(5);
    //    __delaywdt_ms(5);

    //MODULO LINVOR
#ifdef BT_LINVOR
    //9600
    putchFunc = BT_putc;
    printf("AT+PIN3445");
    CLRWDT();
    __delaywdt_ms(5);
    //    __delaywdt_ms(5);
    printf("AT+NAMEbancorolos");

#elif defined BT_BOLUTEK
    AT + NAMEBanco de Rolos
    AT + NAME
    AT + PIN3445
    AT + PIN
    AT + SENM = 3, 1 //Sec_mode3_link,hci_enc_mode_pt_to_pt
            AT + SENM
            AT + RESET
#elif defined BT_HC05
    AT + NAME = BANCODEROLOS
            AT + NAME ?
            AT + PSWD = 3445
            AT + PSWD ?
            AT + SENM = 3, 1
            AT + SENM ?
            AT + RESET
#endif


            //método 1, com velocidade atual
            //	Step 1: Input low level to KEY.
            //	Step 2: Supply power to the module.
            //	Step 3: Input high level to the KEY
            //	The baud rate is as same as the communication time, such as 9600 etc.

            /*	output_bit(BT_Key, 0);
                    output_bit(BT_Reset, 0);
                    printf(lcd_putc,"\fReset");
                    delay_ms(1000);
                    output_bit(BT_Reset, 1);
                    delay_ms(50);
                    output_bit(BT_Key, 1); //Entra no modo AT

                    printf(lcd_putc,"\fMetodo 1");
                    delay_ms(1000);
             */

            //	fprintf(BT,"AT\r\n");
            //	fprintf(BT,"AT+RESET\r\n");
            //	fprintf(BT,"AT+PSWD=3445\r\n");
            //	fprintf(BT,"AT");
            //	string[0]=fgetc(BT);
            //	string[1]=fgetc(BT);
            //	string[2]=0;
            //	printf(lcd_putc,"\fSENHA=%s",string);
            //	do
            //	{
            //		c=aguarda_botao();
            //	}while(!c);

            //método 2, com velocidade 38400
            //	Step 1: Connect PIN34 to the power supply PIN.
            //	Step 2: Supply power to module
            //		(the PIN34 is also supplied with high level since the PIN34 is connected with
            //		power supply PIN). Then the module will enter to AT module.
            //		But at this time, the baud rate is 38400.
            //		In this way, user should change the baud rate at the AT mode, if they
            //		forget the communication baud rate.


            //	fprintf(BT,"AT+NAME=EDU=BANCO DE ROLOS1\r\n");
            //	fprintf(BT,"AT");
            //	string[0]=fgetc(BT);
            //	string[1]=fgetc(BT);
            //	string[2]=0;
            //	fgets(string,BT);
            //	strcpy(resposta,"OK");
            //	printf(lcd_putc,"\fLeitura=%s %s",resposta,string);

            //	if(strcmp(string, resposta))
            //   		printf(lcd_putc,"OK");
            //	fprintf(BT,"AT+PSWD=3A45\r\n");
            //	output_bit(BT_Key, 0); //sai do modo AT
            //	fprintf(BT,"AT+RESET\r\n");

            //modo de operação normal
            //            BT_Key = 0;
            BT_Reset = 1;
}

/**************************************************************************
 *		Rotina para multiplicar por 10
 *
 *	int32 mult_with10(int32 num)
 *
 *	uso: multiplicação utilizando rotação de bytes, ocupando menos recursos
 *
 *	argumentos:
 *		num: valor a ser multiplicado
 *
 *	retorna:
 *		num*10;
 *
 **************************************************************************/
long mult_with10(long num) {
    return ( (num << 1) + (num << 3));
}

/**************************************************************************
 *		Rotina para coletar string numérica
 *
 *	void get_string(char* s, unsigned char max)
 *
 *	uso: captura caracteres e armazena na string, com finalizador 0.
 *		os caracteres são ecoados no display
 *
 *	argumentos:
 *		s: string que vai receber o valor capturado
 *		max: comprimento maximo, em caracteres.
 *
 *	retorna:
 *		void;
 *
 **************************************************************************/
void get_string(char* s, unsigned char max, char dp) {
    unsigned char pos_cursor; //posicao do cursor (posicao)
    unsigned char max_cursor; //tamanho maximo da string no display (apaga)
    unsigned char max_string; //tamanho da string no registro s
    unsigned char pos_string; //ponteiro da string (point)
    char valor; //valor atual do caracter (valor)
    unsigned char contador = 0;
    char key;

    max_cursor = max;
    max_string = max_cursor - 1;
    //cursor do display piscando
    //lcd_send_byte(0,0x0f);
    //cursor do display
    lcd_send_byte(0, 0x0e);

    //desenha valor inicial no display
    //for(pos_cursor=max_cursor;pos_cursor>=1;pos_cursor--)
    for (pos_cursor = 0; pos_cursor < max_cursor; pos_cursor++) {
        if (pos_cursor == dp)
            lcd_putc('.');
        else
            lcd_putc('0');
    }
    //retorna cursor, com backspace, para lsb
    lcd_putc('\b');

    valor = '0';
    //inicia valor da string s[]
    for (pos_string = 0; pos_string < max_string; pos_string++) {
        //reinicia valor em 00.000
        s[pos_string] = valor;
        //retorna cursor, com backspace, para lsb
        lcd_putc('\b');
    }
    pos_string = 0;
    pos_cursor = 0;

    //entrada da string.
    //aguarda B_Zero=1 para rolar número(0-9),
    //aguarda B_Zero=2 para rolar automaticamente, aguarda 50ms entre dígitos
    //B_Func=2 para trocar de caracter, retornando ao início
    //B_Func=4 para aceitar string
    do {
        //aguarda tecla ser pressionada
        do {
            key = aguarda_botao();
        } while (!key);

        //B_Zero==1 incrementa valor do caracter OU
        //B_Zero==3 incrementa automaticamente
        if (key == 1 || key == 3) {
            valor++;
            if (valor > '9')
                valor = '0';
            //incrementa valor
            s[pos_string] = valor;
            //ecoa digito no display
            lcd_putc(valor);
            //retorna cursor
            lcd_putc('\b');
            if (key == 3) {
                for (contador = 0; contador <= 15; contador++)
                    CLRWDT();
                __delaywdt_ms(15);
                //                        __delaywdt_ms(15);
                tick = fator_debounce;
            }
        }
        //aceita valor numerico na string
        //B_Func==2
        if (key == 2) {
            //            if (!pos_cursor) {
            //                if (valor == '0')
            //                    //inibe zero inicial
            //                    lcd_putc(' ');
            //                else
            //                    //ecoa digito no display
            //                    lcd_putc(valor);
            //            } else {
            //ecoa digito no display
            lcd_putc(valor);
            if (pos_cursor == (dp - 1)) {//1
                //salta ponto decimal
                lcd_putc('.');
                pos_cursor++;
            }
            //            }
            pos_cursor++;
            pos_string++;
            if (pos_cursor >= max_cursor) {
                //retorna à posição inicial da string no display
                for (pos_cursor; pos_cursor >= 1; pos_cursor--) {
                    //retorna cursor
                    lcd_putc('\b');
                }
                pos_string = 0;
                pos_cursor = 0;
            }
            valor = s[pos_string];
        }
    } while (key != 4);
    //desativa cursor antes de sair
    lcd_send_byte(0, 0x0c);
    s[max + 1] = 0;
}

/**************************************************************************
 *		Rotina para converter string em int32
 *
 *	long atol(char *s)
 *
 *	uso: captura caracteres e armazena na string, com finalizador 0.
 *
 *	argumentos:
 *		s: string numerica a ser convertida
 *
 *	retorna:
 *		result: valor em 16bits
 *
 **************************************************************************/
long atol(char *s) {

    long result;
    int index;
    char c;

    index = 0;
    result = 0;

    c = s[index++];

    // base 10
    while (c >= '0' && c <= '9') {
        result = mult_with10(result) + (c - '0');
        c = s[index++];
    }
    return (result);
}

/**************************************************************************
 *		Rotina para coletar valor de 16 bits
 *
 *	long get_long()
 *
 *	uso: captura caracteres e armazena na string, com finalizador 0.
 *		os caracteres são ecoados no display
 *
 *	argumentos:
 *
 *
 *	retorna:
 *		l: valor em 16bits
 *
 **************************************************************************/
long get_long(char nchar, char dp) {
    char s[12];
    long l;

    get_string(s, nchar, dp); //6
    l = atol(s);
    return (l);
}

/**
 * atualiza valor apresentado no display
 * rotina chamada a cada 3*100 ms.
 */
void atualizaDistVel() {
    static char periodo = 1;

    /*aguarda 300 ms para atualizar*/
    if (!periodo) {
        /*calcula numero de pulsos e distância percorrida*/
        /*milimetros*/
        /*TODO verificar*/
        _fpover = false;
        _fpunder = false;
        distancia = ((float) Encoder100ms / (float) k) * 1E6;
//        if(_fpover || _fpunder){
//            /*TODO verificar se operacao é necessaria*/
//            distancia = 0;
//        }
        /*vel = km/h, onde:
         * distancia (mm) e
         * dt=0,1s
         */
        //    vel = dVdt * 0.036000 / k;
        /* vel (km/h) = dVdt(pulsos/100ms) * */
        velocidade = ((float) dVdt / k)* 36E3;

        //apresentar resultado no LCD
        putchFunc = lcd_putc;
        /*TODO voltar para lcd_putc*/
        //                putchFunc = BT_putc;
        printf("\f%11.2fmm %2d", distancia, trecho_cont);

        printf("\n%4.1fkm/h%8ld", velocidade, Encoder100ms);
        periodo = 2;
    }
    periodo--;

    flags.atualiza = 0;
}

/**
 * LÊ o valor do buffer de recepção serial
 * 
 * @return valor disponível no buffer
 */
char lerBuffer(void) {
    BuffLidoPtr++;
    if (BuffLidoPtr >= 64)
        BuffLidoPtr = 0;
    return buffer[BuffLidoPtr];
}

/**
 *   
 *     rotina de leitura dos botões.
 *     se pulso curto, debounce de 20ms.
 *     se pulso longo, debounce de 500ms.
 *     retorna:
 * 
 * @return
 *     0 - teclado solto
 *     1 - tecla 1 curto
 *     2 - tecla 2 curto
 *     3 - tecla 1 longo
 *     4 - tecla 2 longo
 */
char aguarda_botao() {
    static bit kbd_down;
    static char last_key;
    char estado_chave;

    estado_chave = 0;
    if (tick >= fator_debounce) {
        if (kbd_down) {
            //tecla estava pressionada
            //testa se foi liberada
            if ((kbd & (ALL_BOTTONS)) == (ALL_BOTTONS)) {
                //tecla foi solta
                kbd_down = false;
                estado_chave = last_key;
                if (last_key > 2)
                    //inibe novo acionamento quando soltar tecla
                    estado_chave = 0;
                last_key = 0;
                decimo = 0;
            }
            //verificar se houve 1/2 segundo
            if (decimo >= fator_repeat) {
                //adiciona 2 para indicar travamento
                if (last_key) {
                    estado_chave = last_key + 2;
                    last_key = 0; //inibe novo acionamento quando soltar tecla
                }
                decimo = fator_repeat; //mantém contador
            }
        } else {
            if ((kbd & (ALL_BOTTONS)) != (ALL_BOTTONS)) {
                //tecla pressionada, testa qual delas
                if ((kbd & Bot0) == 0)
                    last_key = 1;
                else if ((kbd & Bot1) == 0)
                    last_key = 2;
                kbd_down = true;
                decimo = 0;
            }
        }
        tick = 0;
    }
    return (estado_chave);
}

/**
 * rotina de leitura da eeprom
 * o valor da eeprom é copiado na variavel de destino, na quantidade
 * de bytes da variável.
 * 
 * @param ee_addr: endereço inicial da eeprom
 * @param obj_p: endereço da variavel destino
 * @param obj_size: tamanho da variavel destino
 */
void eeprom_read_object(unsigned int ee_addr, void *obj_p, size_t obj_size) {
    unsigned char *p = obj_p;

    while (obj_size--) {
        *p++ = eeprom_read(ee_addr++);
    }
}

/**
 * rotina de escrita da eeprom
 * o bytes da variável de origem são escritos na eeprom, para resgate posterior
 *
 * @param ee_addr: posição inicial da eeprom
 * @param obj_p: variável de origem
 * @param obj_size: tamanho da variável
 */
void eeprom_write_object(unsigned int ee_addr, void *obj_p, size_t obj_size) {
    unsigned char *p = obj_p;

    while (obj_size--) {
        eeprom_write(ee_addr++, *p++);
    }
}

/**
 *  Reinicia os ponteiros e registros de coleta
 * todos os dados serão apagados.
 */
void reinicia_coleta(void) {
    di();
    barreira_pont = 0;
    trecho_cont = 0;
    tempoAbsol.tempo32 = 0;
    EncDecAbsol.dec = 0;
    EncIncAbsol.inc = 0;
    EncoderAbsoluto = 0;
    EncDec100ms.dec = 0;
    EncInc100ms.inc = 0;
    Encoder100ms = 0;

    dVdtMax = 0; //reinicia armazenamento
    WRITETIMER0(0);
    WRITETIMER1(0); //aguarda 1 falling edge para iniciar contagem
    WriteTimer2(0);
    WRITETIMER3(0);

    TMR0IF = 0;
    PIR1 = 0;
    PIR2 = 0;

    ei();
}
//unsigned char READTMR0(void)
//{
//  union Timers timer;
//
//  timer.bt[0] = TMR0;  // Copy Timer0 low byte into union
//  timer.bt[1] = 0;//TMR0H;  // Copy Timer0 high byte into union
//
//  return (TMR0);
//  //return (timer.lt);    // Return the int
//}

//unsigned int READTMR1(void)
//{
//  union Timers timer;
///*TODO ler Examples 12-2 and 12-3 in the PIC Mid-Range MCU
//Family Reference Manual (DS33023)*/
//  timer.bt[0] = TMR1L;    // Read Lower byte
//  timer.bt[1] = TMR1H;    // Read upper byte
//
//  return (timer.lt);      // Return the 16-bit value
//}

//void WRITETMR0(unsigned char timer0)
//{
//  union Timers timer;
//
//  timer.lt = timer0;    // Copy timer value into union
//
////  TMR0H = timer.bt[1];  // Write high byte to Timer0
//  TMR0 = timer.bt[0];  // Write low byte to Timer0
//}

//void WRITETMR1(unsigned int timer1)
//{
//  union Timers timer;
//
//  timer.lt = timer1;    // Save the 16-bit value in local
//
//  TMR1H = timer.bt[1];  // Write high byte to Timer1 High byte
//  TMR1L = timer.bt[0];  // Write low byte to Timer1 Low byte
//}

//void WRITETMR2(unsigned char timer2)
//{
//  TMR2 = timer2;    // Write byte to Timer2
//}

//char DataRdyUSART(void)
//{
//  if(PIR1bits.RCIF)  // If RCIF is set
//    return 1;  // Data is available, return TRUE
//  return 0;  // Data not available, return FALSE
//}
