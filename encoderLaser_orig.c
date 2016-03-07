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
	adicionado modulo bletooth. rotinas de atualiza display e aguarda botoes revista
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
#include <16F876A.h>
//#device *=16  ICD=TRUE //Descomentar Debug

#include <string.h>


/****************************
Define Hardware do equipamento
****************************/
#define TRIS_A 	0b00010000 
#define TRIS_B 	0b00110111
#define TRIS_C 	0b11000001
#define INTCON                    0x0B
#bit INTE = INTCON.4

#define pulso 	PIN_A5
#define sync	PIN_B0
#define B_Zero	PIN_B1
#define B_Func	PIN_B2
#define BT_Key	PIN_C3
#define BT_Reset	PIN_C2

#byte kbd = 6                  // on to port B (at address 6)
#define Bot0 (1 << 1)
#define Bot1 (1 << 2)

#define ALL_BOTTONS (Bot0|Bot1)

#define fator_debounce	20	//20ms para eliminar o repique
#define	fator_repeat	5	//500ms para tecla de repetição

//#define cw		TRUE
//#define ccw		FALSE

#define BT_LINVOR
//#define BT_HC05
//#define BT_BOLUTEK


/****************************
Constantes globais
****************************/
struct{
    int32 pulsos;
    int32 tempo;
}resultado[10];

struct {
	int atualiza : 1;
	int disparo : 1;
	int saida : 1;
}flags;

/*valor para calculo de pulsos de entrada com sentido anti-horário ou decremento*/
//union CaptureDEC {
    signed long dec; /*armazena o valor final para cálculo*/

    struct {
        char lsb; /*armazena leitura do timer0*/
        int mid; /*armazena o contador a cada overflow*/
        char msb; /*armazena overflow para completar 32bits*/
    }teste;

//};

/*valor para calculo de pulsos de entrada com sentido horário ou incremento*/
union CaptureINC {
    signed long inc; /*armazena o valor final para cálculo*/

    struct {
        int lsb; /*armazena leitura do timer1*/
        int msb; /*armazena overflow para completar 32bits*/
    };

};

int buffer[64]; //buffer circular de comunicação serial BT
int i,
	pLido =0, //ponteiro de Leitura do buffer circular BT
	pRecebido =0, //ponteiro de Escrita do buffer circular BT
	Eponteiro, 
	c;			//caracter recebido pelo teclado
long int DeltaL;			
int16 	EncoderINC,
		EncoderDEC;
float 	distancia, 
		k,
		vel; //k do instrumento
signed int32 	EncoderContador,
 				EncoderAjuste,
 				dVdt,
 				dVdtMax;
static char estado,
			tick,
			decimo;
static int estado_chave;
long int time_out,time_count;


enum {PRINCIPAL,
	DIST_MAT,
	AJUSTE,
	MED_DIST,
	COLETA_DIST,
	COLETA_INI1,
	COLETA_TECLADO,
	COLETA_ACK,
	DELAY};

/*****************************
configurações do microcontrolador
*****************************/
#FUSES NOWDT
#FUSES HS                    	//High speed Osc (> 4mhz)
#FUSES NOPROTECT             	//Code not protected from reading
#FUSES NOBROWNOUT          		//Brownout enabled during operation, disabled during SLEEP
#FUSES PUT                   	//Power Up Timer
#FUSES NOCPD                 	//No EE protection
#FUSES NODEBUG               	//No Debug mode for ICD
#FUSES NOLVP                   	//Low Voltage Programming on B3(PIC16) or B5(PIC18)
#FUSES NOWRT                 	//Program memory not write protected

#use delay(clock=20000000,oscillator,restart_wdt)
#use rs232(STREAM=BT, baud=9600, xmit=PIN_C6,rcv=PIN_C7,RESTART_WDT, DISABLE_INTS)

#use fast_io(A)
#use fast_io(B)
#use fast_io(C)


/*************************
driver para o display
*************************/
#include <lcd_870.c>


/*************************
protótipos das funções utilizadas
*************************/
void tratar_sync(void);
void tratar_INC(void);
void tratar_DEC(void);
void tratar_contagem(void);
int aguarda_botao(void);
//void atualiza_LCD(void);
void ajusta_parametros(void);
void inicia_bluetooth(void);
int32 mult_with10(int32 num);
void get_string(char* s, int max);
long atol(char *s);
long get_long(void);
void atualizaDistVel(void);
int lerBuffer(void);
void main(void);

#include <protocolo_taxi.h>

frame rx_frame;
frame tx_frame;

/*************************
rotinas de interrupção
*************************/
#INT_EXT
void tratar_sync()
{
		/*************************
		o pulso de sincronismo externo,
		quando habilitado, inicia e termina
		a contagem dos pulsos.
		*************************/
		int16 soma;
		int sub;
                /*armazenar o valor atual de tempo e de encoder*/
//                CaptureDEC.lsb = get_timer0();
//                CaptureINC.lsb = get_timer1();
                


		if(!FLAGS.disparo)
		{
			//reinicia a contagem de pulsos, na abertura da janela
			set_timer0(0);
			set_timer1(1); //aguarda 1 falling edge para iniciar contagem
			EncoderINC=0;
			EncoderDEC=0;
			FLAGS.disparo=1;
			disable_interrupts(INT_EXT);
		}
		else
		{
			soma = get_timer1();
			sub = get_timer0();
			EncoderAjuste = make32(EncoderINC, soma) - make32(EncoderDEC, sub);
			FLAGS.disparo=0;
			disable_interrupts(INT_EXT);
		}
                resultado[1].pulsos = 0;
                resultado[1].tempo = 1;
}

#INT_TIMER1
void tratar_INC()
{
	/*************************
	A cada overflow do contador, o
	byte superior é incrementado,
	para completar 32 bits
	*************************/
	if(EncoderDEC){
		EncoderDec --;
//                CaptureDEC.mid --;
        }
	else{
		//EncoderINC += 1;
		EncoderINC ++;
//                CaptureINC.msb ++;
        }
}

#INT_TIMER0
void tratar_DEC()
{
	/*************************
	A cada overflow do contador, o
	byte superior é incrementado,
	para completar 32 bits
	*************************/
	if(EncoderINC){
		EncoderINC --;
//                CaptureINC.msb --;
        }
	else{
		//EncoderDEC += 1;
		EncoderDEC ++;
                teste.mid ++;
        }
}


#INT_TIMER2 
void tratar_contagem()
{
	/*
	na interrupção por tempo TIMER2, os contadores serão verificados, onde:
	EncoderContador = EncoderINC - Encoder_DEC;
	*/
	/*************************
	Em intervalos regulares 1/10s
	o valor calculado de:
	distancia percorrida e
	velocidade instantânea
	são atualizados
	*************************/

	int16 soma;
	int sub;
	
	i++;
	if(i>=99)
	{
		soma = get_timer1();
		sub = get_timer0();
		dVdt = EncoderContador;
		EncoderContador = make32(EncoderINC, soma)- make32(EncoderDEC, sub);
		dVdt = EncoderContador - dVdt;
		//armazena o maior valor de dVdT coletado
		if (dVdtMax < dVdt)
			dVdtMax = dVdt;
		i=0;
		FLAGS.atualiza=1; //libera atualização do LCD
		decimo++;
		if (time_out)
			time_out--;
	}
	time_count++;
	tick++;
	
	/*
	 a cada tick de 1ms armazena o byte recebido no buffer circular
	 reinicia ponteiro, se ocorrer overflow (64bytes)
	*/
	if( frame_kbhit() ) //while, não faça isso meu filho!
	{
		pRecebido++;
		if ( pRecebido >= 64 )
		  pRecebido = 0;
		buffer[pRecebido] = fgetc(BT);
	}
}



int aguarda_botao()
{
	
	
	/*************************
	leitura dos botões.
	se pulso curto, debounce de 20ms.
	se pulso longo, debounce de 500ms.
	retorna:
	0 - teclado solto
	1 - tecla 1 curto
	2 - tecla 2 curto
	3 - tecla 1 longo
	4 - tecla 2 longo
	*************************/
	static int1 kbd_down;
	static char last_key;

	estado_chave=0;
	if(tick>=fator_debounce)
	{
		if(kbd_down)
		{
			//tecla estava pressionada
			//testa se foi liberada
			if((kbd & (ALL_BOTTONS))==(ALL_BOTTONS))
			{
				//tecla foi solta
				kbd_down=FALSE;
				estado_chave=last_key;
				if(last_key>2)
					//inibe novo acionamento quando soltar tecla
					estado_chave=0;
				last_key=0;
				decimo = 0;
			}
			//verificar se houve 1/2 segundo
			if(decimo>=fator_repeat)
			{
				//adiciona 2 para indicar travamento
				if(last_key)
				{
					estado_chave=last_key + 2;
					last_key=0; //inibe novo acionamento quando soltar tecla
				}
				decimo = fator_repeat; //mantém contador
			}
		}
		else
		{
			if((kbd & (ALL_BOTTONS))!=(ALL_BOTTONS))
			{
				//tecla pressionada, testa qual delas
				if((kbd & Bot0)==0)
					last_key=1;
				else if((kbd & Bot1)==0)
					last_key=2;
				kbd_down = TRUE;
				decimo=0;
			}
		}
		tick=0;	
	}
//	if(!estado_chave)
//	{
//		if(kbhit(BT))
//				//#use rs232(STREAM=BT, baud=9600, xmit=PIN_C6,rcv=PIN_C7,RESTART_WDT, ERRORS, DISABLE_INTS,TIMEOUT=5)
//			estado_chave=fgetc(BT)-'0';
//				//#use rs232(STREAM=BT, baud=9600, xmit=PIN_C6,rcv=PIN_C7,RESTART_WDT, ERRORS, DISABLE_INTS)
//	}
	return(estado_chave);
}
/*void atualiza_LCD()
{

	//exporta pelo BT
	fprintf(BT,"T%9ldV%9ld\r\n",EncoderContador,dVdt);

	//apresentar no LCD
	lcd_gotoxy(1,1);
	printf(lcd_putc,"%12.2fmm p",distancia);

	lcd_gotoxy(1,2);
	printf(lcd_putc,"%4.1fkm/h%8ld",vel,EncoderContador);

	FLAGS.atualiza=0;
}*/

void ajusta_parametros()
{
	/*************************
	os parâmetros do microcontrolador
	são inicializados
	*************************/

 	SETUP_ADC_PORTS(NO_ANALOGS);
 	//desliga portas a/d
 	SETUP_ADC(ADC_OFF);
	setup_comparator(NC_NC_NC_NC);
	SET_TRIS_A( TRIS_A );
	SET_TRIS_B( TRIS_B );
	SET_TRIS_C( TRIS_C );
	port_b_pullups (true);//comentar debug

	
	//watchdog
	//watchdog: a cada 2s
	setup_wdt(WDT_2304MS);
	//timer0;
	//timer0 entrada externa, sem prescaler,interrupção habilitada
	setup_timer_0 (RTCC_EXT_L_TO_H | RTCC_DIV_1);
	//timer1;
	//timer1: entrada externa, sem prescaler, interrupção habilitada
	setup_ccp1(CCP_OFF);
	setup_timer_1 (T1_EXTERNAL_SYNC | T1_DIV_BY_1);
	//timer2;
	//timer2: prescaler, contador=,postscaler=, interrupção habilitada, quando medir velocidade.
	setup_ccp2(CCP_OFF);
	setup_timer_2 (T2_DIV_BY_4, 125, 10);
	restart_wdt();
	
	lcd_init();
	inicia_bluetooth();
	
	set_timer0(0);
	set_timer1(0); //aguarda 1 falling edge para iniciar contagem
	set_timer2(0);
	Eponteiro=0;
	EncoderContador=0;
	EncoderINC =0;
	EncoderDEC =0;
	FLAGS.saida = false;
	enable_interrupts(INT_TIMER0);
	enable_interrupts(INT_TIMER1);
	enable_interrupts(INT_TIMER2);
	enable_interrupts(global);
}

void inicia_bluetooth()
{
	output_bit(BT_Reset, 0);
	delay_ms(5);
	output_bit(BT_Reset, 1);
	delay_ms(5);

	//MODULO LINVOR
	#ifdef BT_LINVOR
		//9600
		fprintf(BT,"AT+PIN3445");
		delay_ms(5);
		fprintf(BT,"AT+NAMEbancorolos");
	#endif

	#ifdef BT_BOLUTEK
		AT+NAMEBanco de Rolos
		AT+NAME
		AT+PIN3445
		AT+PIN
		AT+SENM=3,1 //Sec_mode3_link,hci_enc_mode_pt_to_pt
		AT+SENM
		AT+RESET
	#endif

	#ifdef BT_HC05
	AT+NAME=BANCODEROLOS
	AT+NAME?
	AT+PSWD=3445
	AT+PSWD?
	AT+SENM=3,1
	AT+SENM?
	AT+RESET
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
	output_bit(BT_Key, 0);
	output_bit(BT_Reset, 1); 
}
int32 mult_with10(int32 num)
{
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
	return ( (num << 1) + (num << 3) );
}
void get_string(char* s, int max)
{
	/**************************************************************************
	*		Rotina para coletar string numérica
	*
	*	void get_string(char* s, int max)
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
	int pos_cursor; //posicao do cursor (posicao)
	int max_cursor; //tamanho maximo da string no display (apaga)
	int max_string; //tamanho da string no registro s
	int pos_string; //ponteiro da string (point)
	char valor; //valor atual do caracter (valor)
	unsigned int8 len;

	if(FLAGS.saida)
	{
		--max;
		len=0;
		do
		{
			c=lerBuffer();
			if(c==8)
			{  // Backspace
				if(len>0)
				{
					len--;
					putc(c);
					putc(' ');
					putc(c);
				}
			}
			else if ((c>='0')&&(c<='9'))//((c>=' ')&&(c<='~'))
       			if(len<=max)
	       		{
		       		s[len++]=c;
		       		putc(c);
		       	}
		} while(c!=13);
		s[len]=0;
	}	
	else
	{
		max_cursor=max;
		max_string=max_cursor-1;
		//cursor do display piscando
		//lcd_send_byte(0,0x0f);
		//cursor do display
		lcd_send_byte(0,0x0e);
	
		//desenha valor inicial no display
		//for(pos_cursor=max_cursor;pos_cursor>=1;pos_cursor--)
		for(pos_cursor=0;pos_cursor<max_cursor;pos_cursor++)
		{
			if(pos_cursor==2)
				lcd_putc('.');
			else
				lcd_putc('0');
		}
		//retorna cursor, com backspace, para lsb
		lcd_putc('\b');
	
		valor = '0';
		//inicia valor da string s[]
		for(pos_string=0;pos_string<max_string;pos_string++)
		{
			//reinicia valor em 00.000
			s[pos_string] = valor;
			//retorna cursor, com backspace, para lsb
			lcd_putc('\b');
		}
		pos_string=0;
		pos_cursor=0;
			
		//entrada da string.
		//aguarda B_Zero=1 para rolar número(0-9),
		//aguarda B_Zero=2 para rolar automaticamente, aguarda 50ms entre dígitos
		//B_Func=2 para trocar de caracter, retornando ao início
		//B_Func=4 para aceitar string
		do
		{
			//aguarda tecla ser pressionada
			do
			{
				c=aguarda_botao();
			}while(!c);
			
			//B_Zero==1 incrementa valor do caracter OU
			//B_Zero==3 incrementa automaticamente
			if(c==1 || c==3)
			{
				valor++;
				if(valor>'9')
					valor='0';
				//incrementa valor
				s[pos_string]=valor;
				//ecoa digito no display
				lcd_putc(valor);
				//retorna cursor
				lcd_putc('\b');
				if(c==3)
				{
					delay_ms(250);
					tick=fator_debounce;
				}	
			}
			//aceita valor numerico na string
			//B_Func==2
		    if(c==2)
		    {
			    if(!pos_cursor)
			    {
			    	if(valor=='0')
			    		//inibe zero inicial
			    		lcd_putc(' ');
			    	else
			    		//ecoa digito no display
			    		lcd_putc(valor);
			    }		
			    else
			    {
				    //ecoa digito no display
			    	lcd_putc(valor);
			    	if(pos_cursor==1)
			    	{
			    		//salta ponto decimal
			    		lcd_putc('.');
			    		pos_cursor++;
			    	}	
			    }
			    pos_cursor++;
			    pos_string++;
			    if(pos_cursor>=max_cursor)
				{
					//retorna à posição inicial da string no display
					for(pos_cursor;pos_cursor>=1;pos_cursor--)
					{
						//retorna cursor
						lcd_putc('\b');
					}
					pos_string=0;
					pos_cursor=0;	
				}
				valor=s[pos_string];
			}
		}while(c!=4);
		//desativa cursor antes de sair
		lcd_send_byte(0,0x0c);
		s[max+1]=0;
	}	
}	

long atol(char *s)
{
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

	long result;
	int index;
	
	index = 0;
	result = 0;
	
	c = s[index++];
	
	// base 10
	while (c >= '0' && c <= '9')
	{
		result = mult_with10(result) + (c - '0');
		c = s[index++];
	}
	return(result);
}

long get_long() 
{
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

	char s[6];
	long l;
	
	get_string(s, 6);
	l=atol(s);
	return(l);
}

void atualizaDistVel() 
{
	//apresenta numero de pulsos e distância percorrida
	distancia= EncoderContador / k; //milimetros
	vel = dVdt*0.036/k;// vel = km/h, onde: distancia (mm) e dt=0,1s 
	//exporta pelo BT
	tx_frame.stx=instrucao;
	tx_frame.comando= envia_leitura;
	tx_frame.formato= 0x00;
	tx_frame.numero= 0x06;
	setSplitedNumber( &tx_frame, EncoderContador, 2, 0 );
	setSplitedNumber( &tx_frame, dVdt, 2, 1 );
	protocolo_tx( &tx_frame );  
	//fprintf(BT,"T%9ldV%9ld\r\n",EncoderContador,dVdt);
	
	//apresentar no LCD
	lcd_gotoxy(1,1);
	printf(lcd_putc,"%12.2fmm p",distancia);
	
	lcd_gotoxy(1,2);
	printf(lcd_putc,"%4.1fkm/h%8ld",vel,EncoderContador);

	//atualiza_LCD();
	FLAGS.atualiza=0;
}	
int lerBuffer(void)
{
	pLido++;
	if ( pLido >= 64 )
		pLido = 0;
	return buffer[pLido];
}
void main()
{
	int c, pLidoPlus;
	//origem do comando 0=botão 1=Bluetooth
	int origem =0;

	ajusta_parametros();
	estado=PRINCIPAL; // inicializa a variavel de controle de estado
	
	/**************************************************************************
	*	1) Loop principal
	*		Apresentar no display
	*			Distancia percorrida em mm
	*			Velocidade em km/h
	*			Numero de pulsos coletados desde o último "Zero" (EncoderContador).
	*		Exportar
	*			Número de pulsos coletados desde o último "Zero" (EncoderContador).
	*			Numero de pulsos coletados a cada intervalo conhecido (dVdt).
	*			T_________V_________\r\n
	*		Aguardar Teclado ou Serial
	*			'1'	- Reinicia contagem dos pulsos (EncoderContador)
	*			'3' - Muda para receber Distancia Materializada
	*			'4' - Muda para ajuste de "k"
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
	
	
	while (true)
	{
		restart_wdt();
		c=aguarda_botao();
		/* ajusta ponteiros, ao final da pilha */
		pLidoPlus = pLido+1;
		if (pLidoPlus == 64) pLidoPlus=0;
		
		//avança até inicio de instrução
		while ( pLido != pRecebido
		 && buffer[pLidoPlus] != instrucao
		 && buffer[pLidoPlus] != resposta
		 && buffer[pLidoPlus] != erro	)
		{
			pLido++;
			if (pLido == 64) pLido = 0;
		}
		if( pRecebido != pLido )
		{			
			//se o frame estiver correto, escolha a proxima ação
			if(protocolo_rx(&rx_frame,&tx_frame))
			{
				if ( rx_frame.stx == instrucao )
				{
					switch(rx_frame.comando)//i
					{						
						case reset_sistema:
							estado=PRINCIPAL;
							c=0;
							reset_cpu();
						break;
						
						case iniciar_sistema:
							estado=PRINCIPAL;
							c=6;
						break;
						
						case maior_vel:
							//muda para estado principal, funcao 1
							do{}
							while(!FLAGS.atualiza); //aguarda sinc
							tx_frame.formato= 0x00;
							tx_frame.numero= 0x03;
							setNumber ( &tx_frame, dVdtmax );
							estado=PRINCIPAL;
						break;
						case zerar_distancia:
							do{}
							while(!FLAGS.atualiza); //aguarda sinc
							tx_frame.formato= 0x00;
							estado=PRINCIPAL;
							c=1;
						break;
						
						case cancelar:
							if ( estado != PRINCIPAL )
							{
								c=2;
							}	
						break;
						
						case ajustar:
							estado=PRINCIPAL;
							c=4;
							origem = 1;
						break;
						
						case medir_dist:
							estado=PRINCIPAL;
							c=3;
							origem = 1;
						break;
						
						default:
						break;
					}
					protocolo_tx(&tx_frame);
				}
			}
			else
			{	 
				 
				 protocolo_tx(&tx_frame);
			}		
		}	
			
		switch (estado)
		{
			case PRINCIPAL: 
			  	//calcula distancia em mm
			  	//calcula velocidade em km/h
			  	//apresenta distancia, velocidade e numero de pulsos
			  	//apresenta T______V_______\r\n
			  	//aguarda teclado
		   		if (FLAGS.atualiza)
				{
					atualizaDistVel();
				}
			  	//em modo de operação normal, zera contagem dos pulsos
				if(c==1)//B_Zero
				{
					set_timer0(0);
					set_timer1(1); //aguarda 1 falling edge para iniciar contagem
					EncoderINC =0;
					EncoderDEC =0;
					dVdtMax = 0; //reinicia armazenamento
				}					
		        if (c==2)
				{
					estado=DELAY;
					printf(lcd_putc,"\fk=%12.6f",k);
			        time_out=50;
				}	        
		        if(c==3)
		        {
			        estado=MED_DIST;
			        time_out=3000; //5min
			        //passa para calibração de distância
					FLAGS.disparo=0;
					printf(lcd_putc,"\fAguarda disparo");
					//aguarda Sync para limpar contadores e
					//iniciar a contagem de pulsos
					//ou B_Func
					clear_interrupt(INT_EXT);
					enable_interrupts(INT_EXT);
					c = 0;
		        }	        
		        if(c==4)
		        {
			        estado=AJUSTE;
			        time_out=3000; //5min
			        //passa para calibração de distância
					FLAGS.disparo=0;
					printf(lcd_putc,"\fAguarda disparo");
					//aguarda Sync para limpar contadores e
					//iniciar a contagem de pulsos
					//ou B_Func
					clear_interrupt(INT_EXT);
					enable_interrupts(INT_EXT);
		        }
		        if(c==6)
		        {
			       estado=DIST_MAT;
			       time_out=50;
			    }		
			break;
			case DELAY: // wait for response	
				if (!time_out) 
		        {
		            estado=PRINCIPAL;
		        }
			break;
			case DIST_MAT: // wait for response		
				DeltaL = getSplitedNumber( &rx_frame,2, 0);
				EncoderAjuste = getSplitedNumber( &rx_frame,2, 1);
				k = (float)EncoderAjuste/(float)DeltaL;
				dVdtMax = 0;
				estado=PRINCIPAL;
			break;
			case MED_DIST: // absorb data and place in a buffer
		        if (!time_out) // debounce timer expired
		        {
		            estado=PRINCIPAL;
		        }
		        if(FLAGS.disparo)//primeiro sensor de piso, inicio do percurso
		        {
			        //apresentar no LCD
					printf(lcd_putc,"\fAjuste...");
					time_count=0;
					estado = COLETA_DIST;
					time_out = 3000;
		        }
		        	
		        if(c == 2)
		        	estado = PRINCIPAL;
//		        if(FLAGS.atualiza)
//		        {
//			        atualizaDistVel();
//		        }	
	        break;
			case COLETA_DIST:
	      		if(c == 2)
	      			estado = PRINCIPAL;
				if (!time_out) // debounce timer expired
		        {
		            estado=PRINCIPAL;
		        }
				if (FLAGS.atualiza)
				{
					atualizaDistVel();	
					lcd_gotoxy(1,1);
					printf(lcd_putc,"MED");
				}
				// tempo para 5m e 80km/h -> 225ms
				// tempo para 5cm e 1km/h -> 185ms				
		        if( time_count>=220 )
		        {
			        if ( !INTE )//Teste se a interrupção está desativada
			        {
				        //time_count=10; //permite Sync
						clear_interrupt(INT_EXT); //***verificar se isso não impede int durante coleta real
						enable_interrupts(INT_EXT);
		        	}
		        	if( !FLAGS.disparo) //segundo sensor de piso, termino da pista
		        	{
			        	//sinaliza fim de coleta
			        	atualizaDistVel();	
			  			printf(lcd_putc,"\fValor medido:!");			  			
						lcd_gotoxy(1,2);
			  			printf(lcd_putc,"%12.2fmm",distancia);
			  			time_out=3000;
			  			if( origem )
						{
							tx_frame.stx=instrucao;
							tx_frame.comando= finaliza_ajuste;
							tx_frame.formato= 0x00;
							tx_frame.numero= 0x03;
							setNumber( &tx_frame, EncoderAjuste );
							protocolo_tx( &tx_frame );  
					        //fprintf(BT,"D%9ld\r\n",EncoderAjuste);
						  	FLAGS.atualiza=0;
						  	time_out = 1000;
						  	origem = 0;
						  	estado=COLETA_ACK;
					  	}
					  	else
					  	{
					        
							//coleta pelo teclado
							time_out=50;
							estado = DELAY;
						}
		        	}
				}
	        break;	  
	/*
		*	3) Durante Ajuste de "k"
		*		Apresenta no display "Aguarda disparo"
		*		Exporta "K_________\r\n"
		*		Aguarda pulso do detector de pista, teclado ou serial
		*			'2'	- Retorna para 1)
		*			Pulso - Inicia contagem quando detectado pulso
	*/
	      	case AJUSTE: // absorb data and place in a buffer
		        if (!time_out) // debounce timer expired
		        {
		            estado=PRINCIPAL;
		        }
		        if(FLAGS.disparo)//primeiro sensor de piso, inicio do percurso
		        {
			        //apresentar no LCD
					printf(lcd_putc,"\fAjuste...");
					time_count=0;
					estado = COLETA_INI1;
					time_out = 3000;
		        }
		        	
		        if(c == 2)
		        	estado = PRINCIPAL;
		        if(FLAGS.atualiza)
		        {
			        tx_frame.stx=instrucao;
					tx_frame.comando= envia_dist;
					tx_frame.formato= 0x00;
					tx_frame.numero= 0x03;
					setNumber( &tx_frame, EncoderContador );
					protocolo_tx( &tx_frame );  
		        	//fprintf(BT,"K%9ld\r\n",EncoderContador);
		        	FLAGS.atualiza=0;
		        }	
	        break;
	/*
		*	4) Contagem de pulsos da pista de Ajuste
		*		Apresentar no display
		*			"Ajuste..."
		*		Exportar
		*			Exporta "K_________\r\n"
		*			Número de pulsos coletados;
		*		Aguarda pulso do detector de pista, teclado ou serial
		*			'2' - Retorna para 1)
		*			pulso - termina contagem e armazena totalizador (EncoderContador)
	*/
	

			//aguarda periodo de 1s para liberar nova interrupção
			//impede novo Sync, no primeiro 1 segundo
	      	case COLETA_INI1:
	      		if(c == 2)
	      			estado = PRINCIPAL;
				if (!time_out) // debounce timer expired
		        {
		            estado=PRINCIPAL;
		        }
				if (FLAGS.atualiza)
				{
					//rotina depende de Timer2 e acontece a cada 1/10s
					//apresenta numero de pulsos
					lcd_gotoxy(1,2);
					printf(lcd_putc,"Encoder%9ld",EncoderContador);
					//exporta pelo BT
					tx_frame.stx=instrucao;
					tx_frame.comando= envia_dist;
					tx_frame.formato= 0x00;
					tx_frame.numero= 0x03;
					setNumber( &tx_frame, EncoderContador );
					protocolo_tx( &tx_frame );  
					//fprintf(BT,"K%9ld\r\n",EncoderContador);
					FLAGS.atualiza=0;	
				}
		        if( time_count>=220 )
		        {
			        if ( !INTE )//Teste se a interrupção está desativada
			        {
//				        time_count=10; //permite Sync
						clear_interrupt(INT_EXT); //***verificar se isso não impede int durante coleta real
						enable_interrupts(INT_EXT);
		        	}
		        	if( !FLAGS.disparo) //segundo sensor de piso, termino da pista
		        	{
			        	//sinaliza fim de coleta
			  			printf(lcd_putc,"\fAjuste Encerrado\nPressione botao!");
			  			time_out=3000;
			  			if( origem )
						{
							tx_frame.stx=instrucao;
							tx_frame.comando= finaliza_ajuste;
							tx_frame.formato= 0x00;
							tx_frame.numero= 0x06;
							//setNumber( &tx_frame, EncoderAjuste );
							setSplitedNumber( &tx_frame, EncoderAjuste, 2, 0 );
							setSplitedNumber( &tx_frame, time_count, 2, 1 );
							protocolo_tx( &tx_frame );  
					        //fprintf(BT,"D%9ld\r\n",EncoderAjuste);
						  	FLAGS.atualiza=0;
						  	time_out = 1000;
						  	estado=COLETA_ACK;
						  	origem = 0;
					  	}
					  	else
					  	{						  	
							//coleta pelo teclado
							estado = COLETA_TECLADO;
					        
						}
		        	}
				}
	        break;	        

	       	case COLETA_ACK:
	       		if (c==2)
	        		estado = PRINCIPAL;
	       		if ( rx_frame.stx == resposta && rx_frame.comando == tx_frame.comando )
	       		{
			  		estado = PRINCIPAL;
			  		
	       		}
	       		else {
		       		if (!time_out || rx_frame.stx == erro ) // reenviar
			        {
			            protocolo_tx( &tx_frame );  
			            rx_frame.stx = 0;
					  	time_out = 1000;
			        }
		        }
	       	break;
	       	
	       	case COLETA_TECLADO:
					//coleta pelo teclado
			  		//solicita distâcia, digito a digito.
			  		printf(lcd_putc,"\fInformeDistancia\n00.000m %7ldp",EncoderAjuste);
			  		lcd_gotoxy(1,2);
			  		time_count=0;
			  		DeltaL=get_long();
			  		estado = PRINCIPAL;
			  		k = (float)EncoderAjuste/(float)DeltaL;
			  		printf(lcd_putc,"\f");
	       	break;
	
	       	default:
	       		estado=PRINCIPAL;
	       	break;
	    }
	}    
}