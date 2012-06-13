// programa per controlar un ADC i un display LCD (LCDADC.c)
//s'implementen les funcions basiques de control i inicialització, es pot fer servir com a base d'altres aplicacions 
//que tinguin LCD o ADC

//Headers:-------------------------------------------------------------------
	
#include <P18F4550.h>	//include del PIC
#include <delays.h>		//funcions de retards
#include <xlcd.h>       //funcions del LCD
#include <stdlib.h>		//nose que fa? operacions matematiques
#include <stdio.h>    	//uso de conversiones printf
#include <adc.h>		//funcions del adc
#include <EEP.h>


//---------------------------

//#include <string.h>

//Bits de configuració: -----------------------------------------------------

        #pragma config PLLDIV   = 1          // PLL PreScaler = 1
        #pragma config CPUDIV   = OSC1_PLL2  // CPU PostScaler = /1  
        #pragma config USBDIV   = 1          // USB Clock Selection Bit
        #pragma config FOSC     = XT_XT      // Configurat a oscilador extern, sense PLL i fet servir pel USB
        #pragma config FCMEN    = OFF		// Fail Safe Clock Monitor Enable bit
        #pragma config IESO     = OFF		// Internal External Oscillator Switcthober bit
        #pragma config PWRT     = ON		// Power Up Timer
        #pragma config BOR      = OFF		// Brown Out Reset, fa un reset quant la tensio baixa del nivel BORV
        #pragma config BORV     = 3			// Tensió de Brown Out, detecta quant la tensio baixa d'aquest nivell i fa un reset
        #pragma config VREGEN   = ON      	// REGULADOR DE VOLTAJE USB
        #pragma config WDT      = OFF		// Watchdog timer enabe
        #pragma config WDTPS    = 32768		// Watchdog timer post scaler
        #pragma config MCLRE    = ON	    // Master Clear Pin Enable	
        #pragma config LPT1OSC  = OFF		// Low power timer 1 oscillator enable bit
        #pragma config PBADEN   = OFF		// Port B A/D enable bit
        #pragma config STVREN   = OFF		// Stack Full/UnderFlow Reset Enable bit, fa un reset quant la pila esta plena o underflow
        #pragma config LVP      = OFF		// Configura el PIC perque pugui ser programat a baixa tensio
        #pragma config XINST    = OFF       // CONJUNTO DE INSTRUCCIONES EXTENDIDAS
        #pragma config CP0      = OFF		// Code protection del primer block 000800 -001FFF
        #pragma config CP1      = OFF		// Code protection del segon block 002000 - 003FFF
        #pragma config CPB      = OFF		// Boot block protection bit 000000 - 0007FF
        #pragma config WRT0     = OFF		// Write protection del primer block
        #pragma config WRT1     = OFF		// Write protection del segon block
        #pragma config WRTB     = OFF       // PROTECCIÓN DE ESCRITURA PARA EL BLOCK DE BOOT
        #pragma config WRTC     = OFF		// Write protection pel registre de configuracio
        #pragma config EBTR0    = OFF		// Table read protection block 0
        #pragma config EBTR1    = OFF		// Table read protection block 1
        #pragma config EBTRB    = OFF		// Table read protection block boot
 

//--- SORTIDES
#define led_verd LATDbits.LATD7
#define led_vermell LATDbits.LATD6
#define escalfador LATDbits.LATD4
#define ventilador LATCbits.LATC7
#define motor_extractor LATCbits.LATC6
#define motor_pellet LATDbits.LATD1
#define motor_escalfador LATDbits.LATD0
#define contacte1 LATDbits.LATD2
#define contacte2 LATDbits.LATD3
//--- tris sortida
#define tris_led_verd TRISDbits.TRISD7
#define tris_led_vermell TRISDbits.TRISD6
#define tris_escalfador TRISDbits.TRISD4
#define tris_ventilador TRISCbits.TRISC7
#define tris_motor_extractor TRISCbits.TRISC6
#define tris_motor_pellet TRISDbits.TRISD1
#define tris_motor_escalfador TRISDbits.TRISD0
#define tris_contacte1 TRISDbits.TRISD2
#define tris_contacte2 TRISDbits.TRISD3


//--- ENTRADES
#define pulsador_EXTRA PORTAbits.RA4  	//P1
#define pulsador_DOWN PORTEbits.RE0  	//P2
#define pulsador_RIGHT PORTEbits.RE1 	//P3
#define pulsador_OK PORTEbits.RE2  		//P4
#define pulsador_LEFT PORTCbits.RC0 	//P5
#define pulsador_UP PORTCbits.RC1 		//P6
#define pulsador_POWER PORTCbits.RC2 	//P7
//--- tris d'entrada
#define tris_P1 TRISAbits.TRISA4  	//P1
#define tris_P2 TRISEbits.TRISE0  	//P2
#define tris_P3 TRISEbits.TRISE1 	//P3
#define tris_P4 TRISEbits.TRISE2  		//P4
#define tris_P5 TRISCbits.TRISC0 	//P5
#define tris_P6 TRISCbits.TRISC1 		//P6
#define tris_P7 TRISCbits.TRISC2 	//P7

//analogiques

#define TERMO_HOTAIR ADC_CH4
#define TERMO_AMBIENT  ADC_CH3
#define LAMBDA ADC_CH2
#define ISENSED_PELLET  ADC_CH1
#define ISENSED_ESCALFADOR  ADC_CH0


//DIRECCIO PORTS
#define ENTRADA 1
#define SORTIDA 0

//ESTAT PORTS
#define PARAT	0
#define ENGEGAT	1

//BOOLEANES
#define TRUE	1
#define FALSE	0

//FASES
#define RUNNING		0
#define PROGRAMACIO		1
#define COOLING_DOWN	2
#define	STOP	3
#define OMPLIR_PELLET 4
#define ESCALFADOR 5
#define ERROR 6

//VALORS PULSADOR
#define UP		1
#define DOWN		2
#define RIGHT		3
#define LEFT		4
#define OK		5
#define EXTRA		6
#define POWER  7

//NUMERO DE REPETICIONS DE LA INTERRUPCIO
#define REPETICIONS 10

#define USE_OR_MASKS // per la eeprom


//definicio de funcions: ---------------------------------------------------

void cmdXLCD(unsigned char cd);
void InterruptHandlerHigh (void);
void config(void);

//Definicio de Variables globals -------------------------------------------

char buf[15];
char buf2[15];
char buf3[15];
char adc[16]={"Temp Comfort:"};
char prog_string[16]={"PROGRAMACIO    "};
char run_string[16]={"running        "};
char saved_string[16]={"Guardat correct"};
char admin_pellet_string[16]={"Administ pellet"};
char jo[20]={"iomoiomo        "};
unsigned int value1,value2,ch,temp_ambient, temp_hotair, isensed_pellet, isensed_escalfador;
unsigned char estat = 0;

//Definicio de variables

unsigned char estat_extractor;
unsigned char estat_pellet;
unsigned char estat_escalfador;
unsigned char estat_ventilador;
unsigned char t_pantalla;
unsigned char temp_sp;
unsigned char th_extractor;                //punter valor s'inicialitza aqui
unsigned char tl_extractor;                //+1
unsigned char th_pellet;                        //+2        
unsigned char tl_pellet;                        //+3
unsigned char ta_pellet;                  //+4 temps administracio pellet inici
unsigned char t_escalfador;                // + 5
unsigned char num_cicle;
unsigned int compt_extractor;
unsigned int compt_pellet;
unsigned int compt_esclafador;
unsigned int compt_ventilador;
unsigned char compt_pantalla;
unsigned char fase;
unsigned char *puntervalor;
unsigned char *punternom;
unsigned char extr_on[10] = "EXTR_ON";                    //punter nom s'inicialitza aqui
unsigned char extr_off[10] = "EXTR_OFF";                //+1
unsigned char pellet_on[10] = "PELLE_ON";                //+2
unsigned char pellet_off[10] = "PELLE_OFF";                //+3
unsigned char pellet_admin[10] = "ADMINI_AR"; 			// + 4
unsigned char escalfador_on[10] = "ESCALF_AR";				// + 5
unsigned char controlpunter;
unsigned char pulsador;
unsigned char last_pulsador;
unsigned int rapid = FALSE;

unsigned char  EEPWrite[15] = "MICROCHIP_TECH", EEPRead[15],Error=0 ;


//Implementacio de funcions -------------------------------------------------

//funcio config del ADC, no retorna res, fa la inicialitzacio del ADC i prou.
void config(void){

	led_verd = PARAT;
	led_vermell = PARAT;
	escalfador = PARAT;
	ventilador = PARAT;
	motor_extractor = PARAT;
	motor_pellet = PARAT;
	motor_escalfador= PARAT; 
	contacte1 = PARAT;
	contacte2 = PARAT;
 

	// configura lcd
    OpenXLCD(FOUR_BIT &     //4-bit, la configuracio dels pins esta en el fitxer xlcd.h
             LINES_5X7);    //

	// configura pins	
	
	tris_P1 = ENTRADA; 	//P1
	tris_P2 = ENTRADA; 	//P2
	tris_P3 = ENTRADA; 	//P3
	tris_P4 = ENTRADA; 	//P4
	tris_P5 = ENTRADA; 	//P5
	tris_P6 = ENTRADA;	//P6
	tris_P7	= ENTRADA;	//P7
	
	tris_motor_pellet = SORTIDA; 			//defineix la pota RB7 com a LED_Pin
	tris_led_verd = SORTIDA;
	tris_led_vermell = SORTIDA; 
	tris_escalfador = SORTIDA; 
	tris_ventilador = SORTIDA;
	tris_motor_extractor = SORTIDA; 
	tris_motor_pellet = SORTIDA; 
	tris_motor_escalfador = SORTIDA; 
	tris_contacte1 = SORTIDA; 
	tris_contacte2 = SORTIDA; 


//CONFIGURA USB
	UCONbits.USBEN = 0;

// configura lcd
    OpenXLCD(FOUR_BIT &     //4-bit, la configuracio dels pins esta en el fitxer xlcd.h
             LINES_5X7);    //

//Configuramos ADC  *****FALTA CONFIGURAR TRIS ANALOGIC (ENTRADA)
    OpenADC(ADC_FOSC_2     &    //Clock Interno
            ADC_RIGHT_JUST    &    //10bit
            ADC_20_TAD        ,    //20TAD
            ADC_CH0            &    //CANAL0
            ADC_CH1            &    //CANAL1
            ADC_CH2            & 
            ADC_CH3            & 
			ADC_CH4            &
            ADC_INT_OFF        &    //INTERRUPCIONES OFF
            ADC_REF_VDD_VSS ,    //+5,GND
            ADC_5ANA);            //canal 0,1 analogo, resto digital


	//inicialitzacio de les interrupcions

	INTCON = 0xA0;					//disable global and enable TMR0 interrupt and RB
	INTCON2bits.TMR0IP = 1;			//TMR0 high priority				//enable priority levels
	TMR0H = 100;						//clear timer and set TMR0H
	TMR0L = 0;						//clear timer low byte and also updates high byte from TMR0H
	T0CON = 0xC6;					//set up timer0 - no prescaler
	INTCONbits.GIEH = 1;			//enable interrupts
}



//funcio cmdXLCD, envia el comando cd al display
void cmdXLCD(unsigned char cd){
    while(BusyXLCD());
    WriteCmdXLCD(cd);
}

//------------------------------------------------------------------------------

#pragma code

// funcions que estan definides a xlcd.h com a extern i estan implementades aqui
// es necessari definir-les perque les funcions de la LCD la fan servir.

void DelayFor18TCY(void){
	Delay10TCYx(4);			// 200 Tcy = 40 us a 4 MHz
	
} 
void DelayPORXLCD(void){
	Delay1KTCYx(15);		// 15 ms a 4 MHz
	//Delay1KTCYx(6);
}
void DelayXLCD(void){
	Delay1KTCYx(5);		// 5 ms a 4 MHz
	//Delay1KTCYx(2);
}


// Ubica cursor en (x = Posicion en linea, y = nº de linea)
void gotoxyXLCD(unsigned char x, unsigned char y){
	unsigned char direccion;

	if(y != 1)
		direccion = 0x40;
	else
		direccion=0;
	
	direccion += x-1;
	WriteCmdXLCD(0x80 | direccion);
}



//Funcio principal --------------------------------------------------------------------

void main(void){

	// Inicialitza les coses
	Delay10KTCYx(100);
    config();
	led_verd = 1;


	// inicialitzacio variables estufa
	estat_extractor = PARAT;
	estat_pellet = PARAT;
	estat_escalfador = PARAT;
	t_pantalla = 250;
	th_extractor  = Read_b_eep (0x0200);
	tl_extractor = Read_b_eep (0x0201);
	th_pellet = Read_b_eep (0x0202);
	tl_pellet = Read_b_eep (0x0203); // sera multiplicat* 100
	ta_pellet = Read_b_eep (0x0204); // sera multiplicat* 100
	t_escalfador = Read_b_eep (0x0205); // sera multiplicat* 100
	temp_sp = Read_b_eep (0x0206);
	num_cicle = 0;
	compt_extractor = 0;
	compt_pellet = 0;
	compt_esclafador = 0;
	compt_pantalla = 0;
	fase = STOP;
	controlpunter = 0;
	puntervalor = &th_extractor;
	punternom = &extr_on[0];
	pulsador = 0;
	isensed_escalfador = 0;
	
	motor_extractor = PARAT;
	motor_pellet = PARAT;
	
	//inicialitzacio lcd

    cmdXLCD(0x0c);    // 000001100 display sense cursor
    cmdXLCD(0x01);    // 000000001 borra la pantalla i posa el cursor al principi

	
    while(1){    //bucle del programa
	//gotoxyXLCD(1,2);
	//sprintf(buf2,"%i   ",fase);
	//putsXLCD(buf2);
	// CONTROL GENERAL BASAT EN TEMPERATURES
	
		if ((fase == STOP)||(fase == COOLING_DOWN)){
			if(temp_ambient < (temp_sp - 2)){
				fase = OMPLIR_PELLET;
			}	
		}

		if (fase == COOLING_DOWN){	
			if (temp_hotair < temp_ambient + 10){
				fase = STOP;
			}
		}

		if (fase == RUNNING){
			if (temp_ambient > temp_sp){
				fase = COOLING_DOWN;		
			}
		}	

	//CONTROL AMB ELS PULSADORS			

		if (pulsador_UP == 0) { pulsador = UP; }
		if (pulsador_DOWN == 0) { pulsador = DOWN; }
		if (pulsador_LEFT == 0) { pulsador = LEFT; }
		if (pulsador_RIGHT == 0) { pulsador = RIGHT; }
		if (pulsador_OK == 0) { pulsador = OK; }
		if (pulsador_EXTRA == 0) { pulsador = EXTRA; }
		if (pulsador_POWER == 0) { pulsador = POWER; }
		
		switch (pulsador) {
			
			case RIGHT:
				if (fase == PROGRAMACIO){
					if (controlpunter < 5){
						punternom += 10;
						puntervalor++;
						controlpunter++;
					}
					gotoxyXLCD(1,2);
					sprintf(buf2,"%s = %i   ",punternom,*puntervalor);
					putsXLCD(buf2);
					Delay10KTCYx(20);
				}
			break;
		
			case LEFT:
				if (fase == PROGRAMACIO){
					if (controlpunter > 0){
						punternom -= 10;
						puntervalor--;
						controlpunter--;
					}
					gotoxyXLCD(1,2);
					sprintf(buf2,"%s = %i   ",punternom,*puntervalor);
					putsXLCD(buf2);
					Delay10KTCYx(20);
				}
			break;
			
			case DOWN:
				if (fase == PROGRAMACIO){
					(*puntervalor)--;
					gotoxyXLCD(1,2);
					sprintf(buf2,"%s = %i   ",punternom,*puntervalor);
					putsXLCD(buf2);
					
				}else if((fase == RUNNING)||(fase == COOLING_DOWN)||(fase == STOP)){
					temp_sp --;
					Write_b_eep (0x0206, temp_sp);
      				Busy_eep (); 
					sprintf(buf,"Temp Comfort:%i ", temp_sp); 
					gotoxyXLCD(1,2);
	    			putsXLCD(buf);        //imprime				
				}
				Delay10KTCYx(20);

			break;
		
			case UP:
				if (fase == PROGRAMACIO){
					(*puntervalor)++;
					gotoxyXLCD(1,2);
					sprintf(buf2,"%s = %i   ",punternom,*puntervalor);
					putsXLCD(buf2);

				}else if((fase == RUNNING)||(fase == COOLING_DOWN)||(fase == STOP)){
					temp_sp ++;
					Write_b_eep (0x0206, temp_sp);
      				Busy_eep ();
					sprintf(buf,"Temp Comfort:%i ", temp_sp); 
					gotoxyXLCD(1,2);
	    			putsXLCD(buf);        //imprime

				}
				Delay10KTCYx(20);

			break;
		
			case OK:
				if (fase == RUNNING){
					fase = PROGRAMACIO;
					gotoxyXLCD(1,1);        //va a la primera linea
		        	putsXLCD(prog_string);
					gotoxyXLCD(1,2);
					sprintf(buf2,"%s = %i   ",punternom,*puntervalor);
					putsXLCD(buf2);
				}else if (fase == PROGRAMACIO){
					Write_b_eep (0x0200, th_extractor);
      				Busy_eep ();
					Write_b_eep (0x0201, tl_extractor);
      				Busy_eep ();
					Write_b_eep (0x0202, th_pellet);
      				Busy_eep ();
					Write_b_eep (0x0203, tl_pellet);
      				Busy_eep ();  
					Write_b_eep (0x0204, ta_pellet);
      				Busy_eep ();    
					Write_b_eep (0x0205, t_escalfador);
      				Busy_eep ();             
					cmdXLCD(0x01); //borra la pantalla
					gotoxyXLCD(1,1);        //va a la primera linea
		        	putsXLCD(saved_string);
					Delay10KTCYx(150);
					fase = RUNNING;
				}
				Delay10KTCYx(20); //per evitar tornar a entrar amb la mateixa pulsada
			break;

			case EXTRA:
				//fotre-li xitxa...
			break;

			case POWER:
				 
			break;

			default:

			break;
		
		}
		last_pulsador = pulsador;
		pulsador = 0;
	}
}

// High priority interrupt vector

#pragma code InterruptVectorHigh = 0x08
void InterruptVectorHigh (void)
{
  _asm
    goto InterruptHandlerHigh
  _endasm
}



// High priority interrupt routine

#pragma code
#pragma interrupt InterruptHandlerHigh

void InterruptHandlerHigh () {

	if (INTCONbits.TMR0IF) {				//check for TMR0 overflow


		// ------- RUNNING, PROGRAMACIO, ESCALFADOR ------------------------
		//	En aquestes fases s'ha de tenir els motors mantenint la estufa.	


		if ((fase == RUNNING) || (fase == PROGRAMACIO) || (fase == ESCALFADOR)) {


			// VENTILADOR
			//if (temp_hotair > (temp_ambient + 5)){
				ventilador = ENGEGAT;
			//}else{
			//	ventilador = PARAT;
			//}

			// CONTROL D'ACTUADORS
			// MOTOR EXTRACTOR
			if (estat_extractor == ENGEGAT){
	        	if (compt_extractor >= th_extractor){
	                estat_extractor = PARAT;
	                motor_extractor = PARAT;
	                compt_extractor = 0;
	        	}else{
	                compt_extractor++;
	        	}
			}else{
	        	if (compt_extractor >= tl_extractor){
	                estat_extractor = ENGEGAT;
	                motor_extractor = ENGEGAT;
	                compt_extractor = 0;
	        	}else{
	                compt_extractor++;
				}
			}
	
			//MOTOR PELLET, falta control intensitat.
			if (estat_pellet == ENGEGAT){
	        	if (compt_pellet >= ((unsigned int)th_pellet)*10){
	                estat_pellet = PARAT;
	                motor_pellet = PARAT;
	                compt_pellet = 0;
	        	}else{
	                compt_pellet++;
	        	}
			}else{
	        	if (compt_pellet >= ((unsigned int)tl_pellet)*100){
	                estat_pellet = ENGEGAT;
	                motor_pellet = ENGEGAT;
	                compt_pellet = 0;
	        	}else{
	                compt_pellet++;
				}
			}
		}


		// ------- OMPLIR PELLET ------------------------
		// En aquesta fase s'omple de pellet la camara de combustio. S'hi passa una vegada
		// i despres es passa a la fase ESCALFADOR.

		if (fase == OMPLIR_PELLET){

			//CONTROL ACTUADORS, falta control de intensitat. Si s'encatlla -> error
			if (estat_pellet == ENGEGAT){
	        	if ((compt_pellet) >= ((unsigned int)(ta_pellet))*100){
					fase = ESCALFADOR;
					estat_pellet = PARAT;
	                motor_pellet = PARAT;
	                compt_pellet = 0;	

	        	}else{
	                compt_pellet++;
	        	}
			}else{
	                estat_pellet = ENGEGAT;
	                motor_pellet = ENGEGAT;
	                compt_pellet = 0;
			}
			//CONTROL PANTALLA
			if (compt_pantalla >= t_pantalla){
				cmdXLCD(0x01); //borra la pantalla
				gotoxyXLCD(1,1);
				putrsXLCD("Arrancant...");        //imprime
				gotoxyXLCD(1,2);
				putrsXLCD("Omplint pellet");

				compt_pantalla = 0;

			}else{
				compt_pantalla ++;
			}		
		}

		// ------- ESCALFADOR ------------------------
		// Per aquesta fase s'hi passa una vegada. Mentres s'hi esta es manté l'escalfador
		// i el ventilador escalfador engegats. Després es passa a la fase RUNNING.

		if (fase == ESCALFADOR){
			//CONTROL ACTUADORS

			//if (isensed_escalfador == 0){
			//	fase = ERROR;
			//	motor_escalfador = PARAT;
			//	estat_escalfador = PARAT;
			//}

			if (estat_escalfador == ENGEGAT){
	        	if ((compt_esclafador) >= ((unsigned int)(t_escalfador))*100){
					fase = RUNNING;
					motor_escalfador = PARAT;
	                estat_escalfador = PARAT;
					escalfador = PARAT;
					compt_esclafador = 0;	
	        	}else{
	                compt_esclafador++;
	        	}
			}else{
				motor_escalfador = ENGEGAT;
	            estat_escalfador = ENGEGAT;
				escalfador = ENGEGAT;
				compt_esclafador = 0;
			}
			
			// CONTROL PANTALLA I LECTURA DE INTENSITATS (EL MATEIX TEMPS)
			if (compt_pantalla >= t_pantalla){
				// LECTURA INTENSITAT MOTOR ESCALFADOR
				SetChanADC(ISENSED_ESCALFADOR);
				ConvertADC();        //start convert
			    while(BusyADC());    //Ha terminado?
				isensed_escalfador = ReadADC();
 				// CONTROL PANTALLA
				cmdXLCD(0x01); //borra la pantalla
				gotoxyXLCD(1,1);
				putrsXLCD("Arrancant...");        //imprime
				gotoxyXLCD(1,2);
				putrsXLCD("Escalfant");

				compt_pantalla = 0;

			}else{
				//sprintf(buf,"comp_pant:%i ",compt_pantalla);
				//gotoxyXLCD(1,1);
				//putsXLCD(buf);
				compt_pantalla ++;
			}
		}

		// ------- STOP ------------------------
		// En aquesta fase esta tot parat, esperant que la temperatura ambient
		// baixi per engegar-se.

		if (fase == STOP){
			if (compt_pantalla >= t_pantalla){
				
				estat_extractor = PARAT;
	            motor_extractor = PARAT;
				estat_pellet = PARAT;
	            motor_pellet = PARAT;
				motor_escalfador = PARAT;
	            estat_escalfador = PARAT;
				escalfador = PARAT;
				ventilador = PARAT;


				//actualitza temp_ambient
				SetChanADC(TERMO_AMBIENT);//canal 3 font del adc
				ConvertADC();        //start convert
	    		while(BusyADC());    //Ha terminado?
	    		temp_ambient = ReadADC()/2;
   
				//actualitza temp_hotair
				SetChanADC(TERMO_HOTAIR);//canal 3 font del adc
				ConvertADC();        //start convert
	    		while(BusyADC());    //Ha terminado?
	    		temp_hotair = ReadADC()/2;

  				// CONTROL PANTALLA
				cmdXLCD(0x01); 		//borra la pantalla 
				gotoxyXLCD(1,1);
	    		putrsXLCD("STOP");        //imprime
				sprintf(buf,"Temp Comfort:%i ", temp_sp); 
				gotoxyXLCD(1,2);
	    		putsXLCD(buf);        //imprime

				compt_pantalla = 0;

			}else{
				compt_pantalla ++;
			}	
		}

		// ------- RUNNING, PANTALLA ------------------------
		if (fase == RUNNING){
			// CONTROL PANTALLA, ACTUADORS CONTROLAT EN IF DE DALT
			if (compt_pantalla >= t_pantalla){

				//actualitza temp_ambient
				SetChanADC(TERMO_AMBIENT);//canal 3 font del adc
				ConvertADC();        //start convert
	    		while(BusyADC());    //Ha terminado?
	    		temp_ambient = ReadADC()/2;
   
				//actualitza temp_hotair
				SetChanADC(TERMO_HOTAIR);//canal 3 font del adc
				ConvertADC();        //start convert
	    		while(BusyADC());    //Ha terminado?
	    		temp_hotair = ReadADC()/2;

  				// ho presenta per pantalla
				sprintf(buf,"Temp Ambient:%i ", temp_ambient); 
				gotoxyXLCD(1,1);
	    		putsXLCD(buf);        //imprime
				sprintf(buf,"T.Cf:%i  T.Ho:%i", temp_sp, temp_hotair); 
				gotoxyXLCD(1,2);
	    		putsXLCD(buf);        //imprime

				compt_pantalla = 0;

			}else{
				compt_pantalla ++;
			}
			
		}	
	}else{
		led_vermell = 1;   // SI ENTRA AQUI ES QUE HI HA ALGU ALTRE GENERANT INTERRUPCIONS
	}

	TMR0H = 0;						//clear timer and set TMR0H
	TMR0L = 200;
	INTCONbits.TMR0IF = 0;				//clear interrupt flag
}	










