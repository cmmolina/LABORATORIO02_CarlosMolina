/*
 * Universidad del Valle de Guatemala
 * Electrónica Digital 2
 * Carlos Mauricio Molina López (#21253)
 * LABORATORIO 02 - LCD
 * Created on 26 de enero de 2023, 08:04 PM
 */

//******************************************************************************
// Palabra de Configuración
//******************************************************************************

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT // Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include "LCD.h"
#include "adc.h"
#include "uart.h"

#define _XTAL_FREQ 8000000
#define tmr0_value 179

//Bits de Control
#define RS PORTBbits.RB7
#define EN PORTBbits.RB6
#define D0 PORTDbits.RD0
#define D1 PORTDbits.RD1
#define D2 PORTDbits.RD2
#define D3 PORTDbits.RD3
#define D4 PORTDbits.RD4
#define D5 PORTDbits.RD5
#define D6 PORTDbits.RD6
#define D7 PORTDbits.RD7

//******************************************************************************
// Variables 
//******************************************************************************
float valorADC;

unsigned int voltaje1; 
unsigned int voltaje2; 

unsigned int cont; 

float VOLTAJE1; 
float VOLTAJE2; 

char ADC1[4];
char ADC2[4];
char prueba[4];

char hundreds;
char tens;
char ones;
char signo;

//******************************************************************************
// Prototipos de Funciones
//******************************************************************************
void setup(void);
unsigned int map(uint8_t value, int inputmin, int inputmax, int outmin, int outmax);
//******************************************************************************
// Interrupción
//******************************************************************************
void __interrupt() isr (void){    
    
    //Interrupción de Envío
    if (PIR1bits.TXIF){
        PIR1bits.TXIF = 0;
    }
    
    //Interrupción de Recepción
    if (PIR1bits.RCIF){
        signo = UART_read_char();
  
        //Suma y Resta de Contador
        if (signo == '+'){
            
            if(cont <= 254){
              cont = cont++;  
            }
            
            else if (cont == 255){
                cont=0;
            }
        }
        
        else if (signo == '-'){
            
            if(cont > 0){
               cont = cont--; 
            }
            else if (cont == 0){
                cont = 255;
            }
        }  
        
        //PORTB = cont;
        PIR1bits.RCIF = 0;
    }
    
    //Interrupción del ADC cuando la lectura termina
    if (PIR1bits.ADIF){
        PIR1bits.ADIF = 0; 
    }
    
    //Interrupción del TMR0
    if (INTCONbits.T0IF){
        TMR0 = tmr0_value;          // Cargamos 5ms de nuevo al TMR0
        INTCONbits.T0IF = 0;
    }
    
    //Interrupción del Puerto B 
    if (INTCONbits.RBIF){ 
        INTCONbits.RBIF = 0;
    }
}

//******************************************************************************
// Código Principal 
//******************************************************************************
void main(void) {
    setup();
    setupADC();
    Lcd_Init();
    Lcd_Clear();
    UART_TX_config(9600);
    UART_RX_config(9600);
    cont=0; 
    
    //Titulo en Terminal
    UART_write_char("\rSigno\r");
     
    //Loop Principal
    while(1){
        
        //Desplegamos Índices de Voltaje
        Lcd_Set_Cursor(1,2);
        Lcd_Write_String("S1");
        Lcd_Set_Cursor(1,8); 
        Lcd_Write_String("S2");
        Lcd_Set_Cursor(1,14);
        Lcd_Write_String("CON");
        
        
        //Desplegamos S1 (Valor del POT1)
        Lcd_Set_Cursor(2,1);
        valorADC = readADC(0);
        VOLTAJE1 = (valorADC*5)/255;
        sprintf(ADC1,"%.2f", VOLTAJE1);
        Lcd_Write_String(ADC1);
        Lcd_Write_String("V");
                
        //Desplegamos S2 (Valor del POT2)
        Lcd_Set_Cursor(2,7);
        valorADC = readADC(1);
        VOLTAJE2 = (valorADC*5)/255;
        sprintf(ADC2,"%.2f", VOLTAJE2);
        Lcd_Write_String(ADC2);
        Lcd_Write_String("V");
                
        //Desplegamos S3
        Lcd_Set_Cursor(2,14);
        
        hundreds = (cont/100);
        tens = (cont/10)%10;
        ones = cont%10; 
        
        Lcd_Write_Char(hundreds+48);
        Lcd_Write_Char(tens+48);
        Lcd_Write_Char(ones+48);
        
    }
}

//******************************************************************************
//Funciones
//******************************************************************************

void setup(void){
    //Configuración de I/O 
    
    ANSEL = 0b00000011;             // RA0, RA1 analógico
    ANSELH = 0; 

            //76543210
    TRISA = 0b00000011;             // RA0, RA1 como Inputs
    TRISB = 0b00000000;             // 
    TRISC = 0b10000000;             // 
    TRISD = 0b00000000;             //
    TRISE = 0b00000000;             // 
    
    PORTA = 0b00000000; 
    PORTB = 0b00000000; 
    PORTC = 0b00000000; 
    PORTD = 0b00000000; 
    PORTE = 0b00000000;
    
    //Configuración del Oscilador
    OSCCONbits.IRCF = 0b111;        // 8MHz
    OSCCONbits.SCS = 1;             // Oscilador Interno
    
    //Configuración del Puerto B 
    //IOCB = 0b00000011;              // Pines de Puerto B con Interrupción
    //OPTION_REGbits.nRBPU = 0;       // Pull-Up/Pull-Down
    //INTCONbits.RBIE = 1;            // Se habilitan las interrupciones del Puerto B
    
    //Configuración de las Interrupciones
    INTCONbits.GIE = 1;             // Se habilitan las interrupciones globales
    //INTCONbits.PEIE = 1;
    
    PIE1bits.ADIE = 1;              // Se habilita la interrupción del ADC
    INTCONbits.TMR0IE = 1;          // Se habilitan las interrupciones del TMR0    
    
    PIR1bits.ADIF = 0;              // Flag de ADC en 0
    INTCONbits.RBIF = 0;            // Flag de Interrupciones del Puerto B en 0

    //Configuración del TMR0
    OPTION_REGbits.T0CS = 0;        // Fosc/4
    OPTION_REGbits.PSA = 0;         // Prescaler para el TMR0
    OPTION_REGbits.PS = 0b011;      // Prescaler 1:16
    TMR0 = tmr0_value;              // Asignamos valor al TMR0 para 4ms
    INTCONbits.T0IF = 0;            // Flag de TMR0 en 0
}

//******************************************************************************
// Función de Conversión 
//******************************************************************************
unsigned int map(uint8_t value, int inputmin, 
                  int inputmax, int outmin, int outmax){
    return ((value - inputmin)*(outmax-outmin)) / ((inputmax-inputmin)+outmin);
}