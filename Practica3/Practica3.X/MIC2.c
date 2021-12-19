/*
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = PRIPLL           // Oscillator Mode (Primary Oscillator (XT, HS, EC) w/ PLL)
#pragma config IESO = ON                // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)

// FOSC
#pragma config POSCMD = EC              // Primary Oscillator Source (EC Oscillator Mode)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow Multiple Re-configurations)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = ON              // Watchdog Timer Enable (Watchdog timer always enabled)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
#pragma config LPOL = ON                // Motor Control PWM Low Side Polarity bit (PWM module low side output pins have active-high output polarity)
#pragma config HPOL = ON                // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-high output polarity)
#pragma config PWMPIN = ON              // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PORT register at device Reset)

// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>          // Defines NULL
#include <stdbool.h>         // Defines true
#include <math.h>

#define LED_GREEN LATAbits.LATA0
#define LED_RED   LATAbits.LATA1
#define LED_ON_STATE 1
#define LED_OFF_STATE 0
#define baud_9600 51

char dataCMD_ISR[50];                           //Rx buffer
char txBuffer_ISR[200];                         //TX buffer
unsigned char BufferLoadDone = 1;
unsigned char AllowPrint = 0;                  //Ver de donde sale esto
unsigned char dummy;
unsigned int nextchar = 0;                     //Size/index variable (Used txBuffer)
unsigned int U1_PrintRate_ISR = 0;             //Aux counter for Print Rate
unsigned int counter = 0;                      //Tx print numeric counter;
volatile unsigned int data_count = 0;          //Size/index variable (Used Rx Buffer)
volatile unsigned char comando_detectado = 0;  //Detect new command

const char cmd0[50] = {0x50, 0x01, 0xAA};
const char cmd1[50] = {0x50, 0x00, 0xAA};
const char cmd2[50] = {0x51, 0x01, 0xAA};
const char cmd3[50] = {0x51, 0x00, 0xAA};
const char cmd4[50] = {0x52, 0x01, 0xAA};
const char cmd5[50] = {0x52, 0x00, 0xAA};

void uart_config (unsigned int baud){    
    //UART PORT/PIN COMMUNICATION ASSIGMENT
    TRISCbits.TRISC0   = 1;  //Pin RC0 input (digital).
    RPINR18bits.U1RXR  = 16; //Pin RC0 connected to recepcion port of UART1.
    RPOR8bits.RP17R    = 3;  //Pin RC1 connected to transmision pin of UART1.
    
    //U1MODE REGISTER CONFIG
    U1MODEbits.UARTEN  = 0; //Unable UART before config.
    U1MODEbits.USIDL   = 0; //Continue operation in idle mode.
    U1MODEbits.IREN    = 0; //Infrared.
    U1MODEbits.RTSMD   = 1; //Flow control. No using RTS/CTS. RX/TX mode.
    U1MODEbits.UEN     = 0; //Only using TX and RX pin.
    U1MODEbits.WAKE    = 0; //Awake when idle mode and recive data.
    U1MODEbits.LPBACK  = 0; //Loopback desable.
    U1MODEbits.ABAUD   = 0; //Auto baud rate desable.
    
    U1MODEbits.URXINV  = 0; //Idle state = '1'.
    U1MODEbits.BRGH    = 1; //High-Speed Mode (1 bit = 4 clock cycles).
    U1MODEbits.PDSEL   = 0; //8 bits data lenght and null parity.
    U1MODEbits.STSEL   = 0; //1-bit Stop at the end of data frame (8N1).
    
    //U1STA REGISTER CONFIG
    U1STAbits.URXISEL0 = 0;
    U1STAbits.URXISEL1 = 0;
    U1STAbits.ADDEN    = 0;
    U1STAbits.UTXBRK   = 0; //Sync frame desable.
    U1STAbits.OERR     = 0; //Buffer is empty (No Overflow problems)
    U1STAbits.PERR     = 0;
    U1STAbits.FERR     = 0;
    U1STAbits.UTXEN    = 1; //Enable transmisor

    // Configuramos la velocidad de transmisión/recepcción de los datos
    U1BRG = baud;
    
    // Prioridades, flags e interrupciones correspondientes a la Uart
    IPC2bits.U1RXIP = 6;    // U1RX con nivel de prioridad 6 (7 es el maximo)
    IFS0bits.U1RXIF = 0;    // Reset Rx Interrupt flag
    IEC0bits.U1RXIE = 1;    // Enable Rx interrupts
    
    IPC3bits.U1TXIP = 5;    // U1TX con nivel de prioridad 6 (7 es el maximo)
    IFS0bits.U1TXIF = 0;    // Reset Tx Interrupt flag
    IEC0bits.U1TXIE = 0;    // Enable Tx interrupts
    
    U1MODEbits.UARTEN = 1;     // Uart habilitada por completo
}

void delay_ms(unsigned long time_ms){
    unsigned long i;
    for(i=0; i < time_ms*100; i++){
        asm("NOP");
    }
}

int main(void) {
    //Fosc = Fpri*M/(N1*N2) => 8MHz*2/(2*2) = 16/4 = 4;
    //Fcpu = Fosc/2 = 4MHz/2 = 2MHz;
    PLLFBD = 0;                     
    CLKDIVbits.PLLPOST = 0;         
    CLKDIVbits.PLLPRE  = 0;         
    while(OSCCONbits.LOCK != 1);
    
    AD1PCFGL = 0xFFFF;
    TRISAbits.TRISA0 = 0;   //Pin A0 -> D1 output (Green LED)
    TRISAbits.TRISA1 = 0;   //Pin A1 -> D2 output (Red LED)
    uart_config(baud_9600);   
    PR1 = 24999;
    
    while(1){
        if(!strcmp(((const char *)dataCMD_ISR), cmd0)) LED_GREEN = LED_ON_STATE;
        if(!strcmp(((const char *)dataCMD_ISR), cmd1)) LED_GREEN = LED_OFF_STATE;
        if(!strcmp(((const char *)dataCMD_ISR), cmd2)) LED_RED = LED_ON_STATE;
        if(!strcmp(((const char *)dataCMD_ISR), cmd3)) LED_RED = LED_OFF_STATE;
        if(!strcmp(((const char *)dataCMD_ISR), cmd4)) AllowPrint = 1;
        if(!strcmp(((const char *)dataCMD_ISR), cmd5)) AllowPrint = 0;
        else{}
        memset(dataCMD_ISR,'\0',sizeof(dataCMD_ISR)); //Resetamos el buffer a NULL
        data_count = 0; //Reseteamos data_count.
        
        if((AllowPrint)&&(BufferLoadDone)&&(U1STAbits.TRMT)){
            //T = 1 / 5Hz = 0,2 segundos = 200ms.
            //delay_ms = 10ms; U1_PrintRate_ISR = 20; 10ms * 20 times = 200ms.
            if(U1_PrintRate_ISR++ >= 20){ // 5Hz printing
                if(counter == 100){
                    counter = 0;
                }
                memset(txBuffer_ISR, '\0',sizeof(txBuffer_ISR));
                sprintf(txBuffer_ISR, "IMPRIMIENDO DATOS: %d \r\n", counter++);
                nextchar = 0;
                BufferLoadDone = 0;
                U1_PrintRate_ISR = 0;
                if(U1STAbits.UTXBF){
                    IFS0bits.U1TXIF = 0; 
                }
                asm("NOP");
                IEC0bits.U1TXIE = 1; 
            }
        }
        delay_ms(10);
    }
    return 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void){
    IFS0bits.T1IF = 0; //Reset Timer1 Interrupt
}

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void){
    dataCMD_ISR[data_count] = U1RXREG;
    data_count++;
    IFS0bits.U1RXIF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void){
    IEC0bits.U1TXIE = 0;                    //Disable UART1 Tx Interrupt
    if(!U1STAbits.UTXBF){                   //Mientras el buffer de transmisión NO se encuentre completo, continuar cargando ek buffer con más datos
        U1TXREG = txBuffer_ISR[nextchar++]; //Cargamos el buffer con un nuevo dato
        asm("NOP");
        if(U1STAbits.UTXBF){        //Si el buffer de transmisión se completó con el último dato incorporado al buffer, reseteanis el flag.
            IFS0bits.U1TXIF = 0;    //Clear UART1 Tx Interrupt Flag
        }
    }else{
        IFS0bits.U1TXIF = 0;        // Clear UART1 Tx Interrupt Flag
    }
    if(nextchar == strlen(txBuffer_ISR)){ // Si se ha finalizado la transmision de todos los caracteres --> Deshabilitar interrupcion y activar flags. strlen cuenta el numero de caracteres hasta encontrar NULL.
        BufferLoadDone = 1;
    }else{
        IEC0bits.U1TXIE = 1; //Enable UART1 Tx Interrupt
    }
}


 */

