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

#include "xc.h"
#include <stdio.h>

#define LED_GREEN LATAbits.LATA0
#define LED_RED   LATAbits.LATA1
#define LED_ON_state 1
#define LED_OFF_state 0

#define baud_9600 103

char TxBuffer[200];
int counter = 0;
int delayCounter = 0;

void uart_config(unsigned short baud){
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
    
    U1BRG = baud;
    U1MODEbits.UARTEN  = 1; //Enable UART after config.
}

void delay_ms(unsigned long time_ms){
    unsigned long i;
    for(i=0; i < time_ms*100; i++){
        asm("NOP");
    }
}
void sendChar(char c){
    while(U1STAbits.UTXBF);
    U1TXREG = c;
}

void sendString(char *s){
    while(*s != '\0'){
        sendChar(*(s++));
    }
}

void arrayInit(){
    int i;
    for(i = 0; i < 200; i++){
        TxBuffer[i] = '\0';
    }
}

int main(void) {
    //Fosc = Fpri*M/(N1*N2) => 8MHz*4/(2*2) = 32/4 = 8;
    //Fcpu = Fosc/2 = 8MHz/2 = 4MHz;
    PLLFBD = 2;
    CLKDIVbits.PLLPRE = 0;
    CLKDIVbits.PLLPOST = 0;
    while(OSCCONbits.LOCK != 1);
    
    //Control de pin para trabajar como analógico/digital
    AD1PCFGL = 0xFFFF;
    
    //Control de direccinalidad de los pines de mi proyecto
    TRISAbits.TRISA0 = 0;   //Pin A0 -> D1 output (Green LED)
    TRISAbits.TRISA1 = 0;   //Pin A1 -> D2 output (Red LED)

    uart_config(baud_9600);
    arrayInit();
    int pressH = 0;
    
    while(1){
        if(delayCounter++ == 1){
            sprintf(TxBuffer, "Luis %d \r \n", counter);
            sendString(TxBuffer);
            counter++;
            delayCounter = 0;
        }
        if(U1STAbits.URXDA){
            if(U1RXREG == 101){ // e
                LED_RED = LED_ON_state;
            }
            if(U1RXREG == 97){ // a
                LED_RED = LED_OFF_state;
            }
            if(U1RXREG == 104){ // h
                pressH++;
            }
            if(U1RXREG == 32){ //Spacebar
                counter = 0;
            }
        }
        if((pressH % 2) == 0){
            LED_GREEN = LED_OFF_state;
        }else{
            LED_GREEN = !PORTAbits.RA0;
        }
        
        delay_ms(250);
    }
    return 0;
}

