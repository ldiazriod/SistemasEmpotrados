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

#define LED_GREEN LATAbits.LATA1
#define LED_RED   LATAbits.LATA0
#define LED_ON_STATE 1
#define LED_OFF_STATE 0
#define baud_9600 51

char dataCMD_ISR[50];               //UART ISR in
char txBuffer[200];                 //UART out
char txBuffer_ISR[200];             //UART ISR out
double tiempo_real_IC1      = 0.0; //Final time calc result
double tiempo_real_IC2      = 0.0; 
unsigned int nextChar       = 0;   //Index tcBuffer_ISR;
unsigned int time_pulso_IC1 = 0;
unsigned int time_pulso_IC2 = 0;
unsigned int rise_pulso_IC1 = 0;
unsigned int rise_pulso_IC2 = 0;
unsigned int duty_OC1       = 4000;
unsigned int duty_OC2       = 4000;
unsigned int pulso1         = 0;
unsigned int pulso2         = 0;
unsigned int flag           = 0;
unsigned int flagMemory     = 0;
unsigned int testResult = 0;
unsigned char memoryBuffer[10] = {0x10, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};
unsigned char memoryAddress[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
unsigned char badMemory_data[10] = {0x10, 0x11, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};
unsigned char memory_data[10];

void uart_config (unsigned int baud){    
    //UART PORT/PIN COMMUNICATION ASSIGMENT
    TRISBbits.TRISB2   = 1;  //Pin RB2 input (digital).
    RPINR18bits.U1RXR  = 2; //Pin RB2 connected to recepcion port of UART1.
    RPOR1bits.RP3R     = 3;  //Pin RB3 connected to transmision pin of UART1.
    
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
    
    IPC2bits.U1RXIP = 6;    // U1RX con nivel de prioridad 6 (7 es el maximo)
    IFS0bits.U1RXIF = 0;    // Reset Rx Interrupt flag
    IEC0bits.U1RXIE = 1;    // Enable Rx interrupts
    
    IPC3bits.U1TXIP = 5;    // U1TX con nivel de prioridad 6 (7 es el maximo)
    IFS0bits.U1TXIF = 0;    // Reset Tx Interrupt flag
    IEC0bits.U1TXIE = 0;    // Enable Tx interrupts
    
    U1MODEbits.UARTEN = 1;     // Uart habilitada por completo
}

void spi_config(void){
    TRISCbits.TRISC0 = 1; //MISO asignado a RC0 (PIN 25)
    TRISCbits.TRISC1 = 0; //MOSI asignado a RC1 (PIN 26)
    TRISCbits.TRISC2 = 0; //SCK  asignado a RC2 (PIN 27)
    TRISCbits.TRISC3 = 0; //CS   asignado a RC3 (PIN 36)
    
    LATCbits.LATC1 = 0;
    
    RPINR20bits.SDI1R = 16; //RCO trabajando como MISO (entrada);
    RPOR8bits.RP17R = 7; //(00111); RC1 es MOSI (salida);
    RPOR9bits.RP18R = 8; //(0100); RC2 es SCK (salida);
    
    SPI1STATbits.SPIEN = 0;
    SPI1STATbits.SPISIDL = 0;
    SPI1STATbits.SPIROV = 0;
    SPI1STATbits.SPITBF = 0;
    SPI1STATbits.SPIRBF = 0;
    
    SPI1CON1bits.DISSCK = 0;
    SPI1CON1bits.DISSDO = 0;
    SPI1CON1bits.MODE16 = 0;
    SPI1CON1bits.SMP = 0;
    
    SPI1CON1bits.CKP = 0;
    SPI1CON1bits.CKE = 0;
    
    SPI1CON1bits.SSEN = 0;
    SPI1CON1bits.MSTEN = 1;
    
    SPI1CON1bits.PPRE = 1;
    SPI1CON1bits.SPRE = 6;
    
    SPI1CON2bits.FRMEN = 0;
    SPI1CON2bits.SPIFSD = 0;
    SPI1CON2bits.FRMPOL = 0;
    SPI1CON2bits.FRMDLY = 0;
    
    IFS0bits.SPI1IF = 0;
    IFS0bits.SPI1EIF = 0;
    IEC0bits.SPI1IE = 0;
    IEC0bits.SPI1EIE = 0;
    IPC2bits.SPI1IP = 6;
    IPC2bits.SPI1EIP = 6;
    
    LATCbits.LATC3 = 1;
    SPI1STATbits.SPIEN = 1;
}

void output_compare_config(void){
    /* Configuración de pines OC */
    TRISCbits.TRISC5   = 0; //Pin configurado como salida
    TRISCbits.TRISC6   = 0; 
    RPOR10bits.RP21R   = 0x12; //OC1 conectado al pin RP21(RC0)
    RPOR11bits.RP22R   = 0x13; //OC2 conectado al pin RP22 (RC1)
    
    /* Configurar modulo OC */
    OC1CONbits.OCM    = 0; //Deshabilitar PWM
    OC1CONbits.OCTSEL = 0; //Trabajamos con el Timer2
    //OC1CONbits.OCM    = 6; //Modo PWM sin pin de fallo
    /*DutyOC1 = (Ton/T) = (2/5)*100 = 40%*/
    OC1R  = 4000; 
    OC1RS = 4000; 
    
    OC2CONbits.OCM    = 0; //Modulo OC2 inicialmente deshabilitado
    OC2CONbits.OCTSEL = 0; //Trabajamos con el Timer2
    //OC2CONbits.OCM    = 6; //Modo PWM sin pin de fallo
    OC2R  = 7000; //Solo registro de lectura
    OC2RS = 7000; //Ciclo de trabajo a 1ms
    
    //Configurar timer2
    T2CONbits.TON     = 0; //Deshabilitar Timer2
    T2CONbits.T32     = 0; //Timer2 modo de 16-bits de resolución-
    T2CONbits.TGATE   = 0; //Gated Timer mode deshabilitado
    T2CONbits.TCKPS   = 0; //Prescaler = 1;
    T2CONbits.TCS     = 0; //Seleccionar clock interno
    
    //Registro interrupcion
    IPC1bits.T2IP = 5; //Priotidad de nivel 5
    IFS0bits.T2IF = 0; //Reset flag
    IEC0bits.T2IE = 0; //Deshabilitar interrupción
    
    PR2   = 10000; //Periodo de 5ms, basado en Freq = 2MHz
    /*tiempo = PRESCALER*(reg + 1)/Fbus -> reg = ((tiempo * fbus)/PRESCALER) - 1 = ((0,005 * 4*10^6) / 1) - 1 = 10000. */
    
    TMR2 = 0; //Valor inicial del contador del Timer
    OC1CONbits.OCM = 6; //Mod PWM
    OC2CONbits.OCM = 6; //Modo PWM
    T2CONbits.TON = 1; //Activar timer2
}

void input_capture_config(void){
    TRISCbits.TRISC8 = 1;   //Pin configurado como entrada
    TRISCbits.TRISC9 = 1;   
    RPINR7bits.IC1R = 21;   // IC1 es el pin RC8 (RP24)
    RPINR7bits.IC2R = 22;   // IC2 es el pin RC9 (RP25)
    
    //Configurar IC1 
    IC1CONbits.ICM    = 0;  //IC deshabilitado
    IC1CONbits.ICSIDL = 0;  
    IC1CONbits.ICTMR  = 0;  //Timer3 seleccionado como base de tiempos para IC
    IC1CONbits.ICI    = 0;  // salta en cada evento
    IFS0bits.IC1IF = 0;
    IEC0bits.IC1IE = 1;  // Enable IC1 interrupts
    
    //Configurar IC2
    IC2CONbits.ICM    = 0;
    IC2CONbits.ICSIDL = 0;  // el IC se detiene si la CPU entra en modo reposo
    IC2CONbits.ICTMR  = 0;  // Trabajamos con Timer 3
    IC2CONbits.ICI    = 0;  // salta en cada evento
    IFS0bits.IC2IF = 0;
    IEC0bits.IC2IE = 1;  // Enable IC2 interrupts
    
    //Configuramos Timer3
    T3CONbits.TSIDL   = 0;
    T3CONbits.TON     = 0;
    T3CONbits.TGATE   = 0;
    T3CONbits.TCKPS   = 0; //Prescaler mas bajo = 1.
    T3CONbits.TCS     = 0;
    PR3 = 10000;
    
    // Activar los modulos IC1, IC2 y Timer3
    T3CONbits.TON     = 1;  //Activar Timer3
    IC1CONbits.ICM    = 3;  //Activar modulo OC1 por pin RC2
    IC2CONbits.ICM    = 3;  //Activar modulo OC2 por pin RC3 
}

unsigned char spi_write(unsigned char data){
    while(SPI1STATbits.SPITBF);
    SPI1BUF = data;
    while(!SPI1STATbits.SPIRBF);
    return SPI1BUF;
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
    delay_ms(10);
    spi_config();
    delay_ms(10);
    output_compare_config();
    delay_ms(10);
    input_capture_config();
    delay_ms(10);
    
    sprintf(txBuffer, "Inicializando EEPROM... \r\n");
    sendString(txBuffer);
    
    unsigned char j;
    for(j=0; j < 10; j++){
        LATCbits.LATC3 = 0;
        delay_ms(5);
        spi_write(0x06); //Enable write operations;
        delay_ms(5);
        LATCbits.LATC3 = 1;
        LATCbits.LATC3 = 0;
        delay_ms(5);
        spi_write(0x02); //Write data instrucction;
        spi_write(0x00); //Address MSB 8 bits;
        spi_write(memoryAddress[j]); //Address LSB 8 bits;
        spi_write(memoryBuffer[j]); //Write data on Address 
        delay_ms(5);
        LATCbits.LATC3 = 1;
        delay_ms(1000);
    }
    sprintf(txBuffer, "EEPROM inicializada \r\n");
    sendString(txBuffer);
    flag = 1;
    while(1){
        if(flag){
            tiempo_real_IC1 = 1.0*((double)time_pulso_IC1)/2000.0;
            tiempo_real_IC2 = 1.0*((double)time_pulso_IC2)/2000.0;
        
            sprintf(txBuffer, "TIME_IC1: %05.3fms \t TIME_IC2: %05.3fms \r\n", tiempo_real_IC1, tiempo_real_IC2);
            sendString(txBuffer);
            delay_ms(100);
            flag = 0;
        }
        if(flagMemory){
            flag = 0;
            int i;
            for(i=0; i < 10; i++){
                LATCbits.LATC3 = 0;
                delay_ms(5);
                spi_write(0x03);//Read data instruction.
        
                spi_write(0x00); //MSB
                spi_write(memoryAddress[i]); //LSB
                memory_data[i] = spi_write(0x00);
                sprintf(txBuffer, "Memory Value: %02X at address %04X, Expected: %02X \r\n", memory_data[i], memoryAddress[i], memoryBuffer[i]);
                sendString(txBuffer);
                delay_ms(5);
                LATCbits.LATC3 = 1;
                delay_ms(1000);
            }
            for(i=0; i < 10; i++){
                if(memory_data[i] != memoryBuffer[i]){
                    LED_RED = LED_ON_STATE;
                    testResult = 1;
                }
            }
            if(testResult == 0){
                LED_GREEN = LED_ON_STATE;
            }
            flagMemory=0;
        }
    }
    return 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _IC1Interrupt(void){ 
    if(pulso1 == 0){
        rise_pulso_IC1 = IC1BUF;
        IC1CONbits.ICM = 2; // Capture next falling edge
        pulso1 = 1;
    }else{
        time_pulso_IC1 = IC1BUF - rise_pulso_IC1;
        IC1CONbits.ICM = 3; // Capture next rising edge
        pulso1 = 0;
    }
    IFS0bits.IC1IF = 0;          // Reset IC1 Interrupt
}

void __attribute__((__interrupt__, no_auto_psv)) _IC2Interrupt(void){ 
    if(pulso2 == 0){
        rise_pulso_IC2 = IC2BUF;
        IC2CONbits.ICM = 2; // Capture next falling edge
        pulso2 = 1;
    }else{
        time_pulso_IC2 = IC2BUF - rise_pulso_IC2;
        IC2CONbits.ICM = 3; // Capture next rising edge
        pulso2 = 0;
    }
        
    IFS0bits.IC2IF = 0;          // Reset IC2 Interrupt
}


void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void){    
    IFS0bits.T1IF = 0;          // Reset Timer1 Interrupt
}


void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void){ 
    dataCMD_ISR[0] = U1RXREG;
    if(dataCMD_ISR[0] == '+') duty_OC2 = duty_OC2 + 150;
    if(dataCMD_ISR[0] == '-') duty_OC2 = duty_OC2 - 150;
    if(dataCMD_ISR[0] == 'p') duty_OC1 = duty_OC1 + 150;
    if(dataCMD_ISR[0] == 'm') duty_OC1 = duty_OC1 - 150;
    OC2RS = duty_OC2;
    OC1RS = duty_OC1;
    flag=1;
    if(dataCMD_ISR[0] == 'i'){
        flag = 0;
        flagMemory = 1;
    }
    
    IFS0bits.U1RXIF = 0;        // Reset Rx Interrupt
}
   
void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void){     // Actualmente configurado para U1STAbits.UTXISEL = 0 (Solo se debe resetear el flag cuando la condicion de UTXISEL no sea cierta.)  
    IEC0bits.U1TXIE = 0;                  // Disable UART1 Tx Interrupt  
    
    if(!U1STAbits.UTXBF){ // Mientras el buffer de transmisión NO se encuentre completo, continuar cargando el buffer con más datos.
        U1TXREG = txBuffer_ISR[nextChar++];  // Cargar el buffer con un nuevo dato.
        asm ("nop");
        if(U1STAbits.UTXBF){ // Si el buffer de transmision se completó con el último dato incorporado al buffer, procedemos a resetear el flag. (Este método se usa para UTXISEL = 0)
            IFS0bits.U1TXIF = 0;          // Clear UART1 Tx Interrupt Flag  
        }
    }else{
        IFS0bits.U1TXIF = 0; // Clear UART1 Tx Interrupt Flag
    }             
          
    if(!(nextChar == strlen(txBuffer_ISR))){
        // Si se ha finalizado la transmision de todos los caracteres --> Deshabilitar interrupcion y activar flags. strlen cuenta el numero de caracteres hasta encontrar NULL.
        IEC0bits.U1TXIE = 1;             // Enable UART1 Tx Interrupt   
    }   
}
