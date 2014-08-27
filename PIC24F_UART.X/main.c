/* 
 * File:   main.c
 * Author: eugene
 *
 * Created on August 26, 2014, 11:12 PM
 */

// #include <stdio.h>
// #include <stdlib.h>
#include <xc.h>

/* MCU configuration bits */
_CONFIG1
(
   JTAGEN_OFF     // JTAG Disabled
   & GCP_OFF      // Code Protect Disabled
   & GWRP_OFF     // Write Protect Disabled
   & FWDTEN_OFF   // Watchdog Timer Disabled
   & WINDIS_OFF   // Windowed Watchdog Timer Disabled
)
CONFIG2
(
   IESO_ON          // Two Speed Start-up
   //& PLL96MHZ_ON    // 96MHz PLL Enabled
   & PLLDIV_DIV3    //input clock divided by 3 to get 4MHz
   & FNOSC_PRIPLL   //FNOSC_FRC //FNOSC_PRIPLL   //primary clock with PLL
   & POSCMOD_HS     //POSCMOD_NONE //POSCMOD_HS
)
_CONFIG3
(
  //SOSCSEL_IO
  SOSCSEL_EC
)
//_CONFIG4
//(
//  RTCOSC_LPRC           //RTCC Source
//)

/* Timer1 period for 1 ms with FOSC = 16 MHz */
// #define TMR1_PERIOD 0x0FA0
#define ON_TIME  0x0200
#define OFF_TIME  0x0400
#define LED_ON()                (LATD |= (1U << 2))
#define LED_OFF()               (LATD &= ~(1U << 2))
#define LED_TOGGLE()            (LATD ^= (1U << 2))


#define INBUFSIZE 40
volatile char inbuf[INBUFSIZE];
volatile int intail;
volatile int inhead;

/*
 *
 *
 */
void _ISR _U1RXInterrupt()
{
    while(U1STAbits.URXDA)
    {
        // put char into buffer
        // c = U1RXREG;
    }
    IFS0bits.U1RXIF = 0;

}
void _ISR _U1TXInterrupt()
{
    while(U1STAbits.UTXBF == 1)
    {
        // send data
        // U1TXREG = c;
    }
    IFS0bits.U1TXIF = 0;

}
void InitMCU()
{
       T2CON = 0x0000U;  /* Use Internal Osc (Fcy), 16 bit mode, prescaler = 1 */
     // PIC24FJ256GB206 doesn't have port A
        TRISB = 0;
        TRISC = 0;
        TRISD = 0;
        TRISE = 0;
        TRISF = 0;
        TRISG = 0;
        //ODCBbits.ODB9 = 1;    //open-drain
        PORTB = 0;
        //VBUS_ON();
        //AD1PCFGL = 0xffff;    //analog off

    // PIC24FJ256GB206 turns off analog differently
        ANSB = 0;
        ANSC = 0;
        ANSD = 0;
        //ANSE = 0;
        ANSF = 0;
        ANSG = 0;
//AD1PCFGbits.PCFG10 = 1;

        //Set serial console pins
        //PPSUnLock;
        //PPSOutput(PPS_RP5, PPS_U2TX);
        //PPSInput(PPS_U2RX, PPS_RP6);
        //PPSLock;

}
void InitUART()
{
    RPINR18bits.U1RXR = 17; // Map RX to RP6
    RPOR2bits.RP4R = X;    // Map TX to RP5 ??
    IPC3bits.U1TXIP2 = 1;  // Interrupt priority
    IPC3bits.U1TXIP1 = 0;
    IPC3bits.U1TXIP0 = 0;
    IPC2bits.U1RXIP2 = 1;
    IPC2bits.U1RXIP1 = 0;
    IPC2bits.U1RXIP0 = 0;
    U1MODE = 0;
    U1STA = 0;
    U1MODEbits.WAKE = 1;    /* wake up from sleep*/
    // U1MODEbits.ABAUD = 1;   /* Automatic baud rate, requires 55h sync*/
    U1BRG = ((16000000/9600)/16)-1 /* Baud rate */
    // U1MODEbits.LPBACK = 1; /* Loopback */
    // U1MODEbits.BRGH - 1; /*High baud rate*/
    U1MODEbits.PDSEL = 0; /* 11 - 9 bit, none
                           * 10 - 8 bit, odd
                           * 01 - 8 bit, even
                           * 00 - 8 bit, none
                           */
    U1MODEbits.STSEL = 1; /* Stop bit set */
    U1MODEbits.UARTEN = 1;  /* Enable UART */
    U1STAbits.UTXEN = 1 ; /* Enable TX, must be after uarten */
    IFS0bits.U1RXIF = 0;  /* Clean interrupt flag */
    IFS0bits.U1TXIF = 0;
    IEC0bits.U1RXIE = 1; /* Enable TX interrupts */
    IEC0bits.U1RXIE = 1; /* Enable RX interrupts*/
}
/*
 * 
 */
int main(int argc, char** argv) {
    inbuf[INBUFSIZE - 1] = '\0';
    InitMCU();
    InitUART();
    while(1) {
        Idle();
    }
    return (0);
}

