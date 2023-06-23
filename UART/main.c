#include <xc.h>
#include <stdint.h>

// PIC16F1778 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover Mode (Internal/External Switchover Mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PPS1WAY = OFF    // Peripheral Pin Select one-way control (The PPSLOCK bit can be set and cleared repeatedly by software)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR)
#pragma config PLLEN = OFF      // Phase Lock Loop enable (4x PLL is enabled when software sets the SPLLEN bit)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

static uint8_t g_MainLoop1msTick = 0;
static uint8_t g_MainLoop_F      = 0;
static uint8_t g_LEDBlinkTimer   = 0;

void TMR2_ISR(void);

void Init_MainClock(void)
{
    // Set main clock = internal clock 32MHz(8MHz -> PLLx4)
    OSCCON = 0xF0;
    // Wait stable PLL
    while( PLLR != 1 ); 
}

void Init_PortDirection(void)
{
    // Set PortA input/output direction
    WPUA   = 0xFF;      // internal pullup 1: enable, 0: disable
    LATA   = 0x00;      // output latch
    ODCONA = 0x00;      // output drain control 0: push-pull, 1: open-drain
    TRISA  = 0xFF;      // port direction 0: output, 1: input
    ANSELA = 0x00;      // 0: digital IO, 1: analog IO
    // SLRCONA = default
    // ILVLA   = default
    
    // Set PortB input/output direction
    WPUB   = 0xFF;      // internal pullup 1: enable, 0: disable
    LATB   = 0x00;      // output latch
    ODCONB = 0x00;      // output drain control 0: push-pull, 1: open-drain
    TRISB  = 0xFF;      // port direction 0: output, 1: input
    ANSELB = 0x00;      // 0: digital IO, 1: analog IO
    // HIDRVB  = default
    // SLRCONB = default
    // ILVLB   = default
    
    // Set PortC input/output direction
    WPUC   = 0xFE;      // internal pullup 1: enable, 0: disable
    LATC   = 0x00;      // output latch
    ODCONC = 0x00;      // output drain control 0: push-pull, 1: open-drain
    TRISC  = 0xFE;      // port direction 0: output, 1: input
    ANSELC = 0x00;      // 0: digital IO, 1: analog IO
    // HIDRVB  = default
    // SLRCONB = default
    // ILVLB   = default
}

void Init_PPS(void)
{
    
}

void Init_Timer(void)
{
    // init system timer(1ms interval timer)
    T2CLKCON = 0x01;        // clock source = Fosc/4
    T2CON    = 0x60;        // prescaler = 1:64, postscaler 1:1
    T2HLT    = 0x00;        // software Freerun mode
    TMR2     = 0;           // Reset 0
    T2PR     = 125 - 1;     // compare value
    // T2RST = default(not use externel reset signal)
}

void Timer_Start(void)
{
    // Set Timer2 interrupt enable
    TMR2IF = 0;         // clear interrupt flag
    TMR2IE = 1;         // enable interrupt
    // Timer start
    T2CONbits.ON = 1;
}

void Init_OpAmp(void)
{
    
}

void EnableInterruptAtSystemWakeup()
{
    PEIE = 1;
    GIE  = 1;
}

void __interrupt() INTERRUPT_MainHandler(void)
{
    // interrupt handler
    if( INTCONbits.PEIE != 1 ){
        return;
    }
    
    if( TMR2IE == 1 && TMR2IF == 1 ){
        TMR2_ISR();
        TMR2IF = 0;
    }
}

void TMR2_ISR(void)
{
    ++g_MainLoop1msTick;
    if( g_MainLoop1msTick >= 10 ){
        g_MainLoop_F = 1;
        g_MainLoop1msTick = 0;
    }
}

void main(void) 
{
    Init_MainClock();
    Init_PortDirection();
    Init_PPS();
    Init_Timer();
    
    EnableInterruptAtSystemWakeup();
    Timer_Start();


    LATC = 0x01;
    while(1){
        if( g_MainLoop_F == 1 ){
            g_MainLoop_F = 0;
            
            ++g_LEDBlinkTimer;
            if( g_LEDBlinkTimer >= 50 ){
                LATC ^= 0x01;
                g_LEDBlinkTimer = 0;
            }
        }
    }
    
    return;
}
