#include <xc.h>
#include <stdint.h>

static uint8_t g_MainLoop1msTick = 0;
static uint8_t g_MainLoop_F      = 0;
static uint8_t g_LEDBlinkTimer   = 0;

void TMR3_ISR(void);


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
    // T2RST = default(not use externel reset signal)
    
    // Set Timer2 interrupt enable
    TMR2IE = 1;
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
    if( INTCONbits.PEIE != 1){
        return;
    }
    
    if( PIE1bits.TMR2IE == 1 && PIR1bits.TMR2IF == 1){
        TMR3_ISR();
    }
}

void TMR3_ISR(void)
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
    
    while(1){
        if( g_MainLoop_F == 1 ){
            g_MainLoop_F = 0;
            
            if( g_LEDBlinkTimer >= 50 ){
                LATA ^= 0x01;
            }
        }
    }
    
    return;
}
