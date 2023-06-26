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

#define UART_TX_BUFFER_SZ      ((uint8_t)64)
#define UART_TX_BUFFER_MASK    ((uint8_t)(UART_TX_BUFFER_SZ - 1))
static uint8_t g_UARTTxBuffer[UART_TX_BUFFER_SZ];
static uint8_t g_UARTTxBufferBegin = 0;
static uint8_t g_UARTTxBufferEnd = 0;

static const uint8_t sk_DecToCharTable[] = {
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F',
};

void TMR2_ISR(void);
int USART_IsWriteBufferFull(void);
void USART_PrintString( const char* str );

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
    WPUB   = 0xDF;      // internal pullup 1: enable, 0: disable
    LATB   = 0x00;      // output latch
    ODCONB = 0x00;      // output drain control 0: push-pull, 1: open-drain
    TRISB  = 0xDF;      // port direction 0: output, 1: input
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
    // Assign UART Tx/Rx port
    // Tx = RB5
    RB5PPS = 0b100100;
    // Rx = not use
    // RXPPS  = 0b001100;
    
    // lock PPS
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 1;
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

void Init_USART(void)
{
    // baudrate = 9600
    // 8bit, none parity
    // disable flow control
    // SEND ONLY
    BAUD1CON = 0x00;
    SP1BRGL = 207;
    SP1BRGH = 0;
    
    TX1STA = 0x04;
    // RC1STA = 0x80;       // not use receive
}

void USART_Start(void)
{
    RC1STAbits.SPEN = 1;
    TX1STAbits.TXEN = 1;
    //RC1STAbits.CREN = 1;
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

void USART_SendByteFromBuffer(void)
{
    // no data in write buffer
    if( g_UARTTxBufferBegin == g_UARTTxBufferEnd ){
        return;
    }
    // Wait until TSR be empty
    while( TRMT == 0 ) ;

    TX1REG = g_UARTTxBuffer[g_UARTTxBufferBegin];
    g_UARTTxBufferBegin = (g_UARTTxBufferBegin + 1) & UART_TX_BUFFER_MASK;
}

void USART_WriteByte( uint8_t ch )
{
    if( USART_IsWriteBufferFull() ){
        return;
    }

    g_UARTTxBuffer[g_UARTTxBufferEnd] = ch;
    g_UARTTxBufferEnd = (g_UARTTxBufferEnd + 1) & UART_TX_BUFFER_MASK;
}

int USART_IsWriteBufferFull(void)
{
    uint8_t next = (g_UARTTxBufferEnd + 1) & UART_TX_BUFFER_MASK;
    if( g_UARTTxBufferBegin == next ){
        return 1;
    }

    return 0;
}

void USART_PrintUInt16( uint16_t v, uint8_t dec )
{
    char c[6];
    int8_t ptr = 5;
    c[5] = '\0';

    do {
        --ptr;
        c[ptr] = sk_DecToCharTable[v % dec];
        v /= dec;
    } while( v > 0 && ptr > 0 );

    USART_PrintString( &c[ptr] );
}

void USART_PrintString( const char* str )
{
    while( *str != '\0' ){
        USART_WriteByte( *str );
        ++str;
    }
}

void main(void) 
{
    uint8_t t = 0;

    Init_MainClock();
    Init_PortDirection();
    Init_PPS();
    Init_Timer();
    Init_USART();

    EnableInterruptAtSystemWakeup();
    Timer_Start();
    USART_Start();

    LATC = 0x01;
    while(1){
        if( g_MainLoop_F == 1 ){
            g_MainLoop_F = 0;

            ++g_LEDBlinkTimer;
            if( g_LEDBlinkTimer >= 50 ){
                LATC ^= 0x01;
                g_LEDBlinkTimer = 0;

                USART_PrintUInt16( t, 10 );
                ++t;
            }
        }

        while( g_MainLoop_F == 0 ){
            USART_SendByteFromBuffer();
        }
    }
    
    return;
}
