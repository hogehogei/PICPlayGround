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

// internal osc 32Mhz
#define _XTAL_FREQ 32000000

// ADC channel
#define ADC_CHS_AN14    ((uint8_t)14)
#define ADC_CHS_AN15    ((uint8_t)15)
#define ADC_CHS_AN16    ((uint8_t)16)

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
    WPUB   = 0xDC;      // internal pullup 1: enable, 0: disable
    LATB   = 0x00;      // output latch
    ODCONB = 0x00;      // output drain control 0: push-pull, 1: open-drain
    TRISB  = 0xDC;      // port direction 0: output, 1: input
    ANSELB = 0x03;      // 0: digital IO, 1: analog IO
    // HIDRVB  = default
    // SLRCONB = default
    // ILVLB   = default
    
    // Set PortC input/output direction
    //WPUC   = 0x02;    // internal pullup 1: enable, 0: disable
    WPUC   = 0x00;      // internal pullup 1: enable, 0: disable TEST
    LATC   = 0x00;      // output latch
    ODCONC = 0x00;      // output drain control 0: push-pull, 1: open-drain
    //TRISC  = 0xFE;    // port direction 0: output, 1: input
    TRISC  = 0xFC;      // port direction 0: output, 1: input TEST
    ANSELC = 0xFC;      // 0: digital IO, 1: analog IO
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
    
    // Assign COG output port
    RB0PPS = 0b000101;
    RB1PPS = 0b000110;
    
    // for PWM test
    RC1PPS = 0b011101;
    
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

void Init_ADC(void)
{
    // ADFM=1(10bit right justified)
    // clock = Fosc/64, adc reference voltage = vdd(+)/vss(-))
    ADCON1 = 0xE0;
    ADCON2 = 0x00;
    ADCON0bits.ADON = 1;
}

uint16_t Get_AnalogValue(uint8_t reg)
{
    uint16_t ret;
    uint8_t  t;
    
    // Select ADC channel
    ADCON0bits.CHS  = reg;
    // Wait acquisition time 5us
    __delay_us(5);

    // Start conversion, wait complete
    ADCON0bits.GO   = 1;
    while(ADCON0bits.GO == 1) ; 
    
    // Get result. 10bit right justified
    ret  = ADRESL;
    t    = ADRESH;
    ret |= ((uint16_t)t << 8);
    
    return ret;
}

void Init_OpAmp(void)
{
    OPA3ORS  = 0x00;
    OPA3NCHS = 0x00;
    OPA3PCHS = 0x00;
    OPA3CON  = 0x80;    // enable OPA3
}

void Init_PWM(void)
{
    PWM5CON    = 0x10;      // output porality = LOW
    PWM5CLKCON = 0x00;      // clock control = No Prescaler, FOSC
    PWM5LDCON  = 0x00;      // Load trigger select, LD5 trigger
    PWM5OFCON  = 0x00;
    // PWM period(10KHz)
    PWM5PRH    = 0x0C;      // period (3200 - 1)
    PWM5PRL    = 0x7F;      // period (3200 - 1)
    // Duty cycle
    PWM5DCH    = 0x00;      // Duty cycle
    PWM5DCL    = 0x00;      // Duty cycle
    PWM5PHH    = 0x00;      // phase 0
    PWM5PHL    = 0x00;      // phase 0
    PWM5OFH    = 0x00;      // offset 0
    PWM5OFL    = 0x00;      // offset 0
    PWM5TMRH   = 0x00;
    PWM5TMRL   = 0x00;
    
    // no use mirror fucntion
    //PWMEN = 0x00;
    //PWMLD = 0x00;
    //PWMOUT = 0x00;
}

void Init_COG(void)
{
    // Half-Bridge mode, COGclock = FOSC
    COG1CON0 = 0x0C;
    COG1CON1 = 0x00;         // output polarity, Active level is low
    COG1RIS0 = 0x00;         // select PWM5 output
    COG1RIS1 = 0x02;         // select PWM5 output
    COG1RSIM0 = 0x00;        // no use phase delay
    COG1RSIM1 = 0x00;        // no use phase delay
    COG1FIS0 = 0x00;         // select PWM5 output         
    COG1FIS1 = 0x02;         // select PWM5 output
    COG1FSIM0 = 0x00;        // no use phase delay
    COG1FSIM1 = 0x00;        // no use phase delay
    COG1ASD0  = 0x7C;        // auto restart enable, COGA, COGB = output '1' logic when shutdown
    COG1ASD1  = 0x00;        // no use auto shutdown function
    //COG1STR = 0x00;
    
    // Dead time setting
    COG1DBR = 5;            // 1/32Mhz = 31.25ns per count after riging edge
    COG1DBF = 5;            // 1/32Mhz = 31.25ns per count after falling edge
    //COG1BLKR = 0x00;
    //COG1BLKF = 0x00;
    //COG1PHR = 0x00;
    //COG1PHF = 0x00;
}

void Start_SyncRectification(void)
{
    PWM5LDCONbits.LDA = 1;
    PWM5CONbits.EN = 1;
    COG1CON0bits.EN = 1;
}

uint16_t Get_Duty(void)
{
    uint16_t duty = 0;
    duty  = PWM5PRH;
    duty <<= 8;
    duty |= PWM5PRL;
    
    return duty;
}

void Set_Duty( uint16_t duty )
{
    uint8_t hi = duty >> 8;
    PWM5DCH = hi;
    PWM5DCL = (uint8_t)duty;
    
    PWM5LDCONbits.LDA = 1;
}

void EnableInterruptAtSystemWakeup(void)
{
    PEIE = 1;
    GIE  = 1;
}

void __interrupt() INTERRUPT_MainHandler(void)
{
    // Interrupt handler
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
    // No data in write buffer
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
    uint16_t advalue;
    
    Init_MainClock();
    Init_PortDirection();
    Init_PPS();
    Init_Timer();
    Init_USART();
    Init_ADC();
    Init_OpAmp();
    Init_PWM();
    Init_COG();
    
    EnableInterruptAtSystemWakeup();
    Timer_Start();
    USART_Start();
    Start_SyncRectification();
    
    Set_Duty(0);

    LATC = 0x01;
    while(1){
        if( g_MainLoop_F == 1 ){
            g_MainLoop_F = 0;

            ++g_LEDBlinkTimer;
            if( g_LEDBlinkTimer >= 50 ){
                LATC ^= 0x01;
                g_LEDBlinkTimer = 0;

                advalue = Get_AnalogValue( ADC_CHS_AN14 );
                USART_PrintUInt16( advalue, 10 );
                advalue = Get_AnalogValue( ADC_CHS_AN15 );
                USART_PrintUInt16( advalue, 10 );
                advalue = Get_AnalogValue( ADC_CHS_AN16 );
                USART_PrintUInt16( advalue, 10 );
            }
        }

        while( g_MainLoop_F == 0 ){
            USART_SendByteFromBuffer();
        }
    }
    
    return;
}
