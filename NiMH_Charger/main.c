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
#pragma config BORV = HI        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), hi trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// internal osc 32Mhz
#define _XTAL_FREQ 32000000

#define FALSE       ((uint8_t)0)
#define TRUE        ((uint8_t)1)

// LED define
#define LED_ON      ((uint8_t)1)
#define LED_OFF     ((uint8_t)0)

// ADC channel
#define ADC_CHS_AN14    ((uint8_t)14)
#define ADC_CHS_AN15    ((uint8_t)15)
#define ADC_CHS_AN16    ((uint8_t)16)

#define UART_TX_BUFFER_SZ      ((uint8_t)64)
#define UART_TX_BUFFER_MASK    ((uint8_t)(UART_TX_BUFFER_SZ - 1))
static uint8_t g_UARTTxBuffer[UART_TX_BUFFER_SZ];
static uint8_t g_UARTTxBufferBegin = 0;
static uint8_t g_UARTTxBufferEnd = 0;

#define SENSE_PANEL_VOLTAGE_OVERSAMPLE  ((uint8_t)4)
#define CHECK_PANEL_OPEN_VOLT_CHECK_MAX ((uint8_t)8)
#define CHECK_PANEL_OPEN_VOLT_DIFF_MV   ((uint16_t)10)
static uint8_t g_MainLoop1msTick = 0;
static uint8_t g_MainLoop_F      = 0;

// Charge sequence
#define PWM_WIDTH_MAX       ((uint16_t)1600)
#define MPPT_CHARGING       ((uint8_t)0)
#define MPPT_FULLCHARGE     ((uint8_t)1)

// Charge finish voltage
#define BATT_FULLCHG_MILLV      ((uint16_t)1420)    // fullcharge    = 1420mv
#define BATT_FULLCHG_CV_MILLV   ((uint16_t)1450)    // fullcharge cv = 1450mv
#define BATT_CHGFIN_CURR        ((uint16_t)50)      // chgfin current
#define BATT_OVERVOLTAGE        ((uint16_t)1480)    // overvoltage   = 1480mv

// Failure protection

static uint8_t g_MPPT_Sequence = MPPT_CHARGING;
// Full charge timer
static uint8_t g_MPPT_FullCharge_Init_F = FALSE;
static uint16_t g_LED_BattFullChargeTimer = 0;

static uint8_t g_MPPT_CheckVoltCnt = 0;
static uint16_t g_MPPT_OpenVolt = 0;
static uint16_t g_MPPT_TargetVolt = 0;
static uint16_t g_MPPT_PWMWidth = 0;
static uint16_t g_MPPT_BattVolt = 0;
static uint16_t g_MPPT_BattCurr = 0;
static uint16_t g_MPPT_BattFullChargingTimer = 0;

static uint16_t g_BattCurrAdValue = 0;
static uint16_t g_Debug_AdTimer = 0;
static uint16_t gAdvalueDebug = 0;

#define SENSE_BATTERY_VOLTAGE_OVERSAMPLE    ((uint8_t)16)
#define SENSE_BATTERY_CURRENT_OVERSAMPLE    ((uint8_t)4)

static const uint8_t sk_DecToCharTable[] = {
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F',
};

static const uint16_t sk_ChgCurrLerp[] = {
    // advalue, current[mA]
    140, 100,
    380, 400,
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
    WPUB   = 0xD8;      // internal pullup 1: enable, 0: disable
    LATB   = 0x00;      // output latch
    ODCONB = 0x00;      // output drain control 0: push-pull, 1: open-drain
    TRISB  = 0xD8;      // port direction 0: output, 1: input
    ANSELB = 0x00;      // 0: digital IO, 1: analog IO
    // HIDRVB  = default
    // SLRCONB = default
    // ILVLB   = default
    
    // Set PortC input/output direction
    WPUC   = 0x03;      // internal pullup 1: enable, 0: disable
    LATC   = 0x00;      // output latch
    ODCONC = 0x00;      // output drain control 0: push-pull, 1: open-drain
    TRISC  = 0xFC;      // port direction 0: output, 1: input
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
    RB2PPS = 0b000101;  // COG1A HiSide
    RB1PPS = 0b000110;  // COG1B LoSide
    
    // lock PPS
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 1;
}

void Init_Timer2(void)
{
    // init system timer(1ms interval timer)
    T2CLKCON = 0x01;        // clock source = Fosc/4
    T2CON    = 0x60;        // prescaler = 1:64, postscaler 1:1
    T2HLT    = 0x00;        // software Freerun mode
    TMR2     = 0;           // Reset 0
    T2PR     = 125 - 1;     // compare value
    // T2RST = default(not use externel reset signal)
}

void Timer2_Start(void)
{
    // Set Timer2 interrupt enable
    TMR2IF = 0;         // clear interrupt flag
    TMR2IE = 1;         // enable interrupt
    // Timer start
    T2CONbits.ON = 1;
}

void Init_Timer4(void)
{
    // Half-bridge buck converter, when discontinuous-mode,
    // for protect reverse current from battery.
    // Force shutdown LO side FET when PWM duty is narrow.
    // (start PWM5 hardware trigger, after timer period, COG output force shutdown.)
    T4CLKCON = 0x01;        // clock source = Fosc/4
    T4CON    = 0x20;        // prescaler = 1:4, postscaler 1:1
    T4HLT    = 0x07;        // 0b00000111 free-running-period, High level reset
    TMR4     = 0;           // Reset 0
    T4PR     = 0;
    T4RST    = 0b00001101;  // reset signal = PWM5 output
}

void Timer4_Start(void)
{
    // Set Timer4 interrupt disable
    //TMR4IF = 0;         // clear interrupt flag
    //TMR4IE = 1;         // enable interrupt
    // Timer start
    T4CONbits.ON = 1;
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
    PWM5CON    = 0x00;      // output porality = HI
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
#if 1
    // Half-Bridge mode, COGclock = FOSC
    COG1CON0 = 0x0C;
    COG1CON1 = 0x00;         // output polarity, Active level below
                             // PWM出力波形はハイサイド=HiActive, ローサイド=HiActive 実機で確認すること
    COG1RIS0 = 0x00;         // select PWM5 output
    COG1RIS1 = 0x02;         // select PWM5 output
    COG1RSIM0 = 0x00;        // no use phase delay
    COG1RSIM1 = 0x00;        // no use phase delay
    COG1FIS0 = 0x00;         // select PWM5 output         
    COG1FIS1 = 0x02;         // select PWM5 output
    COG1FSIM0 = 0x00;        // no use phase delay
    COG1FSIM1 = 0x00;        // no use phase delay
    COG1ASD0  = 0x68;        // shutdown enable, auto restart enable, COGA, COGB = output '0' logic when shutdown
    COG1ASD1  = 0x80;        // shutdown function = Timer4
    //COG1STR = 0x00;
    
    // Dead time setting
    COG1DBR = 10;            // 1/32Mhz = 31.25ns per count after riging edge
    COG1DBF = 10;            // 1/32Mhz = 31.25ns per count after falling edge
    //COG1BLKR = 0x00;
    //COG1BLKF = 0x00;
    //COG1PHR = 0x00;
    //COG1PHF = 0x00;
#endif
}

void Set_ChargeLED( uint8_t on_off )
{
    if( on_off == LED_ON ){
        LATBbits.LATB0 = 1; // LO active
    }
    else {
        LATBbits.LATB0 = 0;
    }
}

void Set_ChargeLED_Invert(void)
{
    uint8_t b = LATBbits.LATB0;
    b ^= 0b00000001;
    LATBbits.LATB0 = b;
}

void Start_SyncRectification(void)
{
    // 1. set auto shutdown loside FET for protect reverse current from battery.
    Timer4_Start();
    
    // 2. start PWM
    PWM5LDCONbits.LDA = 1;
    PWM5CONbits.EN = 1;
    
    // 3. start COG
    COG1ASD0bits.ASE = 0;
    COG1CON0bits.LD = 1;
    COG1CON0bits.EN = 1;
}

void Set_Duty( uint16_t duty )
{
    uint8_t hi = duty >> 8;
    PWM5DCH = hi;
    PWM5DCL = (uint8_t)duty;

    // Set LOside shutdown timer
    // T4timer is prescaled 1:16, PWM clock is prescaled 1:1
    // so set period value is duty/16
    T4PR = (duty / 16) + 1;  
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

/**
 * @brief    Get solarpanel voltage.
 * @return   voltage [mv] 
 */
uint16_t MPPT_SensePanelVoltage(void)
{
    int8_t i = 0;
    uint16_t advalue = 0;
    
    for( i = 0; i < SENSE_PANEL_VOLTAGE_OVERSAMPLE; ++i ){
        advalue += Get_AnalogValue( ADC_CHS_AN14 );
    }
    
    // reference voltage = 3.3V(VCC), voltage divider 1:2
    return (uint16_t)((uint32_t)advalue * 9900 / 1024 / SENSE_PANEL_VOLTAGE_OVERSAMPLE);
}

uint16_t DiffAbs( uint16_t a, uint16_t b )
{
    return a > b ? (a - b) : (b - a);
}

int32_t Lerp( int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t x )
{
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}

void MPPT_SensePanelOpenVolt(void)
{
    uint16_t before_volt;
    uint16_t open_volt;
    uint16_t diff_volt;
    int8_t i;
   
    before_volt = MPPT_SensePanelVoltage();
    for( i = 0; i < CHECK_PANEL_OPEN_VOLT_CHECK_MAX; ++i ){
        open_volt = MPPT_SensePanelVoltage();
        diff_volt = DiffAbs( before_volt, open_volt );
        if( diff_volt < CHECK_PANEL_OPEN_VOLT_DIFF_MV ){
            break;
        }
        __delay_us(50);
    }
    
    g_MPPT_OpenVolt   = open_volt;
    g_MPPT_TargetVolt = (uint16_t)((uint32_t)open_volt * 76 / 100);
}

void MPPT_SenseBatteryVoltage(void)
{
    int8_t i;
    uint16_t advalue = 0;
    
    for( i = 0; i < SENSE_BATTERY_VOLTAGE_OVERSAMPLE; ++i ){
        advalue += Get_AnalogValue( ADC_CHS_AN15 );
    }

    // reference voltage = 3.3V(VCC)
    g_MPPT_BattVolt = (uint16_t)((uint32_t)advalue * 3300 / 1024 / SENSE_BATTERY_VOLTAGE_OVERSAMPLE);
}

void MPPT_SenseVoltage(void)
{
    ++g_MPPT_CheckVoltCnt;
    // sense every 100ms
    if( g_MPPT_CheckVoltCnt >= 10 ){
        g_MPPT_CheckVoltCnt = 0;
        
        Set_Duty(0);
        MPPT_SensePanelOpenVolt();
        MPPT_SenseBatteryVoltage();
    }
}

void MPPT_SenseBatteryCurrent(void)
{
    int8_t i;
    uint16_t advalue = 0;
    
    for( i = 0; i < SENSE_BATTERY_CURRENT_OVERSAMPLE; ++i ){
        advalue += Get_AnalogValue( ADC_CHS_AN16 );
    }
    advalue /= SENSE_BATTERY_CURRENT_OVERSAMPLE;
    
    // current 2V/1A, 0.1V/50mA
    // 1650(3.3V fullscale)
    g_BattCurrAdValue = advalue;
    g_MPPT_BattCurr = Lerp( sk_ChgCurrLerp[0], sk_ChgCurrLerp[1], sk_ChgCurrLerp[2], sk_ChgCurrLerp[3], advalue );
    //t = (uint16_t)((uint32_t)advalue * 1650 / 1024 / SENSE_BATTERY_CURRENT_OVERSAMPLE);
    //g_MPPT_BattCurr = Lerp( sk_ChgCurrLerp[0], sk_ChgCurrLerp[1], sk_ChgCurrLerp[2], sk_ChgCurrLerp[3], curr );
}

void MPPT_SetPWMWidth(void)
{
    uint16_t volt = MPPT_SensePanelVoltage();
    int16_t width = (int16_t)g_MPPT_PWMWidth;
    
    if( volt < 3500 ){
        width -= 2;
    }
    else if( g_MPPT_BattVolt >= BATT_FULLCHG_CV_MILLV ){
        width -= 2;
    }
    else if( volt > g_MPPT_TargetVolt ){
        width += 2;
    }
    else {
        width -= 2;
    }
    
    if( width < 0 ){
        width = 0;
    }
    if( width > PWM_WIDTH_MAX ){
        width = PWM_WIDTH_MAX;
    }
 
    g_MPPT_PWMWidth = (uint16_t)width;
    Set_Duty(g_MPPT_PWMWidth);
}

void MPPT_Charging(void)
{
    MPPT_SetPWMWidth();
    Set_ChargeLED(LED_ON);

    if((g_MPPT_BattVolt >= BATT_FULLCHG_MILLV)
     &&(g_MPPT_BattCurr < BATT_CHGFIN_CURR)){
        ++g_MPPT_BattFullChargingTimer;
    }
    else {
        g_MPPT_BattFullChargingTimer = 0;
    }
    
    if( g_MPPT_BattFullChargingTimer >= 6000 ){
        g_MPPT_Sequence = MPPT_FULLCHARGE;
        g_MPPT_FullCharge_Init_F = TRUE;
    }
}

void InitSeq_MPPT_FullCharge(void)
{
    g_LED_BattFullChargeTimer = 0;
}

void MPPT_FullCharge(void)
{
    ++g_LED_BattFullChargeTimer;
    if( g_LED_BattFullChargeTimer >= 500 ){
        g_LED_BattFullChargeTimer = 0;
        Set_ChargeLED_Invert();
    }
}

void MPPT_ChargeSequence(void)
{
    switch(g_MPPT_Sequence){
        case MPPT_CHARGING:
            MPPT_Charging();
            break;
        case MPPT_FULLCHARGE:
            InitSeq_MPPT_FullCharge();
            MPPT_FullCharge();
            break;
    }
}

void DebugPrint(void)
{
    USART_PrintString("TgtV=");
    USART_PrintUInt16(g_MPPT_TargetVolt, 10);
    USART_PrintString(",");
    
    USART_PrintString("BattV=");
    USART_PrintUInt16(g_MPPT_BattVolt, 10);
    USART_PrintString(",");
    
    USART_PrintString("BattI=");
    USART_PrintUInt16(g_MPPT_BattCurr, 10);
    USART_PrintString(",");

    USART_PrintString("BattIAd=");
    USART_PrintUInt16(g_BattCurrAdValue, 10);
    USART_PrintString("\n");
}

void main(void) 
{
    uint16_t advalue;
    uint16_t pwm = 0;

    Init_MainClock();
    // wait VCC is stable 
    __delay_ms(1000);
    
    Init_PortDirection();
    Init_PPS();
    Init_Timer2();
    Init_Timer4();
    Init_USART();
    Init_ADC();
    Init_OpAmp();
    Init_PWM();
    Init_COG();
    
    EnableInterruptAtSystemWakeup();
    Timer2_Start();
    USART_Start();
    Start_SyncRectification();

    Set_Duty(0);

    while(1){
        if( g_MainLoop_F == 1 ){
            g_MainLoop_F = 0;

            MPPT_SenseBatteryCurrent();
            MPPT_SenseVoltage();
            MPPT_ChargeSequence();
            //MPPT_FailureProtection();

            ++g_Debug_AdTimer;
            if( g_Debug_AdTimer >= 50 ){
                g_Debug_AdTimer = 0;
                DebugPrint();
            }
        }

        while( g_MainLoop_F == 0 ){
            USART_SendByteFromBuffer();
        }
    }
    
    return;
}
