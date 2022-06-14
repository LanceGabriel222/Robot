# Robot

#define _XTAL_FREQ 20000000
#define TMR2PRESCALE 4
#include <xc.h>
#include<stdint.h>
// BEGIN CONFIG
#pragma config FOSC = HS // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = OFF // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF // Flash Program Memory Code Protection bit (Code protection off)
//END CONFIG
//Switch Debounce time in us
#define DEBOUNCE_TIME  240
//Switch Status
#define SWITCH_PRESSED    1
#define SWITCH_BOUNCE     0
//Define pins for motor
#define M_a    RD0
#define M_b    RD1
//Define pins for switch
#define S_1  RB0
#define S_2  RB1
 
// CCP1 module is used here to generate the required PWM
// Timer2 module is used to generate the PWM
// This PWM has 10bit resolution
//max Duty 
uint32_t pwmMaxDuty(const uint32_t freq)
{
  return(_XTAL_FREQ/(freq*TMR2PRESCALE));
}
//Calculate the PR2 value
void initPwm(const uint32_t freq)
{
    //calculate period register value
    PR2 = (uint8_t)((_XTAL_FREQ/(freq*4*TMR2PRESCALE)) - 1);
}
//Give a value in between 0 and 1024 for duty-cycle
void applyPWMDutyCycle(uint16_t dutyCycle, const uint32_t freq)
{
    if(dutyCycle<1024)
    {
        //1023 because 10 bit resolution
        dutyCycle = (uint16_t)(((float)dutyCycle/1023)*pwmMaxDuty(freq));
        CCP1CON &= 0xCF;                 // Make bit4 and 5 zero (Store fraction part of duty cycle)
        CCP1CON |= (0x30&(dutyCycle<<4)); // Assign Last 2 LSBs to CCP1CON
        CCPR1L = (uint8_t)(dutyCycle>>2); // Put MSB 8 bits in CCPR1L
    }
}
//Init the Port pin
void initPort()
{
    TRISB0 = 1;  // Make S_1 pin an input
    TRISB1 = 1;  // Make S_2 pin an input
    TRISD0 = 0;  // Make M_a pin an output
    TRISD1 = 0;  // Make M_b pin an output
    TRISC2 = 0;   //Make pin output for PWM
}
//Run motor clockwise
void motorRunClockWise()
{
    M_a=1;
    M_b=0;
    M_a=1;
    M_b=0;
}
//configure and start PWM1
void startPwm()
{
    CCP1CON = 0x0C; // Configure CCP1 module in PWM mode
    T2CON = 0x01;  // Set Prescaler to be 4
    T2CON |= 0x04; // Enable the Timer2, hence enable the PWM.
}
//Function to check the status of Switch S1
int isS1Pressed()
{
    int switchStatus =  SWITCH_BOUNCE;
    if(S_1 == SWITCH_PRESSED)
    {
        //Wait time more then bouncing period
        __delay_us(DEBOUNCE_TIME);
        switchStatus =  S_1? SWITCH_PRESSED : SWITCH_BOUNCE;
    }
    return switchStatus ;
}
//Function to check the status of Switch S2
int isS2Pressed()
{
    int switchStatus =  SWITCH_BOUNCE;
    if(S_2 == SWITCH_PRESSED)
    {
        //Wait time more then bouncing period
        __delay_us(DEBOUNCE_TIME);
        switchStatus =  S_2? SWITCH_PRESSED : SWITCH_BOUNCE;
    }
    return switchStatus ;
}
//main function
void main()
{
    uint16_t dutycycle = 0;
    uint16_t dutyCycleApply = 0;
    const uint32_t pwmFreq = 5000;
    
    
    initPort(); //Init Gpio port
    motorRunClockWise(); //Run motor clockwise
    initPwm(pwmFreq); // Initialize PWM
    applyPWMDutyCycle(dutycycle,pwmFreq);
    startPwm();
    do
    {
        //Check the switch status for duty cycle
        dutycycle = (isS1Pressed() && isS2Pressed())? 1023: dutycycle; //100% duty cycle
        dutycycle = (isS1Pressed() && !isS2Pressed())? 768: dutycycle; //75% duty cycle
        dutycycle = (!isS1Pressed() && isS2Pressed())? 512: dutycycle; //50% duty cycle
        dutycycle = (!isS1Pressed() && !isS2Pressed())? 256: dutycycle;//25% duty cycle
        if (dutycycle != dutyCycleApply)
        {
            applyPWMDutyCycle(dutycycle,pwmFreq);
            dutyCycleApply = dutycycle;
        }
    }
    while(1);  //super loop
}
