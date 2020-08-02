////////////////////////////////////////////////////////////////////////////////
// © 2013 Microchip Technology Inc.
//
// MICROCHIP SOFTWARE NOTICE AND DISCLAIMER:  You may use this software, and any
// derivatives created by any person or entity by or on your behalf, exclusively
// with Microchip?s products.  Microchip and its licensors retain all ownership
// and intellectual property rights in the accompanying software and in all
// derivatives here to.
//
// This software and any accompanying information is for suggestion only.  It
// does not modify Microchip?s standard warranty for its products.  You agree
// that you are solely responsible for testing the software and determining its
// suitability.  Microchip has no obligation to modify, test, certify, or
// support the software.
//
// THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER
// EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED
// WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR
// PURPOSE APPLY TO THIS SOFTWARE, ITS INTERACTION WITH MICROCHIP?S PRODUCTS,
// COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
//
// IN NO EVENT, WILL MICROCHIP BE LIABLE, WHETHER IN CONTRACT, WARRANTY, TORT
// (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY), STRICT LIABILITY,
// INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, PUNITIVE,
// EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF
// ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWSOEVER CAUSED, EVEN IF
// MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.
// TO THE FULLEST EXTENT ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
// CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES,
// IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
//
// MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
// TERMS.
////////////////////////////////////////////////////////////////////////////////


#include "p33FJ16GS504.h"
#include "init.h"
#include "main.h"

/***************************************************************************
Function: 	init_CLOCK
Description:	Oscillator Settings
 ***************************************************************************/
void init_CLOCK() {
    /* 	Configure Oscillator to operate the device at 40Mhz instruction clock
            Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
            Fosc= 7.37*(43)/(2*2)=80Mhz for Fosc, Fcy = Fosc / 2 = 40Mhz  */

    /* Configure PLL prescaler, PLL postscaler, PLL divisor */
    PLLFBD = 41; /* M = PLLFBD + 2 */
    CLKDIVbits.PLLPOST = 0; /* N1 = 2 */
    CLKDIVbits.PLLPRE = 0; /* N2 = 2 */

    while (OSCCONbits.LOCK != 1); // Wait for PLL to Lock

    /* Config ADC and PWM clock for 120MHz
       ACLK = ((REFCLK * 16) / APSTSCLR ) = (7.37 * 16) / 1 = 117.92 MHz  */

    ACLKCONbits.FRCSEL = 1; /* Reference CLK source for Aux PLL, 1=FRC, 0=source is determined by ASRCSEL  */
    ACLKCONbits.ASRCSEL = 0; /* Clk source for FRCSEL, 1=Primary Oscillator, 0=No Clk */
    ACLKCONbits.SELACLK = 1; /* Auxiliary Oscillator provides clock source for PWM & ADC */
    ACLKCONbits.APSTSCLR = 0b111; /* Divide Auxiliary clock by (0b111=1, 0b110=2, 0b101=4 ...) */
    ACLKCONbits.ENAPLL = 1; /* Enable Auxiliary PLL */

    while (ACLKCONbits.APLLCK != 1); /* Wait for Auxiliary PLL to Lock */
}
/***************************************************************************
End of function
 ***************************************************************************/

/***************************************************************************
Function: 	init_PORTS
Description:	Input Output Pins Settings
 ***************************************************************************/
void init_PORTS() {
    __builtin_write_OSCCONL(OSCCON & ~(1 << 6)); // Unlock Registers (Bit 6 in OSCCON)

    //ODCCbits.ODCC13 = 1;		// configure RP29/RC13 (TxD) as open drain
    RPINR18bits.U1RXR = 16; // U1RX->RP16
    RPOR14bits.RP29R = 3; // U1TX->RP29

    // configure comparators
    RPOR16bits.RP32R = 39; // remap comparator 1 output to virtual pin RP32
    RPOR16bits.RP33R = 40; // remap comparator 2 output to virtual pin RP33
    RPINR29bits.FLT1R = 32; // assign PWM fault1 input to virtual pin RP32
    RPINR30bits.FLT2R = 33; // assign PWM fault2 input to virtual pin RP33
    RPINR31bits.FLT4R = 5; // assign PWM fault4 input to pin RP5=RB5

    // remap PWM4L to Oring Charge Pump
    RPOR12bits.RP24R = 45; // remap PWM4L output to RP24 (Fault Oring actual)

    // remap PWM4H to CS_BUS output
    RPOR11bits.RP23R = 44; // remap PWM4L output to RP23 (CSBUS_OUT)

    __builtin_write_OSCCONL(OSCCON | (1 << 6)); // Lock Registers

    // configure DAC
    CMPCON4bits.CMPON = 1; // enable comparator module
    CMPCON4bits.DACOE = 1; // DAC output enable
    CMPCON4bits.RANGE = 1; // use AVdd/2 as referance
    CMPDAC4 = 0;

    // configure Sync-enable Pins
    TRISBbits.TRISB12 = 0; //configure Pin 9 as output for Sync1_en
    TRISCbits.TRISC12 = 0; //configure Pin 20 as output for Sync2_en
    TRISBbits.TRISB15 = 0; //configure Pin42 as output for green LED
    TRISBbits.TRISB8 = 0; //configure Pin41 as output for red LED
    //LATBbits.LATB12 = 0;
    //LATCbits.LATC12 = 0;
    TRISBbits.TRISB4 = 0; //TestPin
    LATBbits.LATB4 = 0;

}
/***************************************************************************
End of function
 ***************************************************************************/

/***************************************************************************
Function: 	init_TIMER1
Description:	initialize Timer 1
 ***************************************************************************/
void init_TIMER1() {
    T1CONbits.TCS = 0; /* Internal Clock Fcy 40MHz */
    T1CONbits.TCKPS = 3; /* 1:256 Prescaler */
    PR1 = T1_PER; /* Timer1 Period, 5Hz */
    T1CONbits.TON = 1;
}
/***************************************************************************
End of function
 ***************************************************************************/

/***************************************************************************
Function: 	init_TIMER2
Description:	initialize Timer 2
 ***************************************************************************/
void init_TIMER2() {
    T2CONbits.TCS = 0; /* Internal Clock Fcy 40MHz */
    T2CONbits.TCKPS = 0; /* 1:x Prescaler */
    PR2 = T2_PER - 1; /* Timer2 Period, 4,8 kHz */
    T2CONbits.TON = 1; /* if 1, Timer2_on */
}
/***************************************************************************
End of function
 ***************************************************************************/

/***************************************************************************
Function: 	init_INT
Description:	initialize Interrupts
 ***************************************************************************/
void init_INT(void) {
    // enable timer interrupts
    IFS0bits.T2IF = 1; // Reset T2 interrupt has not occured flag
    IPC1bits.T2IP = 3; // set Timer2 interrupt priority (1=lowest, 7=highest)
    IEC0bits.T2IE = 1; // Enable T2 Interrupt Service Routine

    //Timer 1 Interrupt low priority
    IFS0bits.T1IF = 1; // Reset T1 interrupt has not occured flag
    IPC0bits.T1IP = 3; // set Timer1 priority to 1 (1=lowest, 7=highest)
    IEC0bits.T1IE = 1;

    // enable SEVTCMP interrupt
    IFS3bits.PSEMIF = 0; // clear special event match interrupt flag
    IPC14bits.PSEMIP = 6; // set interrupt priority
    IEC3bits.PSEMIE = 1; // enable interrupt

}
/***************************************************************************
End of function
 ***************************************************************************/

/***************************************************************************
Function: 	init_PWM
Description:	initialize PWM Modules
 ***************************************************************************/
void init_PWM() {

    PTCON2bits.PCLKDIV = 2; // Clock divider = 2^n (n=0,1,2,3,4,5,6) don't use 1,5 or 6, see errata
    PTPER = DCDC_PER; // PTPER = ((REFCLK/7.37MHz) * 1/(f*Prescaler*1.04 ns)
    // is the desired switching frequency and 1.04ns is PWM resolution.
    // minimum: 0x0010 (16)	 maximum: 0xFFFB  (65531)
    SEVTCMP = 0;
    PTCONbits.SEIEN = 1; // enable special event interrupt


    // PWM1 (T1 and S1):
    IOCON1bits.PENH = 1; // PWM1H (S1) is controlled by PWM module
    IOCON1bits.PENL = 1; // PWM1L (T1) is controlled by PWM module
    IOCON1bits.PMOD = 3; // Output Mode: 0=Complementary, 1=Redundant, 2=Push-Pull, 3=Independent
    PWMCON1bits.DTC = 0; // Dead Time Control: 0=positive, 1=negative, 2=disabled
    DTR1 = 50; // Deadtime setting
    ALTDTR1 = 0; // Deadtime setting
    FCLCON1 = 0x0103; // CLMOD=1(enabled), CLSRC=fault1, FLTMOD=3(disabled)
    LEBCON1 = 0x2400 | 120; // PHR=0, PLR=1, CLLEBEN=1, LEB=120(500 ns)
    PWMCON1bits.CLIEN = 1;
    //
    PDC1 = DCDC_PER >> 2; // Primary Duty Cycle PWM1, set to 25%
    SDC1 = DCDC_PER >> 2; // Secondary Duty Cycle PWM1, set to 25%
    PHASE1 = 0;
    SPHASE1 = 0;
    STRIG1 = DCDC_PER >> 3; // set trigger position PWM1L (T1)

    // PWM2 (T2 and S2):
    IOCON2bits.PENH = 1; // PWM2H (S2) is controlled by PWM module
    IOCON2bits.PENL = 1; // PWM2L (T2) is controlled by PWM module
    IOCON2bits.PMOD = 3; // Output Mode: 0=Complementary, 1=Redundant, 2=Push-Pull, 3=Independent
    PWMCON2bits.DTC = 0; // Dead Time Control: 0=positive, 1=negative, 2=disabled
    DTR2 = 50; // Deadtime setting
    ALTDTR2 = 0; // Deadtime setting
    FCLCON2 = 0x0503; // CLMOD=1(enabled), CLSRC=fault2, FLTMOD=3(disabled)
    LEBCON2 = 0x2400 | 120; // PHR=0, PLR=1, CLLEBEN=1, LEB=120(500 ns)
    //
    PDC2 = DCDC_PER >> 2; // Primary Duty Cycle PWM2, set to 25%
    SDC2 = DCDC_PER >> 2; // Secondary Duty Cycle PWM2, set to 25%
    PHASE2 = DCDC_PER >> 1;
    SPHASE2 = DCDC_PER >> 1;
    STRIG2 = DCDC_PER >> 3; // set trigger position PWM2L (T2)

    // PWM3 Configuration
    IOCON3bits.PENH = 1; // PWM3H is controlled by PWM module
    IOCON3bits.PENL = 0; // PWM3L is  controlled by PWM module
    IOCON3bits.PMOD = 3; // Output Mode: 0=Complementary, 1=Redundant, 2=Push-Pull, 3=Independent
    PWMCON3bits.DTC = 2; // Dead Time Control: 0=positive, 1=negative, 2=disabled
    PWMCON3bits.ITB = 1; // PHASE Registers provide Time Base period
    DTR3 = 0; // Deadtime setting
    ALTDTR3 = 0; // Deadtime setting
    PHASE3 = FAN_PWM_PER; // phase shift
    SPHASE3 = 0; // 50% Duty Cycle
    PDC3 = 0; // 0% Duty Cycle
    SDC3 = 0; // 200 kHz
    FCLCON3 = 0x0003; // FLTMOD=3(disabled)

    // PWM4 Configuration 
    IOCON4bits.PENH = 1; // PWM4H (CSBUS_OUT) is controlled by PWM module
    IOCON4bits.PENL = 1; // PWM4L (ORING_CP) is  controlled by PWM module
    IOCON4bits.PMOD = 3; // Output Mode: 0=Complementary, 1=Redundant, 2=Push-Pull, 3=Independent
    PWMCON4bits.DTC = 2; // Dead Time Control: 0=positive, 1=negative, 2=disabled
    PWMCON4bits.ITB = 1; // SPHASE Register provides Time Base period for PWM4L
    DTR4 = 0; // Deadtime setting
    ALTDTR4 = 0; // Deadtime setting
    PHASE4 = CS_BUS_PER; // CSBUS_OUT frequency is 50 kHz
    SPHASE4 = ORING_CP_PER; // ORING_CP frequency is 200 kHz
    PDC4 = 0; // 0% Duty Cycle
    SDC4 = PHASE_ORING_CP; // 50% Duty Cycle
    FCLCON4 = 0x0003; // FLTMOD=3(disabled)

    // PWM enable:
    PTCONbits.PTEN = 1; // Enable the PWM Module
}
/***************************************************************************
End of function
 ***************************************************************************/

/***************************************************************************
Function: 	init_ADC
Description:	initialze the AD-Converter
 ***************************************************************************/
void init_ADC() {
    ADCONbits.FORM = 0; // Output in Integer Format
    ADCONbits.SLOWCLK = 1; // if 1, use auxiliary clock source (ACLK = 120 MHz)
    ADCONbits.ADCS = 4; // Clock divider selection (ADCLK = 1/(n+1)*ACLK = 24 MHz)
    ADCONbits.EIE = 1; // Early interrupt enabled
    ADCONbits.ASYNCSAMP = 1; // Asynchronous sampling

    ADPCFGbits.PCFG0 = 0; // AN0 is configured as analog input
    ADPCFGbits.PCFG1 = 0; // AN1 is configured as analog input
    ADPCFGbits.PCFG2 = 0; // AN2 is configured as analog input
    ADPCFGbits.PCFG3 = 0; // AN3 is configured as analog input
    ADPCFGbits.PCFG4 = 0; // AN4 is configured as analog input
    ADPCFGbits.PCFG5 = 0; // AN5 is configured as analog input
    ADPCFGbits.PCFG9 = 0; // AN9 is configured as analog input
    ADPCFGbits.PCFG10 = 0; // AN10 is configured as analog input
    ADPCFGbits.PCFG11 = 0; // AN11 is configured as analog input

    ADSTATbits.P0RDY = 0; // Clear Pair 0 (AN0+1) data ready bit
    ADSTATbits.P1RDY = 0; // Clear Pair 1 (AN2+3) data ready bit
    ADSTATbits.P2RDY = 0; // Clear Pair 2 (AN4+5) data ready bit
    ADSTATbits.P4RDY = 0; // Clear Pair 4 (AN8+9) data ready bit
    ADSTATbits.P5RDY = 0; // Clear Pair 5 (AN10+11) data ready bit

    ADCPC0bits.TRGSRC0 = 14; // PWM1S is Trigger for AN0+1
    ADCPC0bits.TRGSRC1 = 15; // PWM2S is Trigger for AN2+3
    ADCPC1bits.TRGSRC2 = 15; // PWM2S is Trigger for AN4+5
    ADCPC2bits.TRGSRC4 = 31; // Timer2 period is Trigger for AN8+9
    ADCPC2bits.TRGSRC5 = 31; // Timer2 period is Trigger for AN10+11

    ADCONbits.ADON = 1; // Enable ADC module
}
/***************************************************************************
End of function
 ***************************************************************************/

/***************************************************************************
Function: 	init_DAC
Description:	initialize the Comparators for OverCurrent and 
                                Overvoltage Protection
 ***************************************************************************/
void init_DAC() {
    // configure comparator1
    CMPCON1bits.CMPON = 1; // enable comparator
    CMPCON1bits.INSEL = 1; // select CMP1B input pin (RA1)
    CMPCON1bits.RANGE = 1; // select high range, max DAC value = Avdd/2
    CMPDAC1 = CURR_HWLIM; // DAC threshold
    // configure comparator2
    CMPCON2bits.CMPON = 1; // enable comparator
    CMPCON2bits.INSEL = 1; // select CMP2B input pin (RB0)
    CMPCON2bits.RANGE = 1; // select high range, max DAC value = Avdd/2
    CMPDAC2 = CURR_HWLIM; // DAC threshold
}
/***************************************************************************
End of function
 ***************************************************************************/
