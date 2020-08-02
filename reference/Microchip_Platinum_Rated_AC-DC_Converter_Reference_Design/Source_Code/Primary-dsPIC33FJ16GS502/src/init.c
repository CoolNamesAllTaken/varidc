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

#include "p33FJ16GS502.h"
#include "main.h"
#include "init.h"

/***************************************************************************
Function: 	init_CLOCK
Description:	Oscillator Settings
***************************************************************************/
void init_CLOCK()
{
    /* 	Configure Oscillator to operate the device at 40Mhz instruction clock
        Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
        Fosc= 7.37*(43)/(2*2)=80Mhz for Fosc, Fcy = Fosc / 2 = 40Mhz  */

    /* Configure PLL prescaler, PLL postscaler, PLL divisor */
    PLLFBD=41;              /* M = PLLFBD + 2 */
    CLKDIVbits.PLLPOST=0;   /* N1 = 2 */
    CLKDIVbits.PLLPRE=0;    /* N2 = 2 */

    while(OSCCONbits.LOCK != 1);         // Wait for PLL to Lock

    /* Config ADC and PWM clock for 120MHz
       ACLK = ((REFCLK * 16) / APSTSCLR ) = (7.37 * 16) / 1 = 117.92 MHz  */

    ACLKCONbits.FRCSEL = 1;		/* Reference CLK source for Aux PLL, 1=FRC, 0=source is determined by ASRCSEL  */
    ACLKCONbits.ASRCSEL = 0;		/* Clk source for FRCSEL, 1=Primary Oscillator, 0=No Clk */
    ACLKCONbits.SELACLK = 1;		/* Auxiliary Oscillator provides clock source for PWM & ADC */
    ACLKCONbits.APSTSCLR = 0b111;	/* Divide Auxiliary clock by (0b111=1, 0b110=2, 0b101=4 ...) */
    ACLKCONbits.ENAPLL = 1;		/* Enable Auxiliary PLL */

    while(ACLKCONbits.APLLCK != 1);	/* Wait for Auxiliary PLL to Lock */
}
/***************************************************************************
End of function
***************************************************************************/

/***************************************************************************
Function: 	init_PORTS
Description:	Input Output Pins Settings
***************************************************************************/
void init_PORTS()
{
    // enable output ports (disable tristate)
    // ODCBbits.ODCB15   = 1;   // configure RC11 (TxD) as open drain
    TRISBbits.TRISB13 = 0;      //added by AD to test DCM to CCM
    TRISBbits.TRISB11 = 0;      //Relay Control Pin 21
    TRISBbits.TRISB4 = 0;	//Logic Output

    LATBbits.LATB4 = 0;		//logic Pin Output beginning with zero
    LATBbits.LATB11 = 1;	//enable Relay

    __builtin_write_OSCCONL(OSCCON & ~(1<<6));		// Unlock Registers (Bit 6 in OSCCON)

    // configure serial interface
    RPINR18bits.U1RXR = 5;	// U1RX->RP5
    RPOR7bits.RP15R = 3;	// U1TX->RP15

    // assign fault inputs to comparator outputs
    RPOR16bits.RP32R = 39; 	// remap comparator1 (I1) output to virtual pin RP32
    RPOR16bits.RP33R = 40; 	// remap comparator2 (I2) output to virtual pin RP33
    RPOR17bits.RP34R = 41; 	// remap comparator3 (Udc) output to virtual pin RP34
    RPINR29bits.FLT1R = 32;	// assign PWM fault1 (I1) input to virtual pin RP32
    RPINR30bits.FLT2R = 33;	// assign PWM fault2 (I2) input to virtual pin RP33
    RPINR30bits.FLT3R = 34;	// assign PWM fault3 (Udc) input to virtual pin RP34
    RPINR31bits.FLT4R = 12;	// assign PWM fault4 input to pin RP12=RB12

    // route OC1 to RP11/RB11
    RPOR5bits.RP11R = 18;	// output compare 1 -> RP11

    __builtin_write_OSCCONL(OSCCON | (1<<6));		// Lock Registers

    // configure DAC
    CMPCON4bits.CMPON = 1;	// enable comparator module
    CMPCON4bits.DACOE = 1;	// DAC output enable
    CMPCON4bits.RANGE = 1;	// use AVdd/2 as referance

}
/***************************************************************************
End of function
***************************************************************************/

/***************************************************************************
Function: 	init_TIMER2
Description:	initialize Timer 2
***************************************************************************/
void init_TIMER2()
{
    T2CONbits.TCS = 0;		/* Internal Clock Fcy 40MHz */
    T2CONbits.TCKPS = 0;	/* Prescaler, 0,1,2,3 = 1,8,64,256 */
    PR2 = 2083u;		/* Timer2 Period, 19.2 kHz */
    T2CONbits.TON = 1;		/* if 1, Timer2_on */
}
/***************************************************************************
End of function
***************************************************************************/

/***************************************************************************
Function: 	init_TIMER3
Description:	initialize Timer 3
***************************************************************************/
void init_TIMER3()
{
    T3CONbits.TCS = 0;		/* Internal Clock Fcy 40MHz */
    T3CONbits.TCKPS = 1;	/* Prescaler, 0,1,2,3 = 1,8,64,256 */
    PR3 = REL_PWM_PER;		/* Timer3 Period */
    T3CONbits.TON = 1;		/* Timer3 run */
}
/***************************************************************************
End of function
***************************************************************************/

/***************************************************************************
Function: 	init_INT
Description:	initialize Interrupts
***************************************************************************/
void init_INT(void)
{
    // enable timer interrupts
    IFS0bits.T1IF = 0;    	// Reset T1 interrupt has not occured flag
    IPC0bits.T1IP = 3;		// set Timer1 priority to 3 (1=lowest, 7=highest)
    IEC0bits.T1IE = 1;    	// Enable Interrupt Service Routine for Timer1
    // enable timer interrupts
    IFS0bits.T2IF = 0;    	// Reset T2 interrupt has not occured flag
    IPC1bits.T2IP = 3;		// set Timer2 priority to 3 (1=lowest, 7=highest)
    IEC0bits.T2IE = 1;    	// Enable Interrupt Service Routine for Timer2
    // enable INT2 interrupt (used as software triggered interrupt)
    IFS1bits.INT2IF = 0;	// clear INT2 interrupt flag
    IPC7bits.INT2IP = 2;	// set INT2 priority to 3 (1=lowest, 7=highest)
    IEC1bits.INT2IE = 1;	// enable INT2 interrupt
    // enable ADC interrupts
    IFS6bits.ADCP0IF = 0;	// clear ADC Pair 0 interrupt flag
    IPC27bits.ADCP0IP = 6;	// set AD0+1 priority to 6 (1=lowest, 7=highest)
    IEC6bits.ADCP0IE = 1;	// enable interrupt for ADC0+1

    IFS6bits.ADCP1IF = 0;	// clear ADC Pair 1 interrupt flag
    IPC27bits.ADCP1IP = 6;	// set AD2+3 priority to 6 (1=lowest, 7=highest)
    IEC6bits.ADCP1IE = 1;	// enable interrupt for ADC2+3
}
/***************************************************************************
End of function
***************************************************************************/

/***************************************************************************
Function: 	init_PWM
Description:	initialize PWM Modules
***************************************************************************/
void init_PWM(void)
{
    // compensate delay time pwm edge -> mos current
    ///#define SDELAY 85

    PTCON2bits.PCLKDIV = 2;	// Clock divider = 2^n (n=0,1,2,3,4,5,6) don't use 1,5 or 6, see errata
    PTPER = PFC_PER;            // PTPER = ((REFCLK/7.37MHz) * 1/(f*Prescaler*1.04 ns)
                                // is the desired switching frequency and 1.04ns is PWM resolution.
                                // minimal: 0x0010 (16)	 maximum: 0xFFFB  (65531)

    // PWM1 Configuration
    IOCON1bits.PENH = 1;      	// PWM1H is controlled by PWM module
    IOCON1bits.PENL = 0;      	// PWM1L disabled
    IOCON1bits.PMOD = 3; 	// Output Mode: 0=Complementary, 1=Redundant, 2=Push-Pull, 3=Independent
    PWMCON1bits.DTC = 2;	// Dead Time Control: 0=positive, 1=negative, 2=disabled
    DTR1 = 0; 	          	// Deadtime setting
    ALTDTR1 = 0;          	// Deadtime setting
    PWMCON1bits.CLIEN = 1;
    PDC1 = PTPER>>2;		// Primary Duty Cycle PWM1, set to 25%
    TRIG1 = PTPER>>3;		// set trigger position PWM1H
    PHASE1 = 0;
    FCLCON1 = 0x0101;           // CLMOD=1(enabled), CLSRC=fault1, FLTMOD=1(cycle), FLTSRC=none(fault3)
    LEBCON1 = 0xCC00|60;	// PHR=0, PLR=1, CLLEBEN=1, LEB=60(240 ns)

    // PWM2 Configuration
    IOCON2bits.PENH = 1;      	// PWM2H is controlled by PWM module
    IOCON2bits.PENL = 0;      	// PWM2L disabled
    IOCON2bits.PMOD = 3; 	// Output Mode: 0=Complementary, 1=Redundant, 2=Push-Pull, 3=Independent
    PWMCON2bits.DTC = 2;	// Dead Time Control: 0=positive, 1=negative, 2=disabled
    DTR2 = 0;                   // Deadtime setting
    ALTDTR2 = 0;                // Deadtime setting
    PWMCON2bits.CLIEN = 1;
    PDC2 = PTPER>>2;		// Primary Duty Cycle PWM2, set to 25%
    TRIG2 = PTPER>>3;		// set trigger position PWM2H
    PHASE2 = (PFC_PER>>1);	// phase shift
    FCLCON2 = 0x0501;		// CLMOD=1(enabled), CLSRC=fault2, FLTMOD=1(cycle), FLTSRC=none(fault3)
    LEBCON2 = 0xCC00|60;	// PHR=0, PLR=1, CLLEBEN=1, LEB=60(240 ns)

    // PWM4 Configuration
    IOCON4bits.PENH = 1;      	// PWM4H is controlled by PWM module
    IOCON4bits.PENL = 0;      	// PWM4L disabled
    IOCON4bits.PMOD = 3; 	// Output Mode: 0=Complementary, 1=Redundant, 2=Push-Pull, 3=Independent
    PWMCON4bits.DTC = 2;	// Dead Time Control: 0=positive, 1=negative, 2=disabled
    DTR4 = 0; 	        	// Deadtime setting
    ALTDTR4 = 0;        	// Deadtime setting
    PWMCON4bits.CLIEN = 1;
    PHASE4 = 0;			// phase shift
    PDC4 = PTPER-8;             // duty cycle, 100%
    FCLCON4 = 0x0d03;		// CLMOD=1(enabled), CLSRC=fault4, FLTMOD=3(disabled), FLTSRC=fault1

}
/***************************************************************************
End of function
***************************************************************************/

/***************************************************************************
Function: 	init_ADC
Description:	initialze the AD-Converter
***************************************************************************/	
void init_ADC()
{
    // AD0 (PFC1 currrent) &  AD2 (PFC2 current): 93 digits/A, 1 digit = 10.75 mA
    // AD4 (Line voltage) & AD5 (DC link voltage): 2.01 digits/V, 1 digit = 497.5 mV

    ADCONbits.FORM = 0; 	// Output in Integer Format
    ADCONbits.SLOWCLK = 1; 	// if 1, use auxiliary clock source (ACLK = 120 MHz)
    ADCONbits.ADCS = 4; 	// Clock divider selection (ADCLK = 1/(n+1)*ACLK = 24 MHz)
    ADCONbits.EIE = 1;		// Early interrupt enabled
    ADCONbits.ASYNCSAMP = 1;	// Asynchronous sampling

    ADPCFGbits.PCFG0 = 0; 	// AN0 is configured as analog input
    ADPCFGbits.PCFG1 = 0; 	// AN1 is configured as analog input
    ADPCFGbits.PCFG2 = 0; 	// AN2 is configured as analog input
    ADPCFGbits.PCFG3 = 0; 	// AN3 is configured as analog input
    ADPCFGbits.PCFG4 = 0; 	// AN4 is configured as analog input
    ADPCFGbits.PCFG5 = 0; 	// AN5 is configured as analog input
    ADPCFGbits.PCFG6 = 0;	// AN6 is configured as analog input
    ADPCFGbits.PCFG7 = 0;	// AN7 is configured as analog input

    ADSTATbits.P0RDY = 0; 	// Clear Pair 0 (AN0+1) data ready bit
    ADSTATbits.P1RDY = 0; 	// Clear Pair 1 (AN2+3) data ready bit
    ADSTATbits.P2RDY = 0; 	// Clear Pair 2 (AN4+5) data ready bit
    ADSTATbits.P3RDY = 0; 	// Clear Pair 3 (AN6+7) data ready bit

    ADCPC0bits.TRGSRC0=4; 	// PWM1H is Trigger for AN0+1
    ADCPC0bits.TRGSRC1=5;	// PWM2H is Trigger for AN2+3
    ADCPC1bits.TRGSRC2=5;	// PWM2H is Trigger for AN4+5
    ADCPC1bits.TRGSRC3=5;	// PWM2H is Trigger for AN6+7

    ADCONbits.ADON = 1; 	// Enable ADC module
}
/***************************************************************************
End of function
***************************************************************************/

/***************************************************************************
Function: 	init_DAC
Description:	initialize the Comparators for OverCurrent and 
				Overvoltage Protection
***************************************************************************/
void init_DAC()
{
    // CMP1 (PFC1 currrent) &  CMP2 (PFC2 current): 93 digits/A, 1 digit = 10.75 mA
    // CMP3 (DC link voltage): 2.01 digits/V, 1 digit = 497.5 mV

    // configure comparator1 (I1)
    CMPCON1bits.INSEL = 1;		// select CMP1B input pin (RA1)
    CMPCON1bits.RANGE = 1;		// select high range, max DAC value = Avdd/2
    CMPDAC1 = CURR1_LIMIT;		// DAC threshold
    CMPCON1bits.CMPON = 1;		// enable comparator
    // configure comparator2 (I2)

    CMPCON2bits.INSEL = 1;		// select CMP2B input pin (RB0)
    CMPCON2bits.RANGE = 1;		// select high range, max DAC value = Avdd/2
    CMPDAC2 = CURR2_LIMIT;		// DAC threshold
    CMPCON2bits.CMPON = 1;		// enable comparator
    // configure comparator2 (Udc)

    CMPCON3bits.INSEL = 3;		// select CMP3D input pin (RB2)
    CMPCON3bits.RANGE = 1;		// select high range, max DAC value = Avdd/2
    CMPDAC3 = BULKVOLT_LIMIT;	// DAC threshold
    CMPCON3bits.CMPON = 1;		// enable comparator
}
/***************************************************************************
End of function
***************************************************************************/
