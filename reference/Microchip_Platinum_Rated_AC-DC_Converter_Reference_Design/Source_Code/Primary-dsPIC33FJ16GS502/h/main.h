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

#ifndef _MAIN_H_
#define _MAIN_H_


/***************************************************************************
Defines
***************************************************************************/

#define PFC_PER 2448	// use 96 kHz for PWM (100kHz = 2350)

//Current Controller
#define CURR_KP 22000
#define CURR_KI 6000

//Voltage Controller
#define VOLT_KP 32000 //18000
#define VOLT_KI 100

//Protection Limmits
#define CURR1_LIMIT 1023	//Current 1 Limit at 11A 
#define CURR2_LIMIT 1023	//Current 2 Limit at 11A
#define BULKVOLT_LIMIT 857	//Bulkvoltage Limit at 430V (864 at AVdd=3.3V)

//Switching Limits
#define SWON_LO 177 		// switch on limit is 89Vrms
#define SWON_HI 528 		// switch on limit is 264Vrms

// new SWOFF_LO limit due to derating...
#define SWOFF_LO 167 		// switch off limit is 85Vrms
#define SWOFF_LO_AVG 152	// switch off limit for average
#define SWOFF_HI 538 		// switch off limit is 270Vrms
#define SWOFF_HI_AVG 488	// switch off limit for average	
#define SWOFF_TEMP 620  	// switch off limit at Over Temperature (130 °C= int 558)
#define SWON_TEMP 496

//Softstart Settings
#define SS_DELAY 480	// delay is 100 ms
#define SS_RAMP 1440	// ramp is 200 ms => 4.8 = 1ms

//Jitter Settings
#define JITTERSRC 1 //Jitter-Source (0=No Jitter; 1=internally generated; 2=Use PLL)
#define FSWING_MIN 14745 /* decrease PTPER by 10% => frequency is increased by 10.0%, 100% = 16384 fractional */
#define FSWING_MAX 18021 /* increase PTPER by 10% => frequency is decreased by 10.0%, 100% = 16384 fractional */
#define FSWING_FREQ 170   /* one period of fswing is direct in Hz*/
#define FSWING_INT_FREQ 4800 /* Interrupt Frequenz */
#define FSWING_STEPS FSWING_INT_FREQ/(FSWING_FREQ*4) /* STEPS for one edge */
#define FSWING_INC 235//FSWING_MAX-FSWING_MIN/(2*FSWING_STEPS) /* increment per interrupt call */

//Input Relay Control
#define REL_PWM_PER 10000u /* 500Hz @5MHz clock */
#define REL_PWM_HOLD 5000u /* 50% duty */

//Boost Mode
#define BOOST_THRESHOLD 50 // (2*25) 	/* Bulk Voltage Error that Activates Boost, 25V */
#define BOOST_FACTOR 4			/* Boost Mode: Factor by that Voltage Compensator Ki is Multiplied */

// PFC status
#define PFC_PWR_GOOD 	0x0001
#define PFC_UIN_OK	0x0002


#endif
