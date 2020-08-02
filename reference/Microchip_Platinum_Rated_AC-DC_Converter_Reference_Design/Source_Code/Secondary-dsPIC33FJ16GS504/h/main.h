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

// set T2 to 4800 Hz
#define T2_PER 8333u
#define T1_PER 31000 

// set current limits
#define CURR_MAX 802 // 750 // 10.15 * 74A 
#define CURR_HWLIM 1023 // 1023 // 10.15 * 101A 
#define OCP_TIME 500 // time to shut down when overcurrent is pending (5s in 10 ms steps)
// output current derating
#define OUTPUT_DERATE_TRES 220 // tres:=110V, u_scale_pri=2.01/V => tres = 110*2.01 = 220 
#define OUTPUT_DERATE_PARA 1293 // para:=1.0 A/V, u_scale_pri=2.01/V, i_scale_sec=10.15/A  => para = 1.0 *(10.15/2.01) *256 = 1293

//Switching Period Defaults
#define DCDC_PER 2938 //DCDC_PER=2938(80kHz); 2448(96kHz); DCDC_PER=2350(100kHz)
#define DUTY_DEFAULT 26230

// DCDC voltage controller
#define VOLT_KP 32000
#define VOLT_KI 8000
#define VOLT_REF 659 // 55.7 * 12V
#define DUTY_LIMIT 32490 // 99%

//Soft Start
#define SS_RAMP 96 // 1ms = 4.8

// DCDC current controller
#define CURR_KP 12000
#define CURR_KI 2000

//Synchron-Stage Stettings
#define T_SYNC_ON 4	//7 //30
#define T_SYNC_OFF 46	//47
#define T_ADC 61

// use 200 kHz for Charge Pump PWM
#define ORING_CP_PER 1175u
#define PHASE_ORING_CP 587

// Fault declaration
// 1) implemented:
#define	FAULT_PFC_NOT_READY	1	//
#define	FAULT_OVERTEMP		2	//
#define	FAULT_OVERCURRENT	4	//
// 2) reserved:
#define	FAULT_FAN			8	// Any Fan is defective
#define	FAULT_COMMUNICATION 16	// Primay to Secondary Communication
#define	FAULT_MAINS			32	// AC Mains Input Voltage Fault
#define	FAULT_OUTPUTVOLTAGE	64	//

// Over Temperature Protection
#define TEMP_SEC1_DEF 155	//Secondary Heatsink
#define TEMP_SEC2_DEF 155	//Ambient Temperature

//PFC-Stage Parameters
#define PFC_FLAGS 0x0000

//Temperature Default Values
#define TEMP_PRIM_OFFSET		58		//Offset for Prim Temp of 18.7°C
#define TEMP_SEC1_OFFSET		20		//Offset for Sec Heatsink Temp of 6.4°C
#define TEMP_SEC2_OFFSET		115		//Offset for Amb Temp of 43.5°C
#define TEMPLOWSETTING			309		//Temperature=50°C(Heatsink); 32°C(Ambient)
#define TEMPHIGHSETTING			393		//Temperature=75°C(Heatsink); 45°C(Ambient)
#define TEMPEXTREMESETTING		550		//Temperature=127°C(Heatsink); 65°C(Ambient) (468 => old value) 558

//FAN Control
#define FAN_MODE 4	
#define FAN_PWM_PER 			23500		// 10kHz
#define FANSPEED_DEFAULT		13500		// Required DC to drive all PWMs
#define FANSPEED_HALF			18000
#define FANSPEED_FULL			23500

#define FAN_START_VAL 1638		// Speed Offset (32768*5%)
#define FAN_START_TEMP 372		// 70°C * 10mV/°C +500mV *1023 /3.3V = 372
#define FAN_OFF_TEMP 356		// 65°C 
#define FAN_END_TEMP 526		// 120 °C
#define FAN_CONT_FACT 366		// Speed Factor 183 per 1 integer delta Temp 
#define FAN_MAX 13500			// 13500

//enabling SyncFET driver channel
#define IOUTSYNC_EN 0 //
#define IOUT2SYNC_ON  279 //24A * 11.97 = 287
#define IOUT2SYNC_OFF 256 //22A * 11.97 = 263
#define IOUTSYNC_OFF 31
#define IOUTSYNC_ON  31

//Jitter Settings
#define JITTERSRC 1 //Jitter-Source (0=No Jitter; 1=internally generated; 2=Use PLL)
#define FSWING_MIN 14745 /* decrease PTPER by 10% => frequency is increased by 10.0%, 100% = 16384 fractional */
#define FSWING_MAX 18021 /* increase PTPER by 10% => frequency is decreased by 10.0%, 100% = 16384 fractional */
#define FSWING_FREQ 170   /* one period of fswing is direct in Hz*/
#define FSWING_INT_FREQ 4800 /* Interrupt Frequenz */
#define FSWING_STEPS FSWING_INT_FREQ/(FSWING_FREQ*4) /* STEPS for one edge */
#define FSWING_INC 235//FSWING_MAX-FSWING_MIN/(2*FSWING_STEPS) /* increment per interrupt call */

// current share bus (CS_BUS)
#define CS_BUS_OFFS 30		// offset in digits, CS_BUS error signals below this will be supressed (=standalone operation assumed)
#define CS_BUS_SHIFT 8 		// CS_BUS integrator slope, 10:Ti=0.25s; 11:Ti=0.5s; 12:Ti=1s; 13:Ti=2s; ...
#define CS_BUS_SCALE 2114 	// scaling factor for output voltage offset (fractional number 0.0323 => range 0..1023 is scaled to 0..33 => du=600mV)
#define CS_BUS_PER 4700		// use 50 kHz for CS_BUS_OUT PWM


//PFC Bulk Voltage Tresholds for Enabling/Disabling DCDC
/*
#define BULK_TRE_ON 790 //750 old value
#define BULK_TRE_OFF 650
 */



#endif
