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
#include "control_pfc.h"
#include "serial.h"
#include "init.h"

// lookup table PTPER(I_in<<6)
// I1/f1 = 0.55A/20kHz; I2/f2 = 1.1A/100kHz
// int PTPER_TAB[32+1] = {11696, 11696, 11696, 6928, 3600, 2424, 2336, 2336, 2336, 2336, 2336, 2336, 2336, 2336, 2336, 2336, 2336, 2336, 2336, 2336, 2336, 2336, 2336, 2336, 2336, 2336, 2336, 2336, 2336, 2336, 2336, 2336, 2336};
// I1/f1 = 1A/20kHz; I2/f2 = 2A/96kHz
//int PTPER_TAB[32+1] = {11750, 11750, 11750, 11750, 10368, 8103, 6651, 5640, 4895, 4325, 3873, 3507, 3204, 2949, 2732, 2545, 2381, 2350, 2350, 2350, 2350, 2350, 2350, 2350, 2350, 2350, 2350, 2350, 2350, 2350, 2350, 2350, 2350};
int PTPER_TAB[32 + 1] = {5875, 5875, 5875, 5875, 5595, 5026, 4563, 4177, 3852, 3574, 3333, 3122, 2937, 2772, 2625, 2493, 2373, 2350, 2350, 2350, 2350, 2350, 2350, 2350, 2350, 2350, 2350, 2350, 2350, 2350, 2350, 2350, 2350};

// lookup table Kp(PTPER<<2) and Ki(PTPER<<2)
//int CURR_KP_TAB[32+1] = {22000, 22000, 22000, 22000, 22000, 21151, 20302, 19453, 18604, 17755, 16905, 16056, 15207, 14358, 13509, 12660, 11811, 10962, 10113, 9264, 8415, 7566, 6716, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000};
int CURR_KP_TAB[32 + 1] = {4400, 4400, 4400, 4400, 4400, 4400, 4400, 4400, 4400, 4400, 4400, 4400, 4400, 4400, 4400, 4400, 4400, 4400, 4400, 4400, 4400, 4400, 4400, 4400, 4400, 4400, 4400, 4400, 4400, 4400, 4400, 4400, 4400};
//int CURR_KI_TAB[32+1] = {6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000};
int CURR_KI_TAB[32 + 1] = {6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 5000, 5000, 5000, 5000, 5000, 5000, 5000, 5000, 5000, 5000, 5000, 5000, 5000};

// lookup table UDC(Ia<<6)
int UDC_TAB[32 + 1] = {805, 805, 805, 805, 801, 797, 793, 787, 781, 777, 773, 771, 767, 767, 767, 771, 777, 781, 785, 789, 793, 797, 801, 805, 805, 805, 805, 805, 805, 805, 805, 805, 805};
//int UDC_TAB[32+1] = {768, 768, 768, 768, 768, 770, 776, 782, 789, 794, 794, 794, 796, 797, 799, 801, 802, 804, 804, 804, 804, 804, 804, 804, 804, 804, 804, 804, 804, 804, 804, 804, 804};
//int UDC_TAB[32+1] = {794, 794, 794, 794, 794, 794, 794, 794, 794, 794, 794, 794, 796, 797, 799, 801, 802, 804, 804, 804, 804, 804, 804, 804, 804, 804, 804, 804, 804, 804, 804, 804, 804};
//int UDC_TAB[32+1] = {830, 830, 830, 830, 830, 830, 830, 830, 830, 830, 830, 830, 830, 830, 830, 830, 830, 830, 830, 830, 830, 830, 830, 830, 830, 830, 830, 830, 830, 830, 830, 830, 830};

/*
// function returns a int16 lookup value y which corresponds to the uint16 input value x
// value y is interpolated between two lookup table values
int lookup32(int* lookup_tab, unsigned x) {
        unsigned int p1, p2;
        p1 = x>>11; // upper 5 bits
        p2 = x&0x07FF; // lower 11 bits
        p2 <<= 5; // convert to unit16
        return lookup_tab[p1] + MUL16SX16FU(lookup_tab[p1+1]-lookup_tab[p1], p2);
}
 */

/***************************************************************************
Configuration Bit Settings
 ***************************************************************************/

_FOSCSEL(FNOSC_FRCPLL & IESO_ON)
_FOSC(POSCMD_NONE & FCKSM_CSECMD & OSCIOFNC_ON & IOL1WAY_OFF)
_FWDT(FWDTEN_OFF)
_FPOR(FPWRT_PWR128)
#ifndef __DEBUG
_FICD(ICS_NONE & JTAGEN_OFF)
#warning RELEASE MODE SELECTED
#else
_FICD(ICS_PGD1 & JTAGEN_OFF)
#warning DEBUG MODE SELECTED
#endif

/***************************************************************************
Global variables
 ***************************************************************************/

// common
//int test1 = 0x7FFF, test2 = 0x8000;
int ctrl_flags = 0; // control flags which are recieved via the uart
int pfc_status = 0; // pfc staus which are submitted via the uart
int sdelay = 85; // sample delay
int ss_tmr = 0; // soft start
int cnt = 0; //global counter for Temperature reading

// current controller
int i_ref = 0;
tPICURR32 PI_CURR_1 __attribute__((section(".xbss, bss, xmemory"))); // current controller #1
tPICURR32 PI_CURR_2 __attribute__((section(".xbss, bss, xmemory"))); // current controller #2
int curr_kp = CURR_KP;
int curr_ki = CURR_KI;

// voltage controller
int u_set = 0; // reference value for dc link voltage 
tPI32 PI_VOLT __attribute__((section(".xbss, bss, xmemory"))); // voltage controller
int u_ref, u_out;

// Over Temperature Protection
unsigned int temp_prim = 0; //default 0°C

// Global variables for all values read from ADCs
unsigned int pfcCurrentPhase1 = 0;
unsigned int pfcCurrentPhase2 = 0;
unsigned int pfcInputVoltage = 0;
unsigned int pfcOutputVoltage = 0;

// Data from DCDC
int dc_uout = 0;
int dc_iout = 0;

// Relay Control
char relayControlFlag = 0;

// frequency jitter
int pfc_per = PFC_PER; // pfc period 
int fswing_factor = 0x4000; // fractional, scaling is: 0x4000 = 1.0
int fswing_dir = FSWING_INC;

// relay timer
int rel_tmr = 0;

// Filters
// 

// bulk transient detect
tFIL2HISTORY Fil_bulk_trans_history __attribute__((section(".xbss, bss, xmemory"))) = {
    {0, 0},
    {0, 0}};
tFIL2COEFF Fil_bulk_trans_coeff __attribute__((section(".ybss, bss, ymemory"))) = {
    {0, 0, 0},
    {0, 0, 0}};

// line voltage
int uin_fil[48] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,};
int uin_fil_ptr = 0;
long uin_fil_sum_50 = 0;
long uin_fil_sum_60 = 0;
int uin_filtered = 0;
// ... peak detection
int uin_filtered_peak = 0;
int uin_filter_input = 0;
int uin_max_detect = 0;

// dc link voltage
int udc_fil[48] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,};
int udc_fil_ptr = 0;
long udc_fil_sum_50 = 0;
long udc_fil_sum_60 = 0;
int udc_filtered = 0;
int udc_filter_input = 0;

// input current, this is only needed for display purposes
int iin_fil[48] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,};
int iin_fil_ptr = 0;
long iin_fil_sum_50 = 0;
long iin_fil_sum_60 = 0;
int iin_filtered = 0;

/***************************************************************************
ISR: 		TIMER2 Interrupt
Description:	T2 interrupt is called with a frequency of 19200 Hz
 ***************************************************************************/
void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void) {
    static int t2_cnt = 0;
    unsigned int u;
    int i;
    static int uin4 = 0, udc4 = 0;

    IFS0bits.T2IF = 0; // clear the INT0 interrupt flag

    uin4 += pfcInputVoltage;
    udc4 += pfcOutputVoltage;

    ++t2_cnt;
    if ((t2_cnt & 3) == 0) { // do this every 4th cycle
        uin_filter_input = uin4;
        uin4 = 0;
        udc_filter_input = udc4;
        udc4 = 0;
        IFS1bits.INT2IF = 1; // trigger sw interrupt
    }

    // voltage feed forward (division by 0 saturates!)
    u = DIV32UBY16U(MUL16UX16U(2 * u_out, pfcInputVoltage), MUL16UX16U(uin_filtered_peak >> 2, uin_filtered_peak >> 2)); // [30 bits]/[20 bits] = [10 bits]
    if (u > 1023) u = 1023;
    if (u_ref == 0) u = 0;
    i_ref = u;
    //CMPDAC4 = i_ref;
    //CMPDAC4 = pfcInputVoltage;

#define DCM_CORR 1
#if DCM_CORR > 0
    // calculate correction factor for current measurement in dcm
    // corr = Ua/(Ua-Ue), range 1.0..16.0 is scaled to 2048..32768
    i = pfcOutputVoltage - pfcInputVoltage;
    if (i < (pfcOutputVoltage >> 4) + 1) i = (pfcOutputVoltage >> 4) + 1; // limit divisor to >= 1/16 uout
    u = DIV32SBY16S(MUL16SX16S(2048, pfcOutputVoltage), i);
    PI_CURR_1.cor = u;
    PI_CURR_2.cor = u;
#else
    PI_CURR_1.cor = 0xFFFF; // disable dcm correction
    PI_CURR_2.cor = 0xFFFF; // disable dcm correction
#endif

    /*
    // if Ipeak > dU*dt/Lmin we are safely in CCM => disable DCM correction
    if(pfcCurrentPhase1 > (pfcOutputVoltage-pfcInputVoltage)) {
            //CMPDAC4 = 0x3FF;
            PI_CURR_1.cor = 0xFFFF; // disable dcm correction
    PI_CURR_2.cor = 0xFFFF;
} else {
    //CMPDAC4 = 0;
}
     */

}
/***************************************************************************
End of ISR
 ***************************************************************************/

/***************************************************************************
ISR: 		Softwareinterrupt 2
Description: INT2 interrupt is called every 4th cycle of T2 interrupt (4800 Hz)	
                         voltage Controller is included
 ***************************************************************************/
void __attribute__((__interrupt__, no_auto_psv)) _INT2Interrupt(void) {
    int i;
    unsigned u;
    static long sum50 = 0, sum60 = 0;
    static int cor50 = 0, cnt50 = 0, cor60 = 0, cnt60 = 0;
    static int freq_sw_cnt = 0, line_freq = 50;
    static int uin_filter_max = 0;
    static int uin_filter_input_last = 0;
    static char peakDetectFlag = 0;
    static char upcnt = 0, downcnt = 0;
    static int u_start = 0; // start value for soft start ramp;
    static int pfc_per1 = PFC_PER; // internal pfc_per (before jitter is applied)
    static int boost = 0;
    static int off_detect = 0;

    IFS1bits.INT2IF = 0; // clear interrupt flag

    // moving average filter for line voltage (an4)
    uin_fil_sum_50 -= uin_fil[uin_fil_ptr];
    uin_fil_sum_60 -= uin_fil[(uin_fil_ptr >= 40) ? (uin_fil_ptr - 40) : (uin_fil_ptr + 48 - 40)];
    uin_fil[uin_fil_ptr] = uin_filter_input;
    uin_fil_sum_50 += uin_fil[uin_fil_ptr];
    uin_fil_sum_60 += uin_fil[uin_fil_ptr];
    if (++uin_fil_ptr >= 48) uin_fil_ptr = 0;

    // moving average filter for output voltage (an5)
    udc_fil_sum_50 -= udc_fil[udc_fil_ptr];
    udc_fil_sum_60 -= udc_fil[(udc_fil_ptr >= 40) ? (udc_fil_ptr - 40) : (udc_fil_ptr + 48 - 40)];
    udc_fil[udc_fil_ptr] = udc_filter_input;
    udc_fil_sum_50 += udc_fil[udc_fil_ptr];
    udc_fil_sum_60 += udc_fil[udc_fil_ptr];
    if (++udc_fil_ptr >= 48) udc_fil_ptr = 0;

    // moving average filter for input current
    iin_fil_sum_50 -= iin_fil[iin_fil_ptr];
    iin_fil_sum_60 -= iin_fil[(iin_fil_ptr >= 40) ? (iin_fil_ptr - 40) : (iin_fil_ptr + 48 - 40)];
    iin_fil[iin_fil_ptr] = 4 * i_ref; //iin_filter_input;
    iin_fil_sum_50 += iin_fil[iin_fil_ptr];
    iin_fil_sum_60 += iin_fil[iin_fil_ptr];
    if (++iin_fil_ptr >= 48) iin_fil_ptr = 0;

    // detect peak input voltage
    if ((uin_filter_input > uin_filter_max) && (peakDetectFlag == 1)) {
        uin_filter_max = uin_filter_input;
    }
    if (uin_filter_input > uin_filter_input_last) {
        upcnt++;
        downcnt = 0;

        if ((upcnt >= 5) && (peakDetectFlag == 0)) {
            peakDetectFlag = 1;
        }
    } else if (uin_filter_input < uin_filter_input_last) {
        upcnt = 0;
        downcnt++;

        if ((downcnt >= 5) && (peakDetectFlag == 1)) {
            peakDetectFlag = 0;
            uin_max_detect = uin_filter_max >> 2;
            uin_filtered_peak = MUL16SX16FU(uin_filter_max, 46341u) >> 2;
            uin_filter_max = 0;
        }

    }
    uin_filter_input_last = uin_filter_input;

    // determine line frequency by correlation filter (small sums mean good correlation)
    // 50 Hz correlation:
    if (cnt50 < 48)
        sum50 += uin_filter_input;
    else
        sum50 -= uin_filter_input;
    if (++cnt50 >= (2 * 48)) {
        cnt50 = 0;
        cor50 = sum50 >> 5;
        if (cor50 < 0) cor50 = -cor50;
        sum50 = 0;
    }
    // 60 Hz correlation:
    if (cnt60 < 40)
        sum60 += uin_filter_input;
    else
        sum60 -= uin_filter_input;
    if (++cnt60 >= (2 * 40)) {
        cnt60 = 0;
        cor60 = sum60 >> 5;
        if (cor60 < 0) cor60 = -cor60;
        sum60 = 0;
    }
    if (freq_sw_cnt >= 480) // 50 Hz detected for >100 ms
        line_freq = 50;
    else if ((cor60 > 10) && (cor50 < cor60)) // better correlation with 50 Hz
        ++freq_sw_cnt; // 1 step towards 50 Hz position
    if (freq_sw_cnt <= -480) // 60 Hz detected for >100 ms
        line_freq = 60;
    else if ((cor50 > 10) && (cor60 < cor50)) // better correlation with 50 Hz
        --freq_sw_cnt; // 1 step towards 60 Hz position

    // calculate uin_filtered and udc_filtered depending on line frequency
    if (line_freq == 50) {
        uin_filtered = (1365L * uin_fil_sum_50) >> (16 + 2); // div by 48, [10 bits]
        udc_filtered = (1365L * udc_fil_sum_50) >> (16 + 2); // div by 48, [10 bits]
        iin_filtered = (2 * 1365L * iin_fil_sum_50) >> (16 + 2); // div by 48, [10 bits]
    } else { // line_freq==60
        uin_filtered = (1638L * uin_fil_sum_60) >> (16 + 2); // div by 40, [10 bits]
        udc_filtered = (1638L * udc_fil_sum_60) >> (16 + 2); // div by 40, [10 bits]
        iin_filtered = (2 * 1638L * iin_fil_sum_60) >> (16 + 2); // div by 40, [10 bits]
    }
    CMPDAC4 = uin_filtered;
    /*
// detect bulk voltage transients
    {
            static long int udc_fil_sum_old = 0;
            int du;
		
            if(line_freq==50) {
                    du = udc_fil_sum_50 - udc_fil_sum_old;
                    udc_fil_sum_old = udc_fil_sum_50;
            } else { // line_freq==60
                    du = udc_fil_sum_60 - udc_fil_sum_old;
                    udc_fil_sum_old = udc_fil_sum_60;
            }
            du = FIL_2ORD(&Fil_bulk_trans_history, &Fil_bulk_trans_coeff, du);
            // a +20A load step causes du to be approx. -30 ...
            CMPDAC4 = 512 + 4*du;
    }
     */

    // soft start
    // with sinusoidal input voltage => uin_filtered = 2/Pi * Uin,pk = 0.9 * Uin,rms = 1.81*uin_filtered

    if ((uin_filtered_peak > SWON_LO) && (uin_filtered_peak < SWON_HI)) {
        pfc_status |= PFC_UIN_OK;
        off_detect = 0;
    }
    if (((uin_filtered_peak < SWOFF_LO) && (uin_filtered < (SWOFF_LO_AVG))) || ((uin_filtered > (SWOFF_HI_AVG)) && ((uin_filtered_peak) > SWOFF_HI))) {
        //	if(((uin_filtered_peak<SWOFF_LO)&&(uin_filtered<SWOFF_LO))||((uin_filtered>SWOFF_HI)&&((uin_filtered_peak)<SWOFF_HI))) {//uin_filtered_peak>SWOFF_HI
        off_detect++; //pfc_status &= ~PFC_UIN_OK;
    }
    if (off_detect >= 750) { //750 = 150ms
        pfc_status &= ~PFC_UIN_OK;
        off_detect = 0;
        LATBbits.LATB4 = 1;
    }

    if (ss_tmr <= 0) { // state is "delay"
        u_ref = 0;
        u_start = udc_filtered;
        if ((pfc_status & PFC_UIN_OK) != 0) {// uin_filtered within switch on limits
            ++ss_tmr;
        } else {
            ss_tmr = -SS_DELAY; // restart timer
        }
        pfc_status &= ~PFC_PWR_GOOD; // clear bit
    } else { // state is "ramp/on"
        if ((pfc_status & PFC_UIN_OK) == 0) { // out of switch off limits
            ss_tmr = -SS_DELAY; // restart timer
            u_ref = 0;
            pfc_status &= ~PFC_PWR_GOOD; // clear bit
        } else if (ss_tmr < SS_RAMP) { // state is "ramp"
            ++ss_tmr;
            u = DIV32SBY16S(32767L * (long) ss_tmr, SS_RAMP); // 0..32767
            u_ref = u_start + MUL16SX16FS(u_set - u_start, u);
            pfc_status &= ~PFC_PWR_GOOD; // clear bit
            LATBbits.LATB4 = 1;
        } else { // state is "on"
            // u_ref adaption
            if (u_set > u_ref) { // u_ref shall increase => do this immediately!
                u_ref = u_set;
            } else if (u_set < u_ref) { // ...decrease, this is done slowly
                static int cnt256 = 0;
                if ((++cnt256 & 255) == 0) --u_ref;
            }
            //setting for Input Voltage is > Vbulk
            if (uin_max_detect >= (u_ref - 80)) {
                u_ref = uin_max_detect + 80;
            }

            if (u_ref >= UDC_TAB[32]) {
                u_ref = UDC_TAB[32];
            }
            pfc_status |= PFC_PWR_GOOD; // set bit
            LATBbits.LATB4 = 0;
        }
    }
    //CMPDAC4 = u_ref;

    // 'BOOST' mode: whenever (u_ref - udc_filtered) > BOOST_THRESHOLD(25V), boost will be activated
    i = u_ref - udc_filtered;
    if (i <= 0) {
        boost = 0;
    } else if (i > BOOST_THRESHOLD) {
        boost = 1;
    }
    if (boost) {
        PI_VOLT.Kp = VOLT_KP;
        PI_VOLT.Ki = BOOST_FACTOR*VOLT_KI;
    } else {
        PI_VOLT.Kp = VOLT_KP;
        PI_VOLT.Ki = VOLT_KI;
    }
    //CMPDAC4 = boost<<9;

    // voltage controller
    if (u_ref == 0) {
        PI_VOLT.sum = 0;
    }
    u_out = PI32(&PI_VOLT, u_ref, udc_filtered); // [15 bits]

    // pfc frequency adaption, f(i_ref)
    {
        int lu_per;
        lu_per = LOOKUP32_16U_16I(&PTPER_TAB[0], i_ref << 6);
        if (lu_per < pfc_per1) { // frequency shall increase => do this immediately!
            pfc_per1 = lu_per;
        } else if (lu_per > pfc_per1) { // ...decrease, this is done slowly
            static int cnt32 = 0;
            if ((++cnt32 & 31) == 0) ++pfc_per1;
        }
        curr_kp = LOOKUP32_16U_16I(&CURR_KP_TAB[0], pfc_per1 << 2);
        curr_ki = LOOKUP32_16U_16I(&CURR_KI_TAB[0], pfc_per1 << 2);
        PI_CURR_1.Kp = curr_kp;
        PI_CURR_2.Kp = curr_kp;
        PI_CURR_1.Ki = curr_ki;
        PI_CURR_2.Ki = curr_ki;
    }

    // frequency jitter
#if JITTERSRC==1
    // Synchronize PTPER do internally generated fswing_factor
    fswing_factor += fswing_dir;
    if (fswing_factor >= FSWING_MAX) fswing_dir = -FSWING_INC;
    if (fswing_factor <= FSWING_MIN) fswing_dir = FSWING_INC;
    //CMPDAC4 = 512 + (fswing_factor-0x4000)/4;
    pfc_per = 2 * MUL16SX16FS(pfc_per1, fswing_factor);
    //CMPDAC4 = 512 + (pfc_per-pfc_per1)/2;
#else
    // No jitter, do not change PTPER
    pfc_per = pfc_per1;
#endif

    // serve relay
    if (rel_tmr > 0) {
        OC1RS = REL_PWM_PER - 1; // 100% dc
        --rel_tmr;
    } else {
        OC1RS = REL_PWM_HOLD;
    }


}
/***************************************************************************
End of ISR
 ***************************************************************************/

/***************************************************************************
ISR: 		ADCP0Interrupt
Description:	Interrupt for Currentcontroller of PFC-Stage one
 ***************************************************************************/
void __attribute__((__interrupt__, no_auto_psv)) _ADCP0Interrupt() {
    int out;
    static unsigned pfc_per_old = 0;

    ///LATCbits.LATC5 = 1;	// set GPIO7
    IFS6bits.ADCP0IF = 0;

    if (pfc_per != pfc_per_old) {
        PTPER = pfc_per & 0xFFF8;
        PHASE2 = PTPER >> 1;
        pfc_per_old = pfc_per;
    }

    pfcCurrentPhase1 = ADCBUF0;
    //CMPDAC4 = pfcCurrentPhase1;

    PI_CURR_1.psc = PTPER;
    out = PI_CURR_32(&PI_CURR_1, i_ref, pfcCurrentPhase1);
    //CMPDAC4 = out>>2;
    PDC1 = out;
    TRIG1 = (out >> 1) + sdelay;

    ADSTATbits.P0RDY = 0; // Clear the data is ready in buffer bits
}
/***************************************************************************
End of ISR
 ***************************************************************************/

/***************************************************************************
ISR: 		ADCP1Interrupt
Description:	Interrupt for Currentcontroller of PFC-Stage two
 ***************************************************************************/
void __attribute__((__interrupt__, no_auto_psv)) _ADCP1Interrupt() {
    int out;

    IFS6bits.ADCP1IF = 0;

    pfcCurrentPhase2 = ADCBUF2;
    //CMPDAC4 = pfcCurrentPhase2;

    pfcInputVoltage = ADCBUF4;
    pfcOutputVoltage = ADCBUF5;
    temp_prim = ADCBUF6; //Read Primary Temperature
    // CMPDAC4 = ADCBUF5;

    PI_CURR_2.psc = PTPER;
    out = PI_CURR_32(&PI_CURR_2, i_ref, pfcCurrentPhase2);
    //CMPDAC4 = out>>2;
    PDC2 = out;
    TRIG2 = (out >> 1) + sdelay;

    ADSTATbits.P1RDY = 0; // Clear the data is ready in buffer bits
}
/***************************************************************************
End of ISR
 ***************************************************************************/

/***************************************************************************
Function: 	main
Description:	main routine of the programm
 ***************************************************************************/

int main(void) {
    //int cnt = 0;

    // init PI_CURR_1
    PI_CURR_1.Ki = CURR_KI;
    PI_CURR_1.sum = 0;
    PI_CURR_1.Kp = CURR_KP;
    PI_CURR_1.psc = PFC_PER;
    PI_CURR_1.out = 0;
    PI_CURR_1.cor = 0xFFFF;

    // init PI_CURR_2
    PI_CURR_2.Ki = CURR_KI;
    PI_CURR_2.sum = 0;
    PI_CURR_2.Kp = CURR_KP;
    PI_CURR_2.psc = PFC_PER;
    PI_CURR_2.out = 0;
    PI_CURR_2.cor = 0xFFFF;

    // init PI_VOLT (voltage)
    PI_VOLT.Kp = VOLT_KP;
    PI_VOLT.Ki = VOLT_KI;
    PI_VOLT.sum = 0;

    // init filter coefficients (bulk transient), Fg=500 @Fs=4800
    Fil_bulk_trans_coeff.a[0] = 1180;
    Fil_bulk_trans_coeff.a[1] = 2359;
    Fil_bulk_trans_coeff.a[2] = 1180;
    Fil_bulk_trans_coeff.b[0] = 0;
    Fil_bulk_trans_coeff.b[1] = -18170;
    Fil_bulk_trans_coeff.b[2] = 6521;

    // Call Init Fcts
    init_CLOCK();
    init_PORTS();
    init_ADC();
    init_TIMER2();
    init_INT();
    init_PWM();
    init_DAC();
    //
    init_Serial();
    init_TIMER3();
    // init OC1 to generate relay PWM
    OC1CONbits.OCTSEL = 1; // use timer3 as source
    OC1CONbits.OCM = 6; // PWM mode without fault detection
    OC1RS = 0; // 0% DC

    PTCONbits.PTEN = 1; // Enable the PWM Module

    ADCBUF6 = 0;

    rel_tmr = 4800; // force relay on

    while (1) {
        unsigned int databuffer[10];

        // input: read data from serial interface
        if (get_datablock(&databuffer[0]) > 0) { // a new datablock was recieved successfully
            dc_uout = databuffer[0];
            dc_iout = databuffer[1];
            ctrl_flags = databuffer[2];
            //CMPDAC4 = (++cnt&1)<<9;
        }

        // process input values
        if ((ctrl_flags & 0x0001) != 0) { // pfc switched off by serial command
            ss_tmr = -SS_DELAY; // switch off, prepare soft start
        } else if (ss_tmr >= SS_RAMP) { // soft start finished
            u_set = LOOKUP32_16U_16I(&UDC_TAB[0], dc_iout << 6);
            // TODO: maybe it's better to use udc(dc_p_out) instead of udc(dc_i_out)
        } else {
            u_set = UDC_TAB[32]; // start with highest voltage
        }

        //CMPDAC4 = u_set;
        //CMPDAC4 = (u_set-738)<<4;

        // output: write to serial interface
        if (tx_active == 0) { // no transmission in progress
            databuffer[0] = udc_filtered;
            databuffer[1] = uin_filtered_peak;
            databuffer[2] = iin_filtered;
            databuffer[3] = temp_prim;
            databuffer[4] = PTPER;
            databuffer[5] = curr_kp;
            databuffer[6] = curr_ki;
            databuffer[7] = PI_VOLT.Kp;
            databuffer[8] = PI_VOLT.Ki;
            databuffer[9] = pfc_status;
            write_datablock(&databuffer[0]);
            //CMPDAC4 = (++cnt&1)<<9;
        }
    }
}
/***************************************************************************
End of function Main
 ***************************************************************************/








