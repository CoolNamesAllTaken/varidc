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
#include "i2c.h"
#include "serial.h"
#include "control_dcdc.h"
#include "init.h"
#include "main.h"

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
_FICD(ICS_PGD3 & JTAGEN_OFF)
#warning DEBUG MODE SELECTED
#endif

/***************************************************************************
Global variables
 ***************************************************************************/
// lookup table PTPER(i_out<<6)
// I1/f1 = 25A/80kHz; I2/f2 = 35A/90kHz
int PTPER_TAB[32 + 1] = {3357, 3357, 3357, 3357, 3357, 3357, 3321, 3234, 3151, 3133, 3095, 2964, 2937, 2937, 2937, 2931, 2782, 2648, 2526, 2448, 2448, 2448, 2448, 2448, 2448, 2448, 2448, 2448, 2448, 2448, 2448, 2448, 2448};

// Variables for Temperature Test
unsigned int test_time_ms = 0;
unsigned int test_time_h = 0;
int ss_tmr = 0; // soft start timer
int volt_start = 0; //start value for soft start
int dcdc_enabled = 0;

int duty = DUTY_DEFAULT; // 40 %
int i_prim = 0; // primary current
//
int t_sync_on = T_SYNC_ON; // 30 ns
int t_sync_off = T_SYNC_OFF; // 200 
int t_adc = T_ADC; // 260 ns

// voltage controller
tPI32 PI_VOLT __attribute__((section(".xbss, bss, xmemory"))); // voltage controller
int volt_kp = VOLT_KP;
int volt_ki = VOLT_KI;
int volt_ref = VOLT_REF;

// current share bus (CSBUS)
int cs_bus_in = 0;
int cs_bus_out = 0;
int cs_bus_volt_offs = 0; // voltage offset set by current share logic
long int cs_bus_integrator = 0;

// current controller
tPI32 PI_CURR __attribute__((section(".xbss, bss, xmemory"))); // current controller
int curr_ref = 0;
int curr_max = 0;
int climit_mode = 0;
long int climit_cnt = 0;

// Temperature 1 Default Values
int Temp_Sec1 = TEMP_SEC1_DEF; //Semiconductor Temperature
int Temp_Sec2 = TEMP_SEC2_DEF; //Ambient Temperature of Device
//int Temp_Prim = 155;			//Primary Heatsink Temperature

// Fault variables
unsigned int fault_state = 0; // one bit for each type of fault

// Temperature Protection
int maxTempReading = 0;

// enabling SyncFET driver channel
int SyncRecState = 0;
//...filtering primary current
tFIL1HISTORY Fil_iprim_history __attribute__((section(".xbss, bss, xmemory"))) = {
    {0},
    {0}};
tFIL1COEFF Fil_iprim_coeff __attribute__((section(".ybss, bss, ymemory"))) = {
    {0, 0},
    {0, 0}};
//...filtering output current
tFIL1HISTORY Fil_iout_history __attribute__((section(".xbss, bss, xmemory"))) = {
    {0},
    {0}};
tFIL1COEFF Fil_iout_coeff __attribute__((section(".ybss, bss, ymemory"))) = {
    {0, 0},
    {0, 0}};
int i_prim_filtered = 0;
int i_out_filtered = 0;

// frequency jitter
int dcdc_per = DCDC_PER;
int fswing_factor = 0x4000; // fractional, scaling is: 0x4000 = 1.0
int fswing_dir = FSWING_INC;


//*************************************************************************************

// UART communication to PFC stage
//
// the following data is read from the PFC stage
int pfc_udc_filtered = 0;
int pfc_uin_filtered = 0;
int pfc_iin_filtered = 0;
int pfc_temp_prim = 0;
int pfc_ptper = 0;
int pfc_curr_ki = 0;
int pfc_curr_kp = 0;
int pfc_volt_ki = 0;
int pfc_volt_kp = 0;
int pfc_status = 0;
//
// the following parameters are written to the PFC stage
int u_out = 0;
int i_out = 0;
int pfc_ctrl_flags = 0;

/***************************************************************************
ISR: 		TIMER1 Interrupt
Description:	T1 interrupt is called with a frequency of 5 Hz for
                                FAN Control LED Signalizing
 ***************************************************************************/
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void) // 5 Hz
{
    static char timer1InterruptCounter = 0;

    Temp_Sec1 = ADCBUF8; // Read secondary semiconductor Temperature
    /*
    ??? This is the same device (MCP9700A) as for Temp_Sec1, so why scaling it in a different way?
    Temp_Sec2 = ((ADCBUF10*3)>>2);	// Read secondary ambient Temperature of Device
                                                                    // Scaled to other temp sensors
     */
    Temp_Sec2 = ADCBUF10; // Read secondary semiconductor Temperature
    //CMPDAC4=Temp_Sec2;

    // Fan Control Software (Determine Fan Speed based on Temperature)----------------------------------------
    // Determine Max Temp

    if (((Temp_Sec1 + TEMP_SEC1_OFFSET)>(Temp_Sec2 + TEMP_SEC2_OFFSET)) && ((Temp_Sec1 + TEMP_SEC1_OFFSET)>(pfc_temp_prim + TEMP_PRIM_OFFSET))) {
        maxTempReading = Temp_Sec1 + TEMP_SEC1_OFFSET;
    } else if (((Temp_Sec2 + TEMP_SEC2_OFFSET)>(Temp_Sec1 + TEMP_SEC1_OFFSET)) && ((Temp_Sec2 + TEMP_SEC2_OFFSET)>(pfc_temp_prim + TEMP_PRIM_OFFSET))) {
        maxTempReading = Temp_Sec2 + TEMP_SEC2_OFFSET;
    } else {
        maxTempReading = pfc_temp_prim + TEMP_PRIM_OFFSET;
    }

    // Set Fan Speed based on Max Temperature

    if ((maxTempReading > TEMPLOWSETTING) && (maxTempReading < TEMPHIGHSETTING)) {
        //pfc_ctrl_flags = 0; //later if Temp limits are set
        PDC3 = FANSPEED_HALF;
    } else if ((maxTempReading > TEMPHIGHSETTING) && (maxTempReading < TEMPEXTREMESETTING)) {
        PDC3 = FANSPEED_FULL;
    } else {
        PDC3 = FANSPEED_DEFAULT;
    }
    //end of FAN Speed Control------------------------------------------------------------------------------

    //LED Signalizing---------------------------------------------------------------------------------------
    if (++timer1InterruptCounter >= 2) {
        timer1InterruptCounter = 0;
        if (fault_state > 1) { // any fault occured (ignore bit '0')
            static char faultCounter = 0;
            char fault_type; // this number contains highest '1' bit of fault_state
            char i;
            // convert fault_state to fault_type
            i = fault_state & 0xFFFE;
            fault_type = 0;
            while ((i & 1) == 0) {
                ++fault_type;
                i >>= 1;
            }
            // display fault_type (this is the old code)
            if (++faultCounter >= (fault_type << 1)) {
                LATBbits.LATB8 = 1;
                if (faultCounter >= ((fault_type + 2) << 1)) faultCounter = 0;
            } else {
                LATBbits.LATB8 ^= 1; // toggle red LED indicating which fault occured
            }
        } else { // no fault => switch off red led
            LATBbits.LATB8 = 1;
        }
    }
    //end of LED Signalizing--------------------------------------------------------------------------------

    IFS0bits.T1IF = 0; // Reset T1 interrupt has not occured flag
}
/***************************************************************************
End of ISR
 ***************************************************************************/

/***************************************************************************
ISR: 		TIMER2 Interrupt
Description:	T2 interrupt is called with a frequency of 4800 Hz
 ***************************************************************************/
void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void) // 4800 Hz
{
    unsigned volt;
    static int fswing_dir = 1;
    static int dcdc_per1 = DCDC_PER; // internal dcdc_per (before jitter is applied)

    IFS0bits.T2IF = 0; // clear the INT0 interrupt flag

    // 50Hz 1st order filter for primary current
    i_prim_filtered = FIL_1ORD(&Fil_iprim_history, &Fil_iprim_coeff, i_prim);
    // 50Hz 1st order filter for output current
    i_out_filtered = FIL_1ORD(&Fil_iout_history, &Fil_iout_coeff, i_out);


    // do dcdc frequency adaption depending on load conditions
    // we calculate dcdc_per1(i_out) here:
    {
        int lu_per;
        lu_per = LOOKUP32_16U_16I(&PTPER_TAB[0], i_out << 6);
        if (lu_per < dcdc_per1) { // frequency shall increase => do this immediately!
            dcdc_per1 = lu_per;
        } else if (lu_per > dcdc_per1) { // ...decrease, this is done slowly
            static int cnt32 = 0;
            if ((++cnt32 & 31) == 0) ++dcdc_per1;
        }
    }
    //dcdc_per1 = DCDC_PER;

    //Softstart------------------------------------------------------
    if (dcdc_enabled == 0) {
        volt_start = u_out;
        volt_ref = 0;
        cs_bus_integrator = 0;
        ss_tmr = 0;
    } else if (ss_tmr < SS_RAMP) { // state is "ramp"
        ++ss_tmr;
        volt = DIV32SBY16S(32767L * (long) ss_tmr, SS_RAMP); // 0..32767
        volt_ref = volt_start + MUL16SX16FS(VOLT_REF - volt_start, volt);
        cs_bus_integrator = 0;
    } else { // state is "on"
        volt_ref = VOLT_REF + cs_bus_volt_offs;
    }

    // frequency jitter
#if JITTERSRC==1
    // Synchronize PTPER do internally generated fswing_factor
    fswing_factor += fswing_dir;
    if (fswing_factor >= FSWING_MAX) fswing_dir = -FSWING_INC;
    if (fswing_factor <= FSWING_MIN) fswing_dir = FSWING_INC;
    //CMPDAC4 = 512 + (fswing_factor-0x4000)/4;
    dcdc_per = 2 * MUL16SX16FS(dcdc_per1, fswing_factor);
    //CMPDAC4 = 512 + (dcdc_per-dcdc_per1)/2;
#else
    // No jitter, do not change PTPER
    dcdc_per = dcdc_per1;
#endif

    // overcurrent detection
    {
        static char div48 = 0;
        if (++div48 >= 48) { // this is done every 10 ms
            div48 = 0;
            ++climit_cnt;
        }

        // overcurrent condition: (u_out<=0.8*VOLT_REF) && (i_prim_filtered>=0.9*CURR_MAX) for >= 5s
        if (u_out >= (VOLT_REF * 80L / 100L)) climit_cnt = 0;
        if (i_out_filtered <= (CURR_MAX * 90L / 152L)) climit_cnt = 0; // use scaling factor for isec
    }

    // current sense bus (CS_BUS)
    {
        int e, y;
        unsigned u;

        // CS_BUS is driven with output value of _this_ device, 100% load must be scaled to 1023
        // current derating is considered when scaling to 100%
        u = DIV32UBY16U(((unsigned long) i_prim_filtered) << 10, curr_max);
        if (u > 1023) u = 1023;
        if (dcdc_enabled == 0) u = 0; // device is off => don't drive csbus!
        cs_bus_out = u;
        // now map range 0..1023 of cs_bus_out signal to 0..100% pwm duty cycle
        PDC4 = MUL16SX16FU(CS_BUS_PER, (unsigned) cs_bus_out << 6);

        // read input value from csbus, cs_bus_in=1023 means 100% load
        cs_bus_in = ADCBUF11;
        // calculate error e = (cs_bus_in-CS_BUS_OFFS) - cs_bus_out,
        // a positive e means that _this_ device must increase it's power share
        // the offset of CS_BUS_OFFS ensures that in standalone operation e always will be less than 0
        e = (cs_bus_in - CS_BUS_OFFS) - cs_bus_out;
        // the error signal e feeds cs_bus_integrator
        cs_bus_integrator += e;
        // limit integrator sum (y is integrator output)
        if (cs_bus_integrator < 0) cs_bus_integrator = 0;
        if (cs_bus_integrator > (1023L << CS_BUS_SHIFT)) cs_bus_integrator = (1023L << CS_BUS_SHIFT);
        y = cs_bus_integrator >> CS_BUS_SHIFT;
        // integrator output y is scaled to the voltage offset range
        cs_bus_volt_offs = MUL16SX16FU(y, CS_BUS_SCALE); // y = 0..1023 is scaled to cs_bus_volt_offs = 0..33 = 0..600mV
        // cs_bus_volt_offs must be added to the output voltage's reference value
    }

}
/***************************************************************************
End of ISR
 ***************************************************************************/

/***************************************************************************
ISR: 		PWM Special Event Match Interrupt
Description: This Interrupt is called, when PWM Module 1 goes high or
                         PWM2 goes High
 ***************************************************************************/
void __attribute__((__interrupt__, no_auto_psv)) _PWMSpEventMatchInterrupt() {
    int climit;
    static int dc;
    static int di = 0;
    int i;
    static unsigned dcdc_per_old = 0;

    // Modellbasierte Regelung im Kurzschluss-/Hochlaufbetrieb:
    // Da LEB = 0.5us ist, wird die Ausgangsdrossel im ersten Takt um max. dI1 = 0.5us*(16V-ua) - 4.5us*(ua+0.5V) zu hoch aufmagnetisiert.
    // In allen folgenden Takten wird die Drossel mit dI2 = 5us*(ua+0.5V) abmagnetisiert.
    // Wenn dI1 positiv ist (zu hohe Aufmagnetisierung), muss die Taktung für einige Pulse ausgesetzt werden.
    // In jedem der Aussetzpulse sinkt der Drosselstrom um den Betrag dI2.
    climit = ((PWMCON1 | PWMCON2)&0x4000) != 0;
    if (climit) {
        PWMCON1bits.CLIEN = 0;
        PWMCON2bits.CLIEN = 0;
        PWMCON1bits.CLIEN = 1;
        PWMCON2bits.CLIEN = 1;
    }
    if (di > 0) {
        di -= 50 * (25 + ADCBUF0); // 50*(25+ua)
    } else if (climit) {
        di = 2945 - 50 * u_out; // 5*(814-ua) - 45*(25+ua)
    }

    if (SEVTCMP == 0) {
        // in this part, PWM2 (T2 and S2) is inactive,
        // so PWM2 values can be modified
        // ADC2+3 and ADC4+5 results are valid
        ///LATCbits.LATC5 = 1;	// set GPIO7
        i_out = ADCBUF4;
        u_out = ADCBUF5;

        // voltage controller
        i = PI32B(&PI_VOLT, volt_ref, u_out); // -CURR_MAX..CURR_MAX (e.g. 11.97*65A = 778)
        // i += MUL16SX16FU(i_out, 50545u); // Ia feed-forward, Is = 11.97 digits/A, Iout = 15.5 digits/A
        i += MUL16SX16FU(i_out, 42916u); // Ia feed-forward, Is = 10.15 digits/A, Iout = 15.5 digits/A
        if (i > curr_max) i = curr_max; // limit current reference to CURR_MAX
        if (i < 0) i = 0; // negative current reference is not allowed
        if (volt_ref == 0) i = 0; // force zero current in disabled state
        curr_ref = i;
        //
        if (di > 0) { // pulse inhibit active
            SDC2 = 0;
            PDC2 = 0;
        } else if (dc > 0) {
            SDC2 = dc;
            PDC2 = dc + t_sync_off;
        } else {
            SDC2 = 0;
            PDC2 = 0;
        }

        // change PTPER here if neccessary
        if (dcdc_per != dcdc_per_old) {
            PTPER = dcdc_per & 0xFFF8;
            PHASE2 = PTPER >> 1;
            SPHASE2 = PTPER >> 1;
            dcdc_per_old = dcdc_per;
        }

        DTR2 = t_sync_on;
        STRIG2 = (dc >> 1) + t_adc; // AN4+5 trigger
        SEVTCMP = PTPER >> 1;

        ///LATCbits.LATC5 = 0;	// clear GPIO7

    } else {
        long l;
        // in this part, PWM1 (T1 and S1) is inactive,
        // so PWM1 values can be modified
        // ADC0+1 results are valid
        ///LATCbits.LATC6 = 1;	// set GPIO6
        // calculate primary current i_prim
        if (climit || (di > 0)) {
            i_prim = CURR_MAX;
        } else {
            i_prim = ADCBUF0;
        }
        // do the current control job:
        duty = PI32(&PI_CURR, curr_ref, i_prim);
        // set lower limit for integrator
        // at nominal output voltage this limit is 60%, it will decrease at smaller output voltages
        // lower limit is calculated by l = Ua/20.11V
        l = u_out;
        l <<= 16 + 5; // output voltage, u_max(=20.11V) scaled to 0x7FFF0000
        if (PI_CURR.sum < l) PI_CURR.sum = l;
        if (duty > DUTY_LIMIT) duty = DUTY_LIMIT; // upper limit for duty cycle [99%]
        if (curr_ref == 0) duty = 0; // override current controller output when duty==0
        dc = MUL16SX16FU(PTPER, duty); // set to PTPER/2 for duty = 32767
        //
        if (di > 0) { // pulse inhibit active
            SDC1 = 0;
            PDC1 = 0;
        } else if (dc > 0) {
            SDC1 = dc;
            PDC1 = dc + t_sync_off;
        } else {
            SDC1 = 0;
            PDC1 = 0;
        }
        DTR1 = t_sync_on;
        STRIG1 = (dc >> 1) + t_adc; // AN0+1 trigger
        SEVTCMP = 0;

        ///LATCbits.LATC6 = 0;	// clear GPIO6

    }
    IFS3bits.PSEMIF = 0; // clear interrupt flag
}
/***************************************************************************
End of ISR
 ***************************************************************************/

/***************************************************************************
Function: 	serve_SyncRec
Description:	enable/disable sync rectifiers
 ***************************************************************************/
void serve_SyncRec(void) {
    if (i_prim_filtered >= IOUT2SYNC_ON) { // all Sync FETs on
        IOCON1bits.PENH = 1; // PWM1H (S1) is controlled by PWM module
        IOCON2bits.PENH = 1; // PWM2H (S2) is controlled by PWM module
        LATBbits.LATB12 = 1;
        LATCbits.LATC12 = 1;
        SyncRecState = 0;
    } else if (i_prim_filtered <= IOUT2SYNC_OFF) { //all Sync FETs off
        if (i_prim_filtered <= IOUTSYNC_OFF) { //all Sync FETS off
            IOCON1bits.PENH = 0; // PWM1H (S1) is controlled by PWM module
            IOCON2bits.PENH = 0; // PWM2H (S2) is controlled by PWM module
            LATBbits.LATB12 = 0;
            LATCbits.LATC12 = 0;
            SyncRecState = 2;
        } else {//one Sync FET off
            IOCON1bits.PENH = 1; // PWM1H (S1) is controlled by PWM module
            IOCON2bits.PENH = 1; // PWM2H (S2) is controlled by PWM module
            LATBbits.LATB12 = 0;
            LATCbits.LATC12 = 0;
            SyncRecState = 1;
        }
    } else { //all Sync FETs on
        IOCON1bits.PENH = 1; // PWM1H (S1) is controlled by PWM module
        IOCON2bits.PENH = 1; // PWM2H (S2) is controlled by PWM module
        LATBbits.LATB12 = 1;
        LATCbits.LATC12 = 1;
        SyncRecState = 0;
    }
}
/***************************************************************************
End of function
 ***************************************************************************/

/***************************************************************************
Function: 	main
Description:	main routine of the programm
 ***************************************************************************/
int main(void) {
    int i;

    // init PI_CURR
    PI_CURR.Kp = CURR_KP;
    PI_CURR.Ki = CURR_KI;
    PI_CURR.sum = 0;

    // init PI_VOLT
    PI_VOLT.Kp = VOLT_KP;
    PI_VOLT.Ki = VOLT_KI;
    PI_VOLT.sum = 0;
    PI_VOLT.psc = CURR_MAX; // scale output to CURR_MAX (e.g. 65*11.97 = 778)
    PI_VOLT.out = 0;

    // 1st order filter for i_prim, Fg=50 @Fs=4800
    Fil_iprim_coeff.a[0] = 1039;
    Fil_iprim_coeff.a[1] = 1039;
    Fil_iprim_coeff.b[0] = 0;
    Fil_iprim_coeff.b[1] = -30691;

    // 1st order filter for i_out, Fg=50 @Fs=4800
    Fil_iout_coeff.a[0] = 1039;
    Fil_iout_coeff.a[1] = 1039;
    Fil_iout_coeff.b[0] = 0;
    Fil_iout_coeff.b[1] = -30691;

    // Call Init Fcts
    init_CLOCK();
    init_PORTS();
    init_TIMER1();
    init_TIMER2();
    init_INT();
    init_PWM();
    init_ADC();
    init_DAC();
    init_I2C();
    init_Serial();

    // all iic parameters are 'read only' in this version!
    /*
    // Init i2c_inbuf[] with default values
    i2c_inbuf[ 0] = 0;
    i2c_inbuf[ 1] = 0;
    ...
     */


    while (1) { /* ENDLESS MAIN LOOP */

        unsigned int databuffer[10]; // serial communication buffer

        serve_SyncRec();

        // serial input: read new data from pfc stage
        if (get_datablock(&databuffer[0]) > 0) { // a new datablock was recieved successfully
            pfc_udc_filtered = databuffer[0];
            pfc_uin_filtered = databuffer[1];
            pfc_iin_filtered = databuffer[2];
            pfc_temp_prim = databuffer[3];
            pfc_ptper = databuffer[4];
            pfc_curr_kp = databuffer[5];
            pfc_curr_ki = databuffer[6];
            pfc_volt_kp = databuffer[7];
            pfc_volt_ki = databuffer[8];
            pfc_status = databuffer[9];
        }

        // all iic parameters are now read only!
        /*
        // i2c input: read parameters from HostPC
        pfc_flags 		= i2c_inbuf[ 0];
        pfc_test1    	= i2c_inbuf[ 1];
        flags		 	= i2c_inbuf[ 2];
        ...
         */


        // current derating
        i = OUTPUT_DERATE_TRES - pfc_uin_filtered;
        if (i < 0) i = 0; // i = 0..1023
        i <<= 5; // shift 5 bits left to get full 16 bit (signed) resolution
        i = MUL16SX16FU(i, OUTPUT_DERATE_PARA); // scale i by OUTPUT_DERATE_PARA
        i <<= 3; // shift back 5 bits right, shift 8 bits left (because OUTPUT_DERATE_PARA contains factor 256)
        i = CURR_MAX - i;
        if (i < 0) i = 0; // now i contains the derated current value
        if (curr_max < i) ++curr_max;
        else if (curr_max > i) --curr_max; // curr_max is slowly approximated to i


        // FAULT HANDLING:
        /*
        faults implemented:
        FAULT_PFC_NOT_READY
        FAULT_OVERTEMP
        FAULT_OVERCURRENT
        reserved for later use:
        FAULT_FAN				Any Fan is defective
        FAULT_COMMUNICATION 	Primay to Secondary Communication
        FAULT_MAINS				AC Mains Input Voltage Fault
        FAULT_OUTPUTVOLTAGE
         */


        // 1) 'set' conditions for fault bits
        if ((pfc_status & 0x0001) == 0) fault_state |= FAULT_PFC_NOT_READY; // fault bit #0, waiting for pfc, fault is not displayed
        if (maxTempReading > TEMPEXTREMESETTING) fault_state |= FAULT_OVERTEMP;
        if (climit_cnt >= OCP_TIME) fault_state |= FAULT_OVERCURRENT;
        // 2) 'clear' conditions for fault bits
        if ((pfc_status & 0x0001) != 0) fault_state &= ~FAULT_PFC_NOT_READY;
        if (maxTempReading < (TEMPEXTREMESETTING - 10)) fault_state &= ~FAULT_OVERTEMP;
        if ((pfc_status & 0x0002) == 0) fault_state &= ~FAULT_OVERCURRENT; // pfc input voltage below switch off limit


        // set various stuff depending on fault state
        if (fault_state <= 1) {
            pfc_ctrl_flags &= ~0x0001;
        } else {
            pfc_ctrl_flags |= 0x0001; // !! bit 0 is 'FAULT_PFC_NOT_READY' !!
        }
        dcdc_enabled = (fault_state == 0); // setting 'dcdc_enabled' initiates a soft start
        SDC4 = dcdc_enabled ? PHASE_ORING_CP : 0; // feed pwm to charge pump when dcdc is enabled
        LATBbits.LATB15 = dcdc_enabled; // green LED


        // i2c output: feed i2c_outbuf[] with appropriate values
        i2c_outbuf[ 0] = pfc_uin_filtered;
        i2c_outbuf[ 1] = pfc_iin_filtered;
        i2c_outbuf[ 2] = pfc_ptper;
        i2c_outbuf[ 3] = pfc_udc_filtered;
        i2c_outbuf[ 4] = pfc_temp_prim - 155;
        i2c_outbuf[ 5] = pfc_curr_kp;
        i2c_outbuf[ 6] = pfc_curr_ki;
        i2c_outbuf[ 7] = pfc_volt_kp;
        i2c_outbuf[ 8] = pfc_volt_ki;
        i2c_outbuf[ 9] = pfc_status;
        i2c_outbuf[10] = 0;
        i2c_outbuf[11] = cs_bus_in;
        i2c_outbuf[12] = cs_bus_out;
        i2c_outbuf[13] = cs_bus_integrator >> CS_BUS_SHIFT;
        i2c_outbuf[14] = 0;
        i2c_outbuf[15] = 0;
        i2c_outbuf[16] = PTPER;
        i2c_outbuf[17] = SyncRecState;
        i2c_outbuf[18] = u_out;
        i2c_outbuf[19] = i_out;
        i2c_outbuf[20] = Temp_Sec1 - 155;
        i2c_outbuf[21] = Temp_Sec2 - 155;
        i2c_outbuf[22] = 0; // FanSpeed
        i2c_outbuf[23] = PI_CURR.Kp;
        i2c_outbuf[24] = PI_CURR.Ki;
        i2c_outbuf[25] = PI_VOLT.Kp;
        i2c_outbuf[26] = PI_VOLT.Ki;
        i2c_outbuf[27] = i_prim;
        i2c_outbuf[28] = i_prim_filtered;
        i2c_outbuf[29] = fault_state;
        i2c_outbuf[30] = curr_max;
        i2c_outbuf[31] = climit_cnt;


        // serial output: write new parameters to pfc
        if (tx_active == 0) { // no transmission in progress
            databuffer[0] = u_out;
            databuffer[1] = i_out;
            databuffer[2] = pfc_ctrl_flags;
            write_datablock(&databuffer[0]);
        }

    }

}
/***************************************************************************
End of function
 ***************************************************************************/

