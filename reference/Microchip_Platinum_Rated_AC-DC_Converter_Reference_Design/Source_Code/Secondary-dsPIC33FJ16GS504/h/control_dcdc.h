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

#ifndef _CONTROL_H_
#define _CONTROL_H_



// *** DIFFERENT PI CONTROLLERS ***
//


// PI-controller with 16 bit integrator
///////////////////////////////////////////

typedef struct {
    int Ki;
    int sum; // integrator sum
    int Kp;
    int psc; // output postscaler
} tPI16;

extern int PI16(tPI16 *controller; int error;);


// PI-controllers with 32 bit integrator
/////////////////////////////////////////////

// general PI32 controller

typedef struct {
    int Ki;
    long sum; // integrator sum
    int Kp;
    int psc; // output postscaler
    int out; // unscaled output
} tPI32;

extern int PI32(tPI32 *controller; int reference; int input;); // normal PI32, input range 0..1023
extern int PI32A(tPI32 *controller; int reference; int input;); // modified PI32, input range 0..2047
extern int PI32B(tPI32 *controller; int reference; int input;); // modified PI32, Kp is multiplied by 16


// PI32 current controller with dcm current correction, output limit and output postscale 
/////////////////////////////////////////////////////////////////////////////////////////////

typedef struct {
    int Ki;
    long sum; // integrator sum
    int Kp;
    int psc; // output postscaler
    int out; // output (unscaled)
    int cor; // current correction factor for dcm (2048..32768 means 1.0..16.0)
} tPICURR32;

extern int PI_CURR_32(tPICURR32 *controller; int reference; int input;);




// *** 1ST AND 2ND ORDER FILTER FUNCTIONS ***
// Filter function needs a tFILxHISTORY structure and a tFILxCOEFF structure
// tFILxCOEFF structure may be shared by several filters
// Due to performance optimization which uses MAC prefetch instructions, 
// tFILxHISTORY must be located in xmemory and tFILxCOEFF must be located in ymemory


// 1st order filter
//////////////////////

typedef struct {
    int x[1]; // x1
    int y[1]; // y1
} tFIL1HISTORY;

typedef struct { // Filter coefficients are fractional data type: -32768..32767 is -1.0..1.0
    int a[2]; // a0, a1
    int b[2]; // b0, b1 (b0 is not used!)
} tFIL1COEFF;

extern int FIL_1ORD(
        tFIL1HISTORY *history;
        tFIL1COEFF *coeff;
        int input;
        );


// 2nd order filter
//////////////////////

typedef struct {
    int x[2]; // x1, x2
    int y[2]; // y1, y2
} tFIL2HISTORY;

typedef struct { // Filter coefficients are fractional data type: -32768..32767 is -2.0..2.0
    int a[3]; // a0, a1, a2
    int b[3]; // b0, b1, b2 (b0 is not used!)
} tFIL2COEFF;

extern int FIL_2ORD(
        tFIL2HISTORY *history;
        tFIL2COEFF *coeff;
        int input;
        );


// Moving average filter
///////////////////////////

typedef struct {
    int len; // filter length
    int ptr; // buffer ptr
    int* buf; // address of buffer[len]
    long sum; // filter sum
    unsigned scale; // scale factor for filter sum (fractional data type, 0..65535 is 0..1.0)
} tFIL_MA;

extern int FIL_MA(
        tFIL_MA *filter;
        int input;
        );


// PWM Read Functions
////////////////////////
extern int READ_PWM1();
extern int READ_PWM2();


// assembler functions for 32by16 and 16by16 division
/////////////////////////////////////////////////////////
extern unsigned DIV32UBY16U(long unsigned dividend, unsigned divisor);
extern int DIV32SBY16S(long dividend, int divisor);
extern unsigned DIV16UBY16U(unsigned dividend, unsigned divisor);
extern int DIV16SBY16S(int dividend, int divisor);


// assembler functions for 16x16 bit multiplication with 32 bit result
//////////////////////////////////////////////////////////////////////////
extern long int MUL16SX16S(int x, int y);
extern long int MUL16SX16U(int x, unsigned y);
extern long unsigned MUL16UX16U(unsigned x, unsigned y);
// multiplication functions for fractional data
extern int MUL16SX16FS(int x, int y); // signed * fractional(1.15)
extern int MUL16SX16FU(int x, unsigned y); // signed * fractional(0.16)
extern unsigned MUL16UX16FU(unsigned x, unsigned y); // unsigned * fractional(0.16)


// lookup table processing, this functions read a value from a 32(+1) element lookup table
// the upper 5  bit of the uint16 input value are used to address the table position
// the lower 11 bits are used for interpolation between two table entries
///////////////////////////////////////////////////////////////////////////////////////////
// input:
//   int* lktab: address of lookup table's 1st element
//   x: uint16 input value
// output:
//   int16 output value
extern int LOOKUP32_16U_16I(int* lktab, unsigned int x);





#endif
