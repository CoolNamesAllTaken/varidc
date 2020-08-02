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


// set i2c slave address
#define I2C_ADR 0xA0


// i2c interrupt priority
#define I2C_INT_PRIOR 2


// set i2c buffer lengths, must be 2^x
#define I2C_OUTBUF_LEN 32 /* = 32 words */
#define I2C_INBUF_LEN 32  /* = 32 words */


// globals
unsigned int i2c_inbuf[I2C_INBUF_LEN]; // buffer for i2c input (PC -> dsPIC)
unsigned char *b_i2c_inbuf = (unsigned char*) &i2c_inbuf[0]; // i2c_inbuf as byte array
unsigned int i2c_outbuf[I2C_OUTBUF_LEN]; // buffer for i2c output (dsPIC -> PC)
unsigned char *b_i2c_outbuf = (unsigned char*) &i2c_outbuf[0]; // i2c_outbuf as byte array
unsigned char i2c_buf_ptr = 0; // i2c read/write pointer
unsigned char i2c_state = 0; // i2c fsm state

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void init_I2C(void) {
    // configure i2c module
    I2C1CON = 0x1000; // disable I2C1 module
    I2C1BRG = 0x0188; // set I2C1 baud rate to 100 kHz (0x5D for 400 kHz)
    I2C1ADD = I2C_ADR >> 1;
    I2C1CON = 0x9000; // enable I2C1 module
    // enable I2C interrupt
    IFS1bits.SI2C1IF = 0; // clear I2C1 slave events interrupt flag 
    IPC4bits.SI2C1IP = I2C_INT_PRIOR; // set I2C1 slave events interrupt priority (1=lowest, 7=highest)
    IEC1bits.SI2C1IE = 1; // enable I2C1 slave events interrupt 
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void __attribute__((__interrupt__, no_auto_psv)) _SI2C1Interrupt(void) {
#define ST_IDLE 0
#define ST_WRITE_PTR 1
#define ST_WRITE 2
#define ST_READ 3

    if (I2C1STAT & 0x00C0) { // collision or overflow, error
        I2C1CON = 0x1000; // disable I2C1 module
        I2C1STAT = 0; // clear status bits
        I2C1CON = 0x9000; // re-enable I2C1 module
        i2c_state = ST_IDLE;
    }
    if (I2C1STATbits.D_A == 0) { // valid slave address recieved
        volatile char dummy;
        dummy = I2C1RCV; // dummy read
        i2c_state = I2C1STATbits.R_W == 0 ? ST_WRITE_PTR : ST_READ;
    } else if (i2c_state == ST_WRITE_PTR) {
        i2c_buf_ptr = I2C1RCV;
        i2c_state = ST_WRITE;
    } else if (i2c_state == ST_WRITE) { // write (PC -> dsPIC)
        b_i2c_inbuf[i2c_buf_ptr & (2 * I2C_INBUF_LEN - 1)] = I2C1RCV;
        ++i2c_buf_ptr;
    }
    if (i2c_state == ST_READ) { // read (dsPIC -> PC)
        I2C1TRN = b_i2c_outbuf[i2c_buf_ptr & (2 * I2C_OUTBUF_LEN - 1)]; // write data to output buffer
        I2C1CONbits.SCLREL = 1; // release scl
        ++i2c_buf_ptr;
    }

    IFS1bits.SI2C1IF = 0; // clear i2c interrupt flag
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
