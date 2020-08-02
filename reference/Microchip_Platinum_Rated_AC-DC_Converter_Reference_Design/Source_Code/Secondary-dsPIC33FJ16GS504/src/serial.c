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


// GENERAL SETTINGS
// init_Serial() has to be adapted to hardware, too!
//////////////////////////////////////////////////////
//
// sync primary side with pwm output signal?
#define SYNC_PRIMARY 0
// generate sync signal on secondary side?
#define SYNC_SECONDARY 0
// serial interrupt priority
#define SER_INT_PRIOR 1
// baud rate (1 = 1.25 MBaud, 21 = 114 kBaud, 64 = 38,4 kBaud, 512 = 4,8 kBaud)
#define BAUD_DIV 21
// show interrupt activity?
#define SET_GPIO {/*disabled!*/;}
#define CLR_GPIO {/*disabled!*/;}


// SERIAL TRANSMISSION
#define TX_BUF_LEN 3
#define TX_MAX_BUF_PTR (2*TX_BUF_LEN+2)
unsigned int tx_buffer[TX_BUF_LEN + 1];
unsigned char *tx_bbuffer = (unsigned char*) &tx_buffer[0];
int tx_ptr = TX_MAX_BUF_PTR;
int tx_active = 0;
// SERIAL RECEPTION
#define RX_BUF_LEN 10
#define RX_MAX_BUF_PTR (2*RX_BUF_LEN+2)
unsigned int rx_buffer[RX_BUF_LEN + 1];
unsigned char *rx_bbuffer = (unsigned char*) &rx_buffer[0];
int rx_ptr = 0;


// CRC CALCULATION
#define CRC_STEP(ck, d) {ck = (ck>>8) ^crc_tab[(ck&0x00FFu)^((unsigned char)(d))];} 
#define CRC_START 0xFFFF
const unsigned int crc_tab[] = {
    0x0000U, 0xC0C1U, 0xC181U, 0x0140U, 0xC301U, 0x03C0U, 0x0280U, 0xC241U,
    0xC601U, 0x06C0U, 0x0780U, 0xC741U, 0x0500U, 0xC5C1U, 0xC481U, 0x0440U,
    0xCC01U, 0x0CC0U, 0x0D80U, 0xCD41U, 0x0F00U, 0xCFC1U, 0xCE81U, 0x0E40U,
    0x0A00U, 0xCAC1U, 0xCB81U, 0x0B40U, 0xC901U, 0x09C0U, 0x0880U, 0xC841U,
    0xD801U, 0x18C0U, 0x1980U, 0xD941U, 0x1B00U, 0xDBC1U, 0xDA81U, 0x1A40U,
    0x1E00U, 0xDEC1U, 0xDF81U, 0x1F40U, 0xDD01U, 0x1DC0U, 0x1C80U, 0xDC41U,
    0x1400U, 0xD4C1U, 0xD581U, 0x1540U, 0xD701U, 0x17C0U, 0x1680U, 0xD641U,
    0xD201U, 0x12C0U, 0x1380U, 0xD341U, 0x1100U, 0xD1C1U, 0xD081U, 0x1040U,
    0xF001U, 0x30C0U, 0x3180U, 0xF141U, 0x3300U, 0xF3C1U, 0xF281U, 0x3240U,
    0x3600U, 0xF6C1U, 0xF781U, 0x3740U, 0xF501U, 0x35C0U, 0x3480U, 0xF441U,
    0x3C00U, 0xFCC1U, 0xFD81U, 0x3D40U, 0xFF01U, 0x3FC0U, 0x3E80U, 0xFE41U,
    0xFA01U, 0x3AC0U, 0x3B80U, 0xFB41U, 0x3900U, 0xF9C1U, 0xF881U, 0x3840U,
    0x2800U, 0xE8C1U, 0xE981U, 0x2940U, 0xEB01U, 0x2BC0U, 0x2A80U, 0xEA41U,
    0xEE01U, 0x2EC0U, 0x2F80U, 0xEF41U, 0x2D00U, 0xEDC1U, 0xEC81U, 0x2C40U,
    0xE401U, 0x24C0U, 0x2580U, 0xE541U, 0x2700U, 0xE7C1U, 0xE681U, 0x2640U,
    0x2200U, 0xE2C1U, 0xE381U, 0x2340U, 0xE101U, 0x21C0U, 0x2080U, 0xE041U,
    0xA001U, 0x60C0U, 0x6180U, 0xA141U, 0x6300U, 0xA3C1U, 0xA281U, 0x6240U,
    0x6600U, 0xA6C1U, 0xA781U, 0x6740U, 0xA501U, 0x65C0U, 0x6480U, 0xA441U,
    0x6C00U, 0xACC1U, 0xAD81U, 0x6D40U, 0xAF01U, 0x6FC0U, 0x6E80U, 0xAE41U,
    0xAA01U, 0x6AC0U, 0x6B80U, 0xAB41U, 0x6900U, 0xA9C1U, 0xA881U, 0x6840U,
    0x7800U, 0xB8C1U, 0xB981U, 0x7940U, 0xBB01U, 0x7BC0U, 0x7A80U, 0xBA41U,
    0xBE01U, 0x7EC0U, 0x7F80U, 0xBF41U, 0x7D00U, 0xBDC1U, 0xBC81U, 0x7C40U,
    0xB401U, 0x74C0U, 0x7580U, 0xB541U, 0x7700U, 0xB7C1U, 0xB681U, 0x7640U,
    0x7200U, 0xB2C1U, 0xB381U, 0x7340U, 0xB101U, 0x71C0U, 0x7080U, 0xB041U,
    0x5000U, 0x90C1U, 0x9181U, 0x5140U, 0x9301U, 0x53C0U, 0x5280U, 0x9241U,
    0x9601U, 0x56C0U, 0x5780U, 0x9741U, 0x5500U, 0x95C1U, 0x9481U, 0x5440U,
    0x9C01U, 0x5CC0U, 0x5D80U, 0x9D41U, 0x5F00U, 0x9FC1U, 0x9E81U, 0x5E40U,
    0x5A00U, 0x9AC1U, 0x9B81U, 0x5B40U, 0x9901U, 0x59C0U, 0x5880U, 0x9841U,
    0x8801U, 0x48C0U, 0x4980U, 0x8941U, 0x4B00U, 0x8BC1U, 0x8A81U, 0x4A40U,
    0x4E00U, 0x8EC1U, 0x8F81U, 0x4F40U, 0x8D01U, 0x4DC0U, 0x4C80U, 0x8C41U,
    0x4400U, 0x84C1U, 0x8581U, 0x4540U, 0x8701U, 0x47C0U, 0x4680U, 0x8641U,
    0x8201U, 0x42C0U, 0x4380U, 0x8341U, 0x4100U, 0x81C1U, 0x8081U, 0x4040U
};

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void init_Serial(void) {
    // configure ports
    __builtin_write_OSCCONL(OSCCON & ~(1 << 6)); // Unlock Registers (Bit 6 in OSCCON)
    // configure UART IO pins
    RPINR18bits.U1RXR = 16; // U1RX->RP16
    RPOR14bits.RP29R = 3; // U1TX->RP29
#if(SYNC_PRIMARY==1)
    FIXME!!
            // set sync output from primary pwm (PWM out)
            RPINR18bits.U1CTSR = 21; // U1CTS input -> RP21
#endif
#if(SYNC_SECONDARY==1)
    FIXME!!
            // set (virtual) port for reciever interrupt,
            // must be sync input for secondary pwm
            RPOR9bits.RP19R = 4; // U1RTS output -> RP19 (GPIO5)
    RPINR0bits.INT1R = 19; // INT1 -> RP19
#endif
    __builtin_write_OSCCONL(OSCCON | (1 << 6)); // Lock Registers

    // configure recieve interrupt
#if(SYNC_SECONDARY==1)
    FIXME!!
            // instead of using UART Rx interrupt, an interrupt request is generated by the RTS/ signal
            // at the beginning of recieving the 5th byte
            INTCON2bits.INT1EP = 0; // active edge, 0=positive, 1=negative
    IPC5bits.INT1IP = SER_INT_PRIOR; // set INT1 interrupt priority (1=lowest, 7=highest)
    IEC1bits.INT1IE = 1; // enable INT1 interrupt
#else
    // use UART Rx interrupt
    U1STAbits.URXISEL = 0; // Interrupt after one RX character is received;
    IPC2bits.U1RXIP = SER_INT_PRIOR; // set U1RX interrupt priority (1=lowest, 7=highest)
    IEC0bits.U1RXIE = 1; // Enable UART RX interrupt
#endif

    // configure transmit interrupt
    U1STAbits.UTXISEL1 = 1;
    U1STAbits.UTXISEL0 = 0; // Interrupt when TX buffer becomes empty
    IPC3bits.U1TXIP = SER_INT_PRIOR; // set U1RX interrupt priority (1=lowest, 7=highest)
    IEC0bits.U1TXIE = 1; // Enable UART RX interrupt

    // configure uart
    U1MODEbits.STSEL = 0; // 1 stop bit
    U1MODEbits.PDSEL = 3; // 9 data bits, no parity
    U1MODEbits.ABAUD = 0; // Autobaud disabled
    U1MODEbits.BRGH = 0; // Low speed mode
#if(SYNC_PRIMARY==1)
    U1MODEbits.UEN = 2; // UxTX, UxRX, UxCTS and UxRTS pins are enabled and used
#elif(SYNC_SECONDARY==1)
    U1MODEbits.UEN = 1; // UxTX, UxRX and UxRTS pins are enabled and used
#else
    U1MODEbits.UEN = 0; // UxTX and UxRX pins are enabled and used
#endif
    U1BRG = BAUD_DIV;
    U1MODEbits.UARTEN = 1; // Enable UART
    U1STAbits.UTXEN = 1; // Enable UART Tx
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#if(SYNC_SECONDARY==1)

FIXME!!
// external interrupt INT1 triggered by U1RTS/ signal
// this interrupt replaces the U1RX interrupt!
void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt(void) {
    unsigned int i;
    SET_GPIO
    i = U1RXREG;
    if (U1STA & 0x0006) {
        U1STAbits.OERR = 0;
        U1STAbits.FERR = 0;
    } else if (rx_ptr < RX_MAX_BUF_PTR) {
        if (i >= 0x0100u) {
            rx_bbuffer[0] = i & 0x00FFu;
            rx_ptr = 1;
        } else if (rx_ptr > 0) {
            rx_bbuffer[rx_ptr] = i & 0x00FFu;
            ++rx_ptr;
        }
    }
    CLR_GPIO
    IFS1bits.INT1IF = 0; // clear interrupt flag
}
#else
// serial recieve U1RX interrupt service routine

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void) {
    unsigned int i;
    SET_GPIO
    i = U1RXREG;
    if (U1STA & 0x0006) {
        U1STAbits.OERR = 0;
        U1STAbits.FERR = 0;
    } else if (rx_ptr < RX_MAX_BUF_PTR) {
        if (i >= 0x0100u) {
            rx_bbuffer[0] = i & 0x00FFu;
            rx_ptr = 1;
        } else if (rx_ptr > 0) {
            rx_bbuffer[rx_ptr] = i & 0x00FFu;
            ++rx_ptr;
        }
    }
    CLR_GPIO
    IFS0bits.U1RXIF = 0; // clear RX interrupt flag
}
#endif

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void) {
    unsigned char c;
    while ((tx_ptr < TX_MAX_BUF_PTR) && (U1STAbits.UTXBF == 0)) {
        c = tx_bbuffer[tx_ptr];
        ++tx_ptr;
        U1TXREG = c;
    }
    if (tx_ptr >= TX_MAX_BUF_PTR)
        tx_active = 0;
    IFS0bits.U1TXIF = 0; // clear TX interrupt flag
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int get_datablock(unsigned int *data)
// returns a value >0 when a new datablock was recieved
// recieved data is copied to *data
{
    int i;
    unsigned int crc;

    if (rx_ptr < RX_MAX_BUF_PTR)
        return (0); // return code: no data.
    // check crc
    crc = CRC_START;
    for (i = 0; i < RX_MAX_BUF_PTR; ++i)
        CRC_STEP(crc, rx_bbuffer[i]);
    if (crc != 0) {
        rx_ptr = 0; // invalidate buffer
        return (-1); // return code: crc error!
    }
    // copy datablock
    for (i = 0; i < RX_BUF_LEN; ++i)
        data[i] = rx_buffer[i];
    rx_ptr = 0; // invalidate buffer
    return (RX_BUF_LEN); // return # of words recieved
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int write_datablock(unsigned int *data)
// prepares send buffer and triggers write process
{
    unsigned int i;
    unsigned int crc;
    unsigned char c;
    unsigned char *cp;

    // exit if transmit in progress
    if (tx_active) return (0);

    IFS0bits.U1TXIF = 0; // clear TX interrupt flag
    tx_active = -1; // set 'transmit active' flag

    // copy data to transmit buffer
    cp = (unsigned char*) &data[0];
    crc = CRC_START;
    for (i = 0; i < TX_MAX_BUF_PTR - 2; ++i) {
        c = cp[i];
        CRC_STEP(crc, c);
        tx_bbuffer[i] = c;
    }
    tx_bbuffer[TX_MAX_BUF_PTR - 2] = crc & 0x00FF;
    tx_bbuffer[TX_MAX_BUF_PTR - 1] = (crc >> 8)&0x00FF;

    // send 1st byte|0x100 to start transmission
    tx_ptr = 1;
    U1TXREG = 0x0100u | (unsigned int) tx_bbuffer[0];

    return (1);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
