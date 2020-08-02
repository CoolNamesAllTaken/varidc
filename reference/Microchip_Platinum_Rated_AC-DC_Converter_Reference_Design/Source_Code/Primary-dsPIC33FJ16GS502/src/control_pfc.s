;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; © 2013 Microchip Technology Inc.
;
; MICROCHIP SOFTWARE NOTICE AND DISCLAIMER:  You may use this software, and any
; derivatives created by any person or entity by or on your behalf, exclusively
; with Microchip?s products.  Microchip and its licensors retain all ownership
; and intellectual property rights in the accompanying software and in all
; derivatives here to.
;
; This software and any accompanying information is for suggestion only.  It
; does not modify Microchip?s standard warranty for its products.  You agree
; that you are solely responsible for testing the software and determining its
; suitability.  Microchip has no obligation to modify, test, certify, or
; support the software.
;
; THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER
; EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED
; WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR
; PURPOSE APPLY TO THIS SOFTWARE, ITS INTERACTION WITH MICROCHIP?S PRODUCTS,
; COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
;
; IN NO EVENT, WILL MICROCHIP BE LIABLE, WHETHER IN CONTRACT, WARRANTY, TORT
; (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY), STRICT LIABILITY,
; INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, PUNITIVE,
; EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF
; ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWSOEVER CAUSED, EVEN IF
; MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.
; TO THE FULLEST EXTENT ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
; CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES,
; IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
;
; MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
; TERMS.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


.include "p33FJ16GS502.inc"

;****************************************************************************
;  Auxillary Functions for Debugging                                        *
;****************************************************************************

.global __StackError
.global _StkOvflErrInstructionAddrLow, _StkOvflErrInstructionAddrHigh

.section .bss
_StkOvflErrInstructionAddrLow: 	.space 2
_StkOvflErrInstructionAddrHigh: .space 2

.section .text

__StackError:
	mov		w15, w0					;Load w0 with the stack pointer value
	mov		SPLIM, w1				;Load w1 with the SPLIM value

StackOverflowCheck:
	cp		w0, w1
	bra		lt, StackUnderflow
	mov		#_StkOvflErrInstructionAddrHigh, w1
	pop		[w1--]					;Pop the PCL and PCH from the stack
	pop		[w1++]					;Decrement that value by 2
	bclr	[w1], #7				;Store the result into RAM at:
	inc		w1, w1					;OffendingInstructionAddrHigh:Low
	clr.b	[w1]					;Halt the Debugging process and view
									;the variables named:
									;OffendingInstructionAddrHigh:Low
	bclr	INTCON1, #STKERR 		;Clear the trap flag
StkOverflowErr:						;Stay in this routine
	bra		StkOverflowErr

StackUnderflow:
	bclr	INTCON1, #STKERR 		;Clear the trap flag
StkUnderflowErr:					;Stay in this routine
	bra		StkUnderflowErr			;To find the offending instruction that caused
									;the Stack underflow error use MPLAB SIM
	retfie



.global __AddressError
.global _AddrErrInstructionAddrLow, _AddrErrInstructionAddrHigh

.section .bss
_AddrErrInstructionAddrLow: .space 2
_AddrErrInstructionAddrHigh: .space 2

.section .text

__AddressError:
    mov     #_AddrErrInstructionAddrHigh, w1
    pop     [w1--]                  ;Pop the Program Counter (PC) from the stack
    pop     [w1++]                  ;Remember that the PC(PCH and PCL) is stacked
    bclr    [w1], #7                ;along with the SRL byte and IPL3 bit from
    inc     w1, w1                  ;CORCON. So we need to extract just the 24-bit
    clr.b   [w1]                    ;PCH:PCL information from the stack
    mov     #_AddrErrInstructionAddrLow, w1  ;Decrement that value by 2
    mov     #2, w2                  ;Store the 24-bit result into 2x16-bit words
    subr    w2, [w1], [w1++]        ;in RAM at- AddrErrInstructionAddrHigh:Low
    clr     w2
    subbr   w2, [w1], [w1]
    bclr    INTCON1, #ADDRERR       ;Clear the trap flag
StayTrappedAddrErr:                 ;Stay in this routine
    bra     StayTrappedAddrErr
    ;Place a breakpoint above and halt the Debugging process and view
    ;the variables named: AddrErrInstructionAddrHigh:Low
    retfie




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    ; Local inclusions.
    .nolist
    .include "dspcommon.inc"	;fractsetup
    .list

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    ;.section .libdsp, code

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;




;****************************************************************************
;  Different PI Controllers                                                 *
;****************************************************************************

;............................................................................
;
;
;
;                         ----   Proportional
;                        |    |  Output
;              ----------| Kp |-----------------
;             |          |    |                 |
;             |           ----                  |
;             |                                ---
;             |      --------------  Integral | + |      -------  Control  -------
;   Control   |     |      Ki      | Output   |   |     | outp. | Output  |       |
;   ----------------| ------------ |----------|+  |-----| post- |---------|  PWM  |
;   Input (error)   |  1 - Z^(-1)  |           ---      | scale |         |       | 
;                    --------------                      -------           -------
;                                                                                            
;
;............................................................................


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

.global _PI32                 
;	typedef struct {
;   	int Ki;
;   	long sum; // integrator sum
;   	int Kp;
;		int psc; // output postscaler (not yet used)
;		int out; // unscaled output (not yet used)
;	} tPI32;

_PI32:
;
; Input:
;   w0 = address of tPI32 data structure
;	w1 = reference value
;	w2 = control input
;
; Return:
;   w0 = control output

    ;save working registers
    push    w8
    push    CORCON                  
	;prepare CORCON for fractional computation
    fractsetup      w8

	;calculate 32*error
	sub.w	w1, w2, w4		;w4 = error
	sl		w4, #5, w4		;error *= 32
	;do the PI job
	mov		w0, w8			;w8 = &PI
	disi	#11				;disable interrupts for next 11 cycles
	clr		b, [w8]+=4, w5	;w5 = Ki, w8 = &sum_hi
	lac		[w8--], b		;b = sum_hi, w8 = &sum_lo
	mov 	[w8], w0		;w0 = sum_lo
	mov		w0, ACCBL		;accbl = w0 = sum_lo
	mac		w4*w5, b		;b += Ki*error
	mov		ACCBL, w0		;w0 = accbl
	mov		w0, [w8++]		;sum_lo = w0 = accbl, w8 = &sum_hi
	sac		b, [w8++]		;sum_hi = b, w8=&Kp
	clr		a, [w8]+=2, w5	;w5 = Kp, w8 = &psc
	mac		w4*w5, b		;b += Kp*error
	sac.r	b, w0			;w0 = result
	btsc	w0, #15			;skip next instruction if w0>=0
	mov		#0, w0			;clear result if negative

    pop     CORCON          ;restore CORCON.
    pop     w8              ;restore working registers.
    return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

.global _PI32B
;	typedef struct {
;   	int Ki;
;   	long sum; // integrator sum
;   	int Kp;
;		int psc; // output postscaler 
;		int out; // unscaled output 
;	} tPI32;

_PI32B:
;
; Same as _PI32, but Kp is multiplied by 16
;
; Input:
;   w0 = address of tPI32 data structure
;	w1 = reference value
;	w2 = control input
;
; Return:
;   w0 = control output

    ;save working registers
    push    w8
    push    CORCON                  
	;prepare CORCON for fractional computation
    fractsetup      w8

	;calculate 32*error
	sub.w	w1, w2, w4		;w4 = error
	sl		w4, #5, w4		;error *= 32
	;do the PI job
	mov		w0, w8			;w8 = &PI
	disi	#18				;disable interrupts for next 18 cycles
	clr		b, [w8]+=4, w5	;b = 0, w5 = Ki, w8 = &sum_hi
	lac		[w8--], b		;b = sum_hi, w8 = &sum_lo
	mov 	[w8], w0		;w0 = sum_lo
	mov		w0, ACCBL		;accbl = w0 = sum_lo
	mac		w4*w5, b		;b += Ki*error
	mov		ACCBL, w0		;w0 = accbl
	mov		w0, [w8++]		;sum_lo = w0 = accbl, w8 = &sum_hi
	sac		b, [w8++]		;sum_hi = b, w8=&Kp
	clr		a, [w8]+=2, w5	;a = 0, w5 = Kp, w8 = &psc
	mac		w4*w5, a		;a += Kp*error
	sac.r	a, #-4, w4		;w4 = 16*a
	mov		#0x7FFF, w5		;w5 = 0x7FFF
	mac		w4*w5, b		;b += 16*Kp*error
	sac.r	b, w4			;w4 = unscaled result
	clr		b, [w8]+=2, w5	;b = 0, w8 = &out, w5 = psc
	mov		w4, [w8]		;out = unscaled result
	mac		w4*w5, b		;b += out*psc
	sac.r	b, w0			;w0 = scaled result
    pop     CORCON          ;restore CORCON.
    pop     w8              ;restore working registers.
    return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

.global _PI_CURR_32               
;
;	typedef struct {
;   	int Ki;
;   	long sum; // integrator sum
;   	int Kp;
;		int psc;  // output postscaler
;		int out;  // output (unscaled)
;		int cor;  // current correction factor for dcm (2048..32768 means 1.0..16.0)
;	} tPICURR32;

_PI_CURR_32:
;
; Input:
;   w0 = address of tPICURR32 data structure
;	w1 = reference value
;	w2 = control input
;
; Return:
;   w0 = control output (duty cycle)

    ;save working registers
    push    w8
    push    CORCON                  
	;prepare CORCON for fractional computation
    fractsetup      w8

	mov		w0, w8			;w8 = &PI
	add		w8, #10, w8		;w8 = &PI.out
	mov		[w8+#2], w5		;w5 = cor
	btst.z	w5, #15			;if(cor<0) => z = 0
	bra		nz, PI_CURR_32_1
	;scale w2 register, scale factor is (d*cor)
	disi	#6
	clr		a, [w8], w4		;a = 0, w4 = out
	mac		w4*w5, a		;a = out*corr
	sac.r	a, #-4, w4		;w4 = a = 16*out*cor
	mov		w2, w5			;w5 = ctrl.input
	mpy		w4*w5, a		;a = scaled ctrl.input
	sac.r	a, w2			;w2 = a
PI_CURR_32_1:
	;calculate 32*error
	sub.w	w1, w2, w4		;w4 = error
	sl		w4, #5, w4		;error *= 32
	;do the PI job
	mov		w0, w8			;w8 = &PI
	disi	#20				;disable interrupts for next 20 cycles
	clr		b, [w8]+=4, w5	;w5 = Ki, w8 = &sum_hi
	lac		[w8--], b		;b = sum_hi, w8 = &sum_lo
	mov 	[w8], w0		;w0 = sum_lo
	mov		w0, ACCBL		;accbl = w0 = sum_lo
	mac		w4*w5, b		;b += Ki*error
	mov		ACCBL, w0		;w0 = accbl
	mov		w0, [w8++]		;sum_lo = w0 = accbl, w8 = &sum_hi
	sac		b, [w8++]		;sum_hi = b, w8=&Kp
	clr		a, [w8]+=2, w5	;a = 0, w5 = Kp, w8 = &psc
	mac		w4*w5, b		;b += Kp*error
	sac.r	b, w4			;w4 = result
	;set to 0 if reference is 0
	mov		#0, w0
	cpsgt	w1, w0			;skip next instruction if w1>0
	mov		#0, w4			;clear result
	;set to 0 if output is negative 
	btsc	w4, #15			;skip next instruction if w0>=0
	mov		#0, w4			;clear result if negative
	;limit output to 92%
	mov		#0x75c1, w5		;92% of 0x7FFF
	cpslt	w4, w5			;skip next instruction if w4<w5
	mov		w5, w4			;set w4 to 92% dc
	;scale output with postscaler (psc)
	mov		[w8++], w5		;w5 = psc, [w8] = out
	mov		w4, [w8]		;out = unscaled result
	mac		w4*w5, a		;a = result*scale
	sac.r	a, w0			;w0 = scaled result

	;restore working registers
    pop     CORCON          ;restore CORCON.
    pop     w8              ;restore working registers.
    return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

.global _FIL_1ORD                 
_FIL_1ORD:
;
; Input:
;   w0 = address of tFIL1HISTORY data structure:
;	typedef struct {
;		int x[1]; // x1
;   	int y[1]; // y1
;	} tFIL1HISTORY;
;
;   w1 = address of tFIL1COEFF data structure
;	typedef struct {
;		int a[2]; // a0, a1
;   	int b[2]; // b0, b1 (b0 is not used!)
;	} tFIL1COEFF;
;	Filter coefficients are fractional data type: -32768..32767 is -1.0..1.0
;
;	w2 = input value
;
; Return:
;   w0 = filter output

    ;save working registers
    push    w8
    push    w10
    push    CORCON                  
	;prepare CORCON for fractional computation
    fractsetup      w8

	mov		w0, w8			;w8 = &x1
	mov		w1, w10			;w10 = &a0
	mov		w2, w4			;w4 = input
	;calculate: out = a0*in + a1*x1 - b1*y1
	disi	#5
	clr		a, [w8]+=2, w5, [w10]+=2, w6			;a = 0, w5 = x1, w6 = a0, w8 = &y1, w10 = &a1
	mac		w4*w6, a, [w8], w7, [w10]+=4, w6		;a += input*a0, w7 = y1, w6 = a1, w10 = &b1
	mac		w5*w6, a, [w10], w6						;a += x1*a1, w6 = b1
	msc		w7*w6, a								;a -= y1*b1
	sac.r	a, w0									;w0 = out = 2*a
	;update history: y1 = out, x1 = in
	mov		w0, [w8--]		;y1 = out, w8 = &x1
	mov		w2, [w8]		;x1 = in
		
	;restore working registers
    pop     CORCON          ;restore CORCON.
    pop     w10
    pop     w8
    return


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

.global _FIL_2ORD                
_FIL_2ORD:
;
; Input:
;   w0 = address of tFIL2HISTORY data structure:
;	typedef struct {
;		int x[2]; // x1, x2
;   	int y[2]; // y1, y2
;	} tFIL2HISTORY;
;
;   w1 = address of tFIL2COEFF data structure:
;	typedef struct {
;		int a[3]; // a0, a1, a2
;   	int b[3]; // b0, b1, b2 (b0 is not used!)
;	} tFIL2COEFF;
;	Filter coefficients are fractional data type: -32768..32767 is -2.0..2.0
;
;	w2 = input value
;
; Return:
;   w0 = filter output

    ;save working registers
    push    w8
    push    w10
    push    CORCON                  
	;prepare CORCON for fractional computation
    fractsetup      w8

	mov		w0, w8			;w8 = &x1
	mov		w1, w10			;w10 = &a0
	mov		w2, w4			;w4 = input
	;calculate: out = a0*in + a1*x1 + a2*x2 - b1*y1 - b2*y2
	disi	#7
	clr		a, [w8]+=2, w5, [w10]+=2, w6			;a = 0, w5 = x1, w6 = a0, w8 = &x2, w10 = &a1
	mac		w4*w6, a, [w8]+=2, w7, [w10]+=2, w6		;a += input*a0, w7 = x2, w6 = a1, w8 = &y1, w10 = &a2
	mac		w5*w6, a, [w8]+=2, w5, [w10]+=4, w6		;a += x1*a1, w5 = y1, w6 = a2, w8 = &y2, w10 = &b1
	mac		w7*w6, a, [w8], w7, [w10]+=2, w6		;a += x2*a2, w7 = y2, w6 = b1, w10 = &b2
	msc		w5*w6, a, [w10], w6						;a -= y1*b1, w6 = b2
	msc		w7*w6, a								;a -= y2*b2
	sac.r	a, #-1, w0								;w0 = out = 2*a
	;update history: y2 = y1, y1 = out, x2 = x1, x1 = in
	sub		w8, #2, w10		;w10 = &y1
	mov		[w10--], [w8--]	;y2 = y1, w10 = &x2, w8 = &y1
	mov		w0, [w8--]		;y1 = out, w8 = &x2
	mov		[--w8], [w10]	;w8 = &x1, x2 = x1
	mov		w2, [w8]		;x1 = in
		
	;restore working registers
    pop     CORCON          ;restore CORCON.
    pop     w10
    pop     w8
    return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

.global _FIL_MA                 
_FIL_MA:
;
; Input:
;   w0 = address of tFIL_MA data structure:
;	typedef struct {
;		int      len;	// filter length
;		int      ptr;	// buffer ptr
;   	int*     buf;	// address of buffer[len]
;		long     sum;	// filter sum
;		unsigned scale;	// scale factor for filter sum (fractional data type, 0..65535 is 0..1.0)
;	} tFIL_MA;
;
;	w1 = input value
;
; Return:
;   w0 = filter output

	mov		[w0], w2		;w2 = len
	sl		w2, #1, w2		;w2 = 2*len
	mov		[w0+#2], w3		;w3 = ptr
	mov		[w0+#4], w4		;w4 = &buffer
	;sub oldest value from sum
	mov		[w0+#6], w6		;w6 = sum_lo
	mov		[w0+#8], w7		;w7 = sum_hi
	mov		[w4+w3], w5		;w5 = oldest
	sub		w6, w5, w6		;sub lo-word
	subb	w7, #0, w7		;sub borrow
	;store new input value to buffer
	mov		w1, [w4+w3]		;store input
	;add input value to sum
	add		w6, w1, w6		;add lo-word
	addc	w7, #0, w7		;add carry
	mov		w6, [w0+#6]		;store sum_lo
	mov		w7, [w0+#8]		;store sum_hi
	;update buffer ptr
	add		w3, #2, w3		;ptr += 2
	cpslt	w3, w2			;skip next instruction if ptr < (2*len)
	mov		#0, w3			;ptr = 0
	mov		w3, [w0+#2]		;store ptr
	;scale output, (w6,w7) = sum
	mov		[w0+#10], w1	;w1 = scale
	mul.uu	w6, w1, w2		;(w2,w3) = sum_lo*scale
	mul.su	w7, w1, w4		;(w4,w5) = sum_hi*scale
	sl		w2, w2			;set carry if w2 >= 0x8000
	addc	w3, w4, w0		;w0 = filter out

    return


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

.global _DIV32UBY16U
_DIV32UBY16U:
;
; Input:
;   w0:w1 = 32 bit dividend (unsigned)
;	w2 = 16 bit divisor (unsigned)
;
; Return:
;   w0 = 16 bit quotient (unsigned)

	cp		w2, #0
	bra		eq, DIV32UBY16U_1
	repeat	#17
	div.ud 	w0, w2		;divide w0:w1 by w2, quotient->w0, remainder->w1	
	bra		ov, DIV32UBY16U_1
	return
DIV32UBY16U_1:
	;division by zero or overflow -> saturate
	mov		#0xFFFF, w0	
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

.global _DIV32SBY16S
_DIV32SBY16S:
;
; Input:
;   w0:w1 = 32 bit dividend (signed)
;	w2 = 16 bit divisor (signed)
;
; Return:
;   w0 = 16 bit quotient (signed)

	mov		w1, w3		;save dividend's high word
	cp		w2, #0
	bra		eq, DIV32SBY16S_1
	repeat	#17
	div.sd 	w0, w2		;divide w0:w1 by w2, quotient->w0, remainder->w1	
	bra		ov, DIV32SBY16S_1
	return
DIV32SBY16S_1:
	;division by zero or overflow -> saturate
	mov		#0x7FFF, w0
	xor		w3, w2, w3	;bit15 of w3 now indicates result's sign
	rlc		w3, w3
	addc	w0, #0, w0
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

.global _DIV16UBY16U
_DIV16UBY16U:
;
;
;
; Input:
;   w0 = 16 bit dividend (unsigned)
;	w1 = 16 bit divisor (unsigned)
;
; Return:
;   w0 = 16 bit quotient (unsigned)

	cp		w1, #0
	bra		eq, DIV16UBY16U_1
	mov		w1, w2
	repeat	#17
	div.u 	w0, w2		;divide w0 by w2, quotient->w0, remainder->w1	
	return
DIV16UBY16U_1:
	;division by zero -> saturate
	mov		#0xFFFF, w0
	return


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

.global _DIV16SBY16S
_DIV16SBY16S:
;
;
;
; Input:
;   w0 = 16 bit dividend (signed)
;	w1 = 16 bit divisor (signed)
;
; Return:
;   w0 = 16 bit quotient (signed)

	cp		w1, #0
	bra		eq, DIV16SBY16S_1
	mov		w1, w2
	repeat	#17
	div.s 	w0, w2		;divide w0 by w2, quotient->w0, remainder->w1	
	return
DIV16SBY16S_1:
	;division by zero -> saturate
	mov		#0x7FFF, w0
	xor		w0, w1, w1	;bit15 of w1 now indicates result's sign
	rlc		w1, w1
	addc	w0, #0, w0
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

.global _MUL16SX16S
_MUL16SX16S:
;
;
;
; Input:
;   w0 = 1st 16 bit factor (int)
;	w1 = 2nd 16 bit factor (int)
;
; Return:
;   w0 = 32 bit result (long int)

	mul.ss	w0, w1, w0
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

.global _MUL16SX16U
_MUL16SX16U:
;
;
;
; Input:
;   w0 = 1st 16 bit factor (int)
;	w1 = 2nd 16 bit factor (unsigned)
;
; Return:
;   w0 = 32 bit result (long int)

	mul.su	w0, w1, w0
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

.global _MUL16UX16U
_MUL16UX16U:
;
;
;
; Input:
;   w0 = 1st 16 bit factor (unsigned)
;	w1 = 2nd 16 bit factor (unsigned)
;
; Return:
;   w0 = 32 bit result (long unsigned)

	mul.uu	w0, w1, w0
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

.global _MUL16SX16FS
_MUL16SX16FS:
;
;
;
; Input:
;   w0 = 1st 16 bit factor (int)
;	w1 = 2nd 16 bit factor (fractional 1.15)
;
; Return:
;   w0 = 16 bit result (int)

	mul.ss	w0, w1, w0
	rlc		w0, w0
	rlc		w1, w1
	rlc		w0, w0
	addc	w1, #0, w0
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

.global _MUL16SX16FU
_MUL16SX16FU:
;
;
;
; Input:
;   w0 = 1st 16 bit factor (int)
;	w1 = 2nd 16 bit factor (unsigned fractional 0.16)
;
; Return:
;   w0 = 16 bit result (int)

	mul.su	w0, w1, w0
	rlc		w0, w0
	addc	w1, #0, w0
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

.global _MUL16UX16FU
_MUL16UX16FU:
;
;
;
; Input:
;   w0 = 1st 16 bit factor (unsigned)
;	w1 = 2nd 16 bit factor (unsigned fractional 0.16)
;
; Return:
;   w0 = 16 bit result (unsigned)

	mul.uu	w0, w1, w0
	rlc		w0, w0
	addc	w1, #0, w0
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

.global _LOOKUP32_16U_16I
_LOOKUP32_16U_16I:
;
;
;
; Input:
;   w0 = Address of 1st element
;	w1 = uint16 input value
;
; Return:
;   w0 = int16 output

	lsr		w1, #11, w2		;w2 = table index
	sl		w2, #1, w2		;w2 *= 2 (word addressing!)
	add		w2, w0, w2		;w2 = address of 1st entry
	mov		[w2], w3		;w3 = 1st entry
	mov		[w2+2], w4		;w4 = 2nd entry
	sub		w4, w3, w4		;w4 = 2nd entry - 1st entry
	sl		w1, #5, w1		;w1 = 32 * (lower 11 bits of input)
	mul.su	w4, w1, w6		;fractional multiplication
	rlc		w6, w6			;shift msb of lo word to carry
	addc	w7, #0, w7		;add carry
	add		w7, w3, w0		;add result to 1st entry
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;






.end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;EOF

