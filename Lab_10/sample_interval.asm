;;***************************************************************************
/*

Title: direct_period_meas.asm
Author: Zach Valenti & Mike Kennedy
Version: 1.0
Last updated: 10/28/18
Target: ATmega324 @ 1 MHz

DESCRIPTION:
Uses timer 0 to count to 1 sec. Uses the 256 prescalar. First the timer
fully counts to 255 7 times, then it only counts to 168. At the last count
the output is toggled, generating a 1 Hz  50% duty cycle wave.

INPUTS:
None

OUTPUTS:
PC2 - sets and resets a flip flop 

REGISTERS USED:
r17 is used as a count of full counts.


PORTS USED:
None
*/
;**************************************************************************

.nolist
.include "m324adef.inc"
.list

;interrupt vectors
	rjmp RESET      ;Reset/Cold start vector
	reti
	reti
	reti
	reti
	reti
	reti
	reti
	reti
	reti
	reti
	reti
	reti
	reti
	reti
	reti
	rjmp one_sec			;timer compare A vector
	reti
	rjmp inc_full_counts	;Timer overflow vector

RESET:
	clr r17

	ldi r16, low(RAMEND)  ; init stack/pointer
	out SPL, r16          ;
	ldi r16, high(RAMEND) ;
	out SPH, r16

					;intialize timer 0
	ldi r16, 0		;clear all relevant registers
	sts TCNT1H, r16
	sts TCNT1L, r16
	sts TCCR1A, r16		;sets the timer with clock off,
	ldi r16, (1<<CS02)	;sets the timer with a prescalar of 256.
	sts TCCR1B, r16

start:
	cpi r17, 7				;see if the timer fully counted 7 times.
	brne start				;if not keep looping
	ldi r16, 168<<OCR0A		;set 168 in count compare
	sts OCR0A, r16	
	ldi r16, 2				;enable compare interrupt
	sts TIMSK0, r16			;for OCF0A

			

	 


inc_full_counts:	;interrupt for timer 0 overflow.
	inc r17			;increments r17 (full timer counts)
	reti

one_sec:
	