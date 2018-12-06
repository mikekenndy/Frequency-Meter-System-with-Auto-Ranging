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
	.org OC0Baddr
		rjmp half_sec			;timer compare A vector
	.org OVF0addr	
		rjmp inc_full_counts	;Timer overflow vector

RESET:
	.EQU WAVE = 0
		
	clr r17

	ldi r16, low(RAMEND)  ; init stack/pointer
	out SPL, r16          ;
	ldi r16, high(RAMEND) ;
	out SPH, r16
	
	ldi r16, 0b01000000		;enable output pin
	out DDRD, r16
	;intialize timer 0
	LDI R16, 0b00000010
	OUT TCCR0B, R16
	LDI R16, 1 << TOIE0
	sts TIMSK0, r16
	ldi r16, 168		;set 168 in count compare
	;ldi r16, 10		;set 168 in count compare
	OUT OCR0B, r16	

	SEI


start:
	
	cpi r17, 243				;see if the timer fully counted 7 times.
	brne start				;if not keep looping
	
	;LDI R16, 1 << TOV0
	;OUT TIFR0, R16
	clr r17
	ldi r16, (1 << OCIE0B)		;enable compare interrupt
	sts TIMSK0, r16				;for OCF0A
	nop
	rjmp start
	 


inc_full_counts:	;interrupt for timer 0 overflow.
	inc r17			;increments r17 (full timer counts)
	reti

half_sec:
	push R16
	IN R16, SREG
	PUSH R16
	
	SBI PIND, 6
	ldi r16, (1 << TOIE0)	;enable overflow interrupt
	sts TIMSK0, r16			;for OCF0A
	LDI R16, 0		;Clear TC0
	sts TCNT0, r16

	POP R16
	OUT SREG, R16
	POP R16

	reti