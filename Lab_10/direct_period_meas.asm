;;***************************************************************************
/*

Title: direct_period_meas.asm
Author: Zach Valenti & Mike Kennedy
Version: 1.0
Last updated: 10/28/18
Target: ATmega324 @ 1 MHz

DESCRIPTION:
This program measures the frequency of a given test signal at INT1 (PD3). It accomplishes
this by triggering an interrupt on the positive edge of the test signal, and on the first interrupt
the internal 16 bit timer starts to count, and a flag is set. On the next interrupt the flag is taken
into consideration and the program knows to stop the timer, record the value, clear the timer, and
then set a different flag to let the program know to update the display. Once the period is known,
the program computes the frequency by dividing 1 million (for the microseconds) by the measured 
period. 

INPUTS:
-Signal into PD3

OUTPUTS:
-LCD shows in ASCII characters the current frequency and period of the inputted signal. 

REGISTERS USED:
r16-r19 = general purpose registers
r20 = PERIODL
r21 = PERIODH
r22 = FREQL
r23 = FREQH

PORTS USED:
-PORTB = Outputs
-PIN D3 = Signal input
*/
;**************************************************************************

.nolist
.include "m324adef.inc"
.list

	rjmp RESET      ;Reset/Cold start vector
	reti
	.org INT1addr
		rjmp period_calc

;---------------------------- SUBROUTINES ----------------------------


;====================================
.include "lcd_dog_asm_driver_m324a.inc"  ; LCD DOG init/update procedures.
;====================================


;***************************************************************************
;*
;* "bin2BCD16" - 16-bit Binary to BCD conversion
;*
;* This subroutine converts a 16-bit number (fbinH:fbinL) to a 5-digit 
;* packed BCD number represented by 3 bytes (tBCD2:tBCD1:tBCD0).
;* MSD of the 5-digit number is placed in the lowermost nibble of tBCD2.
;*  
;* Number of words:	25
;* Number of cycles: 751/768 (Min/Max)
;* Low registers used: 3 (tBCD0,tBCD1,tBCD2) 
;* High registers used: 4 (fbinL,fbinH,cnt16a,tmp16a)	
;* Pointers used: Z
;*
;***************************************************************************

;***** Subroutine Register Variables

.equ	AtBCD0	=13		;address of tBCD0
.equ	AtBCD2	=15		;address of tBCD1

.def	tBCD0	=r13		;BCD value digits 1 and 0
.def	tBCD1	=r14		;BCD value digits 3 and 2
.def	tBCD2	=r15		;BCD value digit 4
.def	fbinL	=r16		;binary value Low byte
.def	fbinH	=r17		;binary value High byte
.def	cnt16a	=r18		;loop counter
.def	tmp16a	=r19		;temporary value

;***** Code

bin2BCD16:
	ldi	cnt16a,16	;Init loop counter	
	clr	tBCD2		;clear result (3 bytes)
	clr	tBCD1		
	clr	tBCD0		
	clr	ZH		;clear ZH (not needed for AT90Sxx0x)
bBCDx_1:lsl	fbinL		;shift input value
	rol	fbinH		;through all bytes
	rol	tBCD0		;
	rol	tBCD1
	rol	tBCD2
	dec	cnt16a		;decrement loop counter
	brne	bBCDx_2		;if counter not zero
	ret			;   return

bBCDx_2:ldi	r30,AtBCD2+1	;Z points to result MSB + 1
bBCDx_3:
	ld	tmp16a,-Z	;get (Z) with pre-decrement
;----------------------------------------------------------------
;For AT90Sxx0x, substitute the above line with:
;
;	dec	ZL
;	ld	tmp16a,Z
;
;----------------------------------------------------------------
	subi	tmp16a, -$03	;add 0x03
	sbrc	tmp16a, 3	;if bit 3 not clear
	st		Z, tmp16a	;	store back
	ld		tmp16a, Z	;get (Z)
	subi	tmp16a, -$30	;add 0x30
	sbrc	tmp16a, 7	;if bit 7 not clear
	st		Z, tmp16a	;	store back
	cpi		ZL, AtBCD0	;done all three?
	brne	bBCDx_3		;loop again if not
	rjmp	bBCDx_1		

;***************************************************************************
;*
;* "div32u" - 32/32 Bit Unsigned Division
;*
;* Ken Short
;*
;* This subroutine divides the two 32-bit numbers 
;* "dd32u3:dd32u2:dd32u1:dd32u0" (dividend) and "dv32u3:dv32u2:dv32u3:dv32u2"
;* (divisor). 
;* The result is placed in "dres32u3:dres32u2:dres32u3:dres32u2" and the
;* remainder in "drem32u3:drem32u2:drem32u3:drem32u2".
;*  
;* Number of words	:
;* Number of cycles	:655/751 (Min/Max) ATmega16
;* #Low registers used	:2 (drem16uL,drem16uH)
;* #High registers used  :5 (dres16uL/dd16uL,dres16uH/dd16uH,dv16uL,dv16uH,
;*			    dcnt16u)
;* A $0000 divisor returns $FFFF
;*
;***************************************************************************

;***** Subroutine Register Variables

.def	drem32u0=r12    ;remainder
.def	drem32u1=r13
.def	drem32u2=r14
.def	drem32u3=r15

.def	dres32u0=r18    ;result (quotient)
.def	dres32u1=r19
.def	dres32u2=r20
.def	dres32u3=r21

.def	dd32u0	=r18    ;dividend
.def	dd32u1	=r19
.def	dd32u2	=r20
.def	dd32u3	=r21

.def	dv32u0	=r22    ;divisor
.def	dv32u1	=r23
.def	dv32u2	=r24
.def	dv32u3	=r25

.def	dcnt32u	=r17

;***** Code

div32u:
	clr	drem32u0	;clear remainder Low byte
    clr drem32u1
    clr drem32u2
	sub	drem32u3,drem32u3;clear remainder High byte and carry
	ldi	dcnt32u,33	;init loop counter
d32u_1:
	rol	dd32u0		;shift left dividend
	rol	dd32u1
	rol	dd32u2    
	rol	dd32u3
	dec	dcnt32u		;decrement counter
	brne	d32u_2		;if done
	ret			;    return
d32u_2:
	rol	drem32u0	;shift dividend into remainder
    rol	drem32u1
    rol	drem32u2
	rol	drem32u3

	sub	drem32u0,dv32u0	;remainder = remainder - divisor
    sbc	drem32u1,dv32u1
    sbc	drem32u2,dv32u2
	sbc	drem32u3,dv32u3	;
	brcc	d32u_3		;   branch if reult is pos or zero

	add	drem32u0,dv32u0	;    if result negative restore remainder
	adc	drem32u1,dv32u1
	adc	drem32u2,dv32u2
	adc	drem32u3,dv32u3
	clc			;    clear carry to be shifted into result
	rjmp	d32u_1		;else
d32u_3:	sec			;    set carry to be shifted into result
	rjmp	d32u_1


;*******************
;NAME:      load_decimal_period
;FUNCTION:  Loads a predefined string msg and the 
;			calculated period in decimal into a specified display buffer.
;ASSUMES:   Z = offset of message to be loaded. Msg format is 
;           defined below.
;RETURNS:   nothing.
;MODIFIES:  r16, Y, Z
;CALLS:     nothing
;CALLED BY:  
;********************************************************************
; Message structure:
;   label:  .db <buff num>, <text string/message>, <end of string>
;
; Message examples (also see Messages at the end of this file/module):
;   msg_1: .db 1,"First Message ", 0   ; loads msg into buff 1, eom=0
;   msg_2: .db 1,"Another message ", 0 ; loads msg into buff 1, eom=0
;
; Notes: 
;   a) The 1st number indicates which buffer to load (either 1, 2, or 3).
;   b) The last number (zero) is an 'end of string' indicator.
;   c) Y = ptr to disp_buffer
;      Z = ptr to message (passed to subroutine)
;********************************************************************
load_decimal_period:
     ldi YH, high (dsp_buff_1) ; Load YH and YL as a pointer to 1st
     ldi YL, low (dsp_buff_1)  ; byte of dsp_buff_1 (Note - assuming 
                               ; (dsp_buff_1 for now).
     lpm R16, Z+               ; get dsply buff number (1st byte of msg).
     cpi r16, 1                ; if equal to '1', ptr already setup.
     breq get_msg_byte         ; jump and start message load.
     adiw YH:YL, 16            ; else set ptr to dsp buff 2.
     cpi r16, 2                ; if equal to '2', ptr now setup.
     breq get_msg_byte         ; jump and start message load.
     adiw YH:YL, 16            ; else set ptr to dsp buff 2.
        
get_msg_byte:
     lpm R16, Z+              ; get next byte of msg and see if '0'.        
     cpi R16, 0               ; if equal to '0', end of message reached.
     breq decimal_period_load       ; jump and start storing of calibrate message. 
     st Y+, R16               ; else, store next byte of msg in buffer.
     rjmp get_msg_byte        ; jump back and continue...

decimal_period_load:

	;r15 is the 5th digit.
	;r14 high is the 4th digit.
	;r14 low is the 3rd digit.
	;r13 high is the 2nd digit.
	;r13 low is the 1st digit.
	
	;print 5th digit
	mov r16, r15
	rcall load_number

	;print 4th digit
	mov r16, r14
	andi r16, $F0
	lsr r16
	lsr r16
	lsr r16
	lsr r16
	rcall load_number

	;print 3rd digit
	mov r16, r14			;copy bcd digit into r16 for subroutine
	andi r16, $0F			;clear irrelevant parts of r14.
	rcall load_number		;loads the correct number
	;print 2nd digit
	mov r16, r13			;copy r13 into r16 and clear low nibble
	andi r16, $f0			
	lsr r16					;shift high nibble down to low nibble
	lsr r16
	lsr r16
	lsr r16					
	rcall load_number		;load bcd digit to display

	;print 1st digit
	mov r16, r13			;copy bcd digit into r16 and clear high nibble
	andi r16, $0f			
	rcall load_number		;load bcd digit to display

	ret

;*******************
;NAME:      load_decimal_frequency
;FUNCTION:  Loads a predefined string msg and the 
;			calculated frequency in decimal into a specified display buffer.
;ASSUMES:   Z = offset of message to be loaded. Msg format is 
;           defined below.
;RETURNS:   nothing.
;MODIFIES:  r16, Y, Z
;CALLS:     nothing
;CALLED BY:  
;********************************************************************
; Message structure:
;   label:  .db <buff num>, <text string/message>, <end of string>
;
; Message examples (also see Messages at the end of this file/module):
;   msg_1: .db 1,"First Message ", 0   ; loads msg into buff 1, eom=0
;   msg_2: .db 1,"Another message ", 0 ; loads msg into buff 1, eom=0
;
; Notes: 
;   a) The 1st number indicates which buffer to load (either 1, 2, or 3).
;   b) The last number (zero) is an 'end of string' indicator.
;   c) Y = ptr to disp_buffer
;      Z = ptr to message (passed to subroutine)
;********************************************************************
load_decimal_frequency:
     ldi YH, high (dsp_buff_1) ; Load YH and YL as a pointer to 1st
     ldi YL, low (dsp_buff_1)  ; byte of dsp_buff_1 (Note - assuming 
                               ; (dsp_buff_1 for now).
     lpm R16, Z+               ; get dsply buff number (1st byte of msg).
     cpi r16, 1                ; if equal to '1', ptr already setup.
     breq get_msg_byte         ; jump and start message load.
     adiw YH:YL, 16            ; else set ptr to dsp buff 2.
     cpi r16, 2                ; if equal to '2', ptr now setup.
     breq get_msg_byte         ; jump and start message load.
     adiw YH:YL, 16            ; else set ptr to dsp buff 2.
        
get_msg_byte_2:
     lpm R16, Z+              ; get next byte of msg and see if '0'.        
     cpi R16, 0               ; if equal to '0', end of message reached.
     breq decimal_period_load       ; jump and start storing of calibrate message. 
     st Y+, R16               ; else, store next byte of msg in buffer.
     rjmp get_msg_byte        ; jump back and continue...

decimal_frequency_load:

	;r15 is the 5th digit.
	;r14 high is the 4th digit.
	;r14 low is the 3rd digit.
	;r13 high is the 2nd digit.
	;r13 low is the 1st digit.
	
	;print 5th digit
	mov r16, r15
	rcall load_number

	;print 4th digit
	mov r16, r14
	andi r16, $F0
	lsr r16
	lsr r16
	lsr r16
	lsr r16
	rcall load_number

	;print 3rd digit
	mov r16, r14			;copy bcd digit into r16 for subroutine
	andi r16, $0F			;clear irrelevant parts of r14.
	rcall load_number		;loads the correct number
	;print 2nd digit
	mov r16, r13			;copy r13 into r16 and clear low nibble
	andi r16, $f0			
	lsr r16					;shift high nibble down to low nibble
	lsr r16
	lsr r16
	lsr r16					
	rcall load_number		;load bcd digit to display

	;print 1st digit
	mov r16, r13			;copy bcd digit into r16 and clear high nibble
	andi r16, $0f			
	rcall load_number		;load bcd digit to display

	ldi r16, $48			;load an H into the buffer
	st Y+, r16				

	ldi r16, $7A			;load a z into the buffer
	st Y+, r16

	ret

;*******************
;NAME:      load_number
;FUNCTION:  Loads a BCD number into the buffer. 
;MODIFIES:  r16, Y
;CALLS:     load_0-9
;CALLED BY: load_decimal_period 
;********************************************************************
load_number:
	cpi r16, 0
	brne check_1
	rcall load_0
	ret
check_1:
	cpi r16, 1
	brne check_2
	rcall load_1
	ret
check_2:
	cpi r16, 2
	brne check_3
	rcall load_2
	ret
check_3:
	cpi r16, 3
	brne check_4
	rcall load_3
	ret
check_4:
	cpi r16, 4
	brne check_5
	rcall load_4
	ret
check_5:
	cpi r16, 5
	brne check_6
	rcall load_5
	ret
check_6:
	cpi r16, 6
	brne check_7
	rcall load_6
	ret
check_7:
	cpi r16, 7
	brne check_8
	rcall load_7
	ret
check_8 :
	cpi r16, 8
	brne check_9
	rcall load_8
	ret
check_9:
	cpi r16, 9
	brne check_10
	rcall load_9
	ret
check_10:
	cpi r16, 10
	brne check_11
	rcall load_1
	rcall load_0
	ret
check_11:
	cpi r16, 11
	brne check_12
	rcall load_1
	rcall load_1
	ret
check_12:
	cpi r16, 12
	brne check_13
	rcall load_1
	rcall load_2
	ret
check_13:
	cpi r16, 13
	brne check_14
	rcall load_1
	rcall load_3
	ret
check_14:
	cpi r16, 14
	brne check_15
	rcall load_1
	rcall load_4
	ret
check_15:
	rcall load_1
	rcall load_2
	rcall load_3
	ret

;************************
;NAME:      clr_dsp_buffs
;FUNCTION:  Initializes dsp_buffers 1, 2, and 3 with blanks (0x20)
;ASSUMES:   Three CONTIGUOUS 16-byte dram based buffers named
;           dsp_buff_1, dsp_buff_2, dsp_buff_3.
;RETURNS:   nothing.
;MODIFIES:  r25,r26, Z-ptr
;CALLS:     none
;CALLED BY: main application and diagnostics
;********************************************************************
clr_dsp_buffs:
     ldi R25, 48               ; load total length of both buffer.
     ldi R26, ' '              ; load blank/space into R26.
     ldi ZH, high (dsp_buff_1) ; Load ZH and ZL as a pointer to 1st
     ldi ZL, low (dsp_buff_1)  ; byte of buffer for line 1.
   
    ;set DDRAM address to 1st position of first line.
store_bytes:
     st  Z+, R26       ; store ' ' into 1st/next buffer byte and
                       ; auto inc ptr to next location.
     dec  R25          ; 
     brne store_bytes  ; cont until r25=0, all bytes written.
     ret

;*******************
;NAME:      load_1
;FUNCTION:  Loads the ASCII code for a 1 into the buffer.
;ASSUMES:   The buffer is partly filled already.
;RETURNS:   nothing.
;MODIFIES:  Y, r16
;CALLS:     nothing
;CALLED BY:  
;********************************************************************
;********************************************************************
load_1:
	ldi r16, $31	;the ASCII code for a 1.
	st Y+, r16		;stores the ASCII byte in the buffer.
ret

;*******************
;NAME:      load_0
;FUNCTION:  Loads the ASCII code for a 0 into the buffer.
;ASSUMES:   The buffer is partly filled already.
;RETURNS:   nothing.
;MODIFIES:  Y, r16
;CALLS:     nothing
;CALLED BY:  
;********************************************************************
;********************************************************************
load_0:
	ldi r16, $30	;the ASCII code for a 0.
	st Y+, r16		;stores the ASCII byte in the buffer.
ret

;*******************
;NAME:      load_3
;FUNCTION:  Loads the ASCII code for a 3 into the buffer.
;ASSUMES:   The buffer is partly filled already.
;RETURNS:   nothing.
;MODIFIES:  Y, r16
;CALLS:     nothing
;CALLED BY:  
;********************************************************************
;********************************************************************

;*******************
;NAME:      load_2
;FUNCTION:  Loads the ASCII code for a 2 into the buffer.
;ASSUMES:   The buffer is partly filled already.
;RETURNS:   nothing.
;MODIFIES:  Y, r16
;CALLS:     nothing
;CALLED BY:  
;********************************************************************
;********************************************************************
load_2:
	ldi r16, $32
	st Y+, r16
ret


load_3:
	ldi r16, $33
	st Y+, r16
ret

;*******************
;NAME:      load_4
;FUNCTION:  Loads the ASCII code for a 4 into the buffer.
;ASSUMES:   The buffer is partly filled already.
;RETURNS:   nothing.
;MODIFIES:  Y, r16
;CALLS:     nothing
;CALLED BY:  
;********************************************************************
;********************************************************************
load_4:
	ldi r16, $34
	st Y+, r16
ret
;*******************
;NAME:      load_5
;FUNCTION:  Loads the ASCII code for a 5 into the buffer.
;ASSUMES:   The buffer is partly filled already.
;RETURNS:   nothing.
;MODIFIES:  Y, r16
;CALLS:     nothing
;CALLED BY:  
;********************************************************************
;********************************************************************
load_5:
	ldi r16, $35
	st Y+, r16
ret
;*******************
;NAME:      load_6
;FUNCTION:  Loads the ASCII code for a 6 into the buffer.
;ASSUMES:   The buffer is partly filled already.
;RETURNS:   nothing.
;MODIFIES:  Y, r16
;CALLS:     nothing
;CALLED BY:  
;********************************************************************
;********************************************************************
load_6:
	ldi r16, $36
	st Y+, r16
ret

;*******************
;NAME:      load_7
;FUNCTION:  Loads the ASCII code for a 7 into the buffer.
;ASSUMES:   The buffer is partly filled already.
;RETURNS:   nothing.
;MODIFIES:  Y, r16
;CALLS:     nothing
;CALLED BY:  
;********************************************************************
;********************************************************************
load_7:
	ldi r16, $37
	st Y+, r16
ret

;*******************
;NAME:      load_8
;FUNCTION:  Loads the ASCII code for a 8 into the buffer.
;ASSUMES:   The buffer is partly filled already.
;RETURNS:   nothing.
;MODIFIES:  Y, r16
;CALLS:     nothing
;CALLED BY:  
;********************************************************************
;********************************************************************
load_8:
	ldi r16, $38
	st Y+, r16
ret

;*******************
;NAME:      load_9
;FUNCTION:  Loads the ASCII code for a 9 into the buffer.
;ASSUMES:   The buffer is partly filled already.
;RETURNS:   nothing.
;MODIFIES:  Y, r16
;CALLS:     nothing
;CALLED BY:  
;********************************************************************
;********************************************************************
load_9:
	ldi r16, $39
	st Y+, r16
ret


;*******************
;NAME:      load_period_mark
;FUNCTION:  Loads the ASCII code for a . into the buffer.
;ASSUMES:   The buffer is partly filled already.
;RETURNS:   nothing.
;MODIFIES:  Y, r16
;CALLS:     nothing
;CALLED BY:  
;********************************************************************
;********************************************************************
load_period_mark:
	ldi r16, $2E	;the ASCII code for a .
	st Y+, r16		;stores the ASCII byte in the buffer.
ret

;*******************
;NAME:      load_us
;FUNCTION:  Loads the ASCII code for ms into the buffer.
;ASSUMES:   The buffer is partly filled already.
;RETURNS:   nothing.
;MODIFIES:  Y, r16
;CALLS:     nothing
;CALLED BY:  
;********************************************************************
;********************************************************************
load_ms:
	ldi r16, $75	;the ASCII code for u
	ldi r16, $73	;the ASCII code for s
	st Y+, r16		;stores the ASCII byte in the buffer.
ret


;**********************************************************************
;************* M A I N   A P P L I C A T I O N   C O D E  *************
;**********************************************************************

RESET:
    ldi r16, low(RAMEND)  ; init stack/pointer
    out SPL, r16          ;
    ldi r16, high(RAMEND) ;
    out SPH, r16

	;Name registers used
	.DEF PERIODL = R20		;represents low byte of period
	.DEF PERIODH = R21		;represents high byte of period
	.DEF FREQL = R22		;represents low byte of frequency
	.DEF FREQH = R23		;represents high byte of frequency


	;Initialize ports
	;Port A
	LDI R16, 0x0E	;0000 1110
	OUT DDRA, R16	;Outputs for LED
	LDI R16, 0xF1	;1111 0001
	OUT PORTA, R16	;Pull-ups enabled, LEDs off
	;Port B
	LDI R16, 0xFF	;Set port B as all outputs
	OUT DDRB, R16
	sbi PORTB, 4	;set /SS of DOG LCD = 1 (Deselected)
	;PORT C
	LDI R16, 0x00	;All port C pins are inputs
	OUT DDRC, R16
	LDI R16, 0x03	;Enable C6, C7 pull-ups
	OUT PORTC, R16
	;Port D 
	LDI R16, 0x00	;All inputs (desired period)
	OUT DDRD, R16

	;initialize SPI
	rcall init_lcd_dog    ; init display, using SPI serial interface

	;intialize timer 1
	ldi r16, 0		;clear all relevant registers
	sts TCNT1H, r16
	sts TCNT1L, r16
	sts TCCR1A, r16		;sets the timer with clock off,
	sts TCCR1B, r16		;and in normal mode with a prescalar of 1.

	;configure interrupts
		
	
	ldi r16, (1<<ISC11) | (1<<ISC10)	;configure for positive edge detection
	sts EICRA, r16
	bset 7		;global enable for interrupts




start:
	ldi r16, 1<<INT1		;enable interrupt 1 request
	out EIMSK, r16
							;check if a valid count has been found

	brhc start				;if not, do nothing
							;if yes, update
	bclr 5					;clear the half carry flag
	ldi r16, 0<<INT1		;clear interrupt 1 request
	out EIMSK, r16

frequency_calc:
	;need 6 registers:
	;r16-r19 contain 1 million
	;r22 and r23 contain frequency
	mov r22, PERIODL		;prepare divisor registers
	mov r23, PERIODH
	clr r24
	clr r25
	
	ldi r18, LOW(1000000)		;load 1 million into dividend registers
	ldi r19, HIGH(1000000)
	ldi r20, BYTE3(1000000)
	ldi r21, BYTE4(1000000)

	rcall div32u			;1million / period = frequency
	
	push r22		;save period values to stack
	push r23
	
	mov FREQL, r18	;move calculated freq to designated registers
	mov FREQH, r19
	
	pop PERIODH			;return period values to designated registers
	pop PERIODL




display:
	rcall clr_dsp_buffs   ; clear all three buffer lines
	mov r17, FREQH					
	mov r16, FREQL
	rcall Bin2BCD16		;Convert frequency to BCD

	;-----Measure period-----
	;All store values in milliseconds EXCEPT R16
	;load_line_1 into dbuff1:
   ldi  ZH, high(line1_testmessage<<1)  ;
   ldi  ZL, low(line1_testmessage<<1)   ;
   rcall load_decimal_frequency         ; load message into buffer(s).

   MOV r17, PERIODH		;convert period to BCD
   MOV R16, PERIODL
   rcall Bin2BCD16
   ldi  ZH, high(line2_testmessage<<1)  ;
   ldi  ZL, low(line2_testmessage<<1)   ;
   rcall load_decimal_period		    ; load message into buffer(s).

	rcall update_lcd_dog
	rjmp start

period_calc:
	push r16		;save r16 to the stack

	;if this is the start of a wave, the carry bit will not be set
	brcc set_timer
	nop			;forces an equal delay between starting and setting timer
	;otherwise, turn off timer and find calculated period
	ldi r16, 0				;stop timer
	sts TCCR1B, r16
	lds PERIODL, TCNT1L		;get number of cycles
	lds PERIODH, TCNT1H
	
	sts TCNT1H, r16			;clear timer count
	sts TCNT1L, r16
	clc						;clear carry so next interrupt knows to start timer
	bset 5					;set half carry flag to update display
	pop r16					;return r16's value
	reti

set_timer:
	ldi r16, 0b00000001		;sets CS10 = 1, so timer increments on clock
	sts TCCR1B, r16
	sec						;sets the carry, so next interrupt knows to stop timer
	pop r16					;return r16's value
	reti  


;**************************************************************
;***** ALL MESSAGES: Fixed format, flash stored/loaded   ******
;**************************************************************


line1_testmessage: .db 1, "Frq =    ", 0  ; message for line #1.
line2_testmessage: .db 2, "Prd =    ", 0  ; message for line #2.
