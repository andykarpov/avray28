; AY-3-8912 IC emulator version 23.0 for ATMega8 18.02.2016
;
; Sources for Atmel AVRStudio
;
; visit our site for more information
; These source codes are distributed under the GPL v3 license
; If you share these sources you should put the link to web site www.avray.ru
;
; ORIGIN: http://www.avray.ru

#define CHANNELS 2	; choose 2 or 3 channel version
#define STANDARD 1	; chose standard (1) or alternate (0) version

	.cseg
	.org	0x0000

; bit numbers:
.equ	b0	= 0x00
.equ	b1	= 0x01
.equ	b2	= 0x02
.equ	b3	= 0x03
.equ	b4	= 0x04
.equ	b5	= 0x05
.equ	b6	= 0x06
.equ	b7	= 0x07

; register variables:
.def	OutA		= r0
.def	OutC		= r1
.def	TabP		= r2
.def	CntN		= r3
.def	C1F		= r4
.def	BusOut1		= r5
.def	NoiseAddon	= r6
.def	C00		= r7
.def	CC0		= r8
.def	TabE		= r9
.def	C04		= r10
.def	C60		= r11
.def	BusData		= r12
.def	SREGSave	= r13
.def	RNGL		= r14
.def	RNGH		= r15
.def	TMP		= r16
.def	EVal		= r17
.def	BusOut2		= r18
.def	TNLevel		= r19
.def	CntAL		= r20
.def	CntAH		= r21
.def	CntBL		= r22
.def	CntBH		= r23
.def	CntCL		= r24
.def	CntCH		= r25
.def	CntEL		= r26
.def	CntEH		= r27
.def	ADDR		= r30

;------------------------------------------------------
; INTERRUPT VECTORS TABLE
;------------------------------------------------------
	rjmp	_RESET
	rjmp	_INT0_Handler
	rjmp	_INT1_Handler
	reti
	reti
	reti
	reti
	reti
	reti
	reti
	reti
	rjmp	_USART_RX_COMPLETE


; volume table for amplitude
VolTab1:
	.db 0,1,2,3,5,6,8,12,14,18,21,23,27,31,36,41

#define Envelope_Max_Volume 44		; should be as the last value in VolTab2
; volume table for envelopes
VolTab2:
	.db 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,21,22,23,25,27,29,31,33,36,39,41,Envelope_Max_Volume

; mask applied to registers values after receiving
RegsMask:
	.db 0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0x1F,0xFF,0x1F,0x1F,0x1F,0xFF,0xFF,0x0F,0xFF,0xFF


;==========================================================
_INT0_Handler:
	sbic	PinD,b3		; check BDIR bit
	rjmp	LATCH_REG_ADDR0
	// READ MODE (BC1=1,BDIR=0) =======================
	// turn pins to output
	out	DDRD,BusOut2
	out	DDRC,BusOut1
LOOP_NOT_INACTIVE: // loop while BC1=1
	sbic	PinD,b2
	rjmp	LOOP_NOT_INACTIVE
	// turn pins to input
	out	DDRC,C00
	out	DDRD,C00
	reti

LATCH_REG_ADDR0:
	// LATCH ADDRESS MODE in INT 0 (BC1=1, BDIR=1) ====
	in	ADDR,PinC		; receive register number
	ldd	BusOut1,Z+0x20		; load value from SRAM
	ldd	BusOut2,Z+0x30		; load value from SRAM
	out	GIFR,CC0		; reset ext. interrupt flags
	reti

;==========================================================
_INT1_Handler:
	in	BusData,PinC
	sbic	PinD,b2
	rjmp	LATCH_REG_ADDR1
	// WRITE REGISTER MODE (BC=0, BDIR=1) =============
	in	BusOut1,PinD
	in	SREGSave,SREG		; save SREG
	and	BusOut1,CC0		; BusEx &= 0xC0
	or	BusData,BusOut1		; BusData |= BusEx

	ld	BusOut1,Z         	; Load register mask from SRAM
	and	BusData,BusOut1		; apply register mask

	; prepare register data for read mode (port C)
	mov	BusOut1,BusData
	com	BusOut1
	std	Z+0x20,BusOut1
	; --------------------------

#if STANDARD == 0
// Alternate version only part ----------
	; swap bits 5 and 6
	mov	BusOut2,BusData
	subi	BusOut2,0xE0		; BusOut2 += 0x20
	sbrc	BusOut2,b6		; check if bits 5 and 6 are differ, skip next if not
	eor	BusData,C60		; invert bits 5 and 6 if they are differ
// --------------------------------------
#endif	

	; prepare register data for read mode (port D)
	mov	BusOut2,BusOut1
	and	BusOut2,CC0
	std	Z+0x30,BusOut2
	; --------------------------

	std	Z+0x10,BusData		; put register value to SRAM

	cpi	ADDR,0x0D		; check for envelope shape register
	brne	NO_ENVELOPE_CHANGED_P
	ori	TNLevel,0x80		; set envelope change flag (bit 7)
NO_ENVELOPE_CHANGED_P:
	out	SREG,SREGSave		; restore SREG
	reti

LATCH_REG_ADDR1:
	// LATCH ADDRESS MODE in INT 1 (BC1=1, BDIR=1) ====
	mov	ADDR,BusData	; receive register number
	ldd	BusOut1,Z+0x20	; load register from SRAM
	ldd	BusOut2,Z+0x30	; load register from SRAM
	out	GIFR,CC0	; reset ext. interrupt flags
	reti


;==========================================================
_USART_RX_COMPLETE:
	in	BusData,UDR		; get byte from USART
	sbrs	ADDR,b4			; check for address/data mode
	rjmp	RECV_REG_VALUE
	sbrc	BusData,b7		
	rjmp	USART_SYNC
	mov	ADDR,BusData            ; ------ receive register number ------------
	reti
USART_SYNC:				; ------ synchronization mode ---------------
	ldi	ADDR,0x10	; set bit 4 to jump to receive register number on next byte received
	reti
RECV_REG_VALUE:				; ------ receive register value -------------
	in	SREGSave,SREG

	ld	BusOut1,Z		; load register mask
	and	BusData,BusOut1		; apply register mask

	std	Z+0x10,BusData		; put register value to SRAM

	cpi	ADDR,0x0D		; check for envelope shape register
	brne	NO_ENVELOPE_CHANGED_S
	ori	TNLevel,0x80		; set envelope change flag (bit 7)
NO_ENVELOPE_CHANGED_S:
	ldi	ADDR,0x10	; set bit 4 to jump to receive register number on next byte received
	out	SREG,SREGSave
	reti

////////////////////////////////////////////////////////////////////////////////
_RESET:
	// 1-> PUD
	in	r16,SFIOR
	sbr	r16,PUD
	out	SFIOR,r16

	// init stack pointer 0x45F
	ldi	r16,low(RAMEND)
	out	SPL,r16
	ldi	r16,high(RAMEND)
	out	SPH,r16

	// disable Analog Comparator
	sbi ACSR,ACD

	// init constants and variables first part
	clr	C00
	ldi	r16,0xC0
	mov	CC0,r16
	ldi	r16,0x04
	mov	C04,r16
	ldi	r16,0x1F
	mov	C1F,r16
	ldi	r16,0x60
	mov	C60,r16
	ldi	r16,0xFF
	mov	RNGL,r16
	mov	RNGH,r16

	// clear register values in SRAM 0x110-0x11F
	ldi	r18,0x10
	ldi	ZL,0x10
	ldi	ZH,0x01
LOOP0:
	st	Z+,C00
	dec	r18

	// clear values in SRAM 0x120-0x13F
	ldi	r18,0x20
	ldi	ZL,0x20
	ldi	ZH,0x01
LOOP01:
	st	Z+,r16
	dec	r18
	brne	LOOP01

	// set 0xFF values for registers 14,15 (for correct detection in tests)
	sts	AY_REG14,r16
	sts	AY_REG15,r16
	sts	AY_REG14_OUT1,C00	; inverted for using only DDR without PORT
	sts	AY_REG15_OUT1,C00	; inverted for using only DDR without PORT
	sts	AY_REG14_OUT2,C00	; inverted for using only DDR without PORT
	sts	AY_REG15_OUT2,C00	; inverted for using only DDR without PORT

	// load volume table for envelopes to SRAM 0x230, 32 bytes
	ldi	xh,0x02
	ldi	xl,0x30
	ldi 	zl, low(2*VolTab2)
	ldi 	zh, high(2*VolTab2)
	ldi	r18,0x20
LOOP1:
	lpm	r16,Z+
	st	X+,r16
	dec	r18
	brne	LOOP1

	// load volume table for amplitude to SRAM 0x220, 16 bytes
	ldi	xl,0x20
	ldi 	zl, low(2*VolTab1)
	ldi 	zh, high(2*VolTab1)
	ldi	r18,0x10
LOOP2:
	lpm	r16,Z+
	st	X+,r16
	dec	r18
	brne	LOOP2


	// load register masks to SRAM 0x100, 16 bytes
	clr	xl
	ldi	xh,0x01
	ldi 	zl, low(2*RegsMask)
	ldi 	zh, high(2*RegsMask)
	ldi	r18,0x10
LOOP3:
	lpm	r16,Z+
	st	X+,r16
	dec	r18
	brne	LOOP3


	ldi	ZH,0x01		; set high byte of register Z for fast acces to register values
	ldi	YH,0x02		; set high byte of register Y for fast acces to volume table

	// get byte 0 from EEPROM, check value > 0 or skip USART initialization if value = 0
	out	EEARH,C00
	out	EEARL,C00
	sbi	EECR,b0
	in	r16,EEDR
	cp	r16,C00
	breq	NO_USART

	// init USART
	clr	r16
	out	UBRRH,r16
	ldi	r16,0x86
	out	UCSRC,r16
	ldi	r16,0x02
	out	UCSRA,r16
	ldi	r16,0x90
	out	UCSRB,r16
	ldi	r16,0x03
	out	EEARL,r16
	sbi	EECR,b0
	in	r18,EEDR
	out	UBRRL,r18
NO_USART:

	// init Timer1
	out	OCR1AH,C00		; clear OCR values
	out	OCR1AL,C00
	out	OCR1BH,C00
	out	OCR1BL,C00
	sbi	DDRB,b1			; set port B pin 1 to output for PWM (AY channel A)
	sbi	DDRB,b2			; set port B pin 2 to output for PWM (AY channel B)
	ldi	r16,0xA2
	out	TCCR1A,r16
	ldi	r16,0x19
	out	TCCR1B,r16
	out	EEARL,YH		; set EEPROM address 2
	sbi	EECR,b0
	in	r18,EEDR		; load byte 2 from EEPROM to r18
	out	ICR1H,C00
	out	ICR1L,r18		; set PWM speed from byte 2 of EEPROM (affect AY chip frequency)

#if CHANNELS == 3
	// init Timer2
	sbi	DDRB,b3			; set port B pin 3 to output for PWM (AY channel C)
	ldi	r16,0x69
	out	TCCR2,r16
#endif

	// check for external interrupts enabled in byte 1 of EEPROM
	out	EEARL,ZH
	sbi	EECR,b0
	in	r16,EEDR
	cp	r16,C00
	breq	NO_EXT_INT
	// external interrupts initialization
	ldi	r16,0x0F		; fallen edge of INT0, INT1
	out	MCUCR,r16
	out	GIFR,CC0		; clear interrupt flags
	out	GICR,CC0		; enable interrupts
NO_EXT_INT:

	// init constants and variables second part
	ldi	ADDR,0x10
	ldi	TNLevel,0x28
	clr	OutA
	clr	OutC
	mov	TabP,CC0
	clr	TabE
	clr	BusData
	clr	BusOut1
	clr	BusOut2
	clr	NoiseAddon
	clr	CntN
	clr	CntAL
	clr	CntAH
	movw	CntBL,CntAL
	movw	CntCL,CntAL
	movw	CntEL,CntAL
	clr	EVal

	sei			; enable global interrupts

_MAIN_LOOP:
	// MAIN LOOP ========================================================================
	in	YL,TIFR		; check timer1 overflow flag TOV1
	sbrs	YL,TOV1
	rjmp	_MAIN_LOOP	; jump if not set
	
	out	TIFR,YL		; clear timer overflow flag

	/////////////////////////////////////////////////////////////////////////////////////
	/// ENVELOPE GENERATOR
	/////////////////////////////////////////////////////////////////////////////////////
	sbrs	TNLevel,b7	; check if envelope shape register is changed
	rjmp	NO_ENVELOPE_CHANGED

	// initialize envelope generator after change envelope shape register, only first 1/32 part of the first period!
	lds	TabE,AY_REG13	; load envelope shape register value to TabE
	mov	TabP,ZH		; 1 -> TabP
	andi	TNLevel,0x7F	; clear envelope changed flag
	lds	CntEL,AY_REG11	; initialize new envelope frequency counter
	lds	CntEH,AY_REG12
	clr	EVal		; set zero envelope volume as default
	sbrs	TabE,b2		; check ATTACK bit
	ldi	EVal,Envelope_Max_Volume	; set maximal volume if bit is set
	rjmp	ENVELOPE_GENERATOR_END

NO_ENVELOPE_CHANGED:

	sbrc	TabP,b7		; check if envelope generator is disabled
	rjmp	ENVELOPE_GENERATOR_END

	sbiw	CntEL,0x01	; decrease envelope frequency counter
	brcs	EG_PROCESS	; process changes if counter < 0
	brne	ENVELOPE_GENERATOR_END	; no changes if counter != 0

	; also process changes if counter == 0, so if counter <= 0 we are here
EG_PROCESS:	; process each 1/32 of envelope period
	lds	CntEL,AY_REG11	; initialize new envelope frequency counter
	lds	CntEH,AY_REG12
	sbrs	TabP,b5		; check if envelope period counter is overflow (we've already processed 32 parts of period)
	rjmp	ENV_NEXT_VAL	; jump to get next value if not overflow

	// initialize new envelope period (first of 32 parts)
	sbrc	TabE,b3		; check CONTINUE bit
	rjmp	NO_CONT

	; continue bit is cleared, it is first two envelopes from datasheet
	; envelopes 0 - 7 ---------------------------------
	clr	EVal		; set envelope value to 0
	or	TabP,CC0	; disable envelope generator
	rjmp	ENVELOPE_GENERATOR_END

NO_CONT:
	; envelopes 8-15 ----------------------------------
	sbrc	TabE,b0		; check HOLD bit
	rjmp	ENV_HOLD
	
	; envelopes 8, 10, 12, 14
        // init new envelope period
	clr	TabP
	sbrc	TabE,b1		; check ALTERNATE bit
	eor	TabE,C04	; invert ATTACK bit if ALTERNATE is set
	rjmp	ENV_NEXT_VAL
ENV_HOLD:
	; envelopes 9, 11, 13, 15
	or	TabP,CC0	; disable envelope generator
	clr	EVal		; set envelope minimal envelope value as default
	mov	TMP,TabE
	add	TMP,YH
	sbrc	TMP,b2		; check if envelope bits 1 and 2 is differ (TabE += 2 and check bit 3)
	ldi	EVal,Envelope_Max_Volume		; set max value if differ
	rjmp	ENVELOPE_GENERATOR_END
ENV_NEXT_VAL:
	mov	YL,TabP
	sbrs	TabE,b2		; invert envelope value if ATTACK bit is not set
	eor	YL,C1F
	inc	TabP		; increase envelope period counter value
ENV_TO_VOL:
	ldd	EVal,Y+0x30	; translate envelope value to volume, read volume value from SRAM 0x230+YL
ENVELOPE_GENERATOR_END:
	/////////////////////////////////////////////////////////////////////////////////////


	/////////////////////////////////////////////////////////////////////////////////////
	/// NOISE GENERATOR
	/////////////////////////////////////////////////////////////////////////////////////
	dec	CntN		; decrease noise period counter
	brpl	NOISE_GENERATOR_END
	lds	CntN,AY_REG06	; set new noise period counter value from AY register 6
	dec	CntN		; decrease noise period counter
	// pseudo random number generator
	mov	TMP,RNGL
	swap	TMP     	; swap nibbles
	lsr	TMP		; shift right
	eor	TMP,RNGL	; xor
	lsr	NoiseAddon	; move additional bit to carry flag
	ror	RNGH		; rotate right through carry
	ror	RNGL		; rotate right through carry
	; copy CARRY flag no noise bits
	ori TNLevel,0x38
	brcs	NOISE_HIGH
	andi	TNLevel,0xC7	; set noise low if carry is not set
NOISE_HIGH:
	sbrs	TMP,b3		; if bits 0 and 3 (before shift) differ skip next line
	or	NoiseAddon,ZH	; set random generator additional bit
NOISE_GENERATOR_END:
	/////////////////////////////////////////////////////////////////////////////////////


	/////////////////////////////////////////////////////////////////////////////////////
	/// TONE GENERATOR
	/////////////////////////////////////////////////////////////////////////////////////
	; all counters are Int16 values (signed)
	// Channel A -------------
	subi	CntAL,0x01	; CntA - 1
	sbci	CntAH,0x00
	brpl	CNTA_POSITIVE	; if CntA >= 0 goto CNTA_POSITIVE
	lds	CntAL,AY_REG00	; update channel A tone period counter
	lds	CntAH,AY_REG01
	subi	CntAL,0x01	; CntA - 1
	sbci	CntAH,0x00
	eor	TNLevel,ZH	; TNLevel xor 1 (change logical level of channel A)
CNTA_POSITIVE:

	// Channel B -------------
	subi	CntBL,0x01	; CntB - 1
	sbci	CntBH,0x00
	brpl	CNTB_POSITIVE	; if CntB >= 0 goto CNTB_POSITIVE
	lds	CntBL,AY_REG02	; update channel B tone period counter
	lds	CntBH,AY_REG03
	subi	CntBL,0x01	; CntB - 1
	sbci	CntBH,0x00
	eor	TNLevel,YH	; TNLevel xor 2 (change logical level of channel B)
CNTB_POSITIVE:

	// Channel C -------------
	sbiw	CntCL,0x01	; CntC - 1
	brpl	CNTC_POSITIVE	; if CntC >= 0 goto CNTC_POSITIVE
	lds	CntCL,AY_REG04	; update channel C tone period counter
	lds	CntCH,AY_REG05
	sbiw	CntCL,0x01	; CntC - 1
	eor	TNLevel,C04	; TNLevel xor 4 (change logical level of channel C)
CNTC_POSITIVE:
	/////////////////////////////////////////////////////////////////////////////////////



	/////////////////////////////////////////////////////////////////////////////////////
	/// MIXER
	/////////////////////////////////////////////////////////////////////////////////////
	lds	TMP,AY_REG07	; Load Mixer AY Register from SRAM
	or	TMP,TNLevel	; Mixer formula = (Mixer Register Tone | TNLevel Tone) & (Mixer Register Noise | TNLevel Noise)
	mov	YL,TMP
	lsl	YL
	swap	YL
	and	TMP,YL
	/////////////////////////////////////////////////////////////////////////////////////


	/////////////////////////////////////////////////////////////////////////////////////
	/// AMPLITUDE CONTROL
	/////////////////////////////////////////////////////////////////////////////////////

	// Channel A
	lds	YL,AY_REG08	; Load Channel A Amplitude register
	mov	OutA,EVal	; set envelope volume as default value
	sbrs	YL,b4		; if bit 4 is not set in amplitude register then translate it to volume
	ldd	OutA,Y+0x20	; load volume value from SRAM 0x220 + YL
	sbrs	TMP,b0		; if channel is disabled in mixer - set volume to zero
	clr	OutA

	// Channel C
	lds	YL,AY_REG10	; Load Channel C Amplitude register
	mov	OutC,EVal	; set envelope volume as default value
	sbrs	YL,b4		; if bit 4 is not set in amplitude register then translate it to volume
	ldd	OutC,Y+0x20	; load volume value from SRAM 0x220 + YL
	sbrs	TMP,b2		; if channel is disabled in mixer - set volume to zero
	clr	OutC

	// Channel B
#if CHANNELS == 2
// two channel version ----------------------------------------
	sbrs	TMP,b1		; jump if channel B is disabled in mixer
	rjmp	CHANNEL_B_DISABLED
	lds	YL,AY_REG09	; Load Channel B Amplitude register
	mov	TMP,EVal	; set envelope volume as default value
	sbrs	YL,b4
	ldd	TMP,Y+0x20	; load volume value from SRAM 0x220 + YL
	lsr	TMP		; TMP = TMP/2 + TMP/8;
	mov	YL,TMP
	lsr	TMP
	lsr	TMP
	add	YL,TMP
	add	OutA,YL		; add channel B volume to channels A and C
	add	OutC,YL
CHANNEL_B_DISABLED:
// --------------------------------------------------------------
#elif CHANNELS == 3
// three channel version ----------------------------------------
	clr YL
	sbrs	TMP,b1		; jump if channel B is disabled in mixer
	rjmp	CHANNEL_B_DISABLED
	lds	YL,AY_REG09	; Load Channel B Amplitude register
	mov	TMP,EVal	; set envelope volume as default value
	sbrs	YL,b4
	ldd	TMP,Y+0x20	; load volume value from SRAM 0x220 + YL
	lsl	TMP		; TMP = TMP*2 (as OCR2 has 255 levels)
	mov	YL,TMP
CHANNEL_B_DISABLED:
	out	OCR2,YL
// --------------------------------------------------------------
#endif

	out	OCR1AL,OutA	; update PWM counters
	out	OCR1BL,OutC
	rjmp	_MAIN_LOOP
	// MAIN LOOP END ====================================================================


	/////////////////////////////////////////////////////////////////
	/// SRAM MAP
	/////////////////////////////////////////////////////////////////
	; 0x100-0x10F - register masks
	; 0x110-0x11F - register values
	; 0x120-0x12F - inverted register values for read mode
	; 0x130-0x13F - inverted register values for read mode only 2 high bits
	; 0x220-0x22F - Volume Table 1 (for amplitudes) 16 volume levels
	; 0x230-0x24F - Volume Table 2 (for envelopes) 32 volume levels

	/////////////////////////////////////////////////////////////////
	/// AY REGISTERS IN SRAM
	/////////////////////////////////////////////////////////////////
	.dseg
	.org	0x0110
AY_REG00:
	.byte	1
AY_REG01:
	.byte	1
AY_REG02:
	.byte	1
AY_REG03:
	.byte	1
AY_REG04:
	.byte	1
AY_REG05:
	.byte	1
AY_REG06:
	.byte	1
AY_REG07:
	.byte	1
AY_REG08:
	.byte	1
AY_REG09:
	.byte	1
AY_REG10:
	.byte	1
AY_REG11:
	.byte	1
AY_REG12:
	.byte	1
AY_REG13:
	.byte	1
AY_REG14:
	.byte	1
AY_REG15:
	.byte	1

	.org	0x012E
AY_REG14_OUT1:
	.byte	1
AY_REG15_OUT1:
	.byte	1

	.org	0x013E
AY_REG14_OUT2:
	.byte	1
AY_REG15_OUT2:
	.byte	1
