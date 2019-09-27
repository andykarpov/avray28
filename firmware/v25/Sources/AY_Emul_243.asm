; AY-3-8912 IC emulator version 24.3 for ATMega8 20.09.2016
;
; Sources for Atmel AVRStudio 5
;
; visit our site for more information
; These source codes are distributed under the GPL v3 license
; If you share these sources you should put the link to web site www.avray.ru
;
; ORIGIN: http://www.avray.ru

#define CHANNELS 2	; choose 2 or 3 channel version
#define SPEAKER 0	; use SPEAKER port input on PD1 (0 - no, 1 - yes)

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
.def	C1F		= r2
.def	CntN		= r3
.def	TMP2		= r4
.def	BusOut1		= r5
.def	NoiseAddon	= r6
.def	C00		= r7
.def	CC0		= r8
.def	BusOut2		= r9
.def	C04		= r10
.def	TMP		= r11
.def	BusData		= r12
.def	SREGSave	= r13
.def	RNGL		= r14
.def	RNGH		= r15
.def	TabE		= r16
.def	EVal		= r17
.def	TabP		= r18
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
Volumes:
	.db 0,1,2,3,4,5,6,7,9,11,13,16,22,31,42,58

; volume table for envelopes
EVolumes:
	.db 0,1,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,20,22,24,26,28,30,33,36,40,45,51,58

; envelope codes
Envelopes:
	.db 7,7,7,7,4,4,4,4,1,7,3,5,0,6,2,4

; mask applied to registers values after receiving
RegsMask:
	.db 0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0x1F,0xFF,0x1F,0x1F,0x1F,0xFF,0xFF,0x0F,0xFF,0xFF


;==========================================================
_INT0_Handler:
	sbic	PinD,b3		; check BDIR bit
	rjmp	LATCH_REG_ADDR
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
LATCH_REG_ADDR:
	// LATCH ADDRESS MODE (BC1=1, BDIR=1) =============
	in	ADDR,PinC		; receive register number
	ldd	BusOut1,Z+0x20		; load value from SRAM
	ldd	BusOut2,Z+0x30		; load value from SRAM
	out	GIFR,CC0		; reset ext. interrupt flags
	reti

;==========================================================
_INT1_Handler:
	sbic	PinD,b2
	rjmp	LATCH_REG_ADDR
	// WRITE REGISTER MODE (BC=0, BDIR=1) =============
	in	BusData,PinC
	in	BusOut1,PinD
	in	SREGSave,SREG		; save SREG
	and	BusOut1,CC0
	or	BusData,BusOut1		; construct register value from 2 ports
	mov	BusOut1,BusData
	com	BusOut1			; invert register value
	std	Z+0x20,BusOut1		; put inverted register value to SRAM for read mode

	ld	BusOut2,Z         	; Load register mask from SRAM
	and	BusData,BusOut2		; apply register mask

	mov	BusOut2,BusOut1
	and	BusOut2,CC0
	std	Z+0x30,BusOut2		; put inverted register value to SRAM for read mode (2 high bits)
	std	Z+0x10,BusData		; put register value to SRAM
	cpi	ADDR,0x0D		; check for register 13
	brne	NO_ENVELOPE_CHANGED_P
	ori	TNLevel,0x80		; set flag that register 13 changed
NO_ENVELOPE_CHANGED_P:
	out	SREG,SREGSave
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
	ldi	ADDR,0x10
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


;==========================================================
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

	clr	C00
	ldi	r16,0xC0
	mov	CC0,r16
	ldi	r16,0x04
	mov	C04,r16
	ldi	r16,0x1F
	mov	C1F,r16
	ldi	r16,0xFF
	mov	BusOut1,r16
	mov	BusOut2,CC0
	clr	RNGH

	// clear register values in SRAM 0x110-0x13F
	ldi	r18,0x10
	ldi	ZL,0x10
	ldi	ZH,0x01
LOOP0:
	std	Z+0x20,CC0
	std	Z+0x10,r16
	st	Z+,C00
	dec	r18
	brne	LOOP0

	// load envelope codes to SRAM 0x210, 16 bytes
	ldi	xh,0x02
	ldi	xl,0x10
	ldi 	zl, low(2*Envelopes)
	ldi 	zh, high(2*Envelopes)
	ldi	r18,0x10
	rcall _COPY

	// load volume table for amplitude to SRAM 0x220, 16 bytes
	ldi	xl,0x20
	ldi 	zl, low(2*Volumes)
	ldi 	zh, high(2*Volumes)
	ldi	r18,0x10
	rcall _COPY

	// load volume table for envelopes to SRAM 0x230, 32 bytes
	ldi	xl,0x30
	ldi 	zl, low(2*EVolumes)
	ldi 	zh, high(2*EVolumes)
	ldi	r18,0x20
	rcall _COPY

	// load register masks to SRAM 0x100, 16 bytes
	clr	xl
	ldi	xh,0x01
	ldi 	zl, low(2*RegsMask)
	ldi 	zh, high(2*RegsMask)
	ldi	r18,0x10
	rcall _COPY


	ldi	ZH,0x01		; set high byte of register Z for fast acces to register values
	ldi	YH,0x02		; set high byte of register Y for fast acces to volume table
	mov	RNGL,ZH
	mov	NoiseAddon,ZH

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
	; ICR1L value formula (28000000/109375/2 - 1) where 28000000 = 28MHz - AVR oscillator frequency
	; 109375 is for 1.75 MHz version, formula is (PSG frequency / 16) e.g. for 2MHz it is 2000000/16 = 125000

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
	mov	TNLevel,YH
	clr	OutA
	clr	OutC
	mov	TabP,CC0
	clr	TabE
	clr	BusData
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
	sbrs	TNLevel,b7
	rjmp	NO_ENVELOPE_CHANGED

	// initialize envelope generator after change envelope shape register, only first 1/32 part of the first period!
	lds	YL,AY_REG13	; load envelope shape register value to TabE
	ldd	TabE,Y+0x10	; get envelope code from SRAM
	ldi	TabP,0x1F	; set counter for envelope period
	andi	TNLevel,0x7F	; clear envelope shape change flag
	rjmp	E_NEXT_PERIOD

NO_ENVELOPE_CHANGED:
	cpi	TabE,0x80	; check if envelope generator is disabled
	brcc	ENVELOPE_GENERATOR_END
	sbiw	CntEL,0x01
	brcs	E_NEXT_STEP
	brne	ENVELOPE_GENERATOR_END
E_NEXT_STEP:
	dec	TabP
	brpl	E_NEXT_PERIOD
	ldi	TabP,0x1F
	sbrc	TabE,b1
	eor	TabE,ZH
	sbrc	TabE,b2
	or	TabE,CC0
E_NEXT_PERIOD:
	lds	CntEL,AY_REG11
	lds	CntEH,AY_REG12
	mov	YL,TabP
	sbrs	TabE,b0		; invert envelope value if ATTACK bit is not set
	eor	YL,C1F
	ldd	EVal,Y+0x30	; translate envelope value to volume, read volume value from SRAM 0x230+YL
ENVELOPE_GENERATOR_END:
	/////////////////////////////////////////////////////////////////////////////////////


	/////////////////////////////////////////////////////////////////////////////////////
	/// NOISE GENERATOR
	/////////////////////////////////////////////////////////////////////////////////////
	inc	CntN		; increase noise period counter
	lds	TMP,AY_REG06
	cp CntN,TMP		; compare noise period counter with value in AY register 6
	brcs	NOISE_GENERATOR_END
	clr	CntN
	andi	TNLevel,0xC7
	lsr	NoiseAddon
	mov	NoiseAddon,RNGL
	ror	RNGH
	ror	RNGL
	brcc	NO_LEVEL_CHANGED
	ori	TNLevel,0x38
NO_LEVEL_CHANGED:
	lsr	RNGL
	eor	NoiseAddon,RNGL
	rol	RNGL
NOISE_GENERATOR_END:
	/////////////////////////////////////////////////////////////////////////////////////


	/////////////////////////////////////////////////////////////////////////////////////
	/// TONE GENERATOR
	/////////////////////////////////////////////////////////////////////////////////////
	; all counters are Int16 values (signed)
	// Channel A -------------
	subi	CntAL,0x01	; CntA - 1
	sbci	CntAH,0x00
	brcs	RENEW_A		; CntA == -1
	brne	CH_A_NO_CHANGE	; CntA != 0
RENEW_A:
	lds	CntAL,AY_REG00	; update channel A tone period counter
	lds	CntAH,AY_REG01
	eor	TNLevel,ZH	; TNLevel xor 1 (change logical level of channel A)
CH_A_NO_CHANGE:

	// Channel B -------------
	subi	CntBL,0x01	; CntB - 1
	sbci	CntBH,0x00
	brcs	RENEW_B		; CntB == -1
	brne	CH_B_NO_CHANGE	; CntB != 0
RENEW_B:
	lds	CntBL,AY_REG02	; update channel B tone period counter
	lds	CntBH,AY_REG03
	eor	TNLevel,YH	; TNLevel xor 2 (change logical level of channel B)
CH_B_NO_CHANGE:

	// Channel C -------------
	sbiw	CntCL,0x01	; CntC - 1
	brcs	RENEW_C		; CntC == -1
	brne	CH_C_NO_CHANGE	; CntC != 0
RENEW_C:
	lds	CntCL,AY_REG04	; update channel C tone period counter
	lds	CntCH,AY_REG05
	eor	TNLevel,C04	; TNLevel xor 4 (change logical level of channel C)
CH_C_NO_CHANGE:
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
	mov	YL,TMP
	lsr	TMP			; TMP = TMP - (TMP/4 + TMP/8);
	lsr	TMP
	sub	YL,TMP
	lsr	TMP
	sub	YL,TMP
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

// --------------------------------------------------------------
#if SPEAKER == 1
// speaker port enabled -----------------------------------------
	sbis	PinD,b1		; check PD1 (SPEAKER PORT INPUT)
	rjmp	PWM_OUT
	add		OutA,C1F
	add		OutC,C1F
PWM_OUT:
// --------------------------------------------------------------
#endif

	out	OCR1AL,OutA	; update PWM counters
	out	OCR1BL,OutC
	rjmp	_MAIN_LOOP
	// MAIN LOOP END ====================================================================


// copy routine from flash to SRAM
_COPY:
	lpm	r16,Z+
	st	X+,r16
	dec	r18
	brne	_COPY
	ret



/////////////////////////////////////////////////////////////////////////////////////////////
/// SRAM MAP
/////////////////////////////////////////////////////////////////////////////////////////////
; 0x100-0x10F - register masks
; 0x110-0x11F - register values
; 0x120-0x12F - inverted register values for read mode
; 0x130-0x13F - inverted register values for read mode only 2 high bits
;
; 0x210-0x21F - Encoded envelopes
; 0x220-0x22F - Volume Table for Amplitudes, 16 volume levels
; 0x230-0x24F - Volume Table for Envelopes, 32 volume levels

/////////////////////////////////////////////////////////////////////////////////////////////
/// AY REGISTERS IN SRAM
/////////////////////////////////////////////////////////////////////////////////////////////
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
