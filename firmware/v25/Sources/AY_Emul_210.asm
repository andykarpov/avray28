; AY-3-8912 IC emulator for ATMega8
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

.DEVICE ATmega8
.include "m8def.inc"

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
.def	TNLevel		= r3
.def	C38		= r4
.def	C1F		= r5
.def	NoiseAddon	= r6
.def	C00		= r7
.def	CC0		= r8
.def	C3F		= r9
.def	C04		= r10
.def	BusEx		= r11
.def	BusData		= r12
.def	SREGSave	= r13
.def	RNGL		= r14
.def	RNGH		= r15
.def	TMP		= r16
.def	EVal		= r17
.def	TabE		= r18
.def	CntN		= r19
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
	rjmp	_RESET	; L0052
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

VolTab1:
	.db 0,1,2,3,5,6,8,12,14,18,21,23,27,31,36,41
VolTab2:
	.db 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,21,22,23,25,27,29,31,33,36,39,41,44
RegsMask:
	.db 0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0x1F,0xFF,0x1F,0x1F,0x1F,0xFF,0xFF,0x0F,0xFF,0xFF


;==========================================================
_INT0_Handler:
	sbic	PinD,b3		; check BDIR bit
	rjmp	LATCH_REG_ADDR0
	// READ MODE (BC1=1,BDIR=0) =======================
	// turn pins to output
	out	DDRD,CC0
	out	DDRC,C3F
LOOP_NOT_INACTIVE: // loop while BC1=1
	sbic	PinD,b2
	rjmp	LOOP_NOT_INACTIVE
	// turn pins to input
	out	DDRC,C00
	out	DDRD,C00
	reti

LATCH_REG_ADDR0:
	// LATCH ADDRESS MODE in INT 0 (BC1=1, BDIR=1) ====
	in	ADDR,PinC	; receive register number

#if STANDARD == 1
// standard version only -----------------------------------
	ldd	BusData,Z+0x10	; load register from SRAM
//----------------------------------------------------------
#elif STANDARD == 0
// alternate version only ----------------------------------
	ldd	BusData,Z+0x20	; load register from SRAM
//----------------------------------------------------------
#endif
	out	PortC,BusData	; send register value to port (for fast read mode)
	out	PortD,BusData
	out	GIFR,CC0	; reset ext. interrupt flags
	reti

;==========================================================
_INT1_Handler:
	in	BusData,PinC
	sbic	PinD,b2
	rjmp	LATCH_REG_ADDR1
	// WRITE REGISTER MODE (BC=0, BDIR=1) =============
	in	BusEx,PinD
	in	SREGSave,SREG	; save SREG
	and	BusEx,CC0	; BusEx &= 0xC0
	or	BusData,BusEx	; BusData |= BusEx

	ld	BusEx,Z         ; Load register mask from SRAM
	and	BusData,BusEx	; apply register mask

#if STANDARD == 0
// Alternate version only part ----------
	std	Z+0x20,BusData
	mov	BusEx,BusData
	out	PortC,BusData	; send register value to port (for fast read mode)
	out	PortD,BusData
	bst	BusEx,b5	; swap bits 5 and 6
	bld	BusData,b6
	bst	BusEx,b6
	bld	BusData,b5
// --------------------------------------
#endif
	
	std	Z+0x10,BusData	; put register value to SRAM

#if STANDARD == 1
// Standard version only part -----------
	out	PortC,BusData	; send register value to port (for fast read mode)
	out	PortD,BusData
// --------------------------------------
#endif	
	cpi	ADDR,0x0D	; check for envelope shape register
	brne	NO_ENVELOPE_CHANGED_P
	or	TNLevel,CC0
NO_ENVELOPE_CHANGED_P:
	out	SREG,SREGSave	; restore SREG
	reti

LATCH_REG_ADDR1:
	// LATCH ADDRESS MODE in INT 1 (BC1=1, BDIR=1) ====
	mov	ADDR,BusData	; receive register number

#if STANDARD == 1
// standard version only -----------------------------------
	ldd	BusData,Z+0x10	; load register from SRAM
//----------------------------------------------------------
#elif STANDARD == 0
// alternate version only ----------------------------------
	ldd	BusData,Z+0x20	; load register from SRAM
//----------------------------------------------------------
#endif

	out	PortC,BusData	; send to port
	out	PortD,BusData
	out	GIFR,CC0	; reset ext. interrupt flags
	reti


;==========================================================
_USART_RX_COMPLETE:
	in	BusData,UDR
	sbrs	ADDR,b4
	rjmp	RECV_REG_VALUE
	sbrc	BusData,b7
	rjmp	USART_SYNC
	mov	ADDR,BusData
	reti
USART_SYNC:
	mov	ADDR,C3F
	reti
RECV_REG_VALUE:
	in	SREGSave,SREG
	ld	BusEx,Z
	and	BusData,BusEx		; apply registers mask
	std	Z+0x10,BusData

	cpi	ADDR,0x0D		; check for envelope shape register
	brne	NO_ENVELOPE_CHANGED_S
	or	TNLevel,CC0
NO_ENVELOPE_CHANGED_S:
	mov	ADDR,C3F
	out	SREG,SREGSave
	reti

////////////////////////////////////////////////////////////////////////////////
_RESET:
	// 1-> PUD
	in	r16,SFIOR
	ori	r16,0x04
	out	SFIOR,r16

	// init stack pointer 0x45F
	ldi	r16,0x5F
	out	SPL,r16
	ldi	r16,0x04
	out	SPH,r16

	// disable AC
	ldi	r16,0x81
	out	ACSR,r16

	// init variables
	clr	C00
	ldi	r16,0xC0
	mov	CC0,r16
	ldi	r16,0x3F
	mov	C3F,r16
	ldi	r16,0x04
	mov	C04,r16
	ldi	r16,0x38
	mov	C38,r16
	ldi	r16,0x1F
	mov	C1F,r16
	ldi	r16,0xFF
	mov	RNGL,r16
	mov	RNGH,r16

	// clear sram REGS + 0x20
	ldi	r18,0x20
	ldi	ZL,0x10
	ldi	ZH,0x01
L006A:
	st	Z+,C00
	dec	r18
	brne	L006A

	sts	D011E,r16
	sts	D011F,r16
#if STANDARD == 0
// for alternate version only ---------------------------
	sts	D012E,r16
	sts	D012F,r16
// ------------------------------------------------------
#endif
	// load volume table for envelopes to SRAM 0x230, 32 bytes
	ldi	xh,0x02
	ldi	xl,0x30
	ldi 	zl, low(2*VolTab2)
	ldi 	zh, high(2*VolTab2)
	ldi	r18,0x20
LOOP0:
	lpm	r16,Z+
	st	X+,r16
	dec	r18
	brne	LOOP0

	// load volume table for amplitude to SRAM 0x220, 16 bytes
	ldi	xl,0x20
	ldi 	zl, low(2*VolTab1)
	ldi 	zh, high(2*VolTab1)
	ldi	r18,0x10
LOOP1:
	lpm	r16,Z+
	st	X+,r16
	dec	r18
	brne	LOOP1


	// load register masks to SRAM 0x100, 16 bytes
	clr	xl
	ldi	xh,0x01
	ldi 	zl, low(2*RegsMask)
	ldi 	zh, high(2*RegsMask)
	ldi	r18,0x10
LOOP2:
	lpm	r16,Z+
	st	X+,r16
	dec	r18
	brne	LOOP2


	ldi	ZH,0x01
	ldi	YH,0x02

	// get byte 0 from EEPROM, check for value=1 or skip USART initialization
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
	out	OCR1AH,C00
	out	OCR1AL,C00
	out	OCR1BH,C00
	out	OCR1BL,C00
	sbi	DDRB,b1
	sbi	DDRB,b2
	ldi	r16,0xA2
	out	TCCR1A,r16
	ldi	r16,0x19
	out	TCCR1B,r16
	out	EEARL,YH
	sbi	EECR,b0
	in	r18,EEDR
	out	ICR1H,C00
	out	ICR1L,r18 // set PWM speed from 2 byte of EEPROM

#if CHANNELS == 3
	// init Timer2
	sbi	DDRB,b3
	ldi	r16,0x69
	out	TCCR2,r16
#endif

	// check for external interrupts enabled in 1 byte of EEPROM
	out	EEARL,ZH
	sbi	EECR,b0
	in	r16,EEDR
	cp	r16,C00
	breq	NO_EXT_INT
	// external interrupts initialization
	ldi	r16,0x0F
	out	MCUCR,r16
	ldi	r16,0xC0
	out	GIFR,r16
	out	GICR,r16
NO_EXT_INT:
	// init variables
	clr	BusEx
	ldi	ADDR,0x10
	ldi	r16,0x28
	mov	TNLevel,r16
	clr	OutA
	clr	OutC
	mov	TabP,CC0
	clr	TabE
	clr	BusData
	clr	NoiseAddon
	clr	CntN
	clr	CntAL
	clr	CntAH
	movw	CntBL,CntAL
	movw	CntCL,CntAL
	movw	CntEL,CntAL
	clr	EVal
	sei

_MAIN_LOOP:
	// MAIN LOOP =============================================
	in	YL,TIFR		; check timer1 overflow flag TOV1
	sbrs	YL,b2
	rjmp	_MAIN_LOOP	; jump if not set

	out	TIFR,C04	; clear timer overflow flag

	/////////////////////////////////////////////////////////////////////////////////////
	/// ENVELOPE GENERATOR
	/////////////////////////////////////////////////////////////////////////////////////
	sbrs	TNLevel,b7	; check if envelope chape register is changed
	rjmp	NO_ENVELOPE_CHANGED

	// initialize envelope generator
	lds	TabE,D011D	; load envelope shape register value to TabE
	mov	TabP,ZH		; 1 -> TabP
	and	TNLevel,C3F	; clear envelope changed flag
	lds	CntEL,D011B	; initialize envelope counter
	lds	CntEH,D011C
	clr	EVal		; set zero envelope volume as default
	sbrs	TabE,b2		; check ATTACK bit
	ldi	EVal,0x2C	; set maximal volume if bit is set
	rjmp	ENVELOPE_GENERATOR_END

NO_ENVELOPE_CHANGED:
	sbrc	TabP,b7		; check if envelope generator is disabled
	rjmp	ENVELOPE_GENERATOR_END
	sbiw	CntEL,0x01	; decrease envelope period counter
	brcs	INIT_EG		;
	brne	ENVELOPE_GENERATOR_END	; no changes if counter > 0
INIT_EG:
	lds	CntEL,D011B	; initialize envelope counter
	lds	CntEH,D011C
	sbrs	TabP,b5		; check if envelope period counter is overflow
	rjmp	ENV_NEXT_VAL	; jump to get next value if not overflow

	// initialize envelope period
	sbrc	TabE,b3		; check CONTINUE bit
	rjmp	NO_CONT

	clr	EVal		; set envelope value to 0
	or	TabP,CC0	; disable envelope generator
	rjmp	ENVELOPE_GENERATOR_END

NO_CONT:
	sbrc	TabE,b0		; check HOLD bit
	rjmp	ENV_HOLD

        // init new envelope period
	clr	TabP
	sbrc	TabE,b1		; check ALTERNATE bit
	eor	TabE,C04	; invert ATTACK bit if ALTERNATE is set
	rjmp	ENV_NEXT_VAL
ENV_HOLD:
	or	TabP,CC0	; disable envelope generator
	clr	YL
	mov	TMP,TabE
	add	TMP,YH
	sbrc	TMP,b2		; check if envelope bits 1 and 2 is differ
	mov	YL,C1F		; set max value if differ
	rjmp	ENV_TO_VOL
ENV_NEXT_VAL:
	mov	YL,TabP
	sbrs	TabE,b2		; invert envelope if ATTACK bit is not set
	eor	YL,C1F
	inc	TabP		; increase envelope period counter value
ENV_TO_VOL:
	ldd	EVal,Y+0x30	; read volume value from SRAM
ENVELOPE_GENERATOR_END:
	/////////////////////////////////////////////////////////////////////////////////////


	/////////////////////////////////////////////////////////////////////////////////////
	/// NOISE GENERATOR
	/////////////////////////////////////////////////////////////////////////////////////
	dec	CntN		; decrease noise period counter
	brpl	NOISE_GENERATOR_END

	lds	CntN,D0116	; set noise period value
	dec	CntN		; decrease noise period counter

	// pseudo random number generator
	mov	TMP,RNGL
	swap	TMP     	; swap nibbles
	lsr	TMP		; shift left
	eor	TMP,RNGL	; xor
	lsr	NoiseAddon	; move additional bit to carry flag
	ror	RNGH		; rotate right through carry
	ror	RNGL		; rotate right through carry
	brcc	NO_NLVL_CHANGED	
	eor	TNLevel,C38     ; invert noise level if Carry flag is set
NO_NLVL_CHANGED	:
	sbrs	TMP,b3		; if bits 0 and 3 differ skip next line
	or	NoiseAddon,ZH	; set random generator addirional bit
NOISE_GENERATOR_END:
	/////////////////////////////////////////////////////////////////////////////////////


	/////////////////////////////////////////////////////////////////////////////////////
	/// TONE GENERATOR
	/////////////////////////////////////////////////////////////////////////////////////

	// Channel A
	subi	CntAL,0x01	; CntA - 1
	sbci	CntAH,0x00
	brpl	CNTA_POSITIVE	; if CntA >= 0 goto CNTA_POSITIVE
	lds	CntAL,D0110	; update channel A tone period counter
	lds	CntAH,D0111
	subi	CntAL,0x01	; CntA - 1
	sbci	CntAH,0x00
	eor	TNLevel,ZH	; TNLevel xor 1 (change logical level of channel A)
CNTA_POSITIVE:

	// Channel B
	subi	CntBL,0x01	; CntB - 1
	sbci	CntBH,0x00
	brpl	CNTB_POSITIVE	; if CntB >= 0 goto CNTB_POSITIVE
	lds	CntBL,D0112	; update channel B tone period counter
	lds	CntBH,D0113
	subi	CntBL,0x01	; CntB - 1
	sbci	CntBH,0x00
	eor	TNLevel,YH	; TNLevel xor 2 (change logical level of channel B)
CNTB_POSITIVE:

	// Channel C
	sbiw	CntCL,0x01	; CntC - 1
	brpl	CNTC_POSITIVE	; if CntC >= 0 goto CNTC_POSITIVE
	lds	CntCL,D0114	; update channel C tone period counter
	lds	CntCH,D0115
	sbiw	CntCL,0x01	; CntC - 1
	eor	TNLevel,C04	; TNLevel xor 4 (change logical level of channel C)
CNTC_POSITIVE:
	/////////////////////////////////////////////////////////////////////////////////////



	/////////////////////////////////////////////////////////////////////////////////////
	/// MIXER
	/////////////////////////////////////////////////////////////////////////////////////
	lds	TMP,D0117	; Load Mixer Register from SRAM
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
	lds	YL,D0118	; Load Channel A Amplitude register
	mov	OutA,EVal	; set envelope volume as default value
	sbrs	YL,b4		; if bit 4 is not set in amplitude register then translate it to volume
	ldd	OutA,Y+0x20
	sbrs	TMP,b0		; if channel is disabled in mixer - set volume to zero
	clr	OutA

	// Channel C
	lds	YL,D011A	; Load Channel C Amplitude register
	mov	OutC,EVal	; set envelope volume as default value
	sbrs	YL,b4		; if bit 4 is not set in amplitude register then translate it to volume
	ldd	OutC,Y+0x20
	sbrs	TMP,b2		; if channel is disabled in mixer - set volume to zero
	clr	OutC

	// Channel B
#if CHANNELS == 2
// two channel version ----------------------------------------
	sbrs	TMP,b1		; jump if channel B is disabled in mixer
	rjmp	CHANNEL_B_DISABLED
	lds	YL,D0119	; Load Channel B Amplitude register
	mov	TMP,EVal	; set envelope volume as default value
	sbrs	YL,b4
	ldd	TMP,Y+0x20
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
	lds	YL,D0119	; Load Channel B Amplitude register
	mov	TMP,EVal	; set envelope volume as default value
	sbrs	YL,b4
	ldd	TMP,Y+0x20
	lsl	TMP		; TMP = TMP*2 (as OCR2 has 255 levels)
	mov	YL,TMP
CHANNEL_B_DISABLED:
	out	OCR2,YL
// --------------------------------------------------------------
#endif

	out	OCR1AL,OutA	; update PWM counters
	out	OCR1BL,OutC
	rjmp	_MAIN_LOOP

;---------------------------------------
	.dseg
	.org	0x0110
;
D0110:			; AY REGISTER 0
	.byte	1
D0111:			; AY REGISTER 1
	.byte	1
D0112:			; AY REGISTER 2
	.byte	1
D0113:			; AY REGISTER 3
	.byte	1
D0114:			; AY REGISTER 4
	.byte	1
D0115:			; AY REGISTER 5
	.byte	1
D0116:			; AY REGISTER 6
	.byte	1
D0117:			; AY REGISTER 7
	.byte	1
D0118:			; AY REGISTER 8
	.byte	1
D0119:			; AY REGISTER 9
	.byte	1
D011A:			; AY REGISTER 10
	.byte	1
D011B:			; AY REGISTER 11
	.byte	1
D011C:			; AY REGISTER 12
	.byte	1
D011D:			; AY REGISTER 13
	.byte	1
D011E:			; AY REGISTER 14
	.byte	1
D011F:			; AY REGISTER 15
	.byte	15
D012E:			;
	.byte	1
D012F:			;
