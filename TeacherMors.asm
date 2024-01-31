;
; TeacherMorsVFinal.asm
;
; Created: 1.01.2024 22:47:14
; Author : Digdem Yildiz & Zeynep Gencer
;

; define constants
.equ    BAUD_RATE   =9600
.equ    CPU_CLOCK   =4000000
.equ    BAUD_DIV    =(CPU_CLOCK/(16*BAUD_RATE))-1
.equ    FIFOSIZE    = 50     ; establish a buffer of size 50

.equ    TEACHER = 0
.equ    EXAM = 1

;;;;;;;;;;;;;;;
;TEACHING STATE
;.equ    WAITING = 0
;.equ    HASCHAR = 1
.equ    MORSCHAR = 2
.equ    SINGING = 3
.equ    SILENT  = 4
;;;;;;;;;;;;;;
;EXAM STATE
.equ    FALSE  = 5
.equ	TRUE   = 6

.equ    A03 = 220
.equ    maskT_dur = $03

;**************************************************************************
;**************************************************************************
.equ    Xtal = 1048576                  ; system clock frequency
.equ    prescaler_t1 = 64               ; timer1 prescaler
.equ    N_samples = 5                   ; Number of samples in lookup table
.equ    Fck = Xtal/prescaler_t1         ; timer1 working frequency

; define register aliases
.def    SENTCHAR    =r0     ; sent character
.def    RECCHAR     =r2     ; received character
.def    TEMP1       =r17    ; another temporary register
.def    FIFOHEAD        =r22    ; index of beginning of FIFO
.def    FIFOTAIL        =r23    ; index of end of FIFO


.def   tmp  = r16                       ; temp register
.def   T_cnt = r18                      ; Tone duration counter
.def   T_dur = r19                      ; Tone duration
.def   state = R24
.def   playState = R25

.def   char = R3
.def   morsCode = R21

.def	chcheck			=r20	; temporary register
;.def	flag			=r26	; flag for no match GIVES AN ERROR


; data segment
.dseg
fifo:   .byte   FIFOSIZE ; this is the FIFO area of size FIFOSIZE

; code segment
.cseg
.org    $000
   jmp reset						 ; $000 HW reset or watchdog
.org 0x012
   rjmp tim1_ovf					 ; Timer1 overflow Handler
.org	$016
   rjmp tim0_ovf					 ; Timer/Counter0 Overflow Handler
.org    $01A
   jmp REC_INT						 ; $01A UART Rx complete

;;;; Timer 0 Overflow Handler
tim0_ovf:
  push   tmp                   ; Store temporary register
  in     tmp,SREG
  push   tmp                   ; Store status register
  push   ZL
  push   ZH                    ; Store Z-Pointer
  push   R0                    ; Store R0 Register

  cpi state, EXAM
  breq T_exit
  cpi playState, MORSCHAR
  breq readT_dur
  ;cpi playState, SINGING
  ;brne if_notsilent
  ;rjmp continue
 ;if_notsilent:
  ;cpi playState, SILENT
  ;brne T_exit

; continue:
 
T0:
  cp     T_cnt, T_dur
  breq   switchMode
  ldi    tmp, 1
  add    T_cnt, tmp
  cpi    playState, SILENT
  breq   silentSinging
  sbi    DDRD,PD4              ; Set pin PD5 as output
  ldi    playState, SINGING
  rjmp   T_exit
switchMode:
  cpi    playState, SINGING
  brne   toSinging
  ldi    playState, SILENT
  ;ldi    T_dur, $05
  clr    T_cnt
  rjmp   silentSinging
 toSinging:
	ldi playState, SINGING
	cpi morsCode, $00
    breq T1

readT_dur:
  ldi    playState, SINGING
  mov    T_dur, morsCode
  andi   T_dur, maskT_dur
  lsl    T_dur
  lsl    T_dur
  clr    T_cnt
  lsr    morsCode
  lsr    morsCode
  rjmp   T0

silentSinging:
  cbi    DDRD,PD4              ; Clear pin PD5 to disable output
  rjmp T_exit

T1:
  cbi    DDRD,PD4              ; Clear pin PD5 to disable output
  clr    T_cnt
  clr    T_dur
  
T_exit:
  pop     R0                   ; Restore R0 Register
  pop     ZH
  pop     ZL                   ; Restore Z-Pointer
  pop     tmp
  out     SREG,tmp             ; Restore SREG
  pop     tmp                  ; Restore temporary register;
  reti



;;; Timer 1 Overflow Handler
tim1_ovf:
   push   tmp                  ; Store temporary register
   in     tmp,SREG
   push   tmp                  ; Store status register
   push   ZL
   push   ZH                    ; Store Z-Pointer
   push   R0                    ; Store R0 Register

   cpi state, EXAM
   brne T1_exit

   generate:
	   ldi    tmp, Fck/A03        ; load the note A03
	   mov    R0, tmp
       ldi    tmp, 255
       sub    tmp, R0
       out    TCNT1L, tmp          ; Load initial count value to T1
       lsr    R0                   ; Divide R0 by 2
       add    tmp, R0              ; OCR threshold for 50% duty cycle     
       out    OCR1BL,tmp          ; send the OCR value to PWM
          
   T1_exit:
       pop     R0                   ; Restore R0 Register
       pop     ZH
       pop     ZL                   ; Restore Z-Pointer
       pop     tmp
       out     SREG,tmp             ; Restore SREG
       pop     tmp                  ; Restore temporary register;
       reti

; Interrupt Handler
REC_INT:
       push    tmp         ;  registers
       in      tmp, SREG
       push    tmp
       push    RECCHAR
      
       in      RECCHAR, UDR        ; read UART receive data
       mov     tmp, RECCHAR
       out     PORTB, tmp      ; display data on LEDs

       put_char:
           rcall   FIFOput         ; place data in the FIFO

       rec_int_exit:
           pop RECCHAR         ; restore registers
           pop tmp
           out     SREG, tmp               
           pop     tmp
           reti                    ; return from interrupt


; begin main code
reset:
   ldi     tmp, low(RAMEND)        ; initialize stack pointer
   out     SPL, tmp
   ldi     tmp, high(RAMEND)
   out     SPH, tmp


   rcall   clrFIFO             ; initialize FIFO

   ldi state, TEACHER          ; initialize state
   ;ldi playState, WAITING
   ldi morsCode, 0
   ldi T_dur, 0
   ldi T_cnt, 0

   ldi tmp, $FF            ; port D all outputs
   out DDRC, tmp

   ldi tmp, $FF            ; port B all outputs
   out DDRB, tmp
   ldi tmp, $00
   out PORTB, tmp          ; initially all LEDs off

   ldi tmp, low(BAUD_DIV)      ; set UART baud rate
   out UBRRL, tmp
   ldi tmp, high(BAUD_DIV)     ; set UART baud rate
   out UBRRH, tmp
      
   sbi     UCSRB, TXEN         ; enable serial transmitter
   sbi     UCSRB, RXEN         ; enable serial receiver
   sbi     UCSRB, RXCIE            ; enable receiver interrupts
  
   ;enable timer0 ovf interrupt
   ldi   tmp, (1<<CS02)|(1<<CS00)  ; Prepare prescaler
   out   TCCR0, tmp                ; Timer clock = system clock / 1024
                                   ; timer0 ovf frequency 4 Hz

   ;enable timer1 / timer0 interrupt
   ldi   tmp,(1<<TOIE1)+(1<<TOIE0)
   out   TIMSK,tmp                     ; Enable Timer0/Timer1 ovf interrupt

   ; set timer1 PWM mode
   ldi   tmp,(1<<WGM10)+(1<<COM1B1)
   out   TCCR1A,tmp                                ; 8 bit Fast PWM non-inverting (Fck/256)
   ldi   tmp,(1<<WGM12)+(1<<CS10)+(1<<CS11)        ; 8 bit Fast PWM non-inverting (Fck/256)
   out   TCCR1B,tmp       
   sei                 ; enable interrupts
   rcall printTeacherMors
   rcall printNewLine
   rcall printNewLine

   rcall printWelcome
   rcall printNewLine
   rcall printStartMessage
   rcall printNewLine
   rcall printTeacherMode
   rcall printNewLine

mloop:
	out PORTC, state
	cpi state, TEACHER
	brne mloop2
	;cpi playState, WAITING
	;brne mloop
	rcall fillChar
	;cpi playState, HASCHAR
	;brne mloop
	rcall tolowerTeacher
	rcall parseCharA_Z
	cpi state, EXAM
	breq mloop2
    //ldi playState, MORSCHAR
	rcall printNewLine
	rcall printTeacherMode
	rcall printNewLine
	rjmp mloop
mloop2:
    out		PORTC, state
    rcall   FIFOget			; get the next character from the FIFO
	brcc	no_inp			;
	mov		chcheck, RECCHAR; we will receive the testing char
	mov		SENTCHAR, chcheck
	rcall   putchar			; transmit character 
	cpi		chcheck, $21 	;if char is ! change back to teaching state
	breq	toTeacherMode
	cpi		chcheck, $3F	;if char is ? change back to exam state
	breq	toExamMode
	rcall   tolowerExam
	rcall   readCharA_Z
	rcall   printExamMode
	rcall	printNewLine
no_inp:
	rjmp mloop2
toTeacherMode:
	ldi state, Teacher
	;ldi playState, WAITING
	rcall printNewLine
	rcall printTeacherMode
	rcall printNewLine
	jmp mloop
toExamMode:
		ldi state, EXAM
		rcall printNewLine
		rcall printExamMode
		rcall printNewLine
		jmp mloop2	
	
; subroutines
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;TEACHER STATE
tolowerTeacher:
	ldi    tmp, $61 
	cp	 char,tmp 	;61 is hex for A, 41 is for a; to avoid confusion I will turn every uppercase letter to lowercase
	brge   makeLowerTeacher
	ret
makeLowerTeacher:
	ldi  tmp, $20
	sub char, tmp
	ret

parseCharA_Z:
   ;ldi playState, WAITING
   mov tmp, char  ; Get dash character (?)
   cpi tmp,  $3F 
   breq toExam
	   
   ;ldi tmp, $21  ; Get exclamation point character (!)
   cpi tmp, $21
   breq toTeacher

   ldi playState, MORSCHAR

checkA:
   cpi tmp, $41  ; Get A
   ;cp char, tmp
   breq toA

   cpi tmp, $42  ; Get B
   ;cp char, tmp
   breq toB

   cpi tmp, $43  ; Get C
   ;cp char, tmp
   breq toC

   cpi tmp, $44  ; Get D
   ;cp char, tmp
   breq toD

   cpi tmp, $45  ; Get E
   ;cp char, tmp
   breq toE

   cpi tmp, $46  ; Get F
   ;cp char, tmp
   breq toF

   cpi tmp, $47  ; Get G
   ;cp char, tmp
   breq toG

   cpi tmp, $48  ; Get H
   ;cp char, tmp
   breq toH

   cpi tmp, $49  ; Get I
   ;cp char, tmp
   breq toI

   cpi tmp, $4A  ; Get J
   ;cp char, tmp
   breq toJ

   cpi tmp, $4B  ; Get K
   ;cp char, tmp
   breq toK

   cpi tmp, $4C  ; Get L
   ;cp char, tmp
   breq toL

   cpi tmp, $4D  ; Get M
  ; cp char, tmp
   breq toM

   cpi tmp, $4E  ; Get N
   ;cp char, tmp
   breq toN

   cpi tmp, $4F  ; Get O
   ;cp char, tmp
   breq toO

   cpi tmp, $50  ; Get P
   ;cp char, tmp
   breq toP

   cpi tmp, $51  ; Get Q
   ;cp char, tmp
   breq toQ
   rcall parseCharR_Z
   ret

   toExam:
		ldi state, EXAM
		rcall printNewLine
		rcall printExamMode
		rcall printNewLine
		ret

   toTeacher:
		ldi state, TEACHER
		ret
   toA:
		ldi morsCode, 0b00001101
		ret
   toB:
		ldi morsCode, 0b01010111
		ret
   toC:
		ldi morsCode, 0b01110111
		ret
   toD:
	    ldi morsCode, 0b00010111
		ret
   toE:
		ldi morsCode, 0b00000001
		ret
   toF:
		ldi morsCode, 0b01110101
		ret
	toG:
		ldi morsCode, 0b00011111
		ret
	toH:
		ldi morsCode, 0b00010101
		ret
	toI:
		ldi morsCode, 0b00000101
		ret
	toJ:
		ldi morsCode, 0b11111101
		ret
	toK:
		ldi morsCode, 0b00110111
		ret
	toL:
		ldi morsCode, 0b01011101
		ret
	toM:
		ldi morsCode, 0b00001111
		ret
	toN:
		ldi morsCode, 0b00000111
		ret
	toO:
		ldi morsCode, 0b00111111
		ret
	toP:
		ldi morsCode, 0b01111101
		ret
	toQ:
		ldi morsCode, 0b11011111
		ret
	
	
parseCharR_Z:
   cpi tmp, $52  ; Get R
   ;cp char, tmp
   breq toR

    cpi tmp, $53  ; Get S
   ;cp char, tmp
   breq toS

   cpi tmp, $54  ; Get T
   ;cp char, tmp
   breq toT

    cpi tmp, $55  ; Get U
   ;cp char, tmp
   breq toU

    cpi tmp, $56  ; Get V
   ;cp char, tmp
   breq toV

  cpi tmp, $57  ; Get W
   ;cp char, tmp
   breq toW

    cpi tmp, $58  ; Get X
   ;cp char, tmp
   breq toX

    cpi tmp, $59  ; Get Y
   ;cp char, tmp
   breq toY

    cpi tmp, $5A  ; Get Z
   ;cp char, tmp
   breq toZ
   ;ldi  playState, WAITING
   rcall printNewLine
   rcall printInvalidChar
   rcall printNewLine
   rcall printTeacherMode
   rcall printNewLine
   ret

   toR:
		ldi morsCode, 0b00011101
		ret
	toS:
		ldi morsCode, 0b00010101
		ret

   toT:
		ldi morsCode, 0b00000011
		ret
	toU:
		ldi morsCode, 0b00110101
		ret
	toV:
		ldi morsCode, 0b11010101
		ret
    toW:
		ldi morsCode, 0b00111101
		ret
	toX:
		ldi morsCode, 0b11010111
		ret
	toY:
		ldi morsCode, 0b11110111
		ret
	toZ:
	    ldi morsCode, 0b01011111
		ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;EXAM STATE

tolowerExam:
	cpi	 chcheck, $61	;61 is hex for A, 41 is for a; to avoid confusion I will turn every uppercase letter to lowercase
	brge   makeLowerExam
	ret
	;rcall readChar
makeLowerExam:
	subi chcheck, $20
	ret

readCharA_Z:
	cpi		chcheck, $41
	breq	morsA
	cpi		chcheck, $42
	breq	morsB
	cpi		chcheck, $43
	breq	morsC
	cpi		chcheck, $44
	breq	morsD
	cpi		chcheck, $45
	breq	morsE
	cpi		chcheck, $46
	breq	morsF
	rcall   readCharG_L
	ret

morsA: ; .- ascii 46 and 45 => hex 2E and 2D
	rcall	readDot
	cpi     playState, FALSE
	breq    morsA_Exit
	rcall	readDash
	rcall   checkCodeTrue
	ret
morsA_Exit:
	ret
morsB: ; -...
	rcall	readDash
	cpi     playState, FALSE
	breq    morsB_Exit
	rcall	readDot
	cpi     playState, FALSE
	breq    morsB_Exit
	rcall	readDot
	cpi     playState, FALSE
	breq    morsB_Exit
	rcall	readDot
	rcall   checkCodeTrue
morsB_Exit:
	ret
morsC: ; -.-.
	rcall	readDash
	cpi     playState, FALSE
	breq    morsC_Exit
	rcall	readDot
	cpi     playState, FALSE
	breq    morsC_Exit
	rcall	readDash
	cpi     playState, FALSE
	breq    morsC_Exit
	rcall	readDot
	rcall   checkCodeTrue
morsC_Exit:
	ret
morsD: ; -..
	rcall	readDash
	cpi     playState, FALSE
	breq    morsD_Exit
	rcall	readDot
	cpi     playState, FALSE
	breq    morsD_Exit
	rcall	readDot
	rcall   checkCodeTrue
morsD_Exit:
	ret
morsE: ; .
	rcall	readDot
	rcall   checkCodeTrue
	ret
morsF: ; ..-.
	rcall	readDot
	cpi     playState, FALSE
	breq    morsF_Exit
	rcall	readDot
	cpi     playState, FALSE
	breq    morsF_Exit
	rcall	readDash
	cpi     playState, FALSE
	breq    morsF_Exit
	rcall	readDot
	rcall   checkCodeTrue
morsF_Exit:
	ret

readCharG_L:
	cpi		chcheck, $47
	breq	morsG
	cpi		chcheck, $48
	breq	morsH
	cpi		chcheck, $49
	breq	morsI
	cpi		chcheck, $4A
	breq	morsJ
	cpi		chcheck, $4B
	breq	morsK
	cpi		chcheck, $4C
	breq	morsL
	rcall   readCharM_O
	ret

morsG: ; --. 
	rcall	readDash
	cpi     playState, FALSE
	breq    morsG_Exit
	rcall	readDash
	cpi     playState, FALSE
	breq    morsG_Exit
	rcall	readDot
	rcall   checkCodeTrue
morsG_Exit:
	ret
morsH: ; ....
	rcall	readDot
	cpi     playState, FALSE
	breq    morsH_Exit
	rcall	readDot
	cpi     playState, FALSE
	breq    morsH_Exit
	rcall	readDot
	cpi     playState, FALSE
	breq    morsH_Exit
	rcall	readDot
	rcall   checkCodeTrue
morsH_Exit:
	ret
morsI: ; ..
	rcall	readDot
	cpi     playState, FALSE
	breq    morsI_Exit
	rcall	readDot
	rcall   checkCodeTrue
morsI_Exit:
	ret
morsJ: ; .---
	rcall	readDot
	cpi     playState, FALSE
	breq    morsJ_Exit
	rcall	readDash
	cpi     playState, FALSE
	breq    morsJ_Exit
	rcall	readDash
	cpi     playState, FALSE
	breq    morsJ_Exit
	rcall	readDash
	rcall   checkCodeTrue
morsJ_Exit:
	ret
morsK: ; -.-
	rcall	readDash
	cpi     playState, FALSE
	breq    morsK_Exit
	rcall	readDot
	cpi     playState, FALSE
	breq    morsK_Exit
	rcall	readDash
	rcall   checkCodeTrue
morsK_Exit:
	ret
morsL: ; .-..
	rcall	readDot
	cpi     playState, FALSE
	breq    morsL_Exit
	rcall	readDash
	cpi     playState, FALSE
	breq    morsL_Exit
	rcall	readDot
	cpi     playState, FALSE
	breq    morsL_Exit
	rcall	readDot
	rcall   checkCodeTrue
morsL_Exit:
	ret



readCharM_O:
	cpi		chcheck, $4D
	breq	morsM
	cpi		chcheck, $4E
	breq	morsN
	cpi		chcheck, $4F
	breq	morsO
	rcall   readCharP_U
	ret
morsM: ; --
	rcall	readDash
	cpi     playState, FALSE
	breq    morsM_Exit
	rcall	readDash
	rcall   checkCodeTrue
morsM_Exit:
	ret
morsN: ; -.
	rcall	readDash
	cpi     playState, FALSE
	breq    morsN_Exit
	rcall	readDot
	rcall   checkCodeTrue
morsN_Exit:
	ret
morsO: ; ---
	rcall	readDash
	cpi     playState, FALSE
	breq    morsO_Exit
	rcall	readDash
	cpi     playState, FALSE
	breq    morsO_Exit
	rcall	readDash
	rcall   checkCodeTrue
morsO_Exit:
	ret


readCharP_U:
	cpi		chcheck, $50
	breq	morsP
	cpi		chcheck, $51
	breq	morsQ
	cpi		chcheck, $52
	breq	morsR
	cpi		chcheck, $53
	breq	morsS
	cpi		chcheck, $54
	breq	morsT
	cpi		chcheck, $55
	breq	morsU
	rcall   readCharV_Z
	ret
morsP: ; .--.
	rcall	readDot
	cpi     playState, FALSE
	breq    morsP_Exit
	rcall	readDash
	cpi     playState, FALSE
	breq    morsP_Exit
	rcall	readDash
	cpi     playState, FALSE
	breq    morsP_Exit
	rcall	readDot
	rcall   checkCodeTrue
morsP_Exit:
	ret
morsQ: ; --.-
	rcall	readDash
	cpi     playState, FALSE
	breq    morsQ_Exit
	rcall	readDash
	cpi     playState, FALSE
	breq    morsQ_Exit
	rcall	readDot
	cpi     playState, FALSE
	breq    morsQ_Exit
	rcall	readDash
	rcall   checkCodeTrue
morsQ_Exit:
	ret
morsR: ; .-. 
	rcall	readDot
	cpi     playState, FALSE
	breq    morsR_Exit
	rcall	readDash
	cpi     playState, FALSE
	breq    morsR_Exit
	rcall	readDot
	rcall   checkCodeTrue
morsR_Exit:
	ret
morsS: ; ...
	rcall	readDot
	cpi     playState, FALSE
	breq    morsS_Exit
	rcall	readDot
	cpi     playState, FALSE
	breq    morsS_Exit
	rcall	readDot
	rcall   checkCodeTrue
morsS_Exit:
	ret
morsT: ; -
	rcall	readDash
	rcall   checkCodeTrue
	ret
morsU: ; ..-
	rcall	readDot
	cpi     playState, FALSE
	breq    morsU_Exit
	rcall	readDot
	cpi     playState, FALSE
	breq    morsU_Exit
	rcall	readDash
	rcall   checkCodeTrue
morsU_Exit:
	ret

	

readCharV_Z:
    cpi		chcheck, $56
	breq	morsV
	cpi		chcheck, $57
	breq	morsW
	cpi		chcheck, $58
	breq	morsX
	cpi		chcheck, $59
	breq	morsY
	cpi		chcheck, $5A
	breq	morsZ
	rcall printNewLine
    rcall printInvalidChar
	rcall printNewLine
	ret
	morsV: ; ...-
	rcall	readDot
	cpi     playState, FALSE
	breq    morsV_Exit
	rcall	readDot
	cpi     playState, FALSE
	breq    morsV_Exit
	rcall	readDot
	cpi     playState, FALSE
	breq    morsV_Exit
	rcall	readDash
	rcall   checkCodeTrue
morsV_Exit:
	ret
morsW: ; .--
	rcall	readDot
	cpi     playState, FALSE
	breq    morsW_Exit
	rcall	readDash
	cpi     playState, FALSE
	breq    morsW_Exit
	rcall	readDash
	rcall   checkCodeTrue
morsW_Exit:
	ret
morsX: ; -..- 
	rcall	readDash
	cpi     playState, FALSE
	breq    morsX_Exit
	rcall	readDot
	cpi     playState, FALSE
	breq    morsX_Exit
	rcall	readDot
	cpi     playState, FALSE
	breq    morsX_Exit
	rcall	readDash
	rcall   checkCodeTrue
morsX_Exit:
	ret
morsY: ; -.--
	rcall	readDash
	cpi     playState, FALSE
	breq    morsY_Exit
	rcall	readDot
	cpi     playState, FALSE
	breq    morsY_Exit
	rcall	readDash
	cpi     playState, FALSE
	breq    morsY_Exit
	rcall	readDash
	rcall   checkCodeTrue
morsY_Exit:
	ret
morsZ: ; --..
	rcall	readDash
	cpi     playState, FALSE
	breq    morsZ_Exit
	rcall	readDash
	cpi     playState, FALSE
	breq    morsZ_Exit
	rcall	readDot
	cpi     playState, FALSE
	breq    morsZ_Exit
	rcall	readDot
	rcall   checkCodeTrue
morsZ_Exit:
	ret



;;;;;;;;;;;;;;;
readDot:
	rcall   FIFOget			; get the next character from the FIFO
	brcc	readDot
	mov		chcheck, RECCHAR
	mov		SENTCHAR, chcheck
	rcall   putChar
	cpi		chcheck, $2E	; ascii for dot
	brne	dotFalse
	ldi     playState, TRUE
	ret
dotFalse:
	ldi playState, FALSE
	cpi		chcheck, $2D	; ascii for dash 
	breq	prntF
	//if its not dot or dash its invalid 
	rcall printNewLine
	rcall printNotDotNotDash
prntF:
	rcall printNewLine
	rcall printFalse
	ret

;;;;;;;;;;;;;;;
readDash:
	rcall   FIFOget			; get the next character from the FIFO
	brcc	readDash
	mov		chcheck, RECCHAR 
	mov		SENTCHAR, chcheck
	rcall   putChar
	cpi		chcheck, $2D	; ascii for dash
	brne	dashFalse
	ldi playState, TRUE
	ret
dashFalse:
	ldi playState, FALSE
	cpi		chcheck, $2E	; ascii for dash 
	breq	prntF
	//if its not dot or dash its invalid 
	rcall printNewLine
	rcall printNotDotNotDash
	rcall printNewLine
	rcall printFalse
	ret

;;;;;;;;;;;;;;

checkCodeTrue:
	cpi     playState, TRUE
	breq   codeTrue
	ret 
	codeTrue:
	rcall printNewLine
	rcall printTrue
	ret

printNewline:
	ldi tmp, $0A
	mov  SENTCHAR, tmp
	rcall   putchar
	ldi tmp, $0D
	mov  SENTCHAR, tmp
	rcall   putchar
	ret
printFalse:
	ldi tmp, $46
	mov  SENTCHAR, tmp
	rcall   putchar
	rcall printNewline
	ret

printTrue:
	ldi tmp, $54
	mov  SENTCHAR, tmp
	rcall   putchar
	rcall printNewline
	ret

printWelcome:
	ldi ZL, low(WELCOME*2)
	ldi ZH, high(WELCOME*2)
	ldi tmp, $00
WelcomeLoop:
	cpi tmp, $07
	breq WelcomeExit
	lpm
	mov SENTCHAR, R0
	rcall putchar
	inc tmp
	adiw Z, $01
	rjmp WelcomeLoop
WelcomeExit:
	ret

printTeacherMors:
	ldi ZL, low(TEACHERMORS*2)
	ldi ZH, high(TEACHERMORS*2)
	ldi XL, $00
	ldi XH, $00
TeacherMorsLoop:
	cpi XL, low(1455)
	breq TeacherMorsExit
continueLoop:
	lpm
	mov SENTCHAR, R0
	rcall putchar
	adiw X, $01
	adiw Z, $01
	rjmp TeacherMorsLoop
TeacherMorsExit:
	cpi XH, high(1455)
	brne continueLoop
	ret

printInfoTeacherExam:
	ldi ZL, low(INFO_TEACHER_EXAM*2)
	ldi ZH, high(INFO_TEACHER_EXAM*2)
	ldi XL, $00
	ldi XH, $00
	ldi tmp, $00
InfoTeacherExamLoop:
	cpi tmp, 45
	breq InfoTeacherExamExit
	lpm
	mov SENTCHAR, R0
	rcall putchar
	inc tmp
	adiw Z, $01
	rjmp InfoTeacherExamLoop
InfoTeacherExamExit:
	ret

printStartMessage:
	ldi ZL, low(START_MESSAGE*2)
	ldi ZH, high(START_MESSAGE*2)
	ldi XL, $00
	ldi XH, $00
	ldi tmp, $00
StartMessageLoop:
	cpi tmp, 26
	breq StartMessageExit
	lpm
	mov SENTCHAR, R0
	rcall putchar
	inc tmp
	adiw Z, $01
	rjmp StartMessageLoop
StartMessageExit:
	ret

printExamMode:
	ldi ZL, low(EXAM_MODE*2)
	ldi ZH, high(EXAM_MODE*2)
	ldi XL, $00
	ldi XH, $00
	ldi tmp, $00
ExamModeLoop:
	cpi tmp, 27
	breq ExamModeExit
	lpm
	mov SENTCHAR, R0
	rcall putchar
	inc tmp
	adiw Z, $01
	rjmp ExamModeLoop
ExamModeExit:
	ret
	

printTeacherMode:
	ldi ZL, low(TEACHER_MODE*2)
	ldi ZH, high(TEACHER_MODE*2)
	ldi XL, $00
	ldi XH, $00
	ldi tmp, $00
TeacherModeLoop:
	cpi tmp, 30
	breq TeacherModeExit
	lpm
	mov SENTCHAR, R0
	rcall putchar
	inc tmp
	adiw Z, $01
	rjmp TeacherModeLoop
TeacherModeExit:
	ret

printInvalidChar:
	ldi ZL, low(INVALID_CHAR*2)
	ldi ZH, high(INVALID_CHAR*2)
	ldi XL, $00
	ldi XH, $00
	ldi tmp, $00
InvalidCharLoop:
	cpi tmp, 98
	breq InvalidCharExit
	lpm
	mov SENTCHAR, R0
	rcall putchar
	inc tmp
	adiw Z, $01
	rjmp InvalidCharLoop
InvalidCharExit:
	ret
	
printNotDotNotDash:
	ldi ZL, low(NotDotNotDash*2)
	ldi ZH, high(NotDotNotDash*2)
	ldi XL, $00
	ldi XH, $00
	ldi tmp, $00
NotDotNotDashLoop:
	cpi tmp, 118
	breq NotDotNotDashExit
	lpm
	mov SENTCHAR, R0
	rcall putchar
	inc tmp
	adiw Z, $01
	rjmp NotDotNotDashLoop
NotDotNotDashExit:
	ret
	

	
;;;;;;;;;;;;;



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;COMMON SUBROUTINES

fillChar:
  rcall FIFOget
  brcc fillChar
  mov SENTCHAR, RECCHAR
  rcall putchar
  mov char, RECCHAR
  ;ldi playState, HASCHAR
  ret


putchar:
       sbis    UCSRA, UDRE     ; loop until USR:UDRE is 1
       rjmp    putchar
       out     UDR, SENTCHAR       ; write SENTCHAR to transmitter buffer
       ret


;FIFO stuff
clrFIFO:
       clr     FIFOHEAD        ; head = tail -> 0
       clr     FIFOTAIL
       ret


FIFOput:
       push    YH              ; save registers
       push    YL
       push    tmp
      
       clr     tmp
       ldi     YL, low(fifo)   ; Y = fifo base
       ldi     YH, high(fifo)
       add     YL, FIFOTAIL    ; add offset to the end
       adc     YH, tmp     ;


       st      Y, RECCHAR      ; store data in FIFO
       inc     FIFOTAIL        ; update current depth
       cpi     FIFOTAIL, FIFOSIZE
       brlo    tnowrap
       clr     FIFOTAIL        ; start from begining when full
tnowrap:
       pop tmp         ; restore registers
       pop     YL
       pop     YH
       ret             ; return


; function FIFOget
; if data was available, carry is set
; if no data was available, carry is clear
FIFOget:
       cp      FIFOHEAD, FIFOTAIL  ; check if empty
       brne    N_EMP
       clc                         ; clear carry if empty
       ret
N_EMP:
       push    YH                  ; save registers
       push    YL
       clr     RECCHAR
       ldi     YL, low(fifo)       ; Y = FIFO base
       ldi     YH, high(fifo)
       add     YL, FIFOhead        ; add offset to get to
       adc     YH, RECCHAR         ;   the beginning (head) of FIFO
       ld      RECCHAR, Y          ; fetch first element in queue
       inc     FIFOHEAD                ; update FIFO head pointer
       cpi     FIFOHEAD, FIFOSIZE
       brlo    hnowrap
       clr     FIFOHEAD            ; wraparound to 0 when needed
hnowrap:
       pop     YL                  ; restore registers
       pop     YH
       sec                         ; set carry to indicate data
       ret

WELCOME:
	.db 'W', 'E'
	.db 'L', 'C'
	.db 'O', 'M'
	.db 'E', 0

NotDotNotDash:
.db $47, $69
.db $76, $65
.db $6e, $20
.db $69, $6e
.db $70, $75
.db $74, $20
.db $69, $73
.db $20, $6e
.db $65, $69
.db $74, $68
.db $65, $72
.db $20, $61
.db $20, $64
.db $6f, $74
.db $20, $6e
.db $6f, $72
.db $20, $61
.db $20, $64
.db $61, $73
.db $68, $2c
.db $20, $69
.db $6e, $20
.db $65, $78
.db $61, $6d
.db $20, $6d
.db $6f, $64
.db $65, $20
.db $70, $6c
.db $65, $61
.db $73, $65
.db $20, $75
.db $73, $65
.db $20, $64
.db $6f, $74
.db $73, $20
.db $61, $6e
.db $64, $20
.db $64, $61
.db $73, $68
.db $65, $73
.db $20, $61
.db $66, $74
.db $65, $72
.db $20, $74
.db $68, $65
.db $20, $63
.db $68, $61
.db $72, $61
.db $63, $74
.db $65, $72
.db $20, $79
.db $6f, $75
.db $20, $61
.db $72, $65
.db $20, $74
.db $65, $73
.db $74, $69
.db $6e, $67
.db  $21, $0



INVALID_CHAR:
.db $49, $20
.db $64, $6f
.db $6e, $74
.db $20, $6b
.db $6e, $6f
.db $77, $20
.db $77, $68
.db $61, $74
.db $20, $74
.db $6f, $20
.db $64, $6f
.db $20, $77
.db $69, $74
.db $68, $20
.db $74, $68
.db $69, $73
.db $20, $63
.db $68, $61
.db $72, $2c
.db $20, $74
.db $72, $79
.db $20, $61
.db $67, $61
.db $69, $6e
.db $20, $6f
.db $72, $20
.db $75, $73
.db $65, $20
.db $61, $6e
.db $6f, $74
.db $68, $65
.db $72, $20
.db $70, $72
.db $67, $72
.db $61, $6d
.db $20, $69
.db $66, $20
.db $79, $6f
.db $75, $20
.db $63, $61
.db $6e, $20
.db $66, $69
.db $6e, $64
.db $20, $6f
.db $6e, $65
.db $20, $62
.db $65, $74
.db $74, $65
.db $72, $21



TEACHER_MODE:
.db $45, $6e
.db $74, $65
.db $72, $20
.db $61, $20
.db $6c, $65
.db $74, $74
.db $65, $72
.db $20, $28
.db $54, $65
.db $61, $63
.db $68, $65
.db $72, $20
.db $6d, $6f
.db $64, $65
.db $29, $3a

EXAM_MODE:
.db $45, $6e
.db $74, $65
.db $72, $20
.db $61, $20
.db $6c, $65
.db $74, $74
.db $65, $72
.db $20, $28
.db $45, $78
.db $61, $6d
.db $20, $6d
.db $6f, $64
.db $65, $29
.db $3a, $00


START_MESSAGE:
.db $53, $74
.db $61, $72
.db $74, $69
.db $6e, $67
.db $20, $69
.db $6e, $20
.db $54, $65
.db $61, $63
.db $68, $65
.db $72, $20
.db $4d, $6f
.db $64, $65
.db $20, $3a
	

INFO_TEACHER_EXAM:
.db $45, $6e
.db $74, $65
.db $72, $20
.db $21, $20
.db $66, $6f
.db $72, $20
.db $54, $65
.db $61, $63
.db $68, $65
.db $72, $20
.db $6d, $6f
.db $64, $65
.db $20, $61
.db $6e, $64
.db $20, $3f
.db $20, $66
.db $6f, $72
.db $20, $45
.db $78, $61
.db $6d, $20
.db $6d, $6f
.db $64, $65
	


TEACHERMORS:
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $0a
.db $0D, $20
.db $5f, $5f
.db $5f, $5f
.db $5f, $5f
.db $5f, $5f
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $5f
.db $5f, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $0a
.db $0D, $2f
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $7c, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $2f, $20
.db $20, $7c
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $0a
.db $0D, $24
.db $24, $24
.db $24, $24
.db $24, $24
.db $24, $2f
.db $5f, $5f
.db $5f, $5f
.db $5f, $5f
.db $20, $20
.db $20, $20
.db $5f, $5f
.db $5f, $5f
.db $5f, $5f
.db $20, $20
.db $20, $20
.db $5f, $5f
.db $5f, $5f
.db $5f, $5f
.db $5f, $20
.db $24, $24
.db $20, $7c
.db $5f, $5f
.db $5f, $5f
.db $20, $20
.db $20, $20
.db $5f, $5f
.db $5f, $5f
.db $5f, $5f
.db $20, $20
.db $20, $20
.db $5f, $5f
.db $5f, $5f
.db $5f, $5f
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $0a, $0D
.db $20, $20
.db $20, $24
.db $24, $20
.db $7c, $20
.db $2f, $20
.db $20, $20
.db $20, $20
.db $20, $5c
.db $20, $20
.db $2f, $20
.db $20, $20
.db $20, $20
.db $20, $5c
.db $20, $20
.db $2f, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $7c, $24
.db $24, $20
.db $20, $20
.db $20, $20
.db $20, $5c
.db $20, $20
.db $2f, $20
.db $20, $20
.db $20, $20
.db $20, $5c
.db $20, $20
.db $2f, $20
.db $20, $20
.db $20, $20
.db $20, $5c
.db $20, $20
.db $20, $20
.db $20, $20
.db $0a, $0D
.db $20, $20
.db $20, $24
.db $24, $20
.db $7c, $2f
.db $24, $24
.db $24, $24
.db $24, $24
.db $20, $20
.db $7c, $20
.db $24, $24
.db $24, $24
.db $24, $24
.db $20, $20
.db $7c, $2f
.db $24, $24
.db $24, $24
.db $24, $24
.db $24, $2f
.db $20, $24
.db $24, $24
.db $24, $24
.db $24, $24
.db $20, $20
.db $7c, $2f
.db $24, $24
.db $24, $24
.db $24, $24
.db $20, $20
.db $7c, $2f
.db $24, $24
.db $24, $24
.db $24, $24
.db $20, $20
.db $7c, $20
.db $20, $20
.db $20, $20
.db $20, $0a
.db $0D, $20
.db $20, $20
.db $24, $24
.db $20, $7c
.db $24, $24
.db $20, $20
.db $20, $20
.db $24, $24
.db $20, $7c
.db $20, $2f
.db $20, $20
.db $20, $20
.db $24, $24
.db $20, $7c
.db $24, $24
.db $20, $7c
.db $20, $20
.db $20, $20
.db $20, $20
.db $24, $24
.db $20, $7c
.db $20, $20
.db $24, $24
.db $20, $7c
.db $24, $24
.db $20, $20
.db $20, $20
.db $24, $24
.db $20, $7c
.db $24, $24
.db $20, $7c
.db $20, $20
.db $24, $24
.db $2f, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $0a, $0D
.db $20, $20
.db $20, $24
.db $24, $20
.db $7c, $24
.db $24, $24
.db $24, $24
.db $24, $24
.db $24, $2f
.db $20, $2f
.db $24, $24
.db $24, $24
.db $24, $24
.db $24, $20
.db $7c, $24
.db $24, $20
.db $5c, $5f
.db $5f, $5f
.db $5f, $5f
.db $20, $24
.db $24, $20
.db $7c, $20
.db $20, $24
.db $24, $20
.db $7c, $24
.db $24, $24
.db $24, $24
.db $24, $24
.db $24, $2f
.db $20, $24
.db $24, $20
.db $7c, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $0a
.db $0D, $20
.db $20, $20
.db $24, $24
.db $20, $7c
.db $24, $24
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $7c
.db $24, $24
.db $20, $20
.db $20, $20
.db $24, $24
.db $20, $7c
.db $24, $24
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $7c
.db $24, $24
.db $20, $7c
.db $20, $20
.db $24, $24
.db $20, $7c
.db $24, $24
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $7c
.db $24, $24
.db $20, $7c
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $0a, $0D
.db $20, $20
.db $20, $24
.db $24, $2f
.db $20, $20
.db $24, $24
.db $24, $24
.db $24, $24
.db $24, $2f
.db $20, $20
.db $24, $24
.db $24, $24
.db $24, $24
.db $24, $2f
.db $20, $20
.db $24, $24
.db $24, $24
.db $24, $24
.db $24, $2f
.db $20, $24
.db $24, $2f
.db $20, $20
.db $20, $24
.db $24, $2f
.db $20, $20
.db $24, $24
.db $24, $24
.db $24, $24
.db $24, $2f
.db $20, $24
.db $24, $2f
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $0a, $0d
.db $0a, $0d
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $0a
.db $0D, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $5f
.db $5f, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $5f, $5f
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $0a
.db $0D, $20
.db $20, $20
.db $20, $20
.db $20, $2f
.db $20, $20
.db $5c, $20
.db $20, $20
.db $20, $20
.db $2f, $20
.db $20, $7c
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $0a, $0D
.db $20, $20
.db $20, $20
.db $20, $20
.db $24, $24
.db $20, $20
.db $5c, $20
.db $20, $20
.db $2f, $24
.db $24, $20
.db $7c, $20
.db $20, $5f
.db $5f, $5f
.db $5f, $5f
.db $5f, $20
.db $20, $20
.db $20, $5f
.db $5f, $5f
.db $5f, $5f
.db $5f, $20
.db $20, $20
.db $20, $5f
.db $5f, $5f
.db $5f, $5f
.db $5f, $5f
.db $20, $0a
.db $0D, $20
.db $20, $20
.db $20, $20
.db $20, $24
.db $24, $24
.db $20, $20
.db $5c, $20
.db $2f, $24
.db $24, $24
.db $20, $7c
.db $20, $2f
.db $20, $20
.db $20, $20
.db $20, $20
.db $5c, $20
.db $20, $2f
.db $20, $20
.db $20, $20
.db $20, $20
.db $5c, $20
.db $20, $2f
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $7c
.db $0a, $0D
.db $20, $20
.db $20, $20
.db $20, $20
.db $24, $24
.db $24, $24
.db $20, $20
.db $2f, $24
.db $24, $24
.db $24, $20
.db $7c, $2f
.db $24, $24
.db $24, $24
.db $24, $24
.db $20, $20
.db $7c, $2f
.db $24, $24
.db $24, $24
.db $24, $24
.db $20, $20
.db $7c, $2f
.db $24, $24
.db $24, $24
.db $24, $24
.db $24, $2f
.db $20, $0a
.db $0D, $20
.db $20, $20
.db $20, $20
.db $20, $24
.db $24, $20
.db $24, $24
.db $20, $24
.db $24, $2f
.db $24, $24
.db $20, $7c
.db $24, $24
.db $20, $7c
.db $20, $20
.db $24, $24
.db $20, $7c
.db $24, $24
.db $20, $7c
.db $20, $20
.db $24, $24
.db $2f, $20
.db $24, $24
.db $20, $20
.db $20, $20
.db $20, $20
.db $5c, $20
.db $0a, $0D
.db $20, $20
.db $20, $20
.db $20, $20
.db $24, $24
.db $20, $7c
.db $24, $24
.db $24, $2f
.db $20, $24
.db $24, $20
.db $7c, $24
.db $24, $20
.db $5c, $5f
.db $5f, $24
.db $24, $20
.db $7c, $24
.db $24, $20
.db $7c, $20
.db $20, $20
.db $20, $20
.db $20, $20
.db $24, $24
.db $24, $24
.db $24, $24
.db $20, $20
.db $7c, $0a
.db $0D, $20
.db $20, $20
.db $20, $20
.db $20, $24
.db $24, $20
.db $7c, $20
.db $24, $2f
.db $20, $20
.db $24, $24
.db $20, $7c
.db $24, $24
.db $20, $20
.db $20, $20
.db $24, $24
.db $2f, $20
.db $24, $24
.db $20, $7c
.db $20, $20
.db $20, $20
.db $20, $20
.db $2f, $20
.db $20, $20
.db $20, $20
.db $24, $24
.db $2f, $20
.db $0a, $0D
.db $20, $20
.db $20, $20
.db $20, $20
.db $24, $24
.db $2f, $20
.db $20, $20
.db $20, $20
.db $20, $24
.db $24, $2f
.db $20, $20
.db $24, $24
.db $24, $24
.db $24, $24
.db $2f, $20
.db $20, $24
.db $24, $2f
.db $20, $20
.db $20, $20
.db $20, $20
.db $20, $24
.db $24, $24
.db $24, $24
.db $24, $24
.db $2f, $20
.db $20, $20
.db $20, $20
.db $0A, $0D
.db $0A, $0D
.db $42, $59
.db $20, $44
.db $49, $47
.db $44, $45
.db $4d, $20
.db $59, $49
.db $4c, $44
.db $49, $5a
.db $20, $26
.db $20, $5a
.db $45, $59
.db $4e, $45
.db $50, $20
.db $47, $45
.db $4e, $43
.db $45, $52
.db $20, $32
.db $30, $32
.db $34, $00




