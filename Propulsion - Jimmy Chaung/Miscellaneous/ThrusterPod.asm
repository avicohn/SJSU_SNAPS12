	LIST P=18F14K22,F=INHX32,b=8,n=51,t=ON ;define processor and file format
;******************************************************************************
;   This file is the main relocatable assembly source code module for the
;   AE172 SNAPS ThrusterPod implemented on the PIC18F14K22.
;
;rf20120817 Flipped polarity of the Sync signal.
;	
;	This device is designed for the LegoSNAPS project and meets the Lego
;	Mindstorms NXT Plug and Play (PnP) discovery protocol.
;	Similar to the Mindstorms Ultrasonic Sensor, the 7-bit I2C address that this 
;	device responds to is 0000 001. (According to the I2C specification, this
;	address is reserved for "C" bus implementations.)
;	This device address can be modified by changing PIC EEPROM locations 
;	for I2C_Addr and I2C_Mask.
;
;	This code emulates an I2C EEPROM.
;	I2C messages used to access the Emulated EEPROM (EEEPROM) start with either 
;	of the two the following formats.
;
;Read/Write format:
;	byte 1: The I2C device address and !W flag. (a I2C Write message)
;	byte 2:	The first address in the EEEPROM to be read or written. This
;		value is loaded into the Emulated EEPROM's Address Register.
;	byte 3: The number of bytes in the EEPROM to be read or written + 3. The 
;		value of this byte is used to determine when the last byte is written. 
;		(As an I2C slave, we can't determine this from the I2C controller.)
;		After the last character (as indicated by the this byte) is written to
;		the Thruster Time Block, a Thruste Program will be generated from the
;		written data and Sync interrupts will be enabled.
;
;	After byte3, either the master continues to send the data to be written,
;	or a "Read access", Starts (or Restarts) a new I2C message using the 
;	Sequential Read format (see below).
;
;	For Write messages, the data to be written follows...
;	byte 4-n: Data to be written to the Emulated EEPROM. A data byte is 
;	attempted to be written to the location in the Emulated EEPROM designated 
;	by the Address Register. If the Address Register address a read only memory
;	location, the new data will not be written. The Address Register is then 
;	incremented. 
;
;Sequential Read format:
;	byte 1: The I2C device address and R flag. (a I2C Read message)
;	byte 2-n: Data read from the Emulated EEPROM (EEEPROM). The EEEPROM location
; 	designated by the Address Register is sent from the PIC to the I2C bus 
;	master. The Address Register is then incremented. Note that the incrementing
;	of the Address Register may cause it to wrap from the the last location of a 
;	designated block of memory, back to the first location in the block. 
;
;EEEPROM Blocks:
;	Locations in the Emulated EEPROM are assigned special meanings.
;	
;  Plug and Play Block:
;	Locations 0x00 to 0x3F contain Lego Mindstorms NXT PnP discovery data.
;	(see the "EEPROM definition section" for a definition of this data.)
;	Locations 0x19 to 0x3F contain device specific calibration data used as 
;	part of the PnP discovery process. 
;	This block of memory is Read Only. Attempts to write these locations will
;	be ignored.
;	After the Emulated EEPROM accesses location 0x3F, the incrementing of the 
;	Address Register will result it it being wrapped back to location 0x00.
;
;  State and Status Block:
;	Locations 0x40 to 0x47 contain this program's state and status variables. 
;	See xxxx for a definition of this block.
;	This block of memory is Read Only. Attempts to write these locations will
;	be ignored.
;	After the Emulated EEPROM accesses location 0x47, the incrementing of the 
;	Address Register will result it it being wrapped back to location 0x40.
;
;  Thruster Time Block:
;	Locations 0x80 to 0x87 contain the thruster firing times in units of 
;	Thruster Ticks. These locations are readable and writeable.
;	After the Emulated EEPROM accesses location 0x87, the incrementing of the 
;	Address Register will result it it being wrapped back to location 0x80.
;	See xxxx for a definition of this block.
;
;  Thruster Reset Block:
;	Locations 0x90 is a single byte block of EEPROM memory that contains the 
;	Reset Command Register. This location is readable and writeable.
;	After the Emulated EEPROM accesses location 0x90, the incrementing of the 
;	Address Register will result it it being wrapped back to location 0x90.
;	See xxxx for a definition of this block.
;
;Interrupts (in general):
;   The PIC18FXXXX architecture allows two interrupt configurations.
;   This code is written for priority interrupt levels and the IPEN bit
;   in the RCON register is set to enable priority levels.
;	Only the low interrupt priority level is used. The high priority interrupt
;	level is dedicated to the In-Circuit Debugger.
;
;Sync Interrupts:
;	The Sync interface signal (pin 1 of the RJ12NXT connector) is connected to 
;	the PIC's RA2/INT2 pin. A low to high transition of the Sync signal commands
;	the Thruster Pod to begin execution of the previously generated Thruster 
;	Program. The Sync Interrupt Handler will execute the first step of the 
;	Thruster Program by firing the indicated thrusters on this pod.
;	Since the Sync signal is common to all Thruster Pods, all os the thruster 
;	pods on the satellite will execute their Thruster Programs simultaniously.
;
;	Once it has started firing thrusters, the Sync Interrupt Handler will
;	load the Thruster Turn-off Timer, enable its interrupt, and
;	start it.
;
;Thruster Turn-off Timer Interrupts:
;	The Thruster Turn-off Timer (Timer0) is used to know when to turn-off
;	thrusters. Each Timer Program step contains a bit mask corresponding to 
;	thruster control GPIO pins on Port C and a timer value.
;	When a Thruster Turn-off Timer Interrupt occurs, the interrupt handler
;	will execute the next sequential step in the Thruster Program.
;	It will do this by...
;	* zeroing the bits indicated in that step's bit mask
;	* loading the step's timer value into timer and restarting it
;	By the time the last step in the Thruster Program has been executed,
;	all of the pod's thrusters should be off.
;	Once all of a pod's thrusters are turned off, Sync interrupts will be 
;	reenabled.
;
;******************************************************************************
;
;    Filename:  ThrusterPod.asm
;    Date:      2/18/12
;    File Version: 1.0
;
;    Author:    Bob Feretich
;
;******************************************************************************
;
; Files required:   18F14K22.INC
;                   ThrusterPod.INC
;                   ThrusterPod.LKR
;
;   Naming conventions:
;	ACS fields		All uppercase, use _ to separate words
;	Other RAM fields	First letter lowercase, use cap to separate words
;	Flash fields/labels	First letter capatalized, use cap to separate words
;       Constant values for above fields:
;	<ROOT>_<constName>
;		where <ROOT> is built from the name of the field and has same 
;			capitalization rules.
;		where <constName> has first letter uppercase and caps separate words
;	Constants/Literals that are not associated with data fields
;	        All uppercase, use _ to separate words.
;	My assembler variables	All lowercase and uses _ to separate words
;
;	If statement labeling convention:
;	<subroutine_name>If<if_seqnum>t = the beginning of the "then" block (t=true)
;	<subroutine_name>If<if_seqnum>f = the beginning of the "else" block(f=false)
;	<subroutine_name>If<if_seqnum>x = the merge point after "then" and 
;															"else" (x=exit)
;
;   Loop labeling convention:
;	<subroutine_name>L<loop_seqnum>s = start if loop
;	<subroutine_name>L<loop_seqnum>c = bottom, but inside loop, like CONTINUE
;	<subroutine_name>L<loop_seqnum>x = exit loop, similar to BREAK or LEAVE
;
;   Endian conventions:
;	The lowest order bit in a field is bit 0.
;	When data is stored in program memory of PIC18 devices, the first character
;	(lowest memory address) is in least significant byte.
;	In REGs, lowest order byte in lowest numbered register of consecutive 
;	registers.
;
;   Data Memory Map
;       adr. range  size  description
;       000h-05Fh    96 ACS - most used variables and all interrupt handler 
;								variables
;       060h-0FFh   160 Stack - Used to save variables during subroutine calls.
;       100h-13Fh    64 Other program variables.
;       140h-17Fh    64 I2C receive buffer
;       180h-1FFh   128 I2C transmit buffer
;
;   Stack use convention:
;	Stack grows from lower to higher addresses. So "top of stack" is the highest
;         address used.
;	Push operstions pre-increment the stack pointer, then move data the "top of
;	  stack" location.
;	Pop operations post-decriment the stack pointer.
;	PLUSW accesses therefore must index the stack with negative number offsets.
;	Extended mode is not enabled, so indexed access to the stacks are limited.
;	FSR0 is the software stack used by all applications and interrupt levels.
;	FSR1 and FSR2 are available for use by all code. They must be saved and 
;		restored if they are used by interrupt handlers.
;
;      Reset definition:
;	1) Min Reset (MR) - This is the reset that occurs if the Adapter detects an
;	   internal error. The Box Reset resets the minimum amount of Adapter 
;	   facilities needed to continue insrtuction execution. The amount of 
;	   retained state is maximized to aid in the diagnosis of the error. RAM 
;	   locations and groups of RAM locations that are reinitialized upon this 
;	   reset are marked with the letters IMR.
;	2) Software Reset (SR) - This is the reset that is performed by the 
;	   "Full Reset" Command Message. A Software Reset will reinitialize the 
;	   Adapter application to its power-on state so that it will be ready to 
;	   accept USB Command messages.All virtual drive and adapter state 
;	   information will be reinitialized. 
;	   RAM locations and groups of RAM locations that are reinitialized upon 
;	   this reset are marked with the letters ISR.
;	3) Power-On-Reset (PR) - This is the most comprehensive reset. PIC timers 
;	   and I/O Ports are reinialized. All RAM locations are initialized. This 
;	   reset is a superset of Min Reset and Software Reset. RAM locations that 
;	   do not have an architected initial value are set to a filler 
;	   character (0xAA). RAM locations and groups of RAM locations that have an 
;	   architected initial state, but are not reset by MR or SR are marked with 
;	   the letters IPR.
;
;	Hang codes:
;	0x01 = invalid usb state machine state
;	0x02 = false low priority interrupt
;	0x03 = false high priority interrupt
;
;******************************************************************************
;
	radix	dec
F	EQU	1		; used in instructions to indicate "put result in RAM file".
;******************************************************************************
;	Assembler control variables
;
	variable is_simulating	; B'0'= compile for PIC execution
				; B'1'= compile for simulation (no timimg loops)
is_simulating	set	B'1'
;
;******************************************************************************
	#include <P18F14K22.INC>		;processor specific variable definitions
	#include ThrusterPod.INC			;project macro file
	TITLE	"SNAPS ThrusterPod Program,  Rev 0.1"
	SUBTITLE "Configuration setup section"
;******************************************************************************
;Configuration bits
; The CONFIG directive defines configuration data within the .ASM file.
; The labels following the directive are defined in the P18F8722.INC file.
; The PIC18FXX22 Data Sheet explains the functions of the configuration bits.
; Change the following lines to suit your application.
	CONFIG  FOSC=IRC,PLLEN=OFF		;Internal osc withou 4x PLL
	CONFIG	PCLKEN=ON,FCMEN=OFF,IESO=OFF ;fail safe mon & switch disabled
	CONFIG	BOREN=OFF,PWRTEN=OFF	;brown-out reset and power-up-timer disabled
	CONFIG	WDTEN=OFF				;watch-dog-timer disabled
    CONFIG  MCLRE=ON                ;MCLR oin enabled
	CONFIG	STVREN=ON				;stack-ovflw-reset enabled
	CONFIG	LVP=OFF,DEBUG=ON 		;low-voltage ICSP, bkgnd-debug enabled
;	CONFIG	MODE=MC					;use microcontroller memory mode
	CONFIG	CP0=OFF,CP1=OFF         ;code protection off
	CONFIG	CPB=OFF,CPD=OFF		;boot-block and eeprom protection disabled
	CONFIG	WRT0=OFF,WRT1=OFF	;write protection disabled
	CONFIG	WRTC=OFF,WRTB=OFF 	;config, boot-block, &
	CONFIG	WRTD=OFF 			;  data-eeprom protection disabled
	CONFIG	EBTR0=OFF,EBTR1=OFF	;table read protection disabled
	CONFIG	EBTRB=OFF			;boot-block table read protection disabled
;******************************************************************************
LAT_OFFSET	equ  72		;bit offset from port's data reg to LAT reg
TRIS_OFFSET	equ  144	;bit offset from port's data reg to TRIS reg

;	Port A has a bit connected to the RJ12 Input Jack
#define	TSYNC	PIN_A2 	;A 1->0 transition signals the kick-off of the 
;                       ; previously received thruster command set.

;	Port B has the I2C interface
#define	I2C_SCL1	PIN_B6
#define I2C_SDA1	PIN_B4

;   Port C has its low order four bits connected to the solenoid valve drivers.
#define	THRST1	PIN_C6
#define	THRST2	PIN_C4
#define	THRST3	PIN_C5
#define	THRST4	PIN_C7

 PAGE
		SUBTITLE	"ACCESS DATA MEMORY definition section"
		UDATA_ACS

;******************************************************************************
;******************************************************************************
;*******  ACS locations are all CAPS Literals are also all Caps
;******************************************************************************
;******************************************************************************
;	Low Priority Interrupt Level ACS memory		18 bytes
;******************************************************************************
;	Working registers to be used by interrupt handler
FILE00:
LPREG0		RES 1
LPREG1		RES 1
LPREG2		RES 1
LPREG3		RES 1
LPREG4		RES 1
LPREG5		RES 1
LPREG6		RES 1
LPREG7		RES 1

;*****************************************************************************
;	Background Tasks ACS memory		xx bytes
;*****************************************************************************
;	Working registers		8 bytes
REG0		RES 1
REG1		RES 1
REG2		RES 1
REG3		RES 1
REG4		RES 1
REG5		RES 1
REG6		RES 1
REG7		RES 1

SYS_RESR	RES 1	;last reset reason code (RCON Register) (IPR)


;******************************************************************************
;	ISR Thruster Control Block
;	Thruster Pod Configuration
;       3-DoF configurations only have 2 thrusters per pod.
;       6-Dof configurations have all 4 thrusters installed in the pod.
;
;	The below File memory locations are mapped to locations 0x40 to 0x47 in
;	the Emulated EEPROM. 
TCB_STATUS	RES 1				;Pod status
TCBS_DGEN		EQU	 7			;1=Dynamic Program regeneration used
TCBS_PUPD		EQU	 6			;1=Thruster Program needs regeneration
TCBS_ISE		EQU	 5			;1=I2C state error
TCBS_RBOF		EQU  4			;1=Receive buffer overflow error
TCBS_SYN		EQU	 3			;1=Sync still active after last thruster off err
TCBS_WDT		EQU	 2			;1=Sync watch-dog timer event error
TCBS_RAN		EQU	 1			;1=Received "thruster time out of range" error
TCBS_SYS		EQU	 0			;1=A system internal error occured

TCB_ECNT	RES	1				;Status event count

TCB_CNFG	RES 1
TCBC_TTH3       EQU  7			;1=Thruster on RC7 exists
TCBC_TTH1       EQU  6			;1=Thruster on RC6 exists
TCBC_TTH4       EQU  5			;1=Thruster on RC5 exists
TCBC_TTH2       EQU  4			;1=Thruster on RC4 exists
TCBC_MTH3       EQU 0x80        ;Thruster on RC7
TCBC_MTH1       EQU 0x40        ;Thruster on RC6
TCBC_MTH4       EQU 0x20        ;Thruster on RC5
TCBC_MTH2       EQU 0x10        ;Thruster on RC4

TCB_STATE	RES	1
TCBST_UNIT		EQU 0x00		;Unitialized. Thruster Times not set.
TCBST_RDY		EQU 0x01		;Ready for Sync signal
TCBST_FIRE		EQU 0x02		;Firing Thrusters


TCB_CNT		RES 1   			;number of commands in the Thruster Queue

TCB_SSPST	RES	1				;saved I2C SSPSTAT register (in case of a
								; I2C state error

TCB_OTHER	RES	2				;unused

;	End of EEPROM block mapping ***********************************************
;
;	The below File memory locations are mapped to locations 0x80 to 0x87 in
;	the Emulated EEPROM. 
TCB_TH1TKS		RES	2	;Thruster 1 is programmed to fire for this many Ticks
TCB_TH2TKS		RES	2	;Thruster 2 is programmed to fire for this many Ticks
TCB_TH3TKS		RES	2	;Thruster 3 is programmed to fire for this many Ticks
TCB_TH4TKS		RES	2	;Thruster 4 is programmed to fire for this many Ticks
;	End of EEPROM block mapping ***********************************************
;
;	The below File memory locations are mapped to locations 0x90 to 0x90 in
;	the Emulated EEPROM. 
TCB_RESET		RES	1	;Reset command word
TCBR_DONE			EQU	0x00	;Reset completed
TCBR_FULL			EQU	0x01	;Full reset
TCBR_CNCL			EQU	0x02	;Cancel current thruster program
;	End of EEPROM block mapping ***********************************************

TCB_CPTR        RES 2   ;pointer to the next Thruster Command.
; *****************************************************************************
TCB_FMAP        RES 1   ;Bit map of I/O Port C thrusters to fire. The SYNC ISR 
						;  will set these bits in the thruster data port.
TCB_QUEUE:              ;start of Thruster Queue
;           Thruster Turn-off Queue entry 1
TCBQ1_TMR       RES 2   ;Timer ticks for this shut-off command.
                        ; The SYNC ISR will start the timer with this value,
TCBQ1_MAP       RES 1   ;Bit map for I/O Port C of thrusters effected. The Timer
                        ; ISR will clear these bits in the thruster data port.
;           Thruster Turn-off Queue entry 2
TCBQ2_TMR       RES 2   ;Timer ticks for this shut-off command
                        ; The Timer ISR will start the timer with this value,
TCBQ2_MAP       RES 1   ;Bit map for I/O Port C of thrusters effected. The Timer
                        ; ISR will clear these bits in the thruster data port.
;           Thruster Turn-off Queue entry 3
TCBQ3_TMR       RES 2   ;Timer ticks for this shut-off command
                        ; The Timer ISR will start the timer with this value,
TCBQ3_MAP       RES 1   ;Bit map for I/O Port C of thrusters effected. The Timer
                        ; ISR will clear these bits in the thruster data port.
;           Thruster Turn-off Queue entry 4
TCBQ4_TMR       RES 2   ;Timer ticks for this shut-off command
                        ; The Timer ISR will start the timer with this value,
TCBQ4_MAP       RES 1   ;Bit map for I/O Port C of thrusters effected. The Timer
                        ; ISR will clear these bits in the thruster data port.
TCB_QUEUE_END:

;	Timer Run/Stop Semaphore for the Timer Interrupt Service Routine (TISR)
;	Semaphore change sequence for all threads is specified below:
;	 Stop State: (RSS_RUN==0) The Timer ISR should disable itself and return.
;    Run State:  (RSS_RUN!=0) The Timer ISR should respond to interrupts by
;                            shutting off the appropriate thruster(s) and then
;                            scheduling the nest shut-off interrupt.
TRSS_RUN		RES 1	;the Run/Stop Semaphore (init to 0)
;

;       I2C state variables
I2C_DEVADR      RES 1	;device address for which this PIC will answer.
I2C_DEVMASK     RES 1   ;AND mask applied to the above address.
I2C_EEADR       RES 1   ;The Emulated EEPROM address register.
I2C_DEST		RES 1	;Destination next data to be received
I2CD_ADDR			EQU	0x00	;Emulated EEPROM address register
I2CD_LEN			EQU	0x01	;None. The length field is not stored
I2CD_DATA			EQU 0x02	;Data is stored to the EEEPROM location
								; adressed by I2C_EEADR.
I2C_LEN			RES	1	;The number of characters to be stored in a EEEPROM
						; write message.

;******************************************************************************
;	The last address above must be less than 060h       xx bytes
;******************************************************************************

 PAGE
		SUBTITLE "DATA MEMORY definition section"
URAM	UDATA	0x100

;******************************************************************************
;	Thruster Controller State (part 2, non-access ram part)
;******************************************************************************
;
;	More variables (replace the below with real variables)
xxx     	res 1

 PAGE
	SUBTITLE "Non-banked Buffers"
;******************************************************************************
;	Stack
;	Used for saving facilities during subroutine execution
;	Starts at lowest address and grows toward highest
;******************************************************************************
stack		EQU 0x0060	;room for 160 entries
stackEnd	EQU 0x00ff

;******************************************************************************
;	I2C Receive Buffer
;	Messages received from the I2C port are placed in this buffer.
;******************************************************************************
rBuffer		EQU 0x0140	;room for 64 bytes
rBufferEnd	EQU 0x017f

;******************************************************************************
;	I2C Transmit Buffer
;	Messages to be sent to the I2C port are placed in this buffer.
;******************************************************************************
tBuffer		EQU 0x0180	;room for 128 bytes
tBufferEnd	EQU 0x01ff

lastData        EQU 0x1ff       ;the last byte in data memory

 PAGE
;******************************************************************************
;******************************************************************************
;******************************************************************************
		SUBTITLE "Program memory: Interrupt vector definition section"
;******************************************************************************
;******************************************************************************
;******************************************************************************
ResetVector	CODE	0x0000

		goto	PwrOnReset ;go to power-on-reset segment

;******************************************************************************
;High priority interrupt vector
; This code will start executing when a high priority interrupt occurs or
; when any interrupt occurs if interrupt priorities are not enabled.

HighIntVector	CODE	0x0008

;		goto	HighInt	;go to high priority interrupt routine
		goto	LowInt	;go to low priority interrupt routine for testing

;******************************************************************************
;Low priority interrupt vector
; This code will start executing when a low priority interrupt occurs.
; This code can be removed if low priority interrupts are not used.

LowIntVector	CODE	0x0018

		goto	LowInt	;go to low priority interrupt routine

;******************************************************************************
		SUBTITLE "High prority interrupt handler section"
;High priority interrupt routine
; The high priority interrupt code is placed here.

		CODE

HighInt:

;	*** high priority interrupt code goes here ***


		retfie	FAST
 PAGE
;******************************************************************************
		SUBTITLE "Low prority interrupt handler section"
;	This routine polls the sources of all configured "low priority"
;	interrupts. For each source that is requesting service, the corresponding
;	interrupt handler is called.
;	If no source has requested service, the firmware hangs.
;
;	This routine save the below  Background thread facilities:
;		Status Register
;		WREG
;		Bank Select Register
;		FSR1
;	These facilities may be used by interrupt handlers without saving.
;
;******************************************************************************
LowInt:	;Save the STATUS reg, WREG, BSR, and FSR1
			movff	STATUS,PREINC0
			movff	WREG,PREINC0
			movff	BSR,PREINC0
			movff	FSR1L,PREINC0
			movff	FSR1H,PREINC0
;		if the Thruster Timer (Timer0) interrupted
				btfss	INTCON,TMR0IF
				bra	LowIntIf1x
;	  		Call the Timer0 Interrupt Handler
				call	Timer0Int
LowIntIf1x:
;		endif Timer0 interrupted
;		if the Sync Interrupt occured
				btfss	INTCON3,INT2IF
				bra	LowIntIf2x
;	  		Call the Sync signal Interrupt Handler
				call	SyncInt
LowIntIf2x:
;		endif Sync interrupted
;Rtn	Restore the STATUS reg, WREG, BSR, and FSR2
LowIntRtn	movff	POSTDEC0,FSR1H
			movff	POSTDEC0,FSR1L
			movff	POSTDEC0,BSR
			movff	POSTDEC0,WREG
			movff	POSTDEC0,STATUS
			retfie

 PAGE
		SUBTITLE "Power-On-Reset section"
;******************************************************************************
;	Perform the Power-On-Reset
;	Initialize the PIC hardware.
;******************************************************************************
PwrOnReset:
;	Set up the software (save) stack
		lfsr	FSR0,stack
;	Set Bank Select Register = 1
		movlw	1
		movwf	BSR
;   Set processor speed to 4MHz
        movlw   b'01010010'     ;no sleep, 4Mhz, HFINTOSC
        movwf   OSCCON
;	Configure all interrupts to occur at low priority level.
		movlw	b'00000000'	;All low priority
		movwf	IPR1
		movlw	b'00000000'	;All low priority
		movwf	IPR2
;	Initialize all data port directions to inputs
		movlw	0xff
		movwf	TRISA		;SET_TRIS_A	all input bits
		movwf	TRISB		;SET_TRIS_B	all input bits
		movwf	TRISC		;SET_TRIS_C	all input bits
;	Default to all digital I/O
		clrf	ANSEL		;
		clrf	ANSELH		;
;	Disable the A/D converter
		clrf	WREG		;setup_adc(ADC_OFF )
		movwf	ADCON0		;
;******************************************************************************
;	Write the filler pattern throughout all of RAM
;******************************************************************************
		lfsr	FSR2,lastData   ; load with the address of the last data byte
		movlw	0xAA
PORL1b	tstfsz	FSR2L
		bra		PORL1c
		tstfsz	FSR2H
		bra		PORL1c
		bra		PORL1x
PORL1c	movwf	POSTDEC2
		bra		PORL1b
PORL1x:	nop
;******************************************************************************
;	Initialize thruster controls, then set pins to output mode
;******************************************************************************
;	Turn thruster controls off
		output_low	THRST1+LAT_OFFSET
		output_low	THRST2+LAT_OFFSET
		output_low	THRST3+LAT_OFFSET
		output_low	THRST4+LAT_OFFSET
;	Set directions (input or output)
		output_low	THRST1+TRIS_OFFSET	;set TRIS to output
		output_low	THRST2+TRIS_OFFSET	;set TRIS to output
		output_low	THRST3+TRIS_OFFSET	;set TRIS to output
		output_low	THRST4+TRIS_OFFSET	;set TRIS to output
;******************************************************************************
;	Initialize I2C controls
;******************************************************************************
;	Setup the I2C slave port 
		output_high I2C_SDA1+TRIS_OFFSET	;signals as inputs
		output_high I2C_SCL1+TRIS_OFFSET	;signals as inputs
;	Read the I2C Device Address from the PIC EEPROM
		movlw	low eeI2CDevAdr 	;PIC EEPROM address
		movwf 	EEADR 				; Data Memory Address to read
		bcf		EECON1,EEPGD 		; Point to DATA memory
		bcf 	EECON1,CFGS 		; Access EEPROM
		bsf 	EECON1,RD 			; EEPROM Read
		movf	EEDATA,W			; get EEDATA
		movf	I2C_DEVADR			; store it in ACS memory
		movwf	SSPADD				; and in the I2C controller
;	Read the I2C Address Mask from the PIC EEPROM
		movlw	low eeI2CMask	 	;PIC EEPROM address
		movwf 	EEADR 				; Data Memory Address to read
		bcf		EECON1,EEPGD 		; Point to DATA memory
		bcf 	EECON1,CFGS 		; Access EEPROM
		bsf 	EECON1,RD 			; EEPROM Read
		movf	EEDATA,W			; get EEDATA
		movf	I2C_DEVMASK			; store it in ACS memory
		movwf	SSPMSK
;	Configure the I2C port as  7-bit slave
		movlw	0x36		;WCOL SSPOV SSPEN CKP SSPM<3:0>
		movwf	SSPCON1		;  0    0     1    1    0110 {Slave, 7-bit addr}
		movlw	0x80		;SMP CKE r/o r/o r/o r/o r/o r/o
		movwf	SSPSTAT		; 1   0   slow slew rate; disable SMB functions
		movlw	0x01		;GCEN  - - - - - -  SEN
		movwf	SSPCON2		; 0    0 0 0 0 0 0   1
							;gen call disb.; clk stretch enb.
		bcf		PIR1,SSPIF	;Clear MSSP interrupt flag
;		bsf 	PIE1,SSPIE 	;Enable MSSP interrupt enable bit
;*** 	Special adjustments for simulation only
	if is_simulating
;	  Make the PIC inputs into outputs so that they can be stimulated
;	  by code.
		output_low	RA2+TRIS_OFFSET	;set TRIS to output
	endif
;******************************************************************************
;	Perform Software Reset
;******************************************************************************
		call	SoftwareReset
;******************************************************************************
;	Capture the reason for the reset
;******************************************************************************
        movff   RCON,SYS_RESR
;******************************************************************************
;	Initialize Timer0
		call	Timer0Init
;******************************************************************************
;	Enable Interrupts
;******************************************************************************
;	Enable Interrupts
		movlw	b'11000000'	;GIEH+GIEL
		movwf	INTCON
;******************************************************************************
;	Start main loop
;******************************************************************************
		goto	Main		;goto main background loop
 PAGE
;******************************************************************************
		SUBTITLE "Software Reset Subroutine"
; void SoftwareReset()
;	Perform a Software Reset. Reset all locations that are marked ISR.
;	See reset descriptions in block comments at star of this file.
;******************************************************************************
SoftwareReset:
;	Initialize Thruster Controller State
;   First the configuration from the PIC EEPROM
		movlw	low eeConfig 		;PIC EEPROM address
		movwf 	EEADR 				; Data Memory Address to read
		bcf		EECON1,EEPGD 		; Point to DATA memory
		bcf 	EECON1,CFGS 		; Access EEPROM
		bsf 	EECON1,RD 			; EEPROM Read
		movf	EEDATA,W			; get EEDATA
		movwf	TCB_CNFG
;	Thruster state variable and status
		clrf    TCB_STATE
		clrf    TCB_STATUS
		clrf	TCB_ECNT
		clrf	TCB_CNT
		clrf	TCB_OTHER
		clrf	TCB_OTHER+1
;	I2C state variables (some have already been initialized)
        clrf	I2C_EEADR	;The Emulated EEPROM address register.

		movlw	I2CD_ADDR	;Next received data is stored to I2C_EEADR.
		movwf	I2C_DEST

;	Initialize the Thruster time part of the EEEPROM
		clrf	TCB_TH1TKS
		clrf	TCB_TH1TKS+1
		clrf	TCB_TH2TKS
		clrf	TCB_TH2TKS+1
		clrf	TCB_TH3TKS
		clrf	TCB_TH3TKS+1
		clrf	TCB_TH4TKS
		clrf	TCB_TH4TKS+1

;   Initialize the system state
        movlw   0x20        ;HW sets this bit to 0, so we'll use it to 
							;  mean SW Reset
        movwf   SYS_RESR    ;Set the reason for the last reset
                            ;  Power on Reset code overlays this location
;	Initialize the Run/Stop semaphore for the Timer ISR
		clrf	TRSS_RUN
;   Initialize the Timer Action Queue
		clrf	TCB_CNT         ;Empty
;	Initialize Truster Shut-Off Timer
		call	Timer0Init
;	Initialize Sync Interrupts
		call	SyncInit
;*** 	Special initialization for simulation only
;	if is_simulating
;	Initialize the Simulation Command Pointer
;		movlw	low UsbFakeB
;		movwf	usbSimNxt
;		movlw	high UsbFakeB
;		movwf	usbSimNxt+1
;		movlw	upper UsbFakeB
;		movwf	usbSimNxt+2
;	endif
;	Return
		return
 PAGE
;******************************************************************************
		SUBTITLE "Subroutines"
;	Flash LED for WREG times (1 minimum, 0 means 256 flashes)
;	Uses 6 stack locations (two here and four in delay_ms).
;	Returns with W maintained.
FlashLED:
		movwf	PREINC0 	;push saved count value on stack
		movwf	PREINC0 	;push working flash count value on stack
Flash1	output_high	THRST4 	;turn off the LED
		delay_ms    250		;wait 250 mSec
 		output_low  THRST4 	;turn on the LED
		delay_ms    250		;wait 250 mSec
		dcfsnz	INDF0		;reduce flash counter
		bra	Flashx		;exit if counter is zero
  		bra	Flash1
Flashx		;pause a second, then exit
  		output_high THRST4 	;turn off the LED
		delay_ms 250		;wait 250 mSec
		delay_ms 250		;wait 250 mSec
		delay_ms 250		;wait 250 mSec
		delay_ms 250		;wait 250 mSec
		movf	POSTDEC0,W	;pop depleted flash count value from stack
		movf	POSTDEC0,W	;pop saved flash count value from stack
		return

;	Flash a countdown starting with WREG times in the LED (1 minimum, 0 means 
;	256 flashes)
;	Uses 7 stack locations (one here and six in FlashLED).
;	Returns with W maintained.
FlashCntDn:	;flash LED Countdown
		movwf	PREINC0 	;push saved countdown value on stack
FlashC1	call	FlashLED	;flash the LED WReg times
		dcfsnz	WREG		;reduce flash counter
		bra	FlashCx		;exit if counter is zero
		bra	FlashC1		;loop
FlashCx	movf	POSTDEC0,W	;pop saved countdown value from stack
		return


;	Delay the number of 10uSec periods specified by the W reg, 1 minimum, 
;		0 means 2560 uSec
;	Delay time will be exact if not interrupted.
;	Returns with W reg cleared.
;;;; NOT CALIBRATED FOR THIS PIC/OSC seting!
Delay_10us:				;getting here was 2 cycles
		bra	$+2			;2 cycles
		bra	$+2			;2 cycles
		nop				;1 cycle
Delayu1	dcfsnz	WREG	;reduce W ctr ;2 cycles in loop, 1 cycle the last time
		return			;2 cycles ;return if count was set to 1
		bra	$+2			;2 cycles
		bra	$+2			;2 cycles
		bra	$+2			;2 cycles
		bra	Delayu1		;2 cycles

;	Delay the number of 100 uSec intervals specified by the W reg,
;	 1 minimum, 0 means 256*100 uSec
;	Delay time will be exact if not interrupted.
;	Uses 2 stack locations.
;	Returns with W maintained.
Delay_hus:				;getting here was 2 cycles
		movwf	PREINC0	;push saved huSec value on stack, +1 cycle =3
		movwf	PREINC0 ;push working huSec value on stack, +1 cycle =4
Delayh1	dcfsnz	INDF0	;reduce mSec counter; +1 cycle for last time = 5, 
						;  else 2 cycles
		bra		Delayhx	;exit if counter is zero; +2 cycles for last time =7
		nop				;+1 cycle
		movlw	9		;set delay to 90 uSec for intermediate iterations
		call	Delay_10us ;delay 90 uSecs
		bra		$+2		;+2 cyc
		bra		$+2		;+2 cyc 
		bra	Delayh1		;retest
Delayhx	movlw	8		;set delay to 80 uSec for last iteration; +1 cycle =8
		call	Delay_10us ;delay 80 uSecs
		bra		$+2			;+2 cyc =10
		bra		$+2			;+2 cyc =12
		bra		$+2			;+2 cyc =14
		bra		$+2			;+2 cyc =16
		movf	POSTDEC0,W ;pop depleted working mSec value from stack, +1cyc=17
		movf	POSTDEC0,W ;pop saved mSec value from stack, +1 cycle =18
		return			;+2 cycles =20

;	Delay the number of mSec specified by the W reg, 1 minimum, 0 means 256 mSec
;	Delay will be slightly greater than specified value. Some loop overhead and
;	interrupt time is not accounted for.
;	Uses 4 stack locations (two here and two in Delay_hus).
;	Returns with W maintained.
Delay_ms movwf	PREINC0 	;push saved mSec value on stack
		movwf	PREINC0 	;push working mSec value on stack
		movlw	10			;inner loop delays 1 mSec (10 * 100 uSec)
Delaym1	call	Delay_hus	;delay 1 mSec
		dcfsnz	INDF0		;reduce mSec counter
		bra		Delaymx		;exit if counter is zero
		bra		Delaym1
Delaymx	movf	POSTDEC0,W	;pop depleted working mSec value from stack
		movf	POSTDEC0,W	;pop saved mSec value from stack
		return

 PAGE
;******************************************************************************
		SUBTITLE "Sync Signal Interrupt Handler"
;	The Sync interrupt begins the execution of a Thruster Program.
;	When Sync interrupt occurs:
;		1)	The turn-on bit mask will be ORed into the Thruster Control Port
;			(Port C).
;		2)	The Thruster Program Counter is initialized to the first
;			step in the program.
;		3)	That step's Timer Value is loaded into the Thruster Timer (Timer0).
;		4)	Timer0's interrupt is enabled.
;		5)	Timer0 is started.
;		6)	Sync interrupts are disabled.
;
;	The driver consists of three service routines.
;	1) Initialization subroutine
;	2) Enable subroutine
;	3) Interrupt Service Routine (ISR)
;
;******************************************************************************
;	Sync interrupt initialization subroutine
;	This code runs on the Background thread.
;******************************************************************************
SyncInit:
;	Disable Sync interrupts
			bcf	INTCON3,INT2IE
;rf20120817 Flipped polarity of the Sync signal.
;	Configure the Sync interrupt
			bcf	INTCON3,INT2IP 	;low priority
			bcf	INTCON2,INTEDG2	;falling edge causes interrupt
			bcf	INTCON3,INT2IF	;reset interrupt flag
;	return
		return

;******************************************************************************
;	Sync interrupt enable subroutine
;	This code runs on either the Background or Low Priority Interrupt thread.
;******************************************************************************
SyncEnable:
;   Clear any pending interrupt request
			bcf	INTCON3,INT2IF	;reset interrupt flag
;	Enable Sync interrupts
			bsf	INTCON3,INT2IE
;	return
		return

;******************************************************************************
;	Sync interrupt handler
;	This code runs on the Low Priority Interrupt thread.
;******************************************************************************
SyncInt:
;	Disable Sync interrupts
			bcf	INTCON3,INT2IE
;   Clear the pending interrupt request
			bcf	INTCON3,INT2IF	;reset interrupt flag
;	if the RUN semiphore has us enabled && TCB_STATE==TCBST_RDY 
;		&& some thrusters are suposed to fire  
				movf	TRSS_RUN,W
				bz		SyncIf01x		;not allowed to run
				movlw	TCBST_RDY
				xorwf	TCB_STATE,W	
				bnz		SyncIf01x		;invalid state
				movf	TCB_FMAP,W
				bz		SyncIf01x		;no thrusters to be fired
;		Initialize Thruster Program counter
				movlw	low TCB_QUEUE
				movwf	TCB_CPTR       
				movlw	high TCB_QUEUE
				movwf	TCB_CPTR+1       
;		TCB_STATE = TCBST_FIRE	// indicate that thrusters are firing
				movlw	TCBST_FIRE
				movwf	TCB_STATE
;		Fire thrusters
				movf	TCB_FMAP,W
				iorwf	LATC
;		Set Timer0's count
				movff	TCBQ1_TMR+1,TMR0H
				movff	TCBQ1_TMR,TMR0L		;writes all 16 bits
;		Reset old interrupts
				bcf		INTCON,TMR0IF
;		Enable Timer0's interrupt
				bsf		INTCON,TMR0IE
;		Start the timer
				bsf		T0CON,TMR0ON
SyncIf01x:
;	endif
;	return
		return

 PAGE
;******************************************************************************
		SUBTITLE "Thruster Shut-off Timer (Timer0) Driver"
;	This timer is used to determine when to shut-off thrusters.
;	The Background Task sets up the entries in the Thruster Control Block.
;   This driver uses the Thruster Queue entries. When the Timer0 interrupt
;   occurs, the ISR will complete the current entry in the queue (by turning
;   off the indicated thrusters) and determine if more queue entries need to be
;   processed. If there are more entries to be processed, it will advance the
;   queue pointer to the next entry, and restart Timer0 with the tick count
;   from that entry. Once all the entries have been processed, it will disable
;   its own interrupt and enable Sync interrupts.
;
;   This driver assumes that the PIC is running using a 4MHz internal clock.
;   Therefore, the input to the timer will be 1MHz (FOSC/4).
;
;	The timer's prescaler is set to divide by 256, so its counter ticks
;	every 256 uSec. This results in the 16-bit hardware counter overflowing
;	every 16.78 seconds. This is the maximum thruster "on" time per I2C
;   command.
;
;   The timer register will increment on every clock tick out of the
;   prescaler. If interrupt is inabled, a low priority interrupt will be
;   generated upon timer overflow (0xFFFF -> 0x0000).
;
;	The timer is reset by Software Reset. 
;
;	The driver consists of three service routines.
;	1) Initialization subroutine
;	2) Reset subroutine
;	3) Interrupt Service Routine (ISR)
;
;******************************************************************************
;	Thruster Shut-off Timer (Timer0) initialization subroutine
;	This code runs on the Background thread.
;******************************************************************************
Timer0Init:
Timer0Reset:        ;Reset and Initialization are the same for this timer.
;	Disable timer interrupts
		bcf		INTCON,TMR0IE
;	Turn timer off while messing with it
		bcf		T0CON,TMR0ON
;	Configure the timer
		bcf		T0CON,T08BIT	;=0 sets counter in 16-bit mode
		bcf		T0CON,T0CS		;=0 sets clock source to internal (FOSC/4)
		bsf		T0CON,T0SE		;=1 sets counter to increment on falling edge
		bcf		T0CON,PSA		;=0  enables the prescaler
		bsf		T0CON,2			;T0PS<2:0> set prescaler to divide by 256
		bsf		T0CON,1			;"
		bsf		T0CON,0			;"
;	Configure interrupts to appear as Low Priority Interrupts
        bcf 	INTCON2,TMR0IP
;   Clear any pending interrupt request
        bcf     INTCON,TMR0IF   ;clear pending interrupt
;	return
		return
;
;******************************************************************************
;	Thruster Shut-off Timer (Timer0) Interrupt Handler
;	This code runs on the Low Priority Interrupt Thread.
;	This routine is being executed because the Timer0 interrupt request
;	bit was detected to be on. This bit is turned on when the PIC's Timer0
;	wraps from 0xFFFF to 0x0000.
;	
;	When an interrupt occurs the ISR will execute the next sequential step in 
;	the Thruster Program.
;	Summary of actions:
;	1)	zeroing the bits indicated in current step's bit mask
;	2)	if the mask field zero all thruster controls,
;			leave Timer0 interrupts disabled and enable Sync interrupts.
;	3)	else advance the Thruster Program Counter, load the new step's 
;		timer value into timer and restart timer0
;
;	Register usage:
;	Destroys:	LPREG0
;******************************************************************************
Timer0Int:
;	Disable Timer interrupts
		bcf		INTCON,TMR0IE
;	Turn timer off while messing with it
		bcf		T0CON,TMR0ON
;	Turn off the pending interrupt request
        bcf     INTCON,TMR0IF   ;clear pending interrupt
;	if the Run semiphore permits operation
			movf	TRSS_RUN,W
			bz		Tim0If02x		;not allowed to run
;   	Turn off the thrusters indicated in the current mask
Tmr0Agn		movff	TCB_CPTR,FSR1L
			movff	TCB_CPTR+1,FSR1H
			movf	POSTINC1,W		;skip timer value low
			movf	POSTINC1,W		;skip timer value high
			movf	POSTINC1,W		;load the mask value
			andwf	LATC			;turn off the thrusters
;		Update program counter 
			movff	FSR1L,TCB_CPTR	;does not change coondition codes
			movff	FSR1H,TCB_CPTR+1
;		if some thrusters are still on and there are more program steps
				bz		Tim0If01f		;all off
				movlw	low TCB_QUEUE_END
				xorwf	FSR1L,W
				bz		Tim0If01f		;PGM Counter is at end
;			Reload Timer0 Counts
				movff	POSTINC1,LPREG0	
				movff	POSTINC1,TMR0H
				movff	LPREG0,TMR0L	;writes all 16 bits
;			if timer count ==0 
					movf	TMR0L,W
					bnz		Tim0If04x
					movf	TMR0H,W
					bnz		Tim0If04x
;				goto 	Tmr0Agn	; execute that queue instruction also
					bra		Tmr0Agn
Tim0If04x:
;			endif					
;			Reset old interrupts
				bcf		INTCON,TMR0IF
;			Enable Timer0's interrupt
				bsf		INTCON,TMR0IE
;			Start the timer
				bsf		T0CON,TMR0ON
				bra		Tim0If01x
;		endif some thrusters are still on
;		else all thrusters off
;			indicate thrusters are no longer firing
Tim0If01f		movlw	TCBST_RDY
				movwf	TCB_STATE
;			enable Sync interrupts
				call	SyncEnable
Tim0If01x:
;		endelse
Tim0If02x:
;	endif
;	return
		return
;
 PAGE
;******************************************************************************
		SUBTITLE "Main Background Loop -- LegoCntlr action receiver"
;	This routine is the main background loop of the Thruster Pod.
;	Loop sequence: (only one thing identified so far)
;	1) Check I2C port for an I/O transfer. If so, handle it.
;		The I2C handler will take care of EEPROM emulation, setting up
;		thruster firing programs, and enabling the interrupt sequence that
;		executes the programs.
;
;	(The below paragraph should be placed somewhere else.)
;   If a new I2C command was fully received, and it is a Fire Thrusters
;      	command, prepare the command for execution and mark it as "pending".
;           a)  reset the Run/Stop semaphore, to block ISR access.
;           b)  shut-off all thrusters.
;           c)  purge the Thruster Queue (TQ).
;           d)  build a new TQ as appropriate.
;           e)  enable SYNC interrupts
;           f)  set the Run/Stop semaphore, to allow ISR access.
;
;	The "Main Background Loop" will cause thrusters to fire and it will enable
;   interrupt handler actions that will fire the appropriate thrusters and
;   shut-off thrusters per a computed schedule.
;
;	No I2C data buffer is used. Transfers are built into the I2C handler's 
;	state machine and data is generated and consummed.
;
;	A Run/Stop semaphore is maintained with the Sync and Timer Interrupt Service
;   Routines (ISRs). While the semaphore is set, the ISRs owns the
;   Timer Control Block and Queue array. The "Main Background Loop" may reset 
;   this semaphore at any time to take back the control of the timer action 
;   queue array. Obviously, since the "Main Background Loop" runs at a lower 
;   priority than the ISRs, the Main Loop will only be able to change the 
;   semaphore when ISRs are not running.
;
;	Register usage:
;	 REG4 = temporary storage of the masked SSPSTAT value
;	 FSR1 = temporary pointer to source or destination data
;    rest tbd.
;
;******************************************************************************
Main:		nop

MainLoop:
BreakHere   nop         ;beginning of the main background loop.
		
;
;TmLoop		movlw	9
;			call	Delay_10us
;			output_low	THRST4
;			nop
;			movlw	9
;			call	Delay_hus
;			bra		$+2
;			bra		$+2
;			movlw	9
;			call	Delay_10us
;			output_high	THRST4
;			nop
;			movlw	9
;			call	Delay_hus
;			bra		$+2
;			bra		TmLoop




;================================================================================
;*** 	Check for and handle I2C I/O operations
;================================================================================
;	if I2C port 1 is not requesting attention
			btfsc 	PIR1,SSPIF 	; Is this a SSP interrupt?
;		call I2C_Handler   (inherits ability to use REGs & FSRs without saving)
			call I2C_Handler	;note this can't be more than 1 instruction
;	endif


;	goto the top of the Main loop
		goto	MainLoop

 PAGE
;******************************************************************************
;******************************************************************************
;******************************************************************************
		SUBTITLE "Main loop subroutines"
;******************************************************************************
;******************************************************************************
;******************************************************************************
;	I2C I/O handler routine
;----------------------------------------------------------------
; State 1: I2C write operation, last byte was an address byte
; SSPSTAT bits: S = 1, D_A = 0, R_W = 0, BF = 1
;
; State 2: I2C write operation, last byte was a data byte
; SSPSTAT bits: S = 1, D_A = 1, R_W = 0, BF = 1
;
; State 3: I2C read operation, last byte was an address byte
; SSPSTAT bits: S = 1, D_A = 0, R_W = 1 (see Appendix C for more information)
;
; State 4: I2C read operation, last byte was a data byte
; SSPSTAT bits: S = 1, D_A = 1, R_W = 1, BF = 0
;
; State 5: Slave I2C logic reset by NACK from master
; SSPSTAT bits: S = 1, D_A = 1, BF = 0, CKP = 1 (see Appendix C for more information)
; For convenience, WriteI2C and ReadI2C functions have been used.
;-----------------------------------------------------------------
I2C_Handler: 
;	Reset the I2C Interrupt Flag (even though irpts is disabled)
			bcf PIR1,SSPIF
;	switch (I2C hardware state)
			movf	SSPSTAT,W 	; Get the value of SSPSTAT
			movwf	TCB_SSPST	; Save the SSPSTAT in case of error
			andlw	b'00101101' ; Mask out unimportant bits in SSPSTAT.
			movwf 	REG4 		; for comparision checking.

;		case (State1):  Write operation, last byte was an address buffer full
I2C_State1		movlw	b'00001001' ; address, buffer is full.
				xorwf	REG4,W 		;
				btfss 	STATUS,Z 	; Are we in State1?
				goto	I2C_State2	; No, check for next state.....
;		Reset the I/O controller to receive the next character
				movf	SSPBUF,W 		;do a dummy read of the SSPBUF.
				bsf		SSPCON1,CKP		;Release SCK
;		Set write destination to EEEPROM address register
				movlw	I2CD_ADDR	;Next received data is stored to I2C_EEADR.
				movwf	I2C_DEST
;		Clear the expected length of a write
				clrf	I2C_LEN
;		break
				goto	I2CHanSw01x

;		case (State2):	Write operation, last byte was data, 
I2C_State2		movlw 	b'00101001' ; and buffer is full.
				xorwf 	REG4,W
				btfss 	STATUS,Z 	; Are we in State2?
				goto 	I2C_State3 	; No, check for next state.....
I2CHanIf01f:
;			switch (I2C_DEST) ;write destination
;				case (EEEPROM Address Register):
						movlw	I2CD_ADDR
						xorwf	I2C_DEST,W
						bnz		I2CHanSw02C2
;				Receive the character
						movf 	SSPBUF,W 		; Get the byte from the SSP.
						bsf		SSPCON1,CKP		;Release SCK
;				Store it in the Address Register
						movwf	I2C_EEADR
;				Set write destination = length register
						movlw	I2CD_LEN
						movwf	I2C_DEST
;				break;
						bra		I2CHanSw02x

;				case (EEEPROM Length Register):
I2CHanSw02C2			movlw	I2CD_LEN
						xorwf	I2C_DEST,W
						bnz		I2CHanSw02C3
;				Receive the character
						movf 	SSPBUF,W 		; Get the byte from the SSP.
						bsf		SSPCON1,CKP		;Release SCK
;				Store the expected length of the write
						movwf	I2C_LEN
;				Reduce the length by the 3 characters we have already received.
						decf	I2C_LEN
						decf	I2C_LEN
						decf	I2C_LEN
						bnn		$+4				; don't permit negative value
						clrf	I2C_LEN
;				Set write destination = *(Address Register)
						movlw	I2CD_DATA
						movwf	I2C_DEST
;				break;
						bra		I2CHanSw02x

;				case (EEEPROM Data):
I2CHanSw02C3			movlw	I2CD_DATA
						xorwf	I2C_DEST,W
						bnz		I2CHanSw02x
;				call E3promWtNxt
						call	E3promWrNxt
				
I2CHanSw02x:
;			endswitch

I2CHanIf01x:
;		endelse
;		break
				goto	I2CHanSw01x

;		case (State3):  read operation, last byte was an address byte
I2C_State3		movf	REG4,W
				andlw 	b'00101100' 	; Mask BF bit in SSPSTAT
				xorlw 	b'00001100'
				bnz 	I2C_State4 		; No, check for next state.....
;		Reset the I/O controller to receive the next character
				movf	SSPBUF,W 		;do a dummy read of the SSPBUF.		
;		Return next Emulated EEPROM location
				call	E3promRdNxt		;char in W, C=valid
				call 	WriteI2C 		; Write the byte to SSPBUF
;		break
				goto	I2CHanSw01x

;		case (State4):  I2C read operation, last byte was a data byte
I2C_State4		btfsc 	SSPCON1,CKP ;
				goto 	I2C_State5
				movlw 	b'00101100' 	; buffer is empty.
				xorwf 	REG4,W
				bnz 	I2C_State5 		; No, check for next state....
;		Return next Emulated EEPROM location
				call	E3promRdNxt		;char in W, C=valid
				call 	WriteI2C 		; Write to SSPBUF
;		break
				goto	I2CHanSw01x

;		case (State5):  Slave I2C logic reset by NACK from master
I2C_State5		movf 	REG4,W
				andlw 	b'00101000' 	; Mask RW bit in SSPSTAT
				xorlw 	b'00101000'
				bnz 	I2C_StErr 		; No, check for next state....
;		break
				goto	I2CHanSw01x
;		default:	I2C state error
;		Clear the character counter
I2C_StErr		clrf 	I2C_LEN
;		Report the error
				bsf		TCB_STATUS,TCBS_ISE	;set state error bit
				incfsz	TCB_ECNT	;errors++
				decf	TCB_ECNT	;don't wrap the error counter
;		Reset the MSSP adapter
				bsf		SSPCON1,WCOL	;Reset write collision
				bcf		SSPCON1,SSPOV	;Reset receive overflow
				bsf		SSPCON1,CKP		;Release SCK
;	endswitch
I2CHanSw01x:
;	return
		return

;---------------------------------------------------------------------
; WriteI2C	we should check Carry, if clear terminate read.
;---------------------------------------------------------------------
WriteI2C:	
			bcf 	SSPCON1,WCOL 	; Clear the WCOL flag.
			movwf 	SSPBUF 			; Write the byte in WREG
			btfsc 	SSPCON1,WCOL 	; Was there a write collision?
			goto WriteI2C
			bsf		SSPCON1,CKP		;Release SCK
			return

; *****************************************************************************
; 	void E3promWrNxt()
;
;	This routine receives the pending character in the I2C SSPBUF and
;	attempts to store it into the emulated EEPROM.
;
;	I2C_EEADR - contains the EEEPROM address where the data is to be stored.
;				this field may be updated.
;	Other EEEPROM mapped locations may be updated.
;
;	Destroys: WREG, FSR1, REG0
;
E3promWrNxt:
;	switch (EEPROM block to be written)
;		case (PnP 0x00-0x3F): // read only block, ignore write
				movlw	0x40
				cpfslt	I2C_EEADR	;skip if I2C_EEADR < 0x40
				bra		E3WrSw01C40
;		Receive the character
				movf 	SSPBUF,W 		; Get the byte from the SSP.
				bsf		SSPCON1,CKP		;Release SCK
;		Ignore the write attempt
				nop
;		Increment the EEEPROM Address register
				incf	I2C_EEADR
;		Wrap to beginning of block if necessary						
				movlw	0x40
				cpfseq	I2C_EEADR
				bra		$+6				;skip next two instructions
				movlw	0x00
				movwf	I2C_EEADR
;		break;
				bra		E3WrSw01x

;		case (Status 0x40-0x47): // read only block, ignore write
E3WrSw01C40		movlw	0x48
				cpfslt	I2C_EEADR	;skip if I2C_EEADR < 0x48
				bra		E3WrSw01C48
;		Receive the character
				movf 	SSPBUF,W 		; Get the byte from the SSP.
				bsf		SSPCON1,CKP		;Release SCK
;		Ignore the write attempt
				nop
;		Increment the EEEPROM Address register
				incf	I2C_EEADR
;		Wrap to beginning of block if necessary						
				movlw	0x48
				cpfseq	I2C_EEADR
				bra		$+6				;skip next two instructions
				movlw	0x40
				movwf	I2C_EEADR
;		break;
				bra		E3WrSw01x

;		case (0x48-0x7F): //unimplimented block, ignore write
E3WrSw01C48		movlw	0x80
				cpfslt	I2C_EEADR	;skip if I2C_EEADR < 0x80
				bra		E3WrSw01C80
;		Receive the character
				movf 	SSPBUF,W 		; Get the byte from the SSP.
				bsf		SSPCON1,CKP		;Release SCK
;		Ignore the write attempt
				nop
;		Increment the EEEPROM Address register
				incf	I2C_EEADR
;		break;
				bra		E3WrSw01x

;		case (Thruster Times 0x80-0x87): // Thruster Time values
E3WrSw01C80		movlw	0x88
				cpfslt	I2C_EEADR	;skip if I2C_EEADR < 0x88
				bra		E3WrSw01C88
;		Receive the character
				movf 	SSPBUF,W 		; Get the byte from the SSP.
				bsf		SSPCON1,CKP		;Release SCK
				movwf	REG0
;		Write the character into the block
				lfsr	FSR1,low(TCB_TH1TKS-FILE00-0x80)
				movf	I2C_EEADR,W
				addwf	FSR1L			;add will not cause carry
				movff	REG0,INDF1
;		Indicate that Thruster Program Regeneration is needed
				bsf		TCB_STATUS,TCBS_PUPD
;		Increment the EEEPROM Address register
				incf	I2C_EEADR
;		Decriment the count of characters to be written
				decf	I2C_LEN
;		if count==0
				bnz		E3WrIf01x
;			Convert the Received data to a Thruster Program
					call	NewThrusterValues
;			Reset Program Generation Needed flag
					bcf		TCB_STATUS,TCBS_PUPD
;			Allow Sync and Timer interrupt handlers to run
					movlw	0xFF
					movwf	TRSS_RUN
;			Enable Sync interrupts for the first time (they auto-reenable)
					call SyncEnable
E3WrIf01x:
;		endif
;		Wrap to beginning of block if necessary						
				movlw	0x88
				cpfseq	I2C_EEADR
				bra		$+6				;skip next two instructions
				movlw	0x80
				movwf	I2C_EEADR
;		break;
				bra		E3WrSw01x

;		case (0x88-0x8F): //unimplimented block, ignore write
E3WrSw01C88		movlw	0x90
				cpfslt	I2C_EEADR	;skip if I2C_EEADR < 0x90
				bra		E3WrSw01C90
;		Receive the character
				movf 	SSPBUF,W 		; Get the byte from the SSP.
				bsf		SSPCON1,CKP		;Release SCK
;		Ignore the write attempt
				nop
;		Increment the EEEPROM Address register
				incf	I2C_EEADR
;		break;
				bra		E3WrSw01x

;		case (Reset 0x90-0x90): // Reset block
E3WrSw01C90		movlw	0x91
				cpfslt	I2C_EEADR	;skip if I2C_EEADR < 0x91
				bra		E3WrSw01C91
;		Receive the character
				movf 	SSPBUF,W 		; Get the byte from the SSP.
				bsf		SSPCON1,CKP		;Release SCK
;		Write the character into the block
				movwf	TCB_RESET
;		switch (reset requested)
;			case (full reset)
				movlw	TCBR_FULL
				xorwf	TCB_RESET,W
				bnz		E3WrSw02C2
;			Do a full reset (does not return)
				reset
				bra		$				;should never get here

;			case (cancel current program)
E3WrSw02C2	movlw	TCBR_CNCL
				xorwf	TCB_RESET,W
				bnz		E3WrSw02x
;			Call "Cancel the current Thruster program"
				call	ThstrCancel
;			break;
				bra		E3WrSw02x
E3WrSw02x:
;		endswitch

;		Indicated Reset completed
				clrf	TCB_RESET
;		break;
				bra		E3WrSw01x

;		default: //addr>90; unimplimented block, ignore write
E3WrSw01C91:
;		Receive the character
				movf 	SSPBUF,W 		; Get the byte from the SSP.
				bsf		SSPCON1,CKP		;Release SCK
;		Ignore the write attempt
				nop
;		No point to incrementing the address field

E3WrSw01x:
;	endswitch
;	return
			return

; *****************************************************************************
; (WReg=char,STATUS.C=bool) E3promRdNxt()
;Returns:
;		WREG		The data from the addressed location of the emulated
;					EEPROM.		
;		STATUS.C  	The carry bit is set if the character was from an 
;					implemented block.
;					Else the carry bit is cleared. 
;
;	This routine returns emulated_EEPROM[I2C_EEADR++].
;
;	I2C_EEADR - contains the EEEPROM address where the data is to be stored.
;				this field may be updated.
;
;	Usage:
;		REG0	temporarily stores the return character.
;
;	Destroys: WREG, FSR1, REG0, PIC EEPROM registers.
;
E3promRdNxt:
;	switch (EEPROM block to be read)
;		case (PnP 0x00-0x3F): // PIC EEPROM data
				movlw	0x40
				cpfslt	I2C_EEADR	;skip if I2C_EEADR < 0x40
				bra		E3RdSw01C40
;		Read the PIC EEPROM
				movf	I2C_EEADR,W 	;PIC EEPROM address
				movwf 	EEADR 			; Data Memory Address to read
				bcf		EECON1,EEPGD 	; Point to DATA memory
				bcf 	EECON1,CFGS 	; Access EEPROM
				bsf 	EECON1,RD 		; EEPROM Read
				movff	EEDATA,REG0		;save EEDATA into REG0
;		Increment the EEEPROM Address register
				incf	I2C_EEADR
;		Wrap to beginning of block if necessary						
				movlw	0x40
				cpfseq	I2C_EEADR
				bra		$+6				;skip next two instructions
				movlw	0x00
				movwf	I2C_EEADR
;		Set STATE.C to 1
				bsf		STATUS,C
;		break;
				bra		E3RdSw01x

;		case (Status 0x40-0x47): // read Status
E3RdSw01C40		movlw	0x48
				cpfslt	I2C_EEADR	;skip if I2C_EEADR < 0x48
				bra		E3RdSw01C48
;		Read the status location
				lfsr	FSR1,low(TCB_STATUS-FILE00-0x40)
				movf	I2C_EEADR,W
				addwf	FSR1L			;add will not cause carry
				movff	INDF1,REG0
;		Increment the EEEPROM Address register
				incf	I2C_EEADR
;		Wrap to beginning of block if necessary						
				movlw	0x48
				cpfseq	I2C_EEADR
				bra		$+6				;skip next two instructions
				movlw	0x40
				movwf	I2C_EEADR
;		Set STATE.C to 1
				bsf		STATUS,C
;		break;
				bra		E3RdSw01x

;		case (Status 0x48-0x7F): //unimplimented block, read 0x00
E3RdSw01C48		movlw	0x80
				cpfslt	I2C_EEADR	;skip if I2C_EEADR < 0x80
				bra		E3RdSw01C80
;		Unimplimented, read a zero
				clrf	REG0
;		Set STATE.C to 0
				bcf		STATUS,C
;		break;
				bra		E3RdSw01x

;		case (Thruster Times 0x80-0x87): // Thruster Time values
E3RdSw01C80		movlw	0x88
				cpfslt	I2C_EEADR	;skip if I2C_EEADR < 0x88
				bra		E3RdSw01C88
;		Read the character from the block
				lfsr	FSR1,low(TCB_TH1TKS-FILE00-0x80)
				movf	I2C_EEADR,W
				addwf	FSR1L			;add will not cause carry
				movff	INDF1,REG0
;		Increment the EEEPROM Address register
				incf	I2C_EEADR
;		Wrap to beginning of block if necessary						
				movlw	0x88
				cpfseq	I2C_EEADR
				bra		$+6				;skip next two instructions
				movlw	0x80
				movwf	I2C_EEADR
;		Set STATE.C to 1
				bsf		STATUS,C
;		break;
				bra		E3RdSw01x

;		case (0x88-0x8F): //unimplimented block, read 0x00
E3RdSw01C88		movlw	0x90
				cpfslt	I2C_EEADR	;skip if I2C_EEADR < 0x90
				bra		E3RdSw01C90
;		Unimplimented, read a zero
				clrf	REG0
;		Set STATE.C to 0
				bcf		STATUS,C
;		break;
				bra		E3RdSw01x

;		case (Reset 0x90-0x90): // Reset block
E3RdSw01C90		movlw	0x91
				cpfslt	I2C_EEADR	;skip if I2C_EEADR < 0x91
				bra		E3RdSw01C88
;		Read the character from the block
				movf	TCB_RESET,W
				movwf	REG0
;		Set STATE.C to 1
				bsf		STATUS,C
;		break;
				bra		E3RdSw01x

;		default: //addr>90; unimplimented block, read a 0x00
E3RdSw01C91:
;		Unimplimented, read a zero
				clrf	REG0
;		No point to incrementing the address field
;		Set STATE.C to 0
				bcf		STATUS,C
E3RdSw01x:
;	endswitch
;	return	REG0 and STATE.C
			movf	REG0,W
			return

; *****************************************************************************
; void ThstrCancel()
;
;	This routine cancels the current Thruster operation
;
;	Usage:
;	
;	Destroys: STATUS register.
;
ThstrCancel:
;	Save the WREG on the stack
		movwf	PREINC0
;	Block Sync and Timer interrupt handlers from running
		clrf	TRSS_RUN
;	Turn off all thrusters
		clrf	LATC
;	Set the Thruster Program State to Unitialized
		movlw	TCBST_UNIT
		movwf	TCB_STATE
		clrf	TCB_CNT
;	Reinitialize the thruster timer
		call	Timer0Init
;	Reinitialize Sync interrupts
		call	SyncInit
;	Allow Sync and Timer interrupt handlers to run
		movlw	0xFF
		movwf	TRSS_RUN
;Rtn: Restore the WREG
		movf	POSTDEC0,W
;	return
		return

; *****************************************************************************
; *****************************************************************************
; *****************************************************************************
;	include Alyssa's programs
; *****************************************************************************
; *****************************************************************************
; *****************************************************************************
	#include	NewThrusterValues.inc

;End of program
;******************************************************************************
 PAGE
 		SUBTITLE "Fake Test data from the I2C port"
I2cFakeB 	org	0x1000	;for multibyte integers, low order byte comes first
;	Specify 1st faked command
		dw	2,9,0
UsbFakeE 	dw	0

 PAGE
;******************************************************************************
;******************************************************************************
;******************************************************************************
		SUBTITLE "EEPROM definition section"
;******************************************************************************
;******************************************************************************
;******************************************************************************
			org	0xF00000 	
;	The absolue address F00000h is mapped to the 0000 location of
;	EE data memory for PIC18 devices.
;
;	Locations 0x00-3F of the emulated I2C attached EEPROM is contains the Plug
;	and Play device description. These locations are mapped to locations 
;	00-3F of the PIC's internal EEPROM.
;
;	Locations 00-07 - PnP Version 
eeVersion	de	"V1.0\x00\x00\x00\x00"
;	Locations 08-0F - Organization
eeOrg		de	"SNAPS\x00\x00\x00"
;	Locations 10-17 - Sensor/Device
eeDevice	de	"TPOD4001"	;Quad Thruster Pod Model 001
;	Locations 18-1B - Thruster Ticks per second (32-bits)
eeTickRt	de	low(3906),high(3906),upper(3906),0
;	Location 1C-1D 	-	Thruster configuration (See definition of TCB_CNFG.)
eeConfig	de	TCBC_MTH1+TCBC_MTH2+TCBC_MTH3+TCBC_MTH4,0
;	Locations 1E-1F - Unused
			de	0,0
;	Locations 20-23 - Thruster #1 Calibration value in Newtons (32-bits)
eeCal1		de	0,0,0,0	
;	Locations 24-27 - Thruster #2 Calibration value in Newtons (32-bits)
eeCal2		de	0,0,0,0	
;	Locations 28-2B - Thruster #3 Calibration value in Newtons (32-bits)
eeCal3		de	0,0,0,0	
;	Locations 2C-2F - Thruster #4 Calibration value in Newtons (32-bits)
eeCal4		de	0,0,0,0	
;	Locations 30-3F	Unused, set to zeros
			de	0,0,0,0,0,0,0,0
			de	0,0,0,0,0,0,0,0
;	End of the mapped PnP Block
; *****************************************************************************
;	Start Configuration Variables
eeI2CDevAdr	de	0x02			;I2C device 1, the PIC will answer to this addr
eeI2CMask	de	0xFE			;I2C device mask, all bits will be compared

		END
