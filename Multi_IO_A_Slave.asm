;--------------------------------------------------------------------------------------------------
; Project:  Multi-IO Board x Slave PIC
; Date:     4/02/15
; Revision: See Revision History notes below.
;
; Overview:
;
; This program runs on the Slave PICs on a Multi-IO Board Configuration x. Currently, it should
; work for any of the systems (Longitudinal, Transverse, Wall) as it captures peaks for all
; channels as well as mapping data.
;
; The Slave PICs communicate with the Master PIC via the I2C bus.
;
; The system clock is configured to use an external 4 Mhz source which is boosted to 16 MHz by the
; PLL. The CLKOUT pin outputs a clock at Fosc/4 which drives the input clock of the next Slave PIC.
; Each Slave PIC uses the PLL to generate a 16 MHz from the 4 Mhz input supplied by the Slave which
; precedes it in the daisy-chain. The CLKOUT pin of each Slave PIC drives the next Slave PIC and
; so forth. Each PIC's CLKOUT output is FOSC/4 into the next PIC, so each must use its PLL to run
; its system clock at 16 MHz.
;
; The 16 MHz is used since the Slave PICs run their A/D convertors at Fosc/16 to achieve a sample
; period of 1 us. This is the fastest specified A/D conversion rate.
;
; On half of the Slaves, an I/O pin is connected to a digital pot's I2C address configuration
; pins. To address the pot, the pin must be set ???. The Master PIC sends a command to the
; appropriate PIC so that it will set the pin ???, thus enabling the pot connected to that PIC
; for access by the Master PIC via the I2C bus.
;
; Those PICs which are not connected to digital pots still have the code in place to control the
; pin by command from the Master PIC. The output can be used as a debugging tool.
;
; NOTE: This method of enabling the digital pots was put in place as it was mistakenly assumed
; that there would not be enough address space for all the pots on the I2C bus. It turns out that
; there are only 4 such pots required as each pot handles two channels. Even with 8 pots, there
; would have been enough addresses as there are three address configuration inputs on each pot. The
; PIC chips occupy a different address space as the upper nibble of their address can be set to
; anything, unlike the pots which have a fixed upper nibble.
;
; Sync & Sync Reset I/O Configuration
;
; NOTE: The sync input is defined as SYNCT in this program as SYNC is already a pre-defined term.
;
;   Master PIC
;
;    Sync Reset connected to RA0 and RA5, either pin can be used to monitor the signal.
;    RA0 input allows Interrupt on Change (IOC) of the signal.
;    RA5 input allows both IOC and tracking by counter Timer1.
;
;    As RA5 also provides the IOC option, the connection to RA0 is only to maintain design
;    consistency with the Slave PICs which can only use RA0 as they must use RA5 as the system
;    clock input.
;
;    Normally, the Master PIC monitors the Sync Reset RA0 using the IOC option. The Sync line is
;    driven as an output on RC5 to the Slave PICs while RA1 is set to an input to avoid conflict.
;
;   Slave PICs
;
;    Sync Reset is monitored via RA0 using the IOC option. The Sync line is monitored via RC5
;    configured to count pulses with Timer 0. The Sync Reset pulse triggers a reset of the counter.
;
;--------------------------------------------------------------------------------------------------
; Notes on PCLATH
;
; The program counter (PC) is 13 bits. The lower 8 bits can be read and written as register PCL.
; The upper bits cannot be directly read or written.
;
; When the PCL register is written, PCLATH<4:0> is copied at the same time to the upper 5 bits of
; PC.
;
; When a goto is executed, 11 bits embedded into the goto instruction are loaded into the PC<10:0>
; while bits PCLATH<4:3> are copied to bits PC<12:11>
;
; Changing PCLATH does NOT instantly change the PC register. The PCLATH will be used the next time
; a goto is executed (or similar opcode) or the PCL register is written to. Thus, to jump farther
; than the 11 bits (2047 bytes) in the goto opcode will allow, the PCLATH register is adjusted
; first and then the goto executed.
;
;--------------------------------------------------------------------------------------------------
;
; Revision History:
;
; 1.0   Initial code.
;
;
;--------------------------------------------------------------------------------------------------
; Miscellaneous Notes
;
; incf vs decf rollover
;
; When incrementing multi-byte values, incf can be used because it sets the Z flag - then the Z
; flag is set, the next byte up should then be incremented.
; When decrementing multi-byte values, decf CANNOT be used because it sets the Z flag but NOT the
; C flag.  The next byte up is not decremented when the lower byte reaches zero, but when it rolls
; under zero.  This can be caught by loading w with 1 and then using subwf and catching the C flag
; cleared. (C flag is set for a roll-over with addwf, cleared for roll-under for subwf.
; For a quickie but not perfect count down of a two byte variable, decf and the Z flag can be used
; but the upper byte will be decremented one count too early.
;
;--------------------------------------------------------------------------------------------------
; Operational Notes
;
;
;--------------------------------------------------------------------------------------------------
; Hardware Control Description
;
; Function by Pin
;
; Port A        Pin/Options/Selected Option/Description  (only the most common options are listed)
;
; RA0   I/*,IOC,USB-D+                  ~ I ~ Sync Reset
; RA1   I/*,IOC,USB-D-                  ~ I ~ not used, tied to Sync on RC5
; RA2   not implemented in PIC16f1459
; RA3   I/*,IOC,T1G,MSSP-SS,Vpp,MCLR    ~ Vpp
; RA4   I/O,IOC,T1G,CLKOUT,CLKR, AN3    ~ CLKOUT
; RA5   I/O,IOC,T1CKI,CLKIN             ~ CLKIN
; RA6   not implemented in PIC16f1459
; RA7   not implemented in PIC16f1459
;
; Port B        Pin/Options/Selected Option/Description  (only the most common options are listed)
;
; RB0   not implemented in PIC16f1459
; RB1   not implemented in PIC16f1459
; RB2   not implemented in PIC16f1459
; RB3   not implemented in PIC16f1459
; RB4   I/O,IOC,MSSP-SDA/SDI,AN10       ~ I ~ I2CSDA, I2C bus data line to slaves
; RB5   I/O,IOC,EUSART-RX/DX,AN11       ~ O ~ digital pot select on 1/2 of slaves, NC on others
; RB6   I/O,IOC,MSSP-SCL/SCK            ~ I ~ I2CSCL, I2C bus clock line to slaves
; RB7   I/O,IOC,EUSART-TX/CK            ~ 0 ~ no connection
;
; Port C        Pin/Options/Selected Option/Description  (only the most common options are listed)
;
; RC0   I/O,AN4,C1/2IN+,ICSPDAT,Vref    ~ ICSPDAT ~ in circuit programming data line
; RC1   I/O,AN5,C1/2IN1-,ICSPCLK,INT    ~ ICSPCLK ~ in circuit programming clock line
; RC2   I/O,AN6,C1/2IN2-,DACOUT1        ~ I ~ slave I2C bus address bit 0
; RC3   I/O,AN7,C1/2IN3-,DACOUT2,CLKR   ~ I ~ slave I2C bus address bit 1
; RC4   I/O,C1/2OUT                     ~ I ~ slave I2C bus address bit 2
; RC5   I/O,T0CKI,PWM1                  ~ TOCKI ~ sync from master
; RC6   I/O,AN8,PWM2,MSSP-SS            ~ O ~ no connection
; RC7   I/O,AN9,MSSP-SDO                ~ AN9 ~ A/D converter input
;
;end of Hardware Control Description
;--------------------------------------------------------------------------------------------------
;
; User Inputs
;
;
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Defines
;

; COMMENT OUT "#define debug" line before using code in system.
; Defining debug will insert code which simplifies simulation by skipping code which waits on
; stimulus and performing various other actions which make the simulation run properly.
; Search for "ifdef debug" to find all examples of such code.

;#define debug 1     ; set debug testing "on"

; version of this software

SOFTWARE_VERSION_MSB    EQU 0x01
SOFTWARE_VERSION_LSB    EQU 0x01

; upper nibble of I2C address for all Slave PICs is 1110b
; 3 lsbs will be set to match the address inputs on each Slave PIC
; note that 1111b upper nibble has special meaning in 10 bit address mode, so it is avoided here

I2C_SLAVE_ADDR_UPPER            EQU     b'11100000'

; Master PIC to Slave PIC Commands -- sent by Master to Slaves via I2C to trigger actions

PIC_NO_ACTION_CMD               EQU 0x00
PIC_GET_ALL_STATUS_CMD          EQU 0x01
PIC_START_CMD                   EQU 0x02
PIC_GET_PEAK_PKT_CMD            EQU 0X03
PIC_ENABLE_POT_CMD              EQU 0x04
PIC_DISABLE_POT_CMD             EQU 0x05

I2C_RCV_BUF_LEN      EQU .5

I2C_XMT_BUF_LEN      EQU .20

; end of Defines
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Configurations, etc. for the Assembler Tools and the PIC

	LIST p = PIC16F1459	;select the processor

    errorlevel  -306 ; Suppresses Message[306] Crossing page boundary -- ensure page bits are set.

    errorLevel  -302 ; Suppresses Message[302] Register in operand not in bank 0.

	errorLevel	-202 ; Suppresses Message[205] Argument out of range. Least significant bits used.
					 ;	(this is displayed when a RAM address above bank 1 is used -- it is
					 ;	 expected that the lower bits will be used as the lower address bits)

#INCLUDE <p16f1459.inc> 		; Microchip Device Header File


;#include <xc.h>

; Specify Device Configuration Bits

; CONFIG1
; __config 0xF9E4
 __CONFIG _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _BOREN_OFF & _CLKOUTEN_ON & _IESO_OFF & _FCMEN_OFF
; CONFIG2
; __config 0xFFFF
 __CONFIG _CONFIG2, _WRT_ALL & _CPUDIV_NOCLKDIV & _USBLSCLK_48MHz & _PLLMULT_4x & _PLLEN_DISABLED & _STVREN_ON & _BORV_LO & _LPBOR_OFF & _LVP_OFF

; _FOSC_INTOSC -> internal oscillator, I/O function on OSC1/CLKIN pin
; _WDTE_OFF -> watch dog timer disabled
; _PWRTE_OFF -> Power Up Timer disabled
; _MCLRE_ON -> MCLR/VPP pin is Master Clear with weak pull-up automatically enabled
; _CP_OFF -> Flash Program Memory Code Protection off
; _BOREN_OFF -> Power Brown-out Reset off
; _CLKOUTEN_ON -> CLKOUT function on, Fosc/4 -> CLKOUT pin
; _IESO_OFF -> Internal/External Oscillator Switchover off
;   (not used for this application since there is no external clock)
; _FCMEN_OFF -> Fail-Safe Clock Monitor off
;   (not used for this application since there is no external clock)
; _WRT_ALL -> Flash Memory Self-Write Protection on -- no writing to flash
;
; _CPUDIV_NOCLKDIV -> CPU clock not divided
; _USBLSCLK_48MHz -> only used for USB operation
; _PLLMULT_4x -> sets PLL (if enabled) multiplier -- 4x allows software override
; _PLLEN_DISABLED -> the clock frequency multiplier is not used
;
; _STVREN_ON -> Stack Overflow/Underflow Reset on
; _BORV_LO -> Brown-out Reset Voltage Selection -- low trip point
; _LPBOR_OFF -> Low-Power Brown-out Reset Off
; _LVP_OFF -> Low Voltage Programming off
;
; end of configurations
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Hardware Definitions
;
; Note regarding Port Latches vs Pins
;
; Writing to individial port pins can cause problems because that uses a read-modify-write
; operation. If the pins are read, some outputs could read differently than they are programmed in
; the port latch if the external hardware is causing the outputs to change state slowly. Thus,
; when the read-modify-write operation writes back to the latch, the wrong value for some pins
; might be permanently written.
;
; To avoid this, such operations should be done to the port latches (LATA, LATB, LATC) instead of
; the port (PORTA, PORTB, PORTC). This ensures that the values read are what have actually been
; programmed.
;
; For convenience, the port defines below are set to the Port when the associated signal is an
; input and to the Latch when the associated signal is an output so that the proper register will
; be used for each case.
;

; Port A defines

SYNC_RESET      EQU 0           ; input on RA0
SYNCT_RA1       EQU 1           ; not used, must be input, same signal as RC5
RA2             EQU 2           ; not implemented in PIC16f1459
RA3             EQU 3           ; Vpp ~ not used as I/O
RA4             EQU 4           ; CLKOUT ~ not used as I/O
RA5             EQU 5           ; CLKIN ~ not used as I/O

SYNC_RESET_RD   EQU PORTA

; Port B defines

RB0             EQU 0           ; not implemented in PIC16f1459
RB1             EQU 1           ; not implemented in PIC16f1459
RB2             EQU 2           ; not implemented in PIC16f1459
RB3             EQU 3           ; not implemented in PIC16f1459
I2CSDA          EQU 4           ; I2C bus SDA line
DIG_POT_EN      EQU 5           ; digital pot enable
I2CSCL          EQU 6           ; I2C bus SCL line
RB7             EQU 7           ; not used, set as input with weak pullup

DIG_POT_EN_WR   EQU LATB

; Port C defines

RC0             EQU 0           ; ICSPDAT ~ not used as I/O
RC1             EQU 1           ; ICSPCLK ~ not used as I/O
I2C_ADDR0       EQU 2           ; slave I2C address bit 0
I2C_ADDR1       EQU 3           ; slave I2C address bit 1
I2C_ADDR2       EQU 4           ; slave I2C address bit 2
SYNCT           EQU 5           ; Sync input from master
RC6             EQU 6           ; not used, set as output
AD_AN9          EQU 7           ; AN9 ~ A/D converter input, set as input

I2C_ADDR_RD     EQU PORTC
SYNCT_RD        EQU PORTC

; end of Hardware Definitions
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Software Definitions

; bits in flags variable

STARTED             EQU 0     ; normal processing code has started
UNUSED_FLAG1        EQU 1     ;

; bits in statusFlags variable

COM_ERROR           EQU 0
AD_OVERRUN          EQU 1

; values for A/D register ADCON0 to select channel and enable A/D

; ADCON0 bits
; bit 7 = 0 : unimplemented
; bit 6 = 0 : bits 6-2 : analog input channel
; bit 5 = 1 :    01001 -> AN9
; bit 4 = 0 :
; bit 3 = 0 :
; bit 2 = 1 :
; bit 1 = 0 : Go|/Done
; bit 0 = 1 : ADC is enabled

; A/D convertor selection for analog input channel/pin: AN9 on pin RC7

AD_START_CODE    EQU     b'00100101'

; end of Software Definitions
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Variables in RAM
;
; Note that you cannot use a lot of the data definition directives for RAM space (such as DB)
; unless you are compiling object files and using a linker command file.  The cblock directive is
; commonly used to reserve RAM space or Code space when producing "absolute" code, as is done here.
; 

; Assign variables in RAM - Bank 0 - must set BSR to 0 to access
; Bank 0 has 80 bytes of free space

 cblock 0x20                ; starting address

    flags                   ; bit 0: 0 =
                            ; bit 1: 0 =
                            ; bit 2: 0 = 
                            ; bit 3: 0 = 
                            ; bit 4: 0 = 
                            ; bit 5: 0 = 
							; bit 6: 0 = 
							; bit 7: 0 = 

    statusFlags             ; bit 0: 0 = one or more communication errors have occurred
                            ; bit 1: 0 = an A/D buffer overrun has occurred
                            ; bit 2: 0 =
                            ; bit 3: 0 =
                            ; bit 4: 0 =
                            ; bit 5: 0 =
							; bit 6: 0 =
							; bit 7: 0 =

    slaveI2CAddress

    comErrorCnt             ; tracks the number of communication errors

    maxADBufCnt             ; maximum number of A/D values stored in buffer at any one time
                            ; should never be bigger than the size of the buffer, otherwise it
                            ; is an overrun condition

    adValue                 ; last A/D conversion value

    adTrigger               ; 0 = not ready; 1 = handle A/D input
                            ; set by Timer0 interrupt to trigger the next A/D conversion
                            ; cleared by handleADToLEDArrays
                            ; an entire byte is used to avoid read-modify-write issues between the
                            ; main thread and the interrupt thread
    masterCmd               ; command byte received from Master via I2C bus

    scratch0                ; these can be used by any function
    scratch1
    scratch2
    scratch3
    scratch4
    scratch5
    scratch6
    scratch7
    scratch8
    scratch9
    scratch10

    i2cXmtBufPtrH
    i2cXmtBufPtrL
    i2cXmtBuf:I2C_XMT_BUF_LEN

    i2cRcvBufPtrH
    i2cRcvBufPtrL
    i2cRcvBuf:I2C_RCV_BUF_LEN

 endc

;-----------------

; Assign variables in RAM - Bank 1 - must set BSR to 1 to access
; Bank 1 has 80 bytes of free space

 cblock 0xa0                ; starting address


 endc

;-----------------

; Assign variables in RAM - Bank 3 - must set BSR to 3 to access
; Bank 2 has 80 bytes of free space

 cblock 0x120                ; starting address

	block1PlaceHolder

 endc

;-----------------

; NOTE: use of this block by interrupts is not necessary with the PIC16f1459 as it pushes critical
; registers onto a stack.
;
; Define variables in the memory which is mirrored in all 4 RAM banks.  This area is usually used
; by the interrupt routine for saving register states because there is no need to worry about
; which bank is current when the interrupt is invoked.
; On the PIC16F628A, 0x70 thru 0x7f is mirrored in all 4 RAM banks.

; NOTE:
; This block cannot be used in ANY bank other than by the interrupt routine.
; The mirrored sections:
;
;	Bank 0		Bank 1		Bank 2		Bank3
;	70h-7fh		f0h-ffh		170h-17fh	1f0h-1ffh
;

 cblock	0x70
    W_TEMP
    FSR0L_TEMP
    FSR0H_TEMP
    STATUS_TEMP
    PCLATH_TEMP	
 endc

; end of Variables in RAM
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Power On and Reset Vectors
;

	org 0x00                ; Start of Program Memory

	goto start              ; jump to main code section
	nop			            ; Pad out so interrupt
	nop			            ; service routine gets
	nop			            ; put at address 0x0004.

; interrupt vector at 0x0004

    goto 	handleInterrupt	; points to interrupt service routine

; end of Reset Vectors
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; start
;

start:

    call    setup               ; preset variables and configure hardware

;debug mks

    banksel i2cXmtBufPtrL

    movlw   0xaa
    movwf   i2cXmtBufPtrL

    movlw   0x55
    movwf   i2cRcvBufPtrH

;debug mks end


mainLoop:

    call    handleI2CCommand    ; checks for incoming command on I2C bus

    banksel flags

;    btfsc   flags,HANDLE_LED_ARRAYS ; display A/D inputs on LED arrays
;    call    handleADToLEDArrays

    goto    mainLoop
    
; end of start
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setup
;
; Presets variables and configures hardware.
;

setup:

    banksel flags
    clrf    flags
    clrf    statusFlags
    clrf    comErrorCnt

    call    setupClock      ; set system clock source and frequency

    call    setupPortA      ; prepare Port A for I/O

    call    setupPortB      ; prepare Port B for I/O

    call    setupPortC      ; prepare Port C  for I/O

    call    initializeOutputs

    call    parseSlaveIC2Address

    call    setupI2CSlave7BitMode ; prepare the I2C serial bus for use

    call    setupADConverter ; prepare A/D converter for use

;start of hardware configuration

    clrf   FSR0H            ;high byte of indirect addressing pointers -> 0
    clrf   FSR1H

    clrf    INTCON          ; disable all interrupts

    banksel OPTION_REG
    movlw   0x57
    movwf   OPTION_REG      ; Option Register = 0x57   0101 0111 b
                            ; bit 7 = 0 : weak pull-ups are enabled by individual port latch values
                            ; bit 6 = 1 : interrupt on rising edge
                            ; bit 5 = 0 : TOCS ~ Timer 0 run by internal instruction cycle clock (CLKOUT ~ Fosc/4)
                            ; bit 4 = 1 : TOSE ~ Timer 0 increment on high-to-low transition on RA4/T0CKI/CMP2 pin (not applicable here)
							; bit 3 = 0 : PSA ~ Prescaler assigned to Timer0
                            ; bit 2 = 1 : Bits 2:0 control prescaler:
                            ; bit 1 = 1 :    111 = 1:256 scaling for Timer0 (if assigned to Timer0)
                            ; bit 0 = 1 :
    
;end of hardware configuration
	
; enable the interrupts

    bsf     INTCON,PEIE     ; enable peripheral interrupts (Timer0 is a peripheral)
    bsf     INTCON,T0IE     ; enable TMR0 interrupts
    bsf     INTCON,GIE      ; enable all interrupts

    return

; end of setup
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; parseSlaveIC2Address
;
; Determines the PIC's I2C slave address by reading the address input bits which identify this
; PIC's address . Each Slave PIC's three address inputs are tied uniquely high/low.
;
; The value is stored in slaveI2CAddress variable.
; The address for the I2C module is NOT set...that is done when that module is set up.
;

parseSlaveIC2Address:

    banksel slaveI2CAddress
    clrf    slaveI2CAddress

    ; set each of the 3 lsbs to match I/O input pins

    banksel I2C_ADDR_RD

    btfsc   I2C_ADDR_RD, I2C_ADDR0
    bsf     slaveI2CAddress,0
    btfsc   I2C_ADDR_RD, I2C_ADDR1
    bsf     slaveI2CAddress,1
    btfsc   I2C_ADDR_RD, I2C_ADDR2
    bsf     slaveI2CAddress,2

    return

; end of parseSlaveIC2Address
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; initializeOutputs
;
; Initializes all outputs to known values.
;

initializeOutputs:

    banksel DIG_POT_EN_WR
    bcf DIG_POT_EN_WR, DIG_POT_EN           ; disable the digital pot connected to this pin

    return

; end of initializeOutpus
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupClock
;
; Sets up the system clock source and frequency.
;
; Assumes clock related configuration bits are set as follows:
;
;   _FOSC_INTOSC,  _CPUDIV_NOCLKDIV, _PLLMULT_4x, _PLLEN_DISABLED
;
; Assumes all programmable clock related options are at Reset default values.
;

setupClock:

    ; choose internal clock frequency of 16 Mhz

    banksel OSCCON

    bsf     OSCCON, IRCF0
    bsf     OSCCON, IRCF1
    bsf     OSCCON, IRCF2
    bsf     OSCCON, IRCF3

    return

; end of setupClock
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupPortA
;
; Sets up Port A for I/O operation.
;
; NOTE: Writing to PORTA is same as writing to LATA for PIC16f1459. The code example from the
; data manual writes to both -- probably to be compatible with other PIC chips.
;
; NOTE: RA0, RA1 and RA3 can only be inputs on the PIC16f1459 device. 
;       RA2, RA6, RA7 are not implemented.
;

setupPortA:

    ; start with known safe configuration

    banksel PORTA                       ; init port value
    clrf    PORTA

    banksel LATA                        ; init port data latch (same as writing to PORTA)
    clrf    LATA

    banksel WPUA                        ; disable all weak pullups
    clrf    WPUA

    banksel ANSELA                      ; all pins digital I/O
    clrf    ANSELA

    banksel TRISA                       ; all pins digital inputs
    movlw   b'11111111'
    movwf   TRISA

    ; customize I/O directions
    ; some pins configured further by other functions for setup of serial port, I2C, ADC, etc.

    bsf     TRISA, SYNC_RESET           ; input

    return

; end of setupPortA
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupPortB
;
; Sets up Port B for I/O operation.
;
; NOTE: Writing to PORTB is same as writing to LATB for PIC16f1459. The code example from the
; data manual writes to both -- probably to be compatible with other PIC chips.
;
; NOTE: RB0, RB1, RB2, RB3 are not implemented on the PIC16f1459 device.
;

setupPortB:

    ; start with known safe configuration

    banksel PORTB                       ; init port value
    clrf    PORTB

    banksel LATB                        ; init port data latch (same as writing to PORTA)
    clrf    LATB

    banksel WPUB                        ; disable all weak pullups
    clrf    WPUB

    banksel ANSELB                      ; all pins digital I/O
    clrf    ANSELB

    banksel TRISB                       ; all pins digital inputs
    movlw   b'11111111'
    movwf   TRISB

    ; customize I/O directions
    ; some pins configured further by other functions for setup of serial port, I2C, ADC, etc.

    bcf     TRISB, DIG_POT_EN           ; output ~ digital pot enable
    bcf     TRISB, RB7                  ; output ~ unused, set as output to prevent floating

    return

; end of setupPortB
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupPortC
;
; Sets up Port C for I/O operation.
;
; NOTE: Writing to PORTC is same as writing to LATC for PIC16f1459. The code example from the
; data manual writes to both -- probably to be compatible with other PIC chips.
;

setupPortC:

    ; start with known safe configuration
    ; Port C does not have a weak pull-up register

    banksel PORTC                       ; init port value
    clrf    PORTC

    banksel LATC                        ; init port data latch (same as writing to PORTA)
    clrf    LATC

    banksel ANSELC                      ; all pins digital I/O
    clrf    ANSELC

    banksel TRISC                       ; all pins digital inputs
    movlw   b'11111111'
    movwf   TRISC

    ; customize I/O directions
    ; some pins configured further by other functions for setup of serial port, I2C, ADC, etc.

    bsf     TRISC, I2C_ADDR0            ; input  ~ slave I2C bus address bit 0
    bsf     TRISC, I2C_ADDR1            ; input  ~ slave I2C bus address bit 1
    bsf     TRISC, I2C_ADDR2            ; input  ~ slave I2C bus address bit 2
    bsf     TRISC, SYNCT                ; input  ~ from master for circumferential clock position
    bcf     TRISC, RC6                  ; output ~ unused, set as output to prevent floating
    bsf     TRISC, AD_AN9               ; input  ~ A/D converter analog input

    return

; end of setupPortC
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupI2CSlave7BitMode
;
; Sets the MASTER SYNCHRONOUS SERIAL PORT (MSSP) MODULE to the I2C Slave mode using the 7 bit
; address mode.
;

setupI2CSlave7BitMode:

    banksel TRISB
    bsf TRISB, TRISB4       ; set RB4/I2CSDA to input
    bsf TRISB, TRISB6       ; set RB6/I2CSCL to input

    movlw   I2C_SLAVE_ADDR_UPPER  ; set the I2C slave device address (only upper nibble)
    banksel SSP1ADD
    movwf   SSP1ADD

    ; copy the address input bits which identify this PIC's address on the I2C bus to the
    ; 3 lsbs of the Slave Address register (bit 0 is not used, 7 bit address starts at bit 1)
    ; each Slave PIC's three address inputs are tied uniquely high/low


    banksel I2C_ADDR_RD
    movf    I2C_ADDR_RD, W

    banksel SSP1ADD
    btfsc   WREG, I2C_ADDR0
    bsf     SSP1ADD,1
    btfsc   WREG, I2C_ADDR1
    bsf     SSP1ADD,2
    btfsc   WREG, I2C_ADDR2
    bsf     SSP1ADD,3

    banksel SSP1MSK
    movlw   0xff            ; bits <7:1> of SSP1ADD are used to match address
    movwf   SSP1MSK         ; (bit <0> is ignored in 7 bit address mode)

    banksel SSP1CON2        ; enable clock stretching -- upon receiving a byte this PIC will
    bsf     SSP1CON2,SEN    ; hold clock and halt further transmissions until CKP bit cleared

    banksel SSP1CON1
    bcf     SSP1CON1,SSP1M0	; SSPM = b0110 ~ I2C Slave mode
    bsf     SSP1CON1,SSP1M1
    bsf     SSP1CON1,SSP1M2
    bcf     SSP1CON1,SSP1M3

    bsf	SSP1CON1,SSPEN		;enables the MSSP module

    return

; end setupI2CSlave7BitMode
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupADConverter
;
; Sets up the A/D converter.
;

setupADConverter:

    ;This code block configures the ADC for polling, Vdd and Vss references,
    ; FOSC/16 clock

    ; set ADCON1 to configure the A/D converter
    ; bit 7 = 0 : left justify the result in ADRESH:ADRESL
    ; bit 6 = 1 : bits 6-4 : A/D Conversion Clock Select bits
    ; bit 5 = 0 :    101 -> FOSC/16
    ; bit 4 = 1 : 
    ; bit 3 = 0 : unused
    ; bit 2 = 0 : unused
    ; bit 1 = 0 : bits 1-0: A/D voltage reference source
    ; bit 0 = 0 :    00 -> VREF+ connected to VDD

    banksel ADCON1
    movlw   b'01010000'
    movwf   ADCON1

    ; input channel/pin: AN9/RC7

    banksel TRISA
    bsf     TRISA, AD_AN9                       ;set I/O pin to input

    banksel ANSELA
    bsf     ANSELA, AD_AN9                      ;set I/O pin to analog mode

    ; turn on A/D module and begin conversion

    banksel ADCON0
    movlw   AD_START_CODE
    movwf   ADCON0

    return

; end of setupADConverter
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; doADConversion
;
; Starts the A/D convertor, waits until it finishes, then processes the value. This method is a
; simple, crude, way of obtaining an A/D value. It will only execute when the adTrigger:0 flag has
; been set. This is usually done by a timer interrupt.
;
; If the adTrigger flag has not been set by a Timerx interrupt, function returns immediately.
; If flag is set, it is cleared and the A/D processed.
;
; The frequency of the adTrigger flag being set should be slow enough to allow the sampling circuit
; to settle between processing.
;

doADConversion:

    ; do nothing if adTrigger bit 0 is not set

    banksel adTrigger
    btfss   adTrigger,0
    return

    clrf    adTrigger           ; clear flag so interrupt routine can set it again next period

    goto    getADValue

; end of doADConversion
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; getADValue
;
; Starts the A/D convertor, waits until it finishes, then processes the value. This method is a
; simple, crude, way of obtaining an A/D value.
;

getADValue:

    banksel ADCON0
    bsf     ADCON0,ADGO                 ;start conversion

hcatl1:
    btfsc   ADCON0,ADGO                 ;loop until conversion done
    goto    hcatl1

    ; Store the value from ADRESH to scratch0
    ; It is assumed that the A/D value is left justified, thus the upper 8 bits will be used
    ; and the lower 2 bits will be discarded.

    banksel ADRESH              ;read upper 8 bits of result
    movf    ADRESH,W            ; (result is left justified so this gets
                                ;  the upper 8 bits, ignoring the lsbs)

    banksel scratch0
    movwf   scratch0            ; store the A/D value

    return

; end of getADValue
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; getAllStatus
;
; Transmits various status, flags, and other values back to the Master via I2C. Clock stretching is
; enabled so the bus will wait as necessary between each byte.
;
; The communication error count is cleared.
; The maximum number of bytes in the A/D buffer variable is cleared.
;
; This function is done in the main code and not in interrupt code. It is expected that the A/D
; converter interrupt will occur one or more times during transmission of the peak data.
;

getAllStatus:

    ; set buffer pointer to transmit buffer start
    banksel i2cXmtBuf
    movlw   high i2cXmtBuf
    movwf   i2cXmtBufPtrH
    movlw   i2cXmtBuf
    movwf   i2cXmtBufPtrL

    movf    i2cXmtBufPtrH, W            ; load FSR0 with buffer pointer
    movwf   FSR0H
    movf    i2cXmtBufPtrL, W
    movwf   FSR0L

    ; store various status values in the transmit buffer

    banksel flags

    ;debug mks

    movlw   0x44
    movwf   statusFlags
    movlw   0x45
    movwf   comErrorCnt
    movlw   0x46
    movwf   maxADBufCnt
    movlw   0x47
    movwf   adValue

    ;debug mks end

    movf    slaveI2CAddress,W
    movwi   FSR0++

    movlw   SOFTWARE_VERSION_MSB
    movwi   FSR0++

    movlw   SOFTWARE_VERSION_LSB
    movwi   FSR0++

    movf    flags,W
    movwi   FSR0++

    movf    statusFlags,W
    movwi   FSR0++

    movf    comErrorCnt,W
    movwi   FSR0++

    movf    maxADBufCnt,W
    movwi   FSR0++

    movf    adValue,W
    movwi   FSR0++

    movlw   0x55                        ; unused -- for future use
    movwi   FSR0++

    movlw   0xaa                        ; unused -- for future use
    movwi   FSR0++

    movlw   0x5a                        ; unused -- for future use
    movwi   FSR0++

    movlw   .11                         ; number of data bytes in packet
    movwf   scratch0

    call    calcAndStoreCheckSumForI2CXmtBuf

    ; clear appropriate error counts, flags, etc. to start tracking anew

    banksel flags
    clrf    comErrorCnt
    clrf    maxADBufCnt
    bcf     statusFlags,COM_ERROR
    bcf     statusFlags,AD_OVERRUN

    goto sendI2CBuffer

; end getAllStatus
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; calcAndStoreCheckSumForI2CXmtBuf
;
; Calculates the checksum for a series of bytes in the i2cXmtBuf buffer, the address of which
; should be in i2cXmtBufPtrH:L
;
; On Entry:
;
; scratch0 contains number of bytes in series
; i2cXmtBufPtrH:i2cXmtBufPtrL contains address of the start of the buffer
;
; On Exit:
;
; The checksum will be stored at the end of the series.
; FSR0 points to the location after the checksum.
;

calcAndStoreCheckSumForI2CXmtBuf:

    banksel i2cXmtBufPtrH                   ; load FSR0 with buffer pointer
    movf    i2cXmtBufPtrH, W            
    movwf   FSR0H
    movf    i2cXmtBufPtrL, W
    movwf   FSR0L

    goto    calculateAndStoreCheckSum

; end calcAndStoreCheckSumForI2CXmtBuf
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; calculateAndStoreCheckSum
;
; Calculates the checksum for a series of bytes.
;
; On Entry:
;
; scratch0 contains number of bytes in series
; FSR0 points to first byte in series.
;
; On Exit:
;
; The checksum will be stored at the end of the series.
; FSR0 points to the location after the checksum.
;

calculateAndStoreCheckSum:

    call    sumSeries                       ; add all bytes in the buffer

    comf    WREG,W                          ; use two's complement to get checksum value
    addlw   .1

    movwi   FSR0++                          ; store the checksum at the end of the summed series

    return

; end calculateAndStoreCheckSum
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; sumSeries
;
; Calculates the sum of a series of bytes. Only the least significant byte of the sum is retained.
;
; On Entry:
;
; scratch0 contains number of bytes in series.
; FSR0 points to first byte in series.
;
; On Exit:
;
; The least significant byte of the sum will be returned in WREG.
; Z flag will be set if the LSB of the sum is zero.
; FSR0 points to the location after the last byte summed.
;

sumSeries:

    banksel scratch0

    clrf    WREG

sumSLoop:                       ; sum the series

    addwf   INDF0,W
    addfsr  INDF0,1

    decfsz  scratch0,F
    goto    sumSLoop

    return

; end sumSeries
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; sendI2CBuffer
;
; Sends bytes from the buffer pointed to by i2cXmtBufPtrH:L through the I2C bus. The number of bytes
; sent will be controlled by the Master. If the Master requests more bytes than have been loaded
; into the buffer, the extra bytes sent will be whatever happens to be after the last valid byte
; in the buffer.
;
; On Entry:
;
; i2cXmtBufPtrH:i2cXmtBufPtrL contains address of the buffer to be tranw
;

sendI2CBuffer:

    banksel i2cXmtBuf
    movf    i2cXmtBufPtrH, W            ; load FSR0 with buffer pointer
    movwf   FSR0H
    movf    i2cXmtBufPtrL, W
    movwf   FSR0L

sIBXmtLoop:

    call    clearWCOL                   ; make sure error flag is reset to allow sending

    moviw   FSR0++                      ; send next byte in buffer
    movwf   SSP1BUF

    call    clearSSP1IF                 ; clear the I2C interrupt flag
    call    setCKP                      ; release the I2C clock line so master can send next byte

    call    waitForSSP1IFHighOrStop     ; wait for byte or stop condition to be received

    btfss   STATUS,Z
    goto    cleanUpI2CAndReturn          ; bail out when stop condition received

    banksel SSP1CON2
    btfss   SSP1CON2,ACKSTAT
    goto    sIBXmtLoop

    call    waitForSSP1IFHighOrStop     ; wait for byte or stop condition to be received
    goto    cleanUpI2CAndReturn         ; reset all status and error flags

; end sendI2CBuffer
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; getPeakPacket
;
; Places the current peak data in a buffer and transmits it back to the Master via I2C. Clock
; stretching is enabled so the bus will wait as necessary between each byte.
;
; This function is done in the main code and not in interrupt code. It is expected that the A/D
; converter interrupt will occur one or more times during transmission of the peak data.
;

getPeakPacket:



    return

; end getPeakPacket
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; enableDigitalPot
;
; Enables the digital pot connected to this PIC by setting the I/O pin connected to the pot's A2
; address line high. This makes the pot's 3 lsb address bits 100b so the Master can communicate
; with it at 1010100xb
;

enableDigitalPot:

    banksel DIG_POT_EN_WR
    bsf     DIG_POT_EN_WR, DIG_POT_EN

    return

; end enableDigitalPot
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; disableDigitalPot
;
; Disables the digital pot connected to this PIC by setting the I/O pin connected to the pot's A2
; address line low. This makes the pot's 3 lsb address bits 000b so it will ignore the Master
; when it addresses the lone enabled pot at address 1010100b.
;

disableDigitalPot:

    banksel DIG_POT_EN_WR
    bcf     DIG_POT_EN_WR, DIG_POT_EN

    return

; end disableDigitalPot
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleI2CCommand
;
; Checks to see if this PIC has received an ID byte via the I2C bus which matches its address.
;
; The I2C bus is set up to hold the clock and delay transmission by the master after each byte
; is received. That allows this method to be checked in the normal thread instead of by an
; interrupt as any response delays are not a problem as the transmission is halted.
;

handleI2CCommand:

    ; check if SSP1IF flag set -- if so, matching address byte has been received
    
    ifndef debug               ; pretend flag set if in debug mode
    banksel PIR1
    btfss   PIR1, SSP1IF
    return
    endif

    banksel SSP1BUF
    movf    SSP1BUF,W            ; get incoming value; clears BF flag

    call    clearSSP1IF

; jump to handle receive or transmit request
; if bit 0 of the address byte is 0, the master is sending and this PIC is receiving
; if bit 0 is 1, the master is receiving and this PIC is sending

    btfss   WREG,0
    goto    handleI2CReceive
    goto    handleI2CTransmit

    return

; end handleI2CCommand
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleI2CReceive
;
; Handles the receipt of data from the master on the I2C bus.
;
; The next byte read should be the command byte.
;

handleI2CReceive:

    call    clearSSP1IF                 ; clear the I2C interrupt flag
    call    setCKP                      ; release the I2C clock line so master can send next byte

    call    waitForSSP1IFHighOrStop     ; wait for byte or stop condition to be received

    btfss   STATUS,Z
    goto    cleanUpI2CAndReturn         ; bail out when stop condition received

    call    readI2CByteAndPrepForNext   ; get the byte just received

    banksel masterCmd
    movwf   masterCmd                   ; store the command byte

; parse the command byte by comparing with each command...if it is a command byte meant for use on
; the next write operation, it will have been stored in masterCmd but MAY not match any test here
; and will be ignored for the current operation but will be available for the next read operation,
; if addressing bytes are required for the next read, then the command byte must be caught here
; in order to jump to a function which will read and store those extra bytes

    movf    masterCmd,W
    sublw   PIC_START_CMD
    btfsc   STATUS,Z
    goto    handleHostStartCmd

    movf    masterCmd,W
    sublw   PIC_ENABLE_POT_CMD
    btfsc   STATUS,Z
    goto    enableDigitalPot

    movf    masterCmd,W
    sublw   PIC_DISABLE_POT_CMD
    btfsc   STATUS,Z
    goto    disableDigitalPot

    return

; end handleI2CReceive
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleI2CTransmit
;
; Handles the transmission of data to the master on the I2C bus.
;
; The command byte specifying what data should be sent should have already been received in the
; previous transmission from the master and stored in the masterCmd variable.
;
; I2C Read Protocol
;
; When reading some devices, the Master sends a command and/or address bytes with a write
; operation, performs a restart, sends a read command, then reads the data from the device. The
; only thing a restart does over a stop and start operation is the restart allows the Master to
; maintain control of the bus...another device cannot take it over.
;
; For the Multi-IO Board, there is only one master so it does not need to retain control. In order
; to read data, the Master writes a command and/or further addressing information to the device in
; the normal write protocol, ending with a stop signal. The Slave will save the information and
; will expect that information to apply to the next read operation. The Master will then send a
; normal read command to read the desired data. The Slave will know what information to send based
; on the command and/or addressing info sent in the previous write command.
;

handleI2CTransmit:

    ; parse the command byte received in the last write operation from Master via the I2C bus

    banksel masterCmd

    movf    masterCmd,W
    sublw   PIC_GET_ALL_STATUS_CMD
    btfsc   STATUS,Z
    goto    getAllStatus

    movf    masterCmd,W
    sublw   PIC_GET_PEAK_PKT_CMD
    btfsc   STATUS,Z
    goto    getPeakPacket

    return

; end handleI2CTransmit
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setPWMFromI2C
;
; Sets the registers controlling the PWM cutting current pulse control with values received via
; the I2C bus.
;
; The data bytes from the master should be:
;
;    pwmDutyCycleHiByte      ; cutting current pulse controller duty cycle time
;    pwmDutyCycleLoByte
;    pwmPeriod               ; cutting current pulse controller period time
;    pwmPolarity             ; polarity of the PWM output -- only lsb used
;    pwmCheckSum             ; used to verify PWM values read from eeprom
;
; If the checksum does not validate, the PWM registers are not changed.
;
; The value of 1 is added to the checksum to prevent cases where the values are read as all zeroes
; in error and would match the checksum of zero.
;

setPWMFromI2C:

    movlw   .5                      ; store 2 bytes from I2C
    call    receiveBytesFromI2C

    ; validate the checksum

    banksel scratch3

    clrw                            ; calculate the checksum for all PWM values
    addwf   scratch3,W
    addwf   scratch4,W
    addwf   scratch5,W
    addwf   scratch6,W
    addlw   1                       ; see note in function header

    subwf   scratch7,W              ; compare calculated checksum with that read from I2C
    btfss   STATUS,Z
    return                          ; zero flag clear, checksum NOT matched, exit

    ; checksum matched -- apply values

    ; set the polarity of the PWM output

    banksel scratch6
    movf    scratch6,W          ; byte read from I2C bus

    banksel PWM1CON
    bcf     PWM1CON,PWM1POL     ; clear polarity bit
    btfsc   WREG,0
    bsf     PWM1CON,PWM1POL     ; set polarity bit if bit 0 of polarity byte is set

    ; set the PWM cycle period

    banksel scratch5
    movf    scratch5,W          ; byte read from I2C bus
    banksel PR2                 ; period of the cycle
    movwf   PR2

    ; set the PWM duty cycle

    banksel scratch0

    movf    scratch4,W          ; set up low byte of value
    movwf   scratch1

    movf    scratch3,W          ; set up high byte of value
    movwf   scratch2

    ;call    setPWM1DC           ; set value of PWM1DCH and PWM1DCH duty cycle time registers

    return

; end setPWMFromI2C
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleHostStartCmd
;
; Sets flag to trigger execution of the normal operation code.
;

handleHostStartCmd:

    banksel flags
    bsf     flags,STARTED

    return

; end handleHostStartCmd
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; doReset
;
; Places the PIC in known state.
;

doReset:

    return

; end doReset
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; receiveBytesFromI2C
;
; Receives a specified number of bytes from the I2C bus.
; Normally, this number is equal to the number of bytes read unless the master sends too few or
; too many.
;
; Acking: Slave hardware will generate an ACK response if the AHEN and DHEN bits of the SSPCON3
; register are clear.
;
; On entry:
;  W register: the number of bytes to be stored
;
; Bytes will be read until a stop condition is received from the master -- only the number of bytes
; specified in W will actually be stored.
;
; On exit:
;  scratch1: The number of bytes actually stored (may be less than number of bytes specified to be
;   read if master sends too few and may be less than the number of bytes actually read if master
;   sends too many).
;
; scratch3-scratchx: the received bytes
;
; Other variable usage:
;
; Variable scratch0 is used to count down the number of bytes stored.
; Variable scratch2 is used for control flags.
;

receiveBytesFromI2C:

    banksel scratch0
    movwf   scratch0                ; store number of bytes to record

    clrf    scratch1                ; track number of bytes recorded
    clrf    scratch2                ; clear control flags

    clrf    FSR0H
    movlw   scratch3                ; point to first byte of receive buffer
    movwf   FSR0L

rBFILoop1:

    call    waitForSSP1IFHighOrStop     ; wait for byte or stop condition to be received

    btfss   STATUS,Z
    goto    cleanUpI2CAndReturn         ; bail out when stop condition received

    call    readI2CByteAndPrepForNext   ; get the byte just received

    banksel scratch0

    btfsc   scratch2,0                  ; if bit 0 set, expected number of bytes already stored
    goto    skipByteStore               ;  more bytes will be read until stop condition received
                                        ;  but they are not stored

    ; store the byte in the buffer

    movwi   FSR0++                      ; store in next buffer position

    incf    scratch1,F                  ; count number of bytes stored

    decfsz  scratch0,F                  ; check number of bytes to record
    goto    rBFILoop1                   ; loop to read more bytes

    bsf     scratch2,0                  ; set flag to prevent storing any more bytes
                                        ; function will read and toss bytes until stop condition

skipByteStore:

    goto    rBFILoop1                   ; loop to read more bytes

; end receiveBytesFromI2C
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; readI2CByteAndPrepForNext
;
; Reads value from SSP1BUF into W register, resets SSP1IF, and sets CKP to release I2C clock so
; master can start sending the next byte.
;
; The last byte received is returned in W.
;

readI2CByteAndPrepForNext:

    banksel SSP1BUF
    movf    SSP1BUF,W            ; get incoming value; clears BF flag

    call    clearSSP1IF         ; clear the I2C interrupt flag
    call    setCKP              ; release the I2C clock line so master can send next byte
    call    clearSSP1OV         ; clears the overflow bit to allow new data to be read

    return

; end of readI2CByteAndPrepForNext
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; clearSSP1IF
;
; Sets the SSP1IF bit in register PIR1 to 0.
;

clearSSP1IF:

    banksel PIR1
    bcf     PIR1, SSP1IF

    return

; end of clearSSP1IF
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; waitForSSP1IFHigh
;
; Waits in a loop for SSP1IF bit in register PIR1 to go high.
;

waitForSSP1IFHigh:

    ifdef debug       ; if debugging, don't wait for interrupt to be set high as the MSSP is not
    return            ; simulated by the IDE
    endif

    banksel PIR1

wfsh1:
    btfss   PIR1, SSP1IF
    goto    wfsh1

    return

; end of waitForSSP1IFHigh
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; waitForSSP1IFHighOrStop
;
; Waits in a loop for SSP1IF bit in register PIR1 to go high or P (stop) bit in SSP1STAT to go
; high.
;
; On exit:
;
; If SSP1IF went high, W will be 0 and Z flag set.
; If P (stop condition) went high, W will be 1 and Z flag cleared.
;

waitForSSP1IFHighOrStop:

    ifdef debug       ; if debugging, don't wait for interrupt to be set high as the MSSP is not
    goto  wfshosByteReceived    ; simulated by the IDE
    endif


wfshos1:

    banksel PIR1
    btfsc   PIR1, SSP1IF        ; check interrupt flag
    goto    wfshosByteReceived  ; return if flag set

    banksel SSP1STAT
    btfsc   SSP1STAT, P         ; check stop condition received flag
    goto    wfshosStopReceived  ; return if flag set

    goto    wfshos1

wfshosByteReceived:

    clrw                        ; signal that byte received
    addlw   0x00                ; (must use addlw to set Z bit)
    return

wfshosStopReceived:

    clrw                        ; signal that stop condition received
    addlw   0x01                ; (must use addlw to clear Z bit)
    return

; end of waitForSSP1IFHighOrStop
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; waitForSSP1IFHighThenClearIt
;
; Waits in a loop for SSP1IF bit in register PIR1 to go high and then clears that bit.
;
; The bit must be cleared at some point after it is set before performing most actions with the I2C
; but. In most cases, it can be cleared immediately for which this function is useful.
;
; If the bit is not cleared before an operation, then checking the bit immediately after the
; operation will make it appear that the operation completed immediately and the code will not
; wait until the MSSP module sets the bit after actual completion.
;

waitForSSP1IFHighThenClearIt:

    call    waitForSSP1IFHigh

    call    clearSSP1IF

    return

; end of waitForSSP1IFHighThenClearIt
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; cleanUpI2CAndReturn
;
; Clears all I2C status and error flags to ensure the bus is ready for further use.
;

cleanUpI2CAndReturn:

    call    clearSSP1IF         ; clear the I2C interrupt flag
    call    setCKP              ; release the I2C clock line so master can send next byte
    call    clearSSP1OV         ; clears the overflow bit to allow new data to be read
    call    clearWCOL           ; clears the write collision bit to allow new data to be written

    return

; end of cleanUpI2CAndReturn
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setCKP
;
; Sets the CKP bit in register SSP1CON1 to 1.
;
; This will release the I2C bus clock so the master can transmit the next byte.
;

setCKP:

    banksel SSP1CON1
    bsf     SSP1CON1, CKP

    return

; end of setCKP
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; clearSSP1OV
;
; Clears the MSSP overflow bit to allow new bytes to be read.
;
; The bit is set if a byte was received before the previous byte was read from the buffer.
;

clearSSP1OV:

    banksel SSP1CON1
    bcf     SSP1CON1,SSPOV

    return

; end of clearSSP1OV
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; clearWCOL
;
; Clears the MSSP write collision bit to allow new bytes to be written
;
; The bit is set if a byte was placed in SSP1BUF at an improper time.
;

clearWCOL:

    banksel SSP1CON1
    bcf     SSP1CON1, WCOL

    return

; end of clearWCOL
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleInterrupt
;
; All interrupts call this function.  The interrupt flags must be polled to determine which
; interrupts actually need servicing.
;
; Note that after each interrupt type is handled, the interrupt handler returns without checking
; for other types.  If another type has been set, then it will immediately force a new call
; to the interrupt handler so that it will be handled.
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 16 deep and
; it is very bad for the interrupt routine to use it.
;

handleInterrupt:

	btfsc 	INTCON,T0IF     		; Timer0 overflow interrupt?
	goto 	handleTimer0Interrupt	; YES, so process Timer0

INT_ERROR_LP1:		        		; NO, do error recovery
	;GOTO INT_ERROR_LP1      		; This is the trap if you enter the ISR
                               		; but there were no expected interrupts

	retfie                  ; return and enable interrupts

; end of handleInterrupt
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleTimer0Interrupt
;
; This function is called when the Timer 0 register overflows.
;
; The prescaler is set to 1:256.
; 16 Mhz Fosc = 4 Mhz instruction clock (CLKOUT)
; 4,000,000 Hz / 256 = 15,625 Hz;  15,625 Hz / 156 = 100 Hz
; Interrupt needed every 156 counts of TMR0 -- set to 255-156.
;
; Interrupt triggered when 8 bit TMR0 register overflows, so subtract desired number of increments
; between interrupts from 255 for value to store in register.
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 8 deep and
; it is very bad for the interrupt routine to use it.
;

handleTimer0Interrupt:

	bcf 	INTCON,TMR0IF     ; clear the Timer0 overflow interrupt flag

    ; reload the timer -- see notes in function header

    movlw   (.255 - .156)
    banksel TMR0
    movwf   TMR0

;debug mks -- output a pulse to verify the timer0 period
    banksel LATB
    bsf     LATB, RB7
    bcf     LATB, RB7
;debug mks end

    ; trigger the next A/D conversion

    movlw   0x01
    banksel adTrigger
    movwf   adTrigger

	retfie                  ; return and enable interrupts

; end of handleTimer0Interrupt
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; reallyBigDelay
;
; Delays for a second or two.
;
; To adjust the delay, load scratch6 with any value and enter at rbd1.
; In that case, use "banksel scratch6" before entering.
;

reallyBigDelay:

    banksel scratch6

    movlw   .50
    movwf   scratch6

rbd1:

    movlw   .255
    movwf   scratch7

rbd2:

    movlw   .255
    movwf   scratch8

rbd3:

    decfsz  scratch8,F
    goto    rbd3

    decfsz  scratch7,F
    goto    rbd2

    decfsz  scratch6,F
    goto    rbd1

    return

; end of reallyBigDelay
;--------------------------------------------------------------------------------------------------

    END
