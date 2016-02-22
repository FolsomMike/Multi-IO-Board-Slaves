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
; The system clock is configured to use an external 8 Mhz source which is boosted to 32 MHz by the
; PLL. The CLKOUT pin outputs a clock at Fosc/4 which drives the input clock of the next Slave PIC.
; Each Slave PIC uses the PLL to generate a 32 MHz from the 8 Mhz input supplied by the Slave which
; precedes it in the daisy-chain. The CLKOUT pin of each Slave PIC drives the next Slave PIC and
; so forth. Each PIC's CLKOUT output is FOSC/4 into the next PIC, so each must use its PLL to run
; its system clock at 32 MHz.
;
; The 32 MHz is used since the Slave PICs run their A/D convertors at Fosc/32 to achieve a sample
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
; ----------------
;
; Sample Pre-Buffering
;
; The A/D converter interrupt is made as short as possible to allow for shortest latency for all
; tasks. It simply stores the A/D value in a buffer along with the clock position at the time of
; the sample and increments numPreSamples to reflect the number of samples in the buffer.
;
; The main code extracts the samples and processes them, decrementing numPreSamples. It also
; records the maximum number of pre-samples ever stored at one time in maxNumPreSamples for
; debugging purposes to check for buffer overruns.
;
; samplePreBuf buffer format:
;
; pre-sample 1
; pre-sample clock 1
; pre-sample 2
; pre-sample clock 2
; ...
;
; The samples and clock positions are interleaved in samplePreBuf
;
; ----------------
;
; Sample Buffering
;
; A circular buffer is used to capture the last 80 A/D samples. To save time, three buffers are
; used so that data never needs to be copied from the active catch buffer to a peak buffer and
; from there to a transmit buffer: sampleBuf1, sampleBuf2, sampleBuf3.
;
; Note MKS: should probably use 128 byte circular buffers positioned at 128 byte page boundaries.
; When incrementing pointer, only low byte should be modified then top bit zeroed. This will cause
; the pointer to automatically wrap back to the top of the buffer without having to compare with
; the end of the buffer at each increment. This is similar to the method used in the TI DSPs.
;
; There are three buffer pointers used: catchBuf, peakBuf, xmtBuf. Each pointer is assigned to
; one of the sampleBuf* buffers. When a peak is detected, peakBufFinishCnt is set to 40 (half the
; buffer length) and is thereafter counted down for each sample added to the buffer.
;
; When peakBufFinishCnt reaches 0, 40 samples have been saved after the peak. Thus there will be 40
; samples before and 40 samples after the peak in the buffer. At that point, buffer location of the
; last sample is stored in peakBufFinishCnt (the top bit should be ignored as it is part of the
; bank selection bits) - the bottom 7 bits represent the value 0~80 where the last sample was
; stored. From this, it can determined where the peak value must be in the buffer (40 bytes prior)
; and the data can be rotated to center the peak.
;
; At that point, the catchBuf and peakBuf pointer values are swapped, so new data is now added to
; a different buffer and the peak data stored in the first catch buffer is preserved. Every time
; a new peak is detected, the catchBuf and peakBuf pointers are swapped so that the previous peak
; is overwritten with new data while the last peak is always stored in the buffer pointed to by
; peakBuf.
;
; When a request is made for peak data transmission to the Master PIC, the peakBuf and xmtBuf
; pointer values are swapped. Thus the current peak will be preserved and new peaks will be stored
; in a different buffer. Future swaps between catchBuf and peakBuf will involve the new buffer
; leaving the old one alone while it is being transmitted.
;
; Thus the three buffers are rotated amongst the three pointers allowing data to be preserved
; without ever having to copy it. The pointers are not reset to the start of the buffer on
; swapping, so data collection starts again with whatever the last location was. The important
; location is the 40th sample after the peak, as that dictates where the peak must lie in the
; buffer.
;
; Buffer Swapping Process:
;
; collect samples in catchBuf
; when peak detected:
;   set peakBufFinishCnt to (buffer length)/2 and begin counting down with each sample
; when peakBufFinishCnt reaches 0:
;   swap catchBuf and peakBuf and set peakFlags.NEW_DATA flag
; if new peak found before peakBufFinishCnt reaches 0, simple restart counter
;
; when Peak Data transmit is requested, check peakFlags.NEW_DATA flag
; if not set, return whatever data is in xmtBuf along with unset flag to alert that data is old
; if set, swap peakBuf and xmtBuf, transmit xmtBuf, clear peakFlags.NEW_DATA flag
;
; ----------------
;
; Clock Map Buffering
;
; Clock Map buffer handling is similar to the Sample Buffering described above, but only two
; buffers are needed: a catch buffer and a xmt buffer.
;
; Two buffers are used: mapBuf1 and mapBuf2.
;
; There are two buffer pointers used: mapCatchBuf and mapXmtBuf. Each pointer is assigned to
; one of the mapBuf* buffers.
;
; When a request is made for peak data transmission to the Master PIC, the mapCatchBuf and
; mapXmtBuf pointer values are swapped. Thus the current data will be preserved and new data will
; be stored in a different buffer.
;
; Unlike the Sample Buffer, map data is always valid regardless of the state of the
; peakFlags.NEW_DATA flag.
;
; Buffer Swapping Process:
;
; collect samples in mapCatchBuf
;
; when Peak Data transmit is requested:
;   swap mapCatchBuf and mapXmtBuf, transmit mapXmtBuf
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
; Search for "debug_on" to find all examples of such code.

;#define debug_on 1     ; set debug testing "on"

; version of this software

SOFTWARE_VERSION_MSB    EQU 0x01
SOFTWARE_VERSION_LSB    EQU 0x02

; upper nibble of I2C address for all Slave PICs is 1110b
; 3 lsbs will be set to match the address inputs on each Slave PIC
; note that 1111b upper nibble has special meaning in 10 bit address mode, so it is avoided here

I2C_SLAVE_ADDR_UPPER            EQU     b'11100000'

; Master PIC to Slave PIC Commands -- sent by Master to Slaves via I2C to trigger actions

PIC_NO_ACTION_CMD               EQU .0
PIC_ACK_CMD                     EQU .1
PIC_GET_ALL_STATUS_CMD          EQU .2
PIC_START_CMD                   EQU .3
PIC_GET_PEAK_PKT_CMD            EQU .4
PIC_ENABLE_POT_CMD              EQU .5
PIC_DISABLE_POT_CMD             EQU .6
PIC_GET_LAST_AD_VALUE_CMD       EQU .7

I2C_RCV_BUF_LEN      EQU .5
I2C_XMT_BUF_LEN      EQU .20

MAP_BUF_LEN         EQU .24
SAMPLE_PREBUF_LEN   EQU .60             ; NOTE: this must always be an even number!
SAMPLE_BUF_LEN      EQU .80

; end of Defines
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Configurations, etc. for the Assembler Tools and the PIC

	LIST p = PIC16F1459	;select the processor

    ;errorlevel  -306 ; Suppresses Message[306] Crossing page boundary -- ensure page bits are set.
        ; the above is often used if a lot of calls/gotos across page boundaries are required as
        ; then the warning is useless as it lists each crossing even if it is properly done

    errorLevel  -302 ; Suppresses Message[302] Register in operand not in bank 0.

	errorLevel	-202 ; Suppresses Message[205] Argument out of range. Least significant bits used.
					 ;	(this is displayed when a RAM address above bank 1 is used -- it is
					 ;	 expected that the lower bits will be used as the lower address bits)

#INCLUDE <p16f1459.inc> 		; Microchip Device Header File


;#include <xc.h>

; Specify Device Configuration Bits

; CONFIG1
; __config 0xF9E4
 __CONFIG _CONFIG1, _FOSC_ECH & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _BOREN_OFF & _CLKOUTEN_ON & _IESO_OFF & _FCMEN_OFF
; CONFIG2
; __config 0xFFFF
 __CONFIG _CONFIG2, _WRT_ALL & _CPUDIV_NOCLKDIV & _USBLSCLK_48MHz & _PLLMULT_4x & _PLLEN_ENABLED & _STVREN_ON & _BORV_LO & _LPBOR_OFF & _LVP_OFF

; _FOSC_ECH ?> External Clock, High Power Mode (4-20 MHz): device clock supplied to CLKIN pins
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
; bit 6 = 0 : bits 6-2 : analog input channel - 01001 -> AN9
; bit 5 = 1 :    
; bit 4 = 0 :
; bit 3 = 0 :
; bit 2 = 1 :
; bit 1 = 0 : Go|/Done
; bit 0 = 1 : ADC is enabled

; A/D convertor selection for analog input channel/pin: AN9 on pin RC7

AD_CHANNEL_CODE    EQU     b'00100101'

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

; Assign A/D related variables, the Peak Data variables, and the Clock Peak Map in RAM - Bank 6
; this bank has 80 bytes of free space

 cblock 0x320               ; starting address

    peakFlags               ; bit 0: 0 = no new data, 1 = new data ready
                            ; bit 1: 0 =
                            ; bit 2: 0 =
                            ; bit 3: 0 =
                            ; bit 4: 0 =
                            ; bit 5: 0 =
							; bit 6: 0 =
							; bit 7: 0 =
                            
    lastADSample            ; the last A/D sample recorded
    lastSampleClk           ; clock position of the last A/D sample recorded
    lastSampleLoc           ; linear location of the last A/D sample recorded

    maxPeak
    maxPeakClk
    maxPeakLoc
    minPeak
    minPeakClk
    minPeakLoc

    maxPeakSnap
    maxPeakClkSnap
    maxPeakLocSnap
    minPeakSnap
    minPeakClkSnap
    minPeakLocSnap

    maxABSPeak              ; maximum absolute value

    peakBufFinishCnt
    peakBufLastLoc

    pbScratch0
    pbScratch1
    pbScratch2
    pbScratch3
    pbScratch4

    catchBufH
    catchBufL
    peakBufH
    peakBufL
    xmtBufH
    xmtBufL

    mapCatchBufH
    mapCatchBufL
    mapXmtBufH
    mapXmtBufL

 endc

;-----------------

; Assign A/D Clock Peak Map in RAM - Bank 7
; this bank has 80 bytes of free space

 cblock 0x3a0               ; starting address

    mapBuf1:MAP_BUF_LEN
    mapBuf2:MAP_BUF_LEN

 endc

;-----------------

; Assign A/D sampling pre-buffer and related variables in RAM - Bank 8
;   see notes at top of file "Sample Pre-Buffering"
; this bank has 80 bytes of free space

 cblock 0x420               ; starting address
 
    numPreSamples
    maxNumPreSamples        ; maximum number of A/D values stored in pre-buffer at any one time
                            ; should never be bigger than the size of the buffer, otherwise it
                            ; is an overrun condition
    inPreBufH               ; insertion pointer for sample pre-buffer
    inPreBufL
    outPreBufH              ; extraction pointer for sample pre-buffer
    outPreBufL

    samplePreBuf:SAMPLE_PREBUF_LEN

 endc

;-----------------

; Assign A/D Sample Buffer 1 in RAM - Bank 9 - see notes at top of file "Sample Buffering"
; this bank has 80 bytes of free space

 cblock 0x4a0                ; starting address

    sampleBuf1:SAMPLE_BUF_LEN

 endc

;-----------------

; Assign A/D Sample Buffer 2 in RAM - Bank 10 - see notes at top of file "Sample Buffering"
; this bank has 80 bytes of free space

 cblock 0x520                ; starting address

    sampleBuf2:SAMPLE_BUF_LEN

 endc

;-----------------

; Assign A/D Sample Buffer 3 in RAM - Bank 11 - see notes at top of file "Sample Buffering"
; this bank has 80 bytes of free space

 cblock 0x5a0                ; starting address

    sampleBuf3:SAMPLE_BUF_LEN

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

mainLoop:

    call    handleI2CCommand    ; checks for incoming command on I2C bus

    banksel numPreSamples       ; process if interrupt routine has added samples to the pre-buffer
    movf    numPreSamples,F
    btfss   STATUS,Z
    call    processADSamples

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

    call    setupClock          ; set system clock source and frequency

    call    setupPortA          ; prepare Port A for I/O

    call    setupPortB          ; prepare Port B for I/O

    call    setupPortC          ; prepare Port C  for I/O

    call    setupTimer0         ; Timer 0 counts pulses on RC5 input

    call    setupIntOnChange    ; sets up interrupt-on-change for appropriate inputs

    call    initializeOutputs

    call    parseSlaveIC2Address

    call    setupI2CSlave7BitMode ; prepare the I2C serial bus for use

    call    setupADVars     ; setup A/D sampling variables and pointers

    call    setupADConverter ; prepare A/D converter for use

;start of hardware configuration

    clrf   FSR0H            ;high byte of indirect addressing pointers -> 0
    clrf   FSR1H

    clrf    INTCON          ; disable all interrupts

    banksel OPTION_REG
    movlw   0x7f
    movwf   OPTION_REG      ; Option Register = 0x7f   0111 1111 b
                            ; bit 7 = 0 : weak pull-ups are enabled by individual port latch values
                            ; bit 6 = 1 : interrupt on rising edge
                            ; bit 5 = 1 : TMR0CS ~ Timer 0 increments on TCK0I pin input
                            ; bit 4 = 1 : TMR0SE ~ Timer 0 increment on high-to-low transition on RA4/T0CKI/CMP2 pin
                            ; bit 3 = 1 : PSA ~ Prescaler not assigned to Timer0
                            ; bit 2 = 1 : Bits 2:0 control prescaler:
                            ; bit 1 = 1 :    111 = 1:256 scaling for Timer0 (if assigned to Timer0)
                            ; bit 0 = 1 :
    
;end of hardware configuration
	
; enable the interrupts

    banksel PIE1            ; enable A/D interrupts
    bsf     PIE1, ADIE

    bsf     INTCON,PEIE     ; enable peripheral interrupts (Timers, A/D converter, etc.)

    bsf     INTCON,IOCIE    ; enable interrupt-on-change for enabled inputs

;debug mks -- put this back in    bsf     INTCON,GIE      ; enable all interrupts

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
;   _FOSC_ECH, _CPUDIV_NOCLKDIV, _PLLMULT_4x, _PLLEN_ENABLED
;
; Assumes all programmable clock related options are at Reset default values.
;

setupClock:

    ; Since each slave pic is set up to use an external clock (see notes at top of page),
    ; there is no need to set anything programmatically

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
; setupTimer0
;
; Sets up Timer 0 as a counter triggered by pulses on RC5/T0CKI.
;
; For this program, that input is the SYNC pulse. Each pulse marks the start of a new clock
; position segment. The counter is reset when a RESYNC pulse is received on an interrupt pin. Thus
; TMR0 will reset to 0 at the TDC (RESYNC) pulse and then count through the clock positions.
;
; NOTE:
;
; In the 2012 DS41639A PIC16(L)F1454/5/9 Data Sheet, it seems to imply that the TCK0I signal must
; be synchronized to the clock by external means. It then shows a synchronizing block internally.
; It seems that the input is synchronized by the PIC itself. The text below from the manual shows
; the likely true meaning ([original]->[corrected]:
;
; When in 8-Bit Counter mode, the incrementing edge on the T0CKI pin [must be]->[is] synchronized
; to the instruction clock. Synchronization [can be]->[is] accomplished by sampling the prescaler
; output on the Q2 and Q4 cycles of the instruction clock.
;
; Even though it says specifically that the prescaler output is sampled, it also shows to sample
; the non-prescaled signal when the prescaler is disabled.
;

setupTimer0:

; OPTION_REG configured in setup function:
;  PSA bit = 1 ~ prescaler NOT assigned to TMR0 (no prescaler used)
;  TMR0CS = 1 ~ increments on transition of T0CKI pin
;  TMR0SE = 1 ~ count increments on high-to-low transition on T0CKI pin
;  PSA = 111b ~ prescaler of 1:256 (not used since prescaler not assigned to TMR0)


    banksel ANSELC                      ; pin is digital I/O
    bcf     ANSELC,RC7

    banksel TRISC                       ; pin is input
    bsf     TRISC,RC7

    banksel TMR0                        ; clear the counter
    clrf    TMR0

    return

; end of setupTimer0
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupIntOnChange
;
; Sets up interrupt-on-change for RA0 which is the SYNC_RESET input which resets Timer 0 back
; to zero each time RA0 has a high-to-low transition.
;

setupIntOnChange:

    banksel ANSELA                      ; pin is digital I/O
    bcf ANSELA,RA0

    banksel TRISA                       ; pin is input
    bsf TRISA,RA0

    banksel IOCAP                       ; disable interrupt on low-to-high
    bcf IOCAP,IOCAP0

    banksel IOCAN                       ; enable interrupt on high-to-low
    bsf IOCAN,IOCAN0

    return

; end of setupIntOnChange
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

    banksel TRISC
    bsf     TRISC, AD_AN9                       ;set I/O pin to input

    banksel ANSELC
    bsf     ANSELC, AD_AN9                      ;set I/O pin to analog mode
    
    ; turn on A/D module and select channel

    banksel ADCON0
    movlw   AD_CHANNEL_CODE
    movwf   ADCON0

    bsf     ADCON0,ADGO                         ;start first A/D conversion

    return

; end of setupADConverter
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; processADSamples
;
; Processes all A/D samples which have been placed in the pre-buffer by the A/D interrupt routine
; since the last time this method was called.
;
; Each value is converted from unsigned to signed such that 0V input to the board is at value of 0.
; The absolute value is then compared with the value already in the map buffer at the same clock
; position as the new sample. If the new sample is greater, the old value is replaced.
;
; For peak capture, the max and min values of the new sample replace the old peak values if the new
; is greater/less than the old peaks. The clock positions for those peaks are stored as well.
;
; If a new peak is detected (absolute value) a count down counter is started. When that reaches 0,
; the peaks and and clocks are stored and the catch buffer and peak buffer are switched so that the
; buffer for the stored peak is preserved (now in the peak buffer). There is only one peak buffer;
; it contains the data for the highest absolute value peak, either negative or positive.
; See notes at the top of the page for more details.
;
; Sample Value Sign
;
; The raw value from the A/D is unsigned 0-255 where 0 is 0V and 255 is Vcc.
; The board is designed so that 0V input is actually at Vcc/2 at the A/D input.
; The A/D value can be converted to signed by simply flipping bit 7, such that -128 is 0V and
; +127 is Vcc at the A/D input.
;
; NOTE: before converting, force value of 0 to value of 1. This has the effect after the conversion
;       of limiting the negative swing to -127, which can be converted to +127 with absolute value.
;       When trying to get abs value of -128 (0 before the conversion), it returns -128 as there is
;       no +128 in an 8 bit value.
;
; In some cases the value is left unsigned as it is easy to check for min/max peaks as there is no
; sign to worry about.
;

processADSamples:

    banksel lastADSample
    movf    outPreBufH,W         ; set pointer to current pre-buffer extraction location
    movwf   FSR0H
    movf    outPreBufL,W
    movwf   FSR0L

    banksel peakFlags
    movf    catchBufH,W         ; set pointer to current catch buffer location
    movwf   FSR1H
    movf    catchBufL,W
    movwf   FSR1L

    banksel numPreSamples       ; store the number of samples in pbScratch0
    movf    numPreSamples,W
    banksel pbScratch0
    movwf   pbScratch0

    ; transfer the new samples in pre-buffer to the catch buffer

padsLoop1:

    moviw   FSR0++              ; move sample between buffers
    movwi   FSR1++

    movwf   pbScratch1          ; store the raw value temporarily

    btfsc   STATUS,Z            ; limit bottom end to 1 (-127 as a signed value) see notes above
    incf    WREG,F

    xorlw   0x80                ; flip bit 7 to convert to signed value with 0 at center voltage

    ; get absolute value of the sample

    btfss   WREG,7              ; check for positive value
    goto    padsPosNum          ; don't flip if positive

    comf    WREG,W              ; flip value to positive
    addlw   .1

    movwf   pbScratch2          ; store the sample abs value temporarily

padsPosNum:

    movf    FSR1H,W             ; store FSR1 temporarily
    movwf   pbScratch3
    movf    FSR1L,W
    movwf   pbScratch4

    movf    mapCatchBufH,W      ; add clock position to map start to get address of the clock pos
    movwf   FSR1H               ;   in the map buffer
    movf    mapCatchBufL,W      
    addwf   INDF0,W             ; add clock position (leave FSR0 pointing at clock value)
    movwf   FSR1L

    movf    pbScratch2,W        ; compare abs value sample with value already in map
    subwf   INDF1,W
    btfss   STATUS,C            ; C = 1 => W > f, so store new value in map
    goto    padsNewLTEQMap      ; new value <= old value, so leave old value

    movf    pbScratch2,W        ; store new value in map buffer
    movwf   INDF1

padsNewLTEQMap:

    subwf   maxABSPeak,W        ; compare abs value sample with current abs value peak
    btfss   STATUS,C            ; C = 1 => W > f, so store new value in peak
    goto    padsNewLTEQAbsPeak  ; new value <= old value, so leave old value

    movf    pbScratch2,W        ; store new value in abs value peak
    movwf   maxABSPeak

    movlw   (SAMPLE_BUF_LEN / .2)   ; fill in half of buffer after peak so it will be centered
    movwf   peakBufFinishCnt

padsNewLTEQAbsPeak:

    movf    pbScratch1,W        ; get raw sample for max peak detection
    subwf   maxPeak,W           ; compare new sample with max peak
    btfss   STATUS,C            ; C=1 => W>f, so store new value in peak
    goto    padsNewLTEQMaxPeak

    movf    pbScratch1,W        ; store new sample as peak
    movwf   maxPeak
    moviw   FSR0++
    movwf   maxPeakClk          ; store clock position of peak (location not yet implemented)

    goto    checkPeakBufFinishCnt

padsNewLTEQMaxPeak:

    movf    pbScratch1,W        ; get raw sample for min peak detection
    subwf   maxPeak,W           ; compare new sample with min peak
    btfsc   STATUS,C            ; C=0 => W<=f, so store new value in peak
    goto    checkPeakBufFinishCnt

    movf    pbScratch1,W
    movwf   minPeak
    moviw   FSR0++
    movwf   minPeakClk          ; store clock position of peak (location not yet implemented)

checkPeakBufFinishCnt:

    ; decrement the peak buffer finish counter if it is not zero
    ; when it reaches zero, second half of buffer has been filled after peak, so buffers are
    ; swapped to preserve the buffer

    movf    checkPeakBufFinishCnt,F     ;if counter already zero, ignore
    btfsc   STATUS,Z
    goto    bufNotFinished

    decfsz  checkPeakBufFinishCnt,F     ;count down
    goto    bufNotFinished

    ; second half of buffer filled after last peak, so preserve peak data
    call    preservePeakData

bufNotFinished:

    movf    pbScratch3,W        ; restore FSR1
    movwf   FSR1H
    movf    pbScratch4,W
    movwf   FSR1L

    decfsz  pbScratch0,F        ; loop until all samples in pre-buffer processed
    goto    padsLoop1

    ; store the updated pointers

    banksel lastADSample
    movf    FSR0H,W
    movwf   outPreBufH
    movf    FSR0L,W
    movwf   outPreBufL

    banksel peakFlags
    movf    FSR1H,W
    movwf   catchBufH
    movf    FSR1L,W
    movwf   catchBufL

    return

; end of processADSamples
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
; See Device.handleAllStatusPacket method in Java host code for details on packet structure.
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

    movf    slaveI2CAddress,W
    movwi   FSR0++

    movlw   SOFTWARE_VERSION_MSB
    movwi   FSR0++

    movlw   SOFTWARE_VERSION_LSB
    movwi   FSR0++

    movf    flags,W
    movwi   FSR0++

    movf    statusFlags,W
    bcf     statusFlags,COM_ERROR
    bcf     statusFlags,AD_OVERRUN
    movwi   FSR0++

    movf    comErrorCnt,W
    clrf    comErrorCnt
    movwi   FSR0++

    banksel lastADSample

    movf    maxNumPreSamples,W
    clrf    maxNumPreSamples
    movwi   FSR0++

    movf    lastADSample,W
    movwi   FSR0++

    banksel flags

    movlw   0x01                        ; unused -- for future use
    movwi   FSR0++

    movlw   0x02                        ; unused -- for future use
    movwi   FSR0++

    movlw   0x03                        ; unused -- for future use
    movwi   FSR0++

    movlw   .11                         ; number of data bytes in packet
    movwf   scratch0

    call    calcAndStoreCheckSumForI2CXmtBuf

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
; NOTE: I2C reception does not have to be performed in an interrupt routine in this program because
; it is set up to halt the bus until the code has a chance to respond.
;

handleI2CCommand:

    ; check if SSP1IF flag set -- if so, matching address byte has been received
    
    ifndef  debug_on            ; pretend flag set if in debug mode
    banksel PIR1
    btfss   PIR1, SSP1IF
    return
    endif

    banksel SSP1BUF
    movf    SSP1BUF,W           ; get incoming address byte; clears BF flag

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
    
    movf    masterCmd,W
    sublw   PIC_GET_LAST_AD_VALUE_CMD
    btfsc   STATUS,Z
    goto    transmitTwoByteValueToMaster

    return

; end handleI2CTransmit
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; transmitTwoByteValueToMaster
;
; Determines which two-byte value to send to the master pic and then sends it.
;
; Which value to send is determined by the last command received from the master.
;

transmitTwoByteValueToMaster:
    
    banksel i2cXmtBuf                   ; set buffer pointer to transmit buffer start
    movlw   high i2cXmtBuf
    movwf   i2cXmtBufPtrH
    movlw   low i2cXmtBuf
    movwf   i2cXmtBufPtrL

    movf    i2cXmtBufPtrH, W            ; load FSR0 with buffer pointer
    movwf   FSR0H
    movf    i2cXmtBufPtrL, W
    movwf   FSR0L

    movf    masterCmd,W                 ; get last AD value if command says to
    sublw   PIC_GET_LAST_AD_VALUE_CMD
    btfsc   STATUS,Z
    call    getLastADValue
    
    movlw   .2                          ; number of data bytes in packet
    movwf   scratch0
    
    call    calcAndStoreCheckSumForI2CXmtBuf

    goto    sendI2CBuffer

; end transmitTwoByteValueToMaster
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; getLastADValue
;
; Gets the last value converted from analog to digital and puts the upper byte in INDFO and the
; lower in INDF0++.
;
; ON ENTRY:
;
;   FSR0        =   address where upper byte of last AD value should be put
;   1[FSR0]     =   address where lower byte of last AD value should be put
;
; ON EXIT:
;    
;   INDF0       =   upper byte of last AD value
;   1[INDF0]    =   lower byte of last AD value
;

getLastADValue:
    
;//DEBUG HSS// -- uncomment this stuff
    
    ;movlw   .0                  ; no upper byte so just load INDF0 with 0
    ;movwf   INDF0

    ;movf    lastADSample,W      ; load lower byte into 1[FSR0]
    ;movwi   1[FSR0]
    
;//DEBUG HSS// -- end of uncomment this stuff
    
;//DEBUG HSS// -- remove this stuff (used for testing only)
    
    movlw   0x12
    movwf   INDF0

    movf    slaveI2CAddress,W
    movwi   1[FSR0]
    
;//DEBUG HSS// -- end of remove this stuff (used for testing only)

    return

; end getLastADValue
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

    ifdef debug_on    ; if debugging, don't wait for interrupt to be set high as the MSSP is not
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

    ifdef debug_on              ; if debugging, don't wait for interrupt to be set high as the MSSP is not
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

    banksel PIR1
	btfsc 	PIR1,ADIF               ; A/D sample ready?
	goto 	handleADInterrupt       ; YES, so process

    banksel INTCON
    btfsc   INTCON,IOCIF            ; transition detected on an input pin?
	goto 	handleIntOnChange       ; YES, so process

INT_ERROR_LP1:		        		; NO, do error recovery
	;GOTO INT_ERROR_LP1      		; This is the trap if you enter the ISR
                               		; but there were no expected interrupts

	retfie                  ; return and enable interrupts

; end of handleInterrupt
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleIntOnChange
;
; This function is called when an input changes state on an I/O pin with interrupt-on-change
; enabled.
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 8 deep and
; it is very bad for the interrupt routine to use it.
;

handleIntOnChange:

    banksel IOCAF
	btfss 	IOCAF,IOCAF0            ; transition detected on RA0?
	goto    hiocExit                ; NO, so do nothing

    banksel TMR0                    ; clear the counter (resets to clock position 0)
    clrf    TMR0

hiocExit:

    ; clear all interrupt flags which were set are cleared
    ; this method is explained in the manual
    ; MAKE SURE ALL FLAGS CHECKED AND HANDLED BEFORE DOING THIS

    banksel IOCAF
    movlw   0xff
    xorwf   IOCAF, W
    andwf   IOCAF, F

	retfie                  ; return and enable interrupts

; end of handleIntOnChange
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleADInterrupt
;
; This function is called when a sample is ready in the A/D converter.
;
; The sample is stored in samplePreBuffer along with the clock position at the time the sample
; was stored. The next conversion is started.
;
; On Entry:
;
; bank register should be set to PIR1
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 8 deep and
; it is very bad for the interrupt routine to use it.
;

handleADInterrupt:

	bcf 	PIR1,ADIF           ; clear the interrupt flag

    banksel ADRESH              ; get A/D sample
    movf    ADRESH,W            ; read upper 8 bits of result
                                ; (result is left justified so this gets
                                ;  the upper 8 bits, ignoring the 2 lsbs in ADRESL)

    bsf     ADCON0,ADGO         ; start next A/D conversion
    
    banksel lastADSample        ; store A/D sample
    movwf   lastADSample
    
    banksel TMR0                ; get current clock position
    movf    TMR0,W
    
    banksel peakFlags           ; select bank with A/D related values
    
    movwf   lastSampleClk       ; store current clock position
    
    movf    lastADSample,W      ; check to see if new max
    subwf   maxPeak,W
    btfsc   STATUS,C            ; if clear then last A/D sample > last stored maxPeak
    goto    checkMinADInterrupt
    
    movf    lastADSample,W      ; store last A/D sample as new max peak
    movwf   maxPeak
    movf    lastSampleClk,W     ; store last clock sample as new max clock
    movwf   maxPeakClk
    
checkMinADInterrupt:
    
    movf    lastADSample,W      ; check to see if new min
    subwf   minPeak,W
    btfss   STATUS,C            ; if set then last A/D sample <= last stored minPeak
    goto    exitADInterrupt

    movf    lastADSample,W      ; store last A/D sample as new min peak
    movwf   minPeak
    movf    lastSampleClk,W     ; store last clock sample as new min clock
    movwf   minPeakClk
    
exitADInterrupt:
    
    retfie                      ; return and enable interrupts

; end of handleADInterrupt
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

;--------------------------------------------------------------------------------------------------
; setupADVars
;
; Sets up variables and pointers releated to A/D sampling and storage.
;

setupADVars:

    banksel peakFlags

    clrf    peakFlags

    movlw   0x00                ;preset to min value so any max will exceed
    movwf   maxPeak
    clrf    maxPeakClk
    clrf    maxPeakLoc
    movlw   0xff                ;preset to max value so any min will exceed
    movwf   minPeak
    clrf    minPeakClk
    clrf    minPeakLoc
    clrf    maxABSPeak
    
    clrf    lastADSample        ;clear sample values
    clrf    lastSampleClk
    clrf    lastSampleLoc

    clrf    peakBufFinishCnt
    clrf    peakBufLastLoc

    ; assign all pointers to a buffer

    movlw   high mapBuf1
    movwf   mapCatchBufH
    movlw   mapBuf1
    movwf   mapCatchBufL

    movlw   high mapBuf2
    movwf   mapXmtBufH
    movlw   mapBuf2
    movwf   mapXmtBufL

    movlw   high sampleBuf1
    movwf   catchBufH
    movlw   sampleBuf1
    movwf   catchBufL

    movlw   high sampleBuf2
    movwf   peakBufH
    movlw   sampleBuf2
    movwf   peakBufL

    movlw   high sampleBuf3
    movwf   xmtBufH
    movlw   sampleBuf3
    movwf   xmtBufL

    ; setup A/D pre-buffer variables

    banksel numPreSamples
    clrf    numPreSamples
    clrf    maxNumPreSamples

    movlw   high samplePreBuf
    movwf   inPreBufH
    movwf   outPreBufH
    movlw   samplePreBuf
    movwf   inPreBufL
    movwf   outPreBufL

    ; zero each buffer (makes debugging easier)

    movlw   high samplePreBuf
    movwf   FSR0H
    movlw   samplePreBuf
    movwf   FSR0L
    movlw   SAMPLE_PREBUF_LEN
    call    clearMemBlock

    movlw   high mapBuf1
    movwf   FSR0H
    movlw   mapBuf1
    movwf   FSR0L
    movlw   MAP_BUF_LEN
    call    clearMemBlock

    movlw   high mapBuf2
    movwf   FSR0H
    movlw   mapBuf2
    movwf   FSR0L
    movlw   MAP_BUF_LEN
    call    clearMemBlock

    movlw   high sampleBuf1
    movwf   FSR0H
    movlw   sampleBuf1
    movwf   FSR0L
    movlw   SAMPLE_BUF_LEN
    call    clearMemBlock

    movlw   high sampleBuf2
    movwf   FSR0H
    movlw   sampleBuf2
    movwf   FSR0L
    movlw   SAMPLE_BUF_LEN
    call    clearMemBlock

    movlw   high sampleBuf3
    movwf   FSR0H
    movlw   sampleBuf3
    movwf   FSR0L
    movlw   SAMPLE_BUF_LEN
    call    clearMemBlock

    ; For boards which do not have a clock position sync input, each channel is assigned to a
    ; different clock position (such as for Transverse system where shoes do not spin but are
    ; always in the same circumferential location).
    ; In such case, the PIC's I2C slave address number is used as the clock number and stored in
    ; Timer 0 which will never change as there is no sync input. Thus, when the A/D samples are
    ; stored in the map buffer, they will always be in the same clock position which matches the
    ; I2C address. The host then sorts that all out to spread the data across the display map.
    ;
    ; For systems with rotating shoes, the sync input will reset and increment the Timer 0 to
    ; reflect the appropriate clock position. The mapping code will work in this case as well.

    banksel slaveI2CAddress
    movf    slaveI2CAddress,W
    banksel TMR0
    movwf   TMR0

    return

; end of setupADVars
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; clearMemBlock
;
; Sets all bytes up to 255 in a memory block to zero.
;
; On Entry:
;
; W contains number of bytes to zero
; FSR0 points to start of block
;

clearMemBlock:

    banksel scratch0                        ; store number of bytes in block
    movwf   scratch0

    movlw   0x00

cMBLoop:

    movwi   FSR0++
    decfsz  scratch0,F
    goto    cMBLoop

    return

; end of clearMemBlock
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; preservePeakData
;
; Copies the peak sample values and related variables to snapshot variables. The catch and peak
; buffer pointers are swapped so that the buffer data associated with the peak is also preserved.
;
; The map buffer is not preserved as it is not associated with the peak values.
;
; On Entry:
;
; bank register should be set to peakFlags
;
; On Exit:
;
; bank register points at peakFlags
; WREG is unknown
;

preservePeakData:

    movf    maxPeak,W
    movwf   maxPeakSnap

    movf    maxPeakClk,W
    movwf   maxPeakClkSnap

    movf    maxPeakLoc,W
    movwf   maxPeakLocSnap

    movf    minPeak,W
    movwf   minPeakSnap

    movf    minPeakClk,W
    movwf   minPeakClkSnap

    movf    minPeakLoc,W
    movwf   minPeakLocSnap

    call    swapCatchPeakPtrs

    return

; end of preservePeakData
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; swapCatchPeakPtrs
;
; Swaps the value in catchBufH:L pointer with that in peakBufH:L pointer.
;
; On Entry:
;
; bank register should be set to peakFlags
;
; On Exit:
;
; bank register points at peakFlags
; WREG is unknown
;

swapCatchPeakPtrs:

    movf    catchBufH,W
    movwf   pbScratch0

    movf    peakBufH,W
    movwf   catchBufH

    movf    pbScratch0,W
    movwf   peakBufH

    movf    catchBufL,W
    movwf   pbScratch0

    movf    peakBufL,W
    movwf   catchBufL

    movf    pbScratch0,W
    movwf   peakBufL

    return

; end of swapCatchPeakPtrs
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; swapXmtPeakPtrs
;
; Swaps the value in xmtBufH:L pointer with that in peakBufH:L pointer.
;
; On Entry:
;
; bank register should be set to peakFlags
;
; On Exit:
;
; bank register points at peakFlags
; WREG is unknown
;

swapXmtPeakPtrs:

    movf    xmtBufH,W
    movwf   pbScratch0

    movf    peakBufH,W
    movwf   xmtBufH

    movf    pbScratch0,W
    movwf   peakBufH

    movf    xmtBufL,W
    movwf   pbScratch0

    movf    peakBufL,W
    movwf   xmtBufL

    movf    pbScratch0,W
    movwf   peakBufL

    return

; end of swapXmtPeakPtrs
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; swapMapXmtCatchPtrs
;
; Swaps the value in mapXmtBufH:L pointer with that in mapCatchBufH:L pointer.
;
; On Entry:
;
; bank register should be set to peakFlags
;
; On Exit:
;
; bank register points at peakFlags
; WREG is unknown
;

swapMapXmtCatchPtrs:

    movf    mapXmtBufH,W
    movwf   pbScratch0

    movf    mapCatchBufH,W
    movwf   mapXmtBufH

    movf    pbScratch0,W
    movwf   mapCatchBufH

    movf    mapXmtBufL,W
    movwf   pbScratch0

    movf    mapCatchBufL,W
    movwf   mapXmtBufL

    movf    pbScratch0,W
    movwf   mapCatchBufL

    return

; end of swapMapXmtCatchPtrs
;--------------------------------------------------------------------------------------------------


    END
