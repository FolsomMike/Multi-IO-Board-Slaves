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
; Rundata Buffering
;
; Rundata buffer handling requires two buffers: a catch buffer and a xmt buffer.
;
; When a request is made for run data transmission to the Master PIC, the catch and xmt buffers are 
; swapped. Thus the current data will be preserved while transmitting; new data will be stored in a 
; separate buffer.
;
; ----------------
; 
; Snapshot Buffering
; 
; Three 128-byte circular buffers and three pointers are used for snapshot buffering. None of the 
; three buffers are ever copied; instead, the pointers are swapped.
;
; Capturing:
;
;   When a new peak is found, the A/D interrupt (handleAdIntterupt) will continue to put values into 
;   the catch buffer until it has accumulated 64 samples after the peak. 
;
;   Once all 64 samples are captured, the catch buffer and peak buffer pointers are swapped. So that
;   the samples begin at the top of each buffer, the LSB of the catch buffer pointer (snapCatchBufL)
;   is cleared instead of becoming equal to the LSB of the peak buffer pointer (snapPeakBufL). 
;   snapPeakBufL is still set to snapCatchBufL to keep track of where the peak buffer ends; in other
;   words, snapPeakBufL is set equal to the most recent A/D value put into the peak buffer.
;
;   After swapping, the program continues looking for a higher peak than the one stored in the peak
;   buffer, starting the process over again.
;   
; Transmitting:
;
;   The snapshot is not transmitted with the run data, but the absolute peak is. The Master PIC
;   compares the abosolute peaks retreived from the slaves and then requests the snapshot from the
;   slave who returned the greatest peak.
;
;   To ensure that the peak buffer doesn't change between the time the Master requested the rundata 
;   and snapshot, the snapshot xmt and snapshot catch buffer pointers are swapped when the rundata 
;   is requested. During this swap, snapXmtBufL is set to snapPeakBufL so that we know where the
;   end of the buffer is, but snapPeakBufL is not set to snapXmtBufL because the capturing process 
;   does not need the LSB of the peak buffer pointer to swap(see "Capturing" above).
;
;   When the snapshot buffer is transmitted, the transmission begins at snapXmtBufH:snapXmtBufL + 1.
;   One is added because snapXmtBufL is actually pointing at the end of the buffer. Assuming that we
;   had a successful capture, completely filling the snapshot buffer, the +1 should put us at the
;   start of the buffer.
;
; Ideally, the peak buffer should contain 63 samples before the peak and 64 after, but the buffer 
; swapping can cause there to be less samples before, as few as zero.
;   
; The three buffers are rotated amongst the three pointers allowing data to be preserved without 
; ever having to copy it.
; 
; On device reset, the pointers are set up as follows:
;
;       "catch buffer"      snapCatchBufH:snapCatchBufL         points at snapshot buffer 1
;
;       "peak buffer"       snapPeakBufH:snapPeakBufL           points at snapshot buffer 2
;
;       "xmt buffer"        snapXmtBufH:snapXmtBufL             points at snapshot buffer 3
;
; Used 128-byte circular buffers positioned at 128-byte page boundaries so that when incrementing 
; the pointer, only low byte should be modified then top bit zeroed. This will cause the pointer to 
; automatically wrap back to the top of the buffer without having to compare with the end of the 
; buffer at each increment. This is similar to the method used in the TI DSPs.
;
; So that the buffers are positioned at 128-byte page boundaries, they have to be placed at
; unorthodox start positoins. Refer to these notes when putting the buffers in RAM:
;
;       snapshotBuf1a:.64       Bank 03.2       cblock 0x1b0
;       snapshotBuf1b:.64       Bank 04         cblock 0x220
;
;       snapshotBuf2a:.48       Bank 06.4       cblock 0x340
;       snapshotBuf2b:.80       Bank 07         cblock 0x3a0
;
;       snapshotBuf3a:.32       Bank 09.6       cblock 0x4d0
;       snapshotBuf3b:.80       Bank 10         cblock 0x520
;       snapshotBuf3c:.16       Bank 11         cblock 0x5a0
;
; Since the buffers cross bank boundaries, they are accessed through Linear Data Memory. Formula for
; determining the linear address of a variable in RAM (I think it only works for variables located 
; in Bank 0 through Bank 11):
;
;   OFFSET                  EQU (variable & 0x7f) - 0x20
;   LINEAR_ADDR             EQU ((variable/.128)*.80)+0x2000+variable
;   LINEAR_LOC_H            EQU high LINEAR_ADDR
;   LINEAR_LOC_L            EQU low LINEAR_ADDR
;
; ----------------
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
PIC_GET_RUN_DATA_CMD            EQU .4
PIC_ENABLE_POT_CMD              EQU .5
PIC_DISABLE_POT_CMD             EQU .6
PIC_GET_LAST_AD_VALUE_CMD       EQU .7
PIC_SET_ONOFF_CMD               EQU .8
PIC_GET_SNAPSHOT_CMD            EQU .9
PIC_SET_LOCATION_CMD            EQU .10
PIC_SET_CLOCK_CMD               EQU .11

I2C_RCV_BUF_LEN      EQU .5
I2C_XMT_BUF_LEN      EQU .20

MAP_BUF_LEN         EQU .48
SAMPLE_PREBUF_LEN   EQU .60             ; NOTE: this must always be an even number!
SAMPLE_BUF_LEN      EQU .80
      
SNAPSHOT_BUF_LEN    EQU .128            ; NOTE: Must always be an even number!

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
    
; bits in commonFlags
BIT_CHAN_ONOFF          EQU 0
BIT_RDY_STOP            EQU 1

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
; unless you are compiling object files and using a linker command file.  The  cblock directive is
; commonly used to reserve RAM space or Code space when producing "absolute" code, as is done here.
;

;--------------------------------------------------------------------------
; Bank 00 - 80 bytes of free space

 cblock 0x020                ; starting address

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

    i2cXmtBuf:I2C_XMT_BUF_LEN

    i2cRcvBufPtrH
    i2cRcvBufPtrL
    i2cRcvBuf:I2C_RCV_BUF_LEN

 endc

; end Bank 00
;--------------------------------------------------------------------------
 
;--------------------------------------------------------------------------
; Bank 01 - 80 bytes of free space - A/D related variables

 cblock 0x0a0                ; starting address

    peakFlags               ; bit 0: 0 = 
                            ; bit 1: 0 = 
                            ; bit 2: 0 =
                            ; bit 3: 0 =
                            ; bit 4: 0 =
                            ; bit 5: 0 =
							; bit 6: 0 =
							; bit 7: 0 =

    pbScratch0
    pbScratch1
    pbScratch2
    pbScratch3
    pbScratch4

    ; rundata buffer pointers
    rundataXmtBufH
    rundataXmtBufL
    rundataCatchBufH
    rundataCatchBufL
    
    ; snapshot buffer pointers
    snapXmtBufH
    snapXmtBufL
    snapCatchBufH
    snapCatchBufL
    snapPeakBufH
    snapPeakBufL
    
    ; Snapshot buffer values -- see notes at top of file "Snapshot Bufferring"
    lastADAbsolute          ; absolute value of lastADSample
    peakADAbsolute          ; greatest A/D absolute value
    snapBufCnt              ; number of samples in Snapshot Buffer still needed for 64 after peak
    
    locClk                  ; linear location/clock position -- //WIP HSS// explain this better
    clkLoc                  ; clock position/linear location -- //WIP HSS// explain this better

 endc
 
; end Bank 01
;--------------------------------------------------------------------------

;--------------------------------------------------------------------------
; Bank 02 - 80 bytes of free space

 cblock 0x120               ; starting address

 endc
  
; end Bank 02
;--------------------------------------------------------------------------
  
;--------------------------------------------------------------------------
; Bank 03 - 16 bytes of free space
;
; Only 16 bytes available because of the Snapshot Buffer at Bank 3.2. For
; more info, see notes at top of file "Snapshot Bufferring".
;
  
 cblock 0x1a0               ; starting address
 
 endc
  
; end Bank 03
;--------------------------------------------------------------------------
 
;--------------------------------------------------------------------------
; Bank 03.2 - Variables for snapshot buffer 2
;
; This is one of the three snapshot buffers used for the snapshot of the 
; greatest absolute A/D value. 
;
; See notes at top of file "Snapshot Bufferring"
;
; The other two snapshot buffers must start at Bank 06.4 and Bank 09.6.
;
 
 cblock 0x1b0               ; starting address

    snapshotBuf1a:.64

 endc
 
; Compute address of snapShotBuf1 in linear data memory for use as a large buffer
SNAP_BUF1_OFFSET        EQU (snapshotBuf1a & 0x7f) - 0x20
SNAP_BUF1_LINEAR_ADDR   EQU ((snapshotBuf1a/.128)*.80)+0x2000+SNAP_BUF1_OFFSET
SNAP_BUF1_LINEAR_LOC_H  EQU high SNAP_BUF1_LINEAR_ADDR
SNAP_BUF1_LINEAR_LOC_L  EQU low SNAP_BUF1_LINEAR_ADDR
 
; end Bank 03.2
;--------------------------------------------------------------------------

;--------------------------------------------------------------------------
; Bank 04 - Variables for snapshot buffer 1 continued
;
; The snapshot buffer runs over into this bank.
;

 cblock 0x220               ; starting address
 
    snapshotBuf1b:.64
  
 endc
  
; end Bank 04
;--------------------------------------------------------------------------

;--------------------------------------------------------------------------
; Bank 05 - Variables for rundata buffer 1

 cblock 0x2a0               ; starting address

    maxPeak1H               ; overall A/D max MSB
    maxPeak1L               ; overall A/D max LSB
    maxPeakClk1             ; clock position of max A/D
    maxPeakLoc1             ; linear location of max A/D
    
    minPeak1H               ; overall A/D min MSB
    minPeak1L               ; overall A/D min LSB
    minPeakClk1             ; clock position of min A/D
    minPeakLoc1             ; linear location of min A/D

    mapBuf1:MAP_BUF_LEN

 endc
 
RUNDATA_BUF1_ADDRESS_H  EQU high maxPeak1H
RUNDATA_BUF1_ADDRESS_L  EQU low maxPeak1H
  
; end Bank 05
;--------------------------------------------------------------------------
  
;--------------------------------------------------------------------------
; Bank 06 - 32 bytes of free space
;
; Only 32 bytes available because of the Snapshot Buffer at Bank 6.4. For
; more info, see notes at top of file "Snapshot Bufferring".
;

 cblock 0x320               ; starting address

 endc
 
; end Bank 06
;--------------------------------------------------------------------------
 
;--------------------------------------------------------------------------
; Bank 06.4 - Variables for snapshot buffer 2
;
; This is one of the three snapshot buffers used for the snapshot of the 
; greatest absolute A/D value. 
; 
; See notes at top of file "Snapshot Bufferring"
;
; The other two snapshot buffers must start at Bank 03.2 and Bank 09.6.
;

 
 cblock 0x340               ; starting address

    snapshotBuf2a:.48

 endc
 
; Compute address of snapShotBuf2 in linear data memory for use as a large buffer
SNAP_BUF2_OFFSET        EQU (snapshotBuf2a & 0x7f) - 0x20
SNAP_BUF2_LINEAR_ADDR   EQU ((snapshotBuf2a/.128)*.80)+0x2000+SNAP_BUF2_OFFSET
SNAP_BUF2_LINEAR_LOC_H  EQU high SNAP_BUF2_LINEAR_ADDR
SNAP_BUF2_LINEAR_LOC_L  EQU low SNAP_BUF2_LINEAR_ADDR

; end Bank 06.4
;--------------------------------------------------------------------------
 
;--------------------------------------------------------------------------
; Bank 07 - Variables for snapshot buffer 2 continued
;
; The snapshot buffer occupies this entire bank.
;
  
 cblock 0x3a0                ; starting address

    snapshotBuf2b:.80
  
 endc
  
; end Bank 07
;--------------------------------------------------------------------------

;--------------------------------------------------------------------------
; Bank 08 - Variables for run data buffer 2

 cblock 0x420               ; starting address
 
    maxPeak2H               ; overall A/D max MSB
    maxPeak2L               ; overall A/D max LSB
    maxPeakClk2             ; clock position of max A/D
    maxPeakLoc2             ; linear location of max A/D
    
    minPeak2H               ; overall A/D min MSB
    minPeak2L               ; overall A/D min LSB
    minPeakClk2             ; clock position of min A/D
    minPeakLoc2             ; linear location of min A/D

    mapBuf2:MAP_BUF_LEN

 endc
 
RUNDATA_BUF2_ADDRESS_H  EQU high maxPeak2H
RUNDATA_BUF2_ADDRESS_L  EQU low maxPeak2H

; end Bank 08
;--------------------------------------------------------------------------
  
;--------------------------------------------------------------------------
; Bank 09 - 48 bytes of free space
;
; Only 48 bytes available because of the Snapshot Buffer at Bank 6.6. For
; more info, see notes at top of file "Snapshot Bufferring".
;

 cblock 0x4a0               ; starting address

 endc
 
; end Bank 09
;--------------------------------------------------------------------------
 
;--------------------------------------------------------------------------
; Bank 09.6 - Variables for snapshot buffer 3
;
; This is one of the three snapshot buffers used for the snapshot of the 
; greatest absolute A/D value. 
;
; See notes at top of file "Snapshot Bufferring"
;
; The other two snapshot buffers must be placed at Bank 3.2 and Bank 6.4.
;

 
 cblock 0x4d0               ; starting address

    snapshotBuf3a:.32

 endc
 
; Compute address of snapShotBuf3 in linear data memory for use as a large buffer
SNAP_BUF3_OFFSET        EQU (snapshotBuf3a & 0x7f) - 0x20
SNAP_BUF3_LINEAR_ADDR   EQU ((snapshotBuf3a/.128)*.80)+0x2000+SNAP_BUF3_OFFSET
SNAP_BUF3_LINEAR_LOC_H  EQU high SNAP_BUF3_LINEAR_ADDR
SNAP_BUF3_LINEAR_LOC_L  EQU low SNAP_BUF3_LINEAR_ADDR

; end Bank 09.6
;--------------------------------------------------------------------------
  
;--------------------------------------------------------------------------
; Bank 10 - Variables for snapshot buffer 3 continued
;
; The snapshot buffer occupies this entire bank.
;

 cblock 0x520                ; starting address
 
    snapshotBuf3b:.80

 endc
  
; end Bank 10
;--------------------------------------------------------------------------

;--------------------------------------------------------------------------
; Bank 11 - Variables for snapshot buffer 3 continued
;
; The snapshot buffer runs over into this bank.
;

 cblock 0x5a0                ; starting address
 
    snapshotBuf3c:.16

 endc

; end Bank 11
;--------------------------------------------------------------------------
  
;--------------------------------------------------------------------------
; Bank 12 - 48 bytes of free space

 cblock 0x620                ; starting address

 endc

; end Bank 12
;--------------------------------------------------------------------------
  
;--------------------------------------------------------------------------
; Common Ram mirrored in all banks - 16 bytes of free space

 cblock	0x070
  
    commonFlags             ; bit 0: 0 = channel is off, 1 = channel is on (BIT_CHAN_ONOFF)
                            ; bit 1: 0 = ready to send next byte, 1 = stop condition (BIT_RDY_STOP)
                            ; bit 2: 0 =
                            ; bit 3: 0 =
                            ; bit 4: 0 =
                            ; bit 5: 0 =
							; bit 6: 0 =
							; bit 7: 0 =
                            
    lastADSample            ; last A/D sample recorded
    lastSampleClk           ; clock position of the last A/D sample recorded
    lastSampleLoc           ; linear location of the last A/D sample recorded
    
 endc
    
; end Common Ram
;--------------------------------------------------------------------------

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
    clrf    commonFlags         ; clear common flags to make sure channel is off

    call    setupClock          ; set system clock source and frequency

    call    setupPortA          ; prepare Port A for I/O

    call    setupPortB          ; prepare Port B for I/O

    call    setupPortC          ; prepare Port C  for I/O

    call    setupTimer0         ; Timer 0 counts pulses on RC5 input

    call    setupIntOnChange    ; sets up interrupt-on-change for appropriate inputs

    call    initializeOutputs

    call    parseSlaveI2CAddress

    call    setupI2CSlave7BitMode ; prepare the I2C serial bus for use

    call    setupRundataAndSnapshotVars ; setup rundata and snapshot variables and pointers

    call    setupADConverter ; prepare A/D converter for use

;start of hardware configuration

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

    bsf     INTCON,PEIE     ; enable peripheral interrupts (Timers, A/D converter, etc.)

    bsf     INTCON,IOCIE    ; enable interrupt-on-change for enabled inputs

    bsf     INTCON,GIE      ; enable all interrupts

    return

; end of setup
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; parseSlaveI2CAddress
;
; Determines the PIC's I2C slave address by reading the address input bits which identify this
; PIC's address . Each Slave PIC's three address inputs are tied uniquely high/low.
;
; The value is stored in slaveI2CAddress variable.
; The address for the I2C module is NOT set...that is done when that module is set up.
;

parseSlaveI2CAddress:

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

; end of parseSlaveI2CAddress
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
    bcf     ANSELC,RC5

    banksel TRISC                       ; pin is input
    bsf     TRISC,RC5

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
    ; FOSC/32 clock

    ; set ADCON1 to configure the A/D converter
    ; bit 7 = 0 : left justify the result in ADRESH:ADRESL
    ; bit 6 = 1 : bits 6-4 : A/D Conversion Clock Select bits
    ; bit 5 = 1 :    110 -> FOSC/64 -> 1.33 us
    ; bit 4 = 0 : 
    ; bit 3 = 0 : unused
    ; bit 2 = 0 : unused
    ; bit 1 = 0 : bits 1-0: A/D voltage reference source
    ; bit 0 = 0 :    00 -> VREF+ connected to VDD

    banksel ADCON1
    movlw   b'00100000'
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

    return

; end of setupADConverter
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
    banksel i2cXmtBuf                   ; load FSR0 with buffer pointer
    movlw   high i2cXmtBuf
    movwf   FSR0H
    movlw   low i2cXmtBuf
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

    movf    lastADSample,W
    movwi   FSR0++

    banksel flags

    movlw   0x01                        ; unused -- for future use
    movwi   FSR0++

    movlw   0x02                        ; unused -- for future use
    movwi   FSR0++

    movlw   0x03                        ; unused -- for future use
    movwi   FSR0++

    movlw   .10                         ; number of data bytes in packet
    movwf   scratch0

    call    calcAndStoreCheckSumForI2CXmtBuf
    
    banksel i2cXmtBuf                   ; load FSR0 with buffer pointer
    movlw   high i2cXmtBuf
    movwf   FSR0H
    movlw   low i2cXmtBuf
    movwf   FSR0L
getAllStatus_xmtLoop:
    moviw   FSR0++
    call    sendByteViaI2C
    btfss   commonFlags,BIT_RDY_STOP    ; loop through unil stop condition received
    goto    getAllStatus_xmtLoop

    goto    cleanUpI2CAndReturn         ; made it to here, so stop condition received -- bail out

; end getAllStatus
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; calcAndStoreCheckSumForI2CXmtBuf
;
; Calculates the checksum for a series of bytes in the i2cXmtBuf buffer.
;
; On Entry:
;
; scratch0 contains number of bytes in series
;
; On Exit:
;
; The checksum will be stored at the end of the series.
; FSR0 points to the location after the checksum.
;

calcAndStoreCheckSumForI2CXmtBuf:

    banksel i2cXmtBuf                   ; load FSR0 with buffer pointer
    movlw   high i2cXmtBuf
    movwf   FSR0H
    movlw   low i2cXmtBuf
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

sumSLoop:                 ; sum the series

    addwf   INDF0,W
    addfsr  INDF0,1

    decfsz  scratch0,F
    goto    sumSLoop

    return

; end sumSeries
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; sendByteViaI2C
;
; Sends one byte via I2C.
;
; ON ENTRY:
;
; FSR1L = byte to send
;
; ON EXIT:
;
; commonFlags,BIT_RDY_STOP = 0  if ready to send next byte
; commonFlags,BIT_RDY_STOP = 1  if stop condition received
;

sendByteViaI2C:

    call    clearWCOL                   ; make sure error flag is reset to allow sending

    movwf   SSP1BUF                     ; send byte

    call    clearSSP1IF                 ; clear the I2C interrupt flag
    call    setCKP                      ; release the I2C clock line so master can send next byte

    call    waitForSSP1IFHighOrStop     ; wait for byte or stop condition to be received

    btfsc   commonFlags,BIT_RDY_STOP
    return                              ; need to stop and bail out

    banksel SSP1CON2
    btfss   SSP1CON2,ACKSTAT
    return                              ; good, so next byte can be sent
    
    call    waitForSSP1IFHighOrStop     ; wait for byte or stop condition to be received
    bsf     commonFlags,BIT_RDY_STOP    ; make sure to stop and bail out no matter what
    return

; end sendByteViaI2C
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; getRunData
;
; Transmits all of the run data it to the Master PIC via I2C. This function should be called when
; the PIC_GET_RUN_DATA_CMD is received from the Master PIC.
;
; Rundata currently consists of:
;   02 bytes    max peak
;   01 bytes    max peak clock position
;   01 bytes    max peak linear location
;   02 bytes    min peak
;   01 bytes    min peak clock position
;   01 bytes    min peak linear location
;   48 bytes    clock map
;   01 bytes    absolute peak
;   01 bytes    check sum
;   ---
;   58 bytes    total
;
; This function is done in the main code and not in interrupt code. It is expected that the A/D
; converter interrupt will occur one or more times during transmission of the peak data.
;
; See notes at top of file "Rundata Buffering" and "Snapshot Buffering"
;

getRunData:
    
    banksel peakFlags
    
    movf    rundataXmtBufH,W    ; point FSR0 at rundata xmt buffer
    movwf   FSR0H
    movf    rundataXmtBufL,W
    movwf   FSR0L
    
    movf    snapXmtBufH,W       ; point FSR1 at snapshot xmt buffer
    movwf   FSR1H
    movf    snapXmtBufL,W
    movwf   FSR1L
    
    btfss   commonFlags,BIT_CHAN_ONOFF
    goto    skipBufferResetAndSwap          ; skip buffer reset and swap if channel disabled
    call    resetRundataBuffer  ; reset xmt buffer since it was transmitted last time
    call    resetSnapshotBuffer ; reset snapshot buffer since it was transmitted last time
    
; end reset xmt buffer
    
; switch rundata xmt and catch pointers and snapshot xmt and peak pointers
    
    banksel peakFlags
    
    movf    rundataCatchBufH,W  ; temporarily store rundata catch buffer pointer
    movwf   FSR0H
    movf    rundataCatchBufL,W
    movwf   FSR0L
    
    movf    snapPeakBufH,W  ; temporarily store snapshot peak buffer pointer
    movwf   FSR1H
    movf    snapPeakBufL,W
    movwf   FSR1L
    
    movf    rundataXmtBufH,W
    bcf     INTCON,GIE          ; temporarily disable all interrupts to avoid interference
    movwf   rundataCatchBufH    
    movf    rundataXmtBufL,W
    movwf   rundataCatchBufL
    movf    snapXmtBufH,W
    movwf   snapPeakBufH
    bsf     INTCON,GIE          ; re-enable all interrupts
    
    banksel peakFlags
    
    movf    FSR0H,W             ; point rundata xmt buffer pointer at where catch was pointing
    movwf   rundataXmtBufH
    movf    FSR0L,W
    movwf   rundataXmtBufL
    
    movf    FSR1H,W             ; point snapshot xmt buffer pointer at where peak was pointing
    movwf   snapXmtBufH
    movf    FSR1L,W
    movwf   snapXmtBufL
    
; end switch rundata xmt and catch pointers and snapshot xmt and peak pointers
    
skipBufferResetAndSwap:
    
    banksel peakADAbsolute      ; store the peakADAbsolute so we can have our own value to work with 
    movf    peakADAbsolute,W
    banksel scratch0
    movwf   scratch1            ; (handleADInterrupt: might change peakADAbsolute often)
    
    movlw   .56                 ; sum bytes in rundata xmt buffer
    movwf   scratch0
    call    sumSeries
    addwf   scratch1,W          ; peakADAbsolute will be sent in the rundata packet, so include
    comf    WREG,W              ; use two's complement to get checksum value
    addlw   .1
    movwf   scratch2            ; store checksum for future use
    
    ; xmt rundata buffer via I2C
    
    addfsr  FSR0,-.32           ; move FSR0 to first byte in rundata (-56)
    addfsr  FSR0,-.24           ; addfsr instruction can only handle -32 to 31
    movlw   .56
    movwf   scratch0
getRunData_xmtLoop:
    moviw   FSR0++
    call    sendByteViaI2C
    btfsc   commonFlags,BIT_RDY_STOP
    goto    cleanUpI2CAndReturn ; bail out if stop condition received for some reason
    banksel scratch0
    decfsz  scratch0
    goto    getRunData_xmtLoop  ; loop until all of rundata buffer is sent
    
    ; end xmt rundata buffer via I2C
    
    movf    scratch1,W          ; xmt the peakADAbsolute along with the rundata
    call    sendByteViaI2C
    btfsc   commonFlags,BIT_RDY_STOP
    goto    cleanUpI2CAndReturn ; bail out if stop condition received for some reason
    
    banksel scratch2
    movf    scratch2,W          ; xmt the checksum
    call    sendByteViaI2C

    goto    cleanUpI2CAndReturn ; clean up I2C and return because nothing left to xmt

; end getRunData
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; xmtSnapshotBuffer
;
; Transmits the snapshot buffer to the Master PIC via I2C. This function should be called when the 
; PIC_GET_SNAPSHOT_CMD is received from the Master PIC.
;
; Rundata currently consists of:
;   001 bytes   address of most recent A/D value put in snap peak buffer
;   128 bytes   snapshot buffer
;   001 bytes   check sum
;   ---
;   130 bytes    total
;
; This function is done in the main code and not in interrupt code. It is expected that the A/D
; converter interrupt will occur one or more times during transmission of the data.
;
; See notes at top of file "Snapshot Buffering"
;

xmtSnapshotBuffer:
    
    banksel peakFlags
    
    movf    snapXmtBufH,W       ; point FSR0 at snapshot xmt buffer
    movwf   FSR0H
    movf    snapXmtBufL,W
    movwf   FSR0L
    
    banksel scratch0
    
    movwf   scratch1            ; store address of most recent A/D value put in snap peak buffer
    
    ; calculate checksum
    
    movlw   SNAPSHOT_BUF_LEN    ; set up counter for xmtSnapBuf_SumLoop
    movwf   scratch0
    
    clrf    WREG
    
    addwf   scratch1,W          ; include address of most recent A/D value put in snap peak buffer
    
xmtSnapBuf_SumLoop:             ; sum bytes in snapshot xmt buffer
    addfsr  INDF0,1             ; add first so it points at start of buffer
    bcf     FSR0,.7             ; clear bit 7 so 128-byte cicular buffer automatically rolls around
    addwf   INDF0,W
    decfsz  scratch0,F
    goto    xmtSnapBuf_SumLoop
    
    comf    WREG,W              ; use two's complement to get checksum value
    addlw   .1
    movwf   scratch2            ; store checksum for future use
    
    ; end calculate checksum
    
    ; xmt address of most recent A/D value put in snap peak buffer
    
    movf    scratch1,W
    call    sendByteViaI2C
    btfsc   commonFlags,BIT_RDY_STOP
    goto    cleanUpI2CAndReturn ; bail out if stop condition received for some reason
    
    ; end xmt address of most recent A/D value put in snap peak buffer
    
    ; xmt snapshot buffer via I2C
    banksel snapXmtBufH         ; point FSR0 at snapshot xmt buffer
    movf    snapXmtBufH,W
    movwf   FSR0H
    movf    snapXmtBufL,W
    movwf   FSR0L
    movlw   SNAPSHOT_BUF_LEN    ; set up counter for xmtSnapshotBuffer_xmtLoop
    banksel scratch0
    movwf   scratch0
    addfsr  INDF0,1             ; add first so it points at start of buffer
xmtSnapshotBuffer_xmtLoop:
    bcf     FSR0,.7             ; clear bit 7 so 128-byte cicular buffer automatically rolls around
    moviw   FSR0++
    call    sendByteViaI2C
    btfsc   commonFlags,BIT_RDY_STOP
    goto    cleanUpI2CAndReturn ; bail out if stop condition received for some reason
    banksel scratch0
    decfsz  scratch0
    goto    xmtSnapshotBuffer_xmtLoop  ; loop until all of rundata buffer is sent
    
    ; end xmt snapshot buffer via I2C
    
    banksel scratch2
    movf    scratch2,W          ; xmt the checksum
    call    sendByteViaI2C

    goto    cleanUpI2CAndReturn ; clean up I2C and return because nothing left to xmt

; end xmtSnapshotBuffer
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; turnChannelOff
;
; Turns the channel off for this Slave PIC.
;
; When channel is turned off:
;   flag set indicating channel is off
;   A/D Converter interrupts disabled
;   some important values are cleared
;   rundata and snapshot xmt buffers populated with with 0s
;   rundata catch, snapshot catch and  snapshot peak buffers are reset/prepped for when the channel 
;       is turned on
;

turnChannelOff:
    
    bcf     commonFlags,BIT_CHAN_ONOFF
    
    banksel PIE1                        ; disable A/D interrupts
    bcf     PIE1, ADIE
    
    banksel peakFlags
    
    clrf    lastADSample                ; clear important values -- should remain cleared until
    clrf    lastADAbsolute              ; channel is turned on again
    clrf    lastSampleClk
    clrf    lastSampleLoc
    clrf    peakADAbsolute
    clrf    snapBufCnt
    
    ; set rundata xmt buffer to 0s
    movf    rundataXmtBufH,W
    movwf   FSR0H
    movf    rundataXmtBufL,W
    movwf   FSR0L
    
    movlw   0x00                ; set max to .127 (actually zero with host offset)
    movwi   FSR0++
    movlw   .127
    movwi   FSR0++
    movlw   0x00
    movwi   FSR0++              ; clear maxPeakClk
    movwi   FSR0++              ; clear maxPeakLoc
    
    movlw   0x00                ; set min to .127 (actually zero with host offset)
    movwi   FSR0++
    movlw   .127
    movwi   FSR0++
    movlw   0x00
    movwi   FSR0++              ; clear minPeakClk
    movwi   FSR0++              ; clear minPeakLoc
    
    movlw   MAP_BUF_LEN         ; set all of clock map buffer to 0
    call    clearMemBlock
    ; end set rundata xmt buffer to 0s
    
    ; reset rundata catch buffer for when channel is enabled again
    banksel peakFlags
    movf    rundataCatchBufH,W
    movwf   FSR0H
    movf    rundataCatchBufL,W
    movwf   FSR0L
    call    resetRundataBuffer
    ; end reset rundata catch buffer
    
    ; reset snapshot xmt, peak, and catch buffers
    banksel peakFlags
    movf    snapXmtBufH,W
    movwf   FSR1H
    movf    snapXmtBufL,W
    movwf   FSR1L
    call    resetSnapshotBuffer
    
    banksel peakFlags
    movf    snapPeakBufH,W
    movwf   FSR1H
    movf    snapPeakBufL,W
    movwf   FSR1L
    call    resetSnapshotBuffer
    
    banksel peakFlags
    movf    snapCatchBufH,W
    movwf   FSR1H
    movf    snapCatchBufL,W
    movwf   FSR1L
    call    resetSnapshotBuffer
    ; end reset snapshot xmt, peak, and catch buffers
    
    return

; end turnChannelOff
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; turnChannelOn
;
; Turns the channel on for this Slave PIC.
;
; When channel is turned on:
;   flag set indicating channel is on
;   A/D Converter interrupts are enabled
;   an A/D conversion is started
;

turnChannelOn:
    
    bsf     commonFlags,BIT_CHAN_ONOFF
    
    banksel ADCON0                      ; start A/D conversions
    bsf     ADCON0,ADGO
    
    banksel PIE1                        ; enable A/D interrupts and start converting
    bsf     PIE1,ADIE
    
    return

; end turnChannelOn
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; resetRundataBuffer
;
; Resets a rundata buffer.
;
; ON ENTRY:
;
; FSR0 points at buffer start   
;

resetRundataBuffer:

    movlw   0x00
    movwi   FSR0++              ; put 0s in MSB of max
    movwi   FSR0++              ; set max to anti-peak
    movwi   FSR0++              ; clear maxPeakClk
    movwi   FSR0++              ; clear maxPeakLoc
    
    movwi   FSR0++              ; put 0s in MSB of min
    movlw   0xFF                ; set min to anti-peak
    movwi   FSR0++
    movlw   0x00
    movwi   FSR0++              ; clear minPeakClk
    movwi   FSR0++              ; clear minPeakLoc
    
    banksel pbScratch0
    movlw   MAP_BUF_LEN         ; set all of clock map buffer to 0 (max anti-peak)
    movwf   pbScratch0
    movlw   0x00
getRunData_clockMapLoop:
    movwi   FSR0++
    decfsz  pbScratch0
    goto    getRunData_clockMapLoop
    
    return
    
; end resetRundataBuffer
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; resetSnapshotBuffer
;
; Sets all bytes up to 255 in a memory block to zero. This is used instead of clearMemBlock so that
; FSR1 can be used.
;
; ON ENTRY:
;
; W contains number of bytes to zero
; FSR1 points to start of buffer
;

resetSnapshotBuffer:

    banksel scratch0                        ; store number of bytes in block
    movlw   SNAPSHOT_BUF_LEN
    movwf   scratch0

    movlw   0x00

rSBLoop:

    movwi   FSR1++
    bcf     FSR1,.7             ; clear bit 7 so 128-byte cicular buffer automatically rolls around
    decfsz  scratch0,F
    goto    rSBLoop

    return

; end of resetSnapshotBuffer
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

    call    clearSSP1IF         ; clear the I2C interrupt flag

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
    
    call    setCKP              ; release the I2C clock line so master can send next byte

    call    waitForSSP1IFHighOrStop     ; wait for byte or stop condition to be received

    btfsc   commonFlags,BIT_RDY_STOP
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
    
    movf    masterCmd,W
    sublw   PIC_SET_ONOFF_CMD
    btfsc   STATUS,Z
    goto    handleHostChannelOnOffCmd
    
    movf    masterCmd,W
    sublw   PIC_SET_LOCATION_CMD
    btfsc   STATUS,Z
    goto    handleHostSetLocationCmd
    
    movf    masterCmd,W
    sublw   PIC_SET_CLOCK_CMD
    btfsc   STATUS,Z
    goto    handleHostSetClockCmd

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
    sublw   PIC_GET_RUN_DATA_CMD
    btfsc   STATUS,Z
    goto    getRunData
    
    movf    masterCmd,W
    sublw   PIC_GET_SNAPSHOT_CMD
    btfsc   STATUS,Z
    goto    xmtSnapshotBuffer
    
    movf    masterCmd,W
    sublw   PIC_GET_LAST_AD_VALUE_CMD
    btfsc   STATUS,Z
    goto    transmitTwoByteValueToMaster

    return

; end handleI2CTransmit
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; handleHostChannelOnOffCmd
;
; Turns the channel for this Slave PIC on or off depending on the next byte sent by the Master PIC.
; 
; If the next byte is 0, the channel is turned off; if it is 1, then the channel is turned on.
;

handleHostChannelOnOffCmd:
    
    movlw   .1                          ; store 1 bytes from I2C
    call    receiveBytesFromI2C
    
    banksel scratch3
    movf    scratch3,W                  ; move on/off byte into W to set Z flag
    
    btfss   STATUS,Z
    goto    turnChannelOn               ; if byte received was 1 then turn channel on
    goto    turnChannelOff              ; if byte received was 0 then turn channel on

; end handleHostChannelOnOffCmd
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; handleHostSetLocationCmd
;
; Stores the next byte sent by the Master PIC as the linear location/clock position. 
;
; //WIP HSS// explain why it is called "linear location/clock position"
;

handleHostSetLocationCmd:
    
    movlw   .1                          ; receive 1 byte from Master PIC via I2C
    call    receiveBytesFromI2C
    
    banksel scratch3                    ; store received byte as linear location/clock position
    movf    scratch3,W
    banksel locClk
    movwf   locClk
    
    return

; end handleHostSetLocationCmd
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; handleHostSetClockCmd
;
; Stores the next byte sent by the Master PIC as the clock position/linear location and in TMR0. 
;
; //WIP HSS// explain why it is called "linear location/clock position"
;

handleHostSetClockCmd:
    
    movlw   .1                          ; receive 1 byte from Master PIC via I2C
    call    receiveBytesFromI2C
    
    banksel scratch3                    ; store received byte
    movf    scratch3,W
    banksel clkLoc
    movwf   clkLoc
    banksel TMR0
    movwf   TMR0
    
    return

; end handleHostSetClockCmd
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; transmitTwoByteValueToMaster
;
; Determines which two-byte value to send to the master pic and then sends it.
;
; Which value to send is determined by the last command received from the master.
;

transmitTwoByteValueToMaster:
    
    banksel i2cXmtBuf                   ; load FSR0 with buffer pointer
    movlw   high i2cXmtBuf
    movwf   FSR0H
    movlw    low i2cXmtBuf
    movwf   FSR0L

    movf    masterCmd,W                 ; get last AD value if command says to
    sublw   PIC_GET_LAST_AD_VALUE_CMD
    btfsc   STATUS,Z
    call    getLastADValue
    
    movlw   .2                          ; calculate and store checksum
    movwf   scratch0
    call    calcAndStoreCheckSumForI2CXmtBuf

    banksel i2cXmtBuf                   ; reload FSR0 with buffer pointer
    movlw   high i2cXmtBuf
    movwf   FSR0H
    movlw    low i2cXmtBuf
    movwf   FSR0L
xmtTwoByteValue_xmtLoop:
    moviw   FSR0++
    call    sendByteViaI2C
    btfss   commonFlags,BIT_RDY_STOP    ; loop through unil stop condition received
    goto    xmtTwoByteValue_xmtLoop

    goto    cleanUpI2CAndReturn         ; made it to here, so stop condition received -- bail out

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

    ;movlw   0x34
    ;movwi   1[FSR0]
    
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
; Normally, this number is equal to the number of bytes read unless the master
; sends too few or too many.
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

    movlw   high scratch3           ; point to first byte of receive buffer
    movwf   FSR0H
    movlw   low scratch3
    movwf   FSR0L

rBFILoop1:

    call    waitForSSP1IFHighOrStop     ; wait for byte or stop condition to be received

    btfsc   commonFlags,BIT_RDY_STOP
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
; ON EXIT:
;
; commonFlags,BIT_RDY_STOP = 0  if ready to send next byte
; commonFlags,BIT_RDY_STOP = 1  if stop condition received
;

waitForSSP1IFHighOrStop:

    ifdef debug_on              ; if debugging, don't wait for interrupt to be set high as the MSSP is not
    goto  wfshosByteReceived    ; simulated by the IDE
    endif

wfshos1:

    banksel PIR1
    btfsc   PIR1,SSP1IF         ; check interrupt flag
    goto    wfshosByteReceived  ; return if flag set

    banksel SSP1STAT
    btfsc   SSP1STAT,P          ; check stop condition received flag
    goto    wfshosStopReceived  ; return if flag set

    goto    wfshos1

wfshosByteReceived:

    ; set send next byte flag and return
    bcf     commonFlags,BIT_RDY_STOP
    return

wfshosStopReceived:

    ; set stop flag and return
    bsf     commonFlags,BIT_RDY_STOP
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

	retfie                          ; return and enable interrupts

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
; This function is called when a sample is ready in the A/D converter. The new sample is stored 
; and used to process and handle data pertaining to the Rundata and Snapshot buffers.
;
; See notes at top of file "Rundata Buffering" and "Snapshot Buffering"
;
; ON ENTRY:
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
    
    movwf   lastADSample        ; store A/D sample
    
    banksel TMR0                ; store current clock position
    movf    TMR0,W
    movwf   lastSampleClk
    
    banksel peakFlags
    
    movf    rundataCatchBufH,W  ; point FSR0 at rundata catch buffer
    movwf   FSR0H
    movf    rundataCatchBufL,W
    movwf   FSR0L
    
    addfsr  FSR0,.8             ; point FSR0 at beginning of clock map
    
    moviw   -.7[FSR0]           ; check to see if new max
    subwf   lastADSample,W
    btfss   STATUS,C            ; if set then it is new max (lastADSample > maxPeak)
    goto    handleADInterrupt_checkMin
    
    movf    lastADSample,W      ; store last A/D sample as new max peak (maxPeak)     
    movwi   -.7[FSR0]
    movf    lastSampleClk,W     ; store last clock sample as new max clock (maxPeakClk)
    movwi   -.6[FSR0]
    movlw   0x00                ; store 0s as maxPeakLoc //WIP HSS// -- should actually do stuff
    movwi   -.5[FSR0]
    
handleADInterrupt_checkMin:
    
    moviw   -.3[FSR0]           ; check to see if new min
    subwf   lastADSample,W
    btfsc   STATUS,C            ; if clear then it is new min (lastADSample < minPeak)
    goto    handleADInterrupt_doClockMap
    
    movf    lastADSample,W      ; store last A/D sample as new min peak (minPeak)     
    movwi   -.3[FSR0]
    movf    lastSampleClk,W     ; store last clock sample as new min clock (minPeakClk)
    movwi   -.2[FSR0]
    movlw   0x00                ; store 0s as minPeakLoc //WIP HSS// -- should actually do stuff
    movwi    -.1[FSR0]
    
; end handleADInterrupt_checkMin:
    
handleADInterrupt_doClockMap:
   
    movf    lastSampleClk,W     ; point FSR0 at proper clock position
    addwf   FSR0,F              ; NOTE: For this to work, clock map buffer has to be in one bank
    
    ; Convert lastADSample to absolute value
    movf    lastADSample,W
    xorlw   0x80                ; flip top bit of AD sample to make it a signed byte
    btfss   WREG,.7             ; if top bit is set, take two's compliment to get absolute value
    goto    handleADInterrupt_checkClockMap
    
    comf    WREG,W              ; value was negative; take two's compliment to get absolute value
    addlw   .1
    ; end Convert lastADSample to absolute value
    
handleADInterrupt_checkClockMap:
    
    movwf   lastADAbsolute      ; store the absolute value
    
    subwf   INDF0,W             ; see if lastADAbsolute is new peak for current clock position
    movf    lastADAbsolute,W
    btfss   STATUS,C            ; if clear then it is new peak for current clock position
    movwf   INDF0               ; lastADAbsolute > last peak for clock position, so replace old peak
    
; end handleADInterrupt_checkClockMap:
    
; begin stuff with the snapshot buffer
    
    subwf   peakADAbsolute,W    ; see if lastADAbsolute is new overall peak
    btfsc   STATUS,C            ; if clear then new peak was found (lastADAbsolute > peakADAbsolute)
    goto    adInterrupt_putLastSmplInSnapBuf
    
    movf    lastADAbsolute,W    ; store new peak and reset counter
    movwf   peakADAbsolute
    movlw   SNAPSHOT_BUF_LEN/2
    movwf   snapBufCnt
    goto    adInterrupt_putLastSmplInSnapBuf    ; skip over counter check
    
handleADInterrupt_checkCounter:
    
    movf    snapBufCnt,F
    btfsc   STATUS,Z
    goto    adInterrupt_putLastSmplInSnapBuf    ; skip over counter dec if it is already zero
    
    decfsz  snapBufCnt,F        ; decrement and check counter
    goto    adInterrupt_putLastSmplInSnapBuf    ; skip over buffer switch if not zero yet
    
    ; made it to here, which means the buffers need to be switched
    movf    snapPeakBufH,W      ; temporarily store snapPeakBufH
    movwf   FSR0H
    movf    snapCatchBufH,W     ; replace snapPeakBufH with snapCatchBufH
    movwf   snapPeakBufH
    movf    FSR0H,W             ; replace snapCatchBufH with snapPeakBufH
    movwf   snapCatchBufH
    
    movf    snapCatchBufL,W     ; replace snapPeakBufL with snapCatchBufL
    movwf   snapPeakBufL
    clrf    snapCatchBufL       ; point snapCatchBufL at start of buffer
    ; done switching buffers
    
adInterrupt_putLastSmplInSnapBuf:
    
    movf    snapCatchBufH,W
    movwf   FSR0H
    movf    snapCatchBufL,W
    movwf   FSR0L
    
    movf    lastADSample,W      ; store last A/D sample in the snapshot buffer
    movwf   INDF0
    
    incf    snapCatchBufL       ; increment for next time
    bcf     snapCatchBufL,.7    ; clear bit 7 so 128-byte cicular buffer automatically rolls around
    
    retfie                      ; return and enable interrupts
    
; end of handleADInterrupt
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupRundataAndSnapshotVars
;
; Sets up variables and pointers releated to rundata and snapshot buffering.
;
; See notes at top of file "Rundata Buffering" and "Snapshot Buffering".
;

setupRundataAndSnapshotVars:

    banksel peakFlags

    clrf    peakFlags           ; clear peak flags

    ; assign pointers
    movlw   RUNDATA_BUF1_ADDRESS_H
    movwf   rundataCatchBufH
    movlw   RUNDATA_BUF1_ADDRESS_L
    movwf   rundataCatchBufL

    movlw   RUNDATA_BUF2_ADDRESS_H
    movwf   rundataXmtBufH
    movlw   RUNDATA_BUF2_ADDRESS_L
    movwf   rundataXmtBufL
    
    movlw   SNAP_BUF1_LINEAR_LOC_H
    movwf   snapCatchBufH
    movlw   SNAP_BUF1_LINEAR_LOC_L
    movwf   snapCatchBufL
    
    movlw   SNAP_BUF2_LINEAR_LOC_H
    movwf   snapPeakBufH
    movlw   SNAP_BUF2_LINEAR_LOC_L
    movwf   snapPeakBufL
    
    movlw   SNAP_BUF3_LINEAR_LOC_H
    movwf   snapXmtBufH
    movlw   SNAP_BUF3_LINEAR_LOC_L
    movwf   snapXmtBufL
    
    ; turn the channel off
    call    turnChannelOff

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

; end of setupRundataAndSnapshotVars
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

    END