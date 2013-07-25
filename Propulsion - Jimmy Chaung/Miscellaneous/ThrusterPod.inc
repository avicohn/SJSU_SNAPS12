;Main includes
;	Port Reg = value/8  Bit = value MOD 8
;	All names of the form PIN_<port><bit>

#define PIN_A0  31744
#define PIN_A1  31745
#define PIN_A2  31746
;   ...
	nolist
#define PIN_A3  31747
#define PIN_A4  31748
#define PIN_A5  31749

#define PIN_B0  31752
#define PIN_B1  31753
#define PIN_B2  31754
#define PIN_B3  31755
#define PIN_B4  31756
#define PIN_B5  31757
#define PIN_B6  31758
#define PIN_B7  31759

#define PIN_C0  31760
#define PIN_C1  31761
#define PIN_C2  31762
#define PIN_C3  31763
#define PIN_C4  31764
#define PIN_C5  31765
#define PIN_C6  31766
#define PIN_C7  31767

#define PIN_D0  31768
#define PIN_D1  31769
#define PIN_D2  31770
#define PIN_D3  31771
#define PIN_D4  31772
#define PIN_D5  31773
#define PIN_D6  31774
#define PIN_D7  31775

#define PIN_E0  31776
#define PIN_E1  31777
#define PIN_E2  31778
#define PIN_E3  31779
#define PIN_E4  31780
#define PIN_E5  31781
#define PIN_E6  31782
#define PIN_E7  31783

#define PIN_F0  31784
#define PIN_F1  31785
#define PIN_F2  31786
#define PIN_F3  31787
#define PIN_F4  31788
#define PIN_F5  31789
#define PIN_F6  31790
#define PIN_F7  31791

#define PIN_G0  31792
#define PIN_G1  31793
#define PIN_G2  31794
#define PIN_G3  31795
#define PIN_G4  31796

#define PIN_H0  31800
#define PIN_H1  31801
#define PIN_H2  31802
#define PIN_H3  31803
#define PIN_H4  31804
#define PIN_H5  31805
#define PIN_H6  31806
#define PIN_H7  31807

#define PIN_J0  31808
#define PIN_J1  31809
#define PIN_J2  31810
#define PIN_J3  31811
#define PIN_J4  31812
#define PIN_J5  31813
#define PIN_J6  31814
	list
#define PIN_J7  31815

output_high	macro	pin
		bsf	(pin)>>3&0xff,(pin)&0x07,0
		endm

output_low	macro	pin
		bcf	(pin)>>3&0xff,(pin)&0x07,0
		endm

input		macro	pin
		bcf	STATUS,C
		btfsc	(pin)>>3&0xff,(pin)&0x07,0
		bsf	STATUS,C
		endm

delay_ms	macro	time
		movlw	time
		call	Delay_ms
		endm

hang	macro	code
		movlw	code
		call	FlashLED
		bra		$-4
		endm
