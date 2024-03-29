EESchema Schematic File Version 2
LIBS:PiSupply-rescue
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:nodetech_custom
LIBS:PiSupply-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 5
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L USB_OTG-RESCUE-PiSupply J1
U 1 1 598E04A1
P 850 2250
F 0 "J1" H 650 2700 50  0000 L CNN
F 1 "USB_OTG" H 650 2600 50  0000 L CNN
F 2 "nodetech_custom:usb_micro_smd2" H 1000 2200 50  0000 C CNN
F 3 "" H 1000 2200 50  0001 C CNN
	1    850  2250
	1    0    0    -1  
$EndComp
$Comp
L C_Small C1
U 1 1 598E06D6
P 1650 2350
F 0 "C1" H 1660 2420 50  0000 L CNN
F 1 "10µF" H 1660 2270 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 1650 2350 50  0000 C CNN
F 3 "" H 1650 2350 50  0001 C CNN
	1    1650 2350
	1    0    0    -1  
$EndComp
$Comp
L R_Small R3
U 1 1 598E08CF
P 2750 2250
F 0 "R3" H 2780 2270 50  0000 L CNN
F 1 "10k" H 2780 2210 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 2750 2250 50  0000 C CNN
F 3 "" H 2750 2250 50  0001 C CNN
	1    2750 2250
	1    0    0    -1  
$EndComp
$Comp
L D_Schottky_ALT D4
U 1 1 598E0E41
P 5450 2100
F 0 "D4" H 5450 2200 50  0000 C CNN
F 1 "SS34" H 5450 2000 50  0000 C CNN
F 2 "Diodes_SMD:D_SMA" H 5450 2100 50  0000 C CNN
F 3 "" H 5450 2100 50  0001 C CNN
	1    5450 2100
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR1
U 1 1 598E1666
P 850 2750
F 0 "#PWR1" H 850 2500 50  0001 C CNN
F 1 "GND" H 850 2600 50  0000 C CNN
F 2 "" H 850 2750 50  0001 C CNN
F 3 "" H 850 2750 50  0001 C CNN
	1    850  2750
	1    0    0    -1  
$EndComp
$Comp
L BARREL_JACK J2
U 1 1 59911113
P 1000 1100
F 0 "J2" H 1000 1295 50  0000 C CNN
F 1 "BARREL_JACK" H 1000 945 50  0000 C CNN
F 2 "Connectors:BARREL_JACK" H 1000 1100 50  0000 C CNN
F 3 "" H 1000 1100 50  0001 C CNN
	1    1000 1100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR2
U 1 1 59957272
P 1400 1300
F 0 "#PWR2" H 1400 1050 50  0001 C CNN
F 1 "GND" H 1400 1150 50  0000 C CNN
F 2 "" H 1400 1300 50  0001 C CNN
F 3 "" H 1400 1300 50  0001 C CNN
	1    1400 1300
	1    0    0    -1  
$EndComp
$Comp
L R_Small R4
U 1 1 5995DA85
P 4550 1700
F 0 "R4" H 4580 1720 50  0000 L CNN
F 1 "10k" H 4580 1660 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 4550 1700 50  0000 C CNN
F 3 "" H 4550 1700 50  0001 C CNN
	1    4550 1700
	-1   0    0    -1  
$EndComp
$Comp
L D_Schottky_ALT D2
U 1 1 5995EAB8
P 5000 1850
F 0 "D2" H 5000 1950 50  0000 C CNN
F 1 "SS34" H 5000 1750 50  0000 C CNN
F 2 "Diodes_SMD:D_SMA" H 5000 1850 50  0000 C CNN
F 3 "" H 5000 1850 50  0001 C CNN
	1    5000 1850
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR4
U 1 1 5996652E
P 2750 2450
F 0 "#PWR4" H 2750 2200 50  0001 C CNN
F 1 "GND" H 2750 2300 50  0000 C CNN
F 2 "" H 2750 2450 50  0001 C CNN
F 3 "" H 2750 2450 50  0001 C CNN
	1    2750 2450
	1    0    0    -1  
$EndComp
Text GLabel 1850 2050 2    60   Output ~ 0
5Vusb
Text GLabel 1900 1000 2    60   Output ~ 0
12Vin
$Comp
L GND #PWR7
U 1 1 599F3AF4
P 4550 1850
F 0 "#PWR7" H 4550 1600 50  0001 C CNN
F 1 "GND" H 4550 1700 50  0000 C CNN
F 2 "" H 4550 1850 50  0001 C CNN
F 3 "" H 4550 1850 50  0001 C CNN
	1    4550 1850
	-1   0    0    -1  
$EndComp
NoConn ~ 1150 2250
NoConn ~ 1150 2350
NoConn ~ 1150 2450
NoConn ~ 750  2650
NoConn ~ 1300 1100
$Sheet
S 5900 2650 800  500 
U 59A0982C
F0 "Boost5V" 60
F1 "Boost5V.sch" 60
F2 "Vin" I L 5900 2750 60 
F3 "5Vout" O R 6700 2750 60 
F4 "EN" I L 5900 3050 60 
$EndSheet
$Sheet
S 2950 2650 1650 450 
U 59A11CB4
F0 "Battery Charge + Protection" 60
F1 "BatChargeProtection.sch" 60
F2 "Vchrg" I L 2950 2850 60 
F3 "BAT+" O R 4600 2750 60 
F4 "BAT-" O R 4600 3000 60 
$EndSheet
$Comp
L GND #PWR8
U 1 1 59A1AE7C
P 4700 3050
F 0 "#PWR8" H 4700 2800 50  0001 C CNN
F 1 "GND" H 4700 2900 50  0000 C CNN
F 2 "" H 4700 3050 50  0001 C CNN
F 3 "" H 4700 3050 50  0001 C CNN
	1    4700 3050
	1    0    0    -1  
$EndComp
$Sheet
S 3400 1200 950  550 
U 59A1D659
F0 "Buck5V" 60
F1 "Buck5V.sch" 60
F2 "Vin" I L 3400 1450 60 
F3 "5Vout" O R 4350 1450 60 
$EndSheet
Text GLabel 3350 1450 0    60   Input ~ 0
12Vin
$Comp
L ATTINY45-20SU U1
U 1 1 59A31D46
P 3850 4350
F 0 "U1" H 2700 4750 50  0000 C CNN
F 1 "ATTINY45-20SU" H 4850 3950 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 4800 4350 50  0000 C CNN
F 3 "" H 3850 4350 50  0001 C CNN
	1    3850 4350
	1    0    0    -1  
$EndComp
Text GLabel 4950 2800 3    60   Output ~ 0
MCVcc
Text GLabel 5450 4100 2    60   Input ~ 0
MCVcc
$Comp
L GND #PWR9
U 1 1 59A33CA6
P 5300 4650
F 0 "#PWR9" H 5300 4400 50  0001 C CNN
F 1 "GND" H 5300 4500 50  0000 C CNN
F 2 "" H 5300 4650 50  0001 C CNN
F 3 "" H 5300 4650 50  0001 C CNN
	1    5300 4650
	1    0    0    -1  
$EndComp
Text GLabel 8500 2750 2    60   Output ~ 0
Vout
Text GLabel 2450 4200 0    60   BiDi ~ 0
Ctrl
Text GLabel 2150 4100 0    60   Output ~ 0
B
$Comp
L Polyfuse F1
U 1 1 59A359F6
P 7600 2750
F 0 "F1" V 7500 2750 50  0000 C CNN
F 1 "NANOSMD400LR-C-2" V 7700 2750 50  0000 C CNN
F 2 "Fuse_Holders_and_Fuses:Fuse_SMD1206_Reflow" V 7150 1850 50  0000 L CNN
F 3 "" H 7600 2750 50  0001 C CNN
	1    7600 2750
	0    1    1    0   
$EndComp
Text GLabel 2150 4300 0    60   BiDi ~ 0
LED
Text GLabel 2450 4400 0    60   Input ~ 0
PIon
Text GLabel 2150 4500 0    60   Output ~ 0
PIctrl
Text GLabel 1350 5150 0    60   Input ~ 0
B
Text GLabel 1450 5350 0    60   Input ~ 0
LED
$Comp
L R R1
U 1 1 59A3ECDF
P 1650 5350
F 0 "R1" V 1730 5350 50  0000 C CNN
F 1 "1k5" V 1650 5350 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 1580 5350 50  0000 C CNN
F 3 "" H 1650 5350 50  0001 C CNN
	1    1650 5350
	0    1    1    0   
$EndComp
$Comp
L GND #PWR3
U 1 1 59A3F877
P 1850 5550
F 0 "#PWR3" H 1850 5300 50  0001 C CNN
F 1 "GND" H 1850 5400 50  0000 C CNN
F 2 "" H 1850 5550 50  0001 C CNN
F 3 "" H 1850 5550 50  0001 C CNN
	1    1850 5550
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X03 J6
U 1 1 59A45122
P 3550 5300
F 0 "J6" H 3550 5500 50  0000 C CNN
F 1 "CONN_02X03" H 3550 5100 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x03_Pitch2.54mm" H 3550 5300 50  0000 C CNN
F 3 "" H 3550 4100 50  0001 C CNN
	1    3550 5300
	-1   0    0    1   
$EndComp
Text GLabel 2400 4600 0    60   Input ~ 0
RST
Text GLabel 3250 5200 0    60   Output ~ 0
Ctrl
Text GLabel 3000 5300 0    60   Output ~ 0
LED
Text GLabel 4250 5300 2    60   Input ~ 0
B
Text GLabel 3250 5400 0    60   Output ~ 0
RST
$Comp
L GND #PWR5
U 1 1 59A4F571
P 3950 5400
F 0 "#PWR5" H 3950 5150 50  0001 C CNN
F 1 "GND" H 3950 5250 50  0000 C CNN
F 2 "" H 3950 5400 50  0001 C CNN
F 3 "" H 3950 5400 50  0001 C CNN
	1    3950 5400
	1    0    0    -1  
$EndComp
$Comp
L R_Small R2
U 1 1 59A52315
P 2450 4750
F 0 "R2" H 2480 4770 50  0000 L CNN
F 1 "10k" H 2480 4710 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 2450 4750 50  0000 C CNN
F 3 "" H 2450 4750 50  0001 C CNN
	1    2450 4750
	1    0    0    -1  
$EndComp
Text GLabel 2500 4900 2    60   Input ~ 0
MCVcc
$Comp
L SI3443DDV Q2
U 1 1 599DCA9E
P 5400 1450
F 0 "Q2" H 5400 1550 60  0000 C CNN
F 1 "SI3443DDV" H 5400 1450 60  0000 C CNN
F 2 "nodetech_custom:TSOP-6" H 5400 1450 60  0000 C CNN
F 3 "" H 5400 1450 60  0001 C CNN
	1    5400 1450
	0    1    1    0   
$EndComp
$Comp
L SI3443DDV Q1
U 1 1 599E039F
P 5250 2900
F 0 "Q1" H 5250 3000 60  0000 C CNN
F 1 "SI3443DDV" H 5250 2900 60  0000 C CNN
F 2 "nodetech_custom:TSOP-6" H 5250 2900 60  0000 C CNN
F 3 "" H 5250 2900 60  0001 C CNN
	1    5250 2900
	1    0    0    1   
$EndComp
Text GLabel 5250 1100 1    60   Output ~ 0
5Vusb
Wire Wire Line
	2750 2100 5300 2100
Wire Wire Line
	850  2650 850  2750
Connection ~ 5250 2100
Wire Wire Line
	1300 1200 1600 1200
Wire Wire Line
	1400 1200 1400 1300
Wire Wire Line
	1300 1000 1900 1000
Wire Wire Line
	1150 2050 1850 2050
Wire Wire Line
	1650 2000 1650 2250
Connection ~ 1650 2050
Wire Wire Line
	850  2700 1650 2700
Wire Wire Line
	1650 2700 1650 2450
Connection ~ 850  2700
Wire Wire Line
	4350 1450 5000 1450
Wire Wire Line
	5250 1650 5250 2500
Wire Wire Line
	5050 2750 4600 2750
Wire Wire Line
	2750 2350 2750 2450
Wire Wire Line
	2750 2150 2750 2100
Wire Wire Line
	4750 1850 4850 1850
Connection ~ 4750 1450
Wire Wire Line
	5450 2750 5900 2750
Wire Wire Line
	4550 1850 4550 1800
Wire Wire Line
	4750 1450 4750 1850
Wire Wire Line
	4550 1400 4550 1600
Connection ~ 4550 1450
Wire Wire Line
	4600 3000 4700 3000
Wire Wire Line
	4700 3000 4700 3050
Wire Wire Line
	2850 2100 2850 2850
Wire Wire Line
	2850 2850 2950 2850
Wire Wire Line
	3350 1450 3400 1450
Wire Wire Line
	5150 1850 5300 1850
Connection ~ 5250 1850
Wire Wire Line
	4950 2750 4950 2800
Connection ~ 4950 2750
Wire Wire Line
	5200 4100 5450 4100
Wire Wire Line
	5200 4600 5300 4600
Wire Wire Line
	7750 2750 8500 2750
Wire Wire Line
	2150 4100 2500 4100
Wire Wire Line
	2450 4200 2500 4200
Wire Wire Line
	2500 4300 2150 4300
Wire Wire Line
	2450 4400 2500 4400
Wire Wire Line
	2500 4500 2150 4500
Wire Wire Line
	1450 5350 1500 5350
Wire Wire Line
	1800 5350 1950 5350
Wire Wire Line
	1350 5150 1950 5150
Wire Wire Line
	1150 5250 1950 5250
Wire Wire Line
	1150 5250 1150 5500
Wire Wire Line
	1150 5500 1850 5500
Wire Wire Line
	1950 5450 1850 5450
Wire Wire Line
	1850 5450 1850 5550
Connection ~ 1850 5500
Wire Wire Line
	2400 4600 2500 4600
Wire Wire Line
	3250 5200 3300 5200
Wire Wire Line
	3000 5300 3300 5300
Wire Wire Line
	4250 5300 3800 5300
Wire Wire Line
	3250 5400 3300 5400
Wire Wire Line
	3800 5400 3950 5400
Wire Wire Line
	2450 4600 2450 4650
Connection ~ 2450 4600
Wire Wire Line
	2500 4900 2450 4900
Wire Wire Line
	2450 4900 2450 4850
Wire Wire Line
	5150 1250 5300 1250
Wire Wire Line
	5250 1250 5250 1100
$Sheet
S 6900 4450 800  600 
U 59A5D913
F0 "RaspiDual" 60
F1 "RaspiDual.sch" 60
F2 "Vout" I L 6900 4550 60 
F3 "PIon" O L 6900 4800 60 
F4 "PIctrl" I L 6900 4950 60 
$EndSheet
Text GLabel 6700 4950 0    60   Input ~ 0
PIctrl
Text GLabel 6700 4800 0    60   Output ~ 0
PIon
Text GLabel 6700 4550 0    60   Input ~ 0
Vout
Wire Wire Line
	6700 4550 6900 4550
Wire Wire Line
	6700 4800 6900 4800
Wire Wire Line
	6700 4950 6900 4950
$Comp
L GND #PWR11
U 1 1 59A7C005
P 8250 3050
F 0 "#PWR11" H 8250 2800 50  0001 C CNN
F 1 "GND" H 8250 2900 50  0000 C CNN
F 2 "" H 8250 3050 50  0001 C CNN
F 3 "" H 8250 3050 50  0001 C CNN
	1    8250 3050
	1    0    0    -1  
$EndComp
$Comp
L C_Small C3
U 1 1 59A7CB40
P 8050 2900
F 0 "C3" H 8060 2970 50  0000 L CNN
F 1 "100µF" H 8060 2820 50  0000 L CNN
F 2 "Capacitors_SMD:C_1210" H 8050 2900 50  0000 C CNN
F 3 "" H 8050 2900 50  0001 C CNN
	1    8050 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 3000 8250 3050
Wire Wire Line
	5050 2650 5050 2800
Connection ~ 5050 2700
Connection ~ 5050 2750
Wire Wire Line
	5600 2100 5650 2100
Wire Wire Line
	5650 2100 5650 2750
Connection ~ 5650 2750
Text GLabel 5750 3050 0    60   Input ~ 0
Ctrl
Wire Wire Line
	5750 3050 5900 3050
Connection ~ 2850 2100
$Comp
L LED_ALT D3
U 1 1 59AAE744
P 5000 2250
F 0 "D3" H 5000 2350 50  0000 C CNN
F 1 "LED Green" H 5000 2150 50  0000 C CNN
F 2 "LEDs:LED_0603" H 5000 2250 50  0000 C CNN
F 3 "" H 5000 2250 50  0001 C CNN
	1    5000 2250
	1    0    0    -1  
$EndComp
$Comp
L R_Small R5
U 1 1 59AB0DB8
P 4600 2250
F 0 "R5" H 4630 2270 50  0000 L CNN
F 1 "1k" H 4630 2210 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 4600 2250 50  0000 C CNN
F 3 "" H 4600 2250 50  0001 C CNN
	1    4600 2250
	0    1    1    0   
$EndComp
Connection ~ 5250 2250
$Comp
L GND #PWR6
U 1 1 59AB0F71
P 4400 2250
F 0 "#PWR6" H 4400 2000 50  0001 C CNN
F 1 "GND" H 4400 2100 50  0000 C CNN
F 2 "" H 4400 2250 50  0001 C CNN
F 3 "" H 4400 2250 50  0001 C CNN
	1    4400 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 2250 4400 2250
Connection ~ 5200 1250
Connection ~ 5250 1250
Wire Wire Line
	4700 2250 4850 2250
Wire Wire Line
	4750 2400 4750 2250
Connection ~ 4750 2250
Wire Wire Line
	5150 2250 5250 2250
$Comp
L LED_ALT D1
U 1 1 59ACA3AF
P 4750 2550
F 0 "D1" H 4750 2650 50  0000 C CNN
F 1 "LED Red" H 4750 2450 50  0000 C CNN
F 2 "LEDs:LED_0603" H 4750 2550 50  0000 C CNN
F 3 "" H 4750 2550 50  0001 C CNN
	1    4750 2550
	0    -1   1    0   
$EndComp
Wire Wire Line
	4750 2750 4750 2700
Connection ~ 4750 2750
$Comp
L CONN_01X02 J3
U 1 1 59ACD1B4
P 1800 1150
F 0 "J3" H 1800 1300 50  0000 C CNN
F 1 "CONN_01X02" V 1900 1150 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x02_Pitch2.54mm" H 1800 1150 50  0000 C CNN
F 3 "" H 1800 1150 50  0001 C CNN
	1    1800 1150
	1    0    0    1   
$EndComp
Wire Wire Line
	1550 950  1550 1100
Wire Wire Line
	1550 1100 1600 1100
Connection ~ 1550 1000
Connection ~ 1400 1200
$Comp
L CONN_01X02 J4
U 1 1 59AD2599
P 2150 5200
F 0 "J4" H 2150 5350 50  0000 C CNN
F 1 "CONN_01X02" V 2250 5200 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x02_Pitch2.54mm" H 2150 5200 50  0000 C CNN
F 3 "" H 2150 5200 50  0001 C CNN
	1    2150 5200
	1    0    0    1   
$EndComp
$Comp
L CONN_01X02 J5
U 1 1 59AD264F
P 2150 5400
F 0 "J5" H 2150 5550 50  0000 C CNN
F 1 "CONN_01X02" V 2250 5400 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x02_Pitch2.54mm" H 2150 5400 50  0000 C CNN
F 3 "" H 2150 5400 50  0001 C CNN
	1    2150 5400
	1    0    0    1   
$EndComp
$Comp
L USB_A-RESCUE-PiSupply J7
U 1 1 59AB08FD
P 8250 3600
F 0 "J7" H 8050 4050 50  0000 L CNN
F 1 "USB_A" H 8050 3950 50  0000 L CNN
F 2 "Connectors:USB_A" H 8400 3550 50  0000 C CNN
F 3 "" H 8400 3550 50  0001 C CNN
	1    8250 3600
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7800 2750 7800 3400
Connection ~ 7800 2750
Wire Wire Line
	7650 3400 7950 3400
$Comp
L GND #PWR12
U 1 1 59AB4E49
P 8250 4100
F 0 "#PWR12" H 8250 3850 50  0001 C CNN
F 1 "GND" H 8250 3950 50  0000 C CNN
F 2 "" H 8250 4100 50  0001 C CNN
F 3 "" H 8250 4100 50  0001 C CNN
	1    8250 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 4000 8250 4100
NoConn ~ 8350 4000
NoConn ~ 7950 3600
NoConn ~ 7950 3700
NoConn ~ 3800 5200
$Comp
L C_Small C4
U 1 1 59AC3628
P 8250 2900
F 0 "C4" H 8260 2970 50  0000 L CNN
F 1 "100µF" H 8260 2820 50  0000 L CNN
F 2 "Capacitors_SMD:C_1210" H 8250 2900 50  0000 C CNN
F 3 "" H 8250 2900 50  0001 C CNN
	1    8250 2900
	1    0    0    -1  
$EndComp
$Comp
L C_Small C5
U 1 1 59AC36CC
P 8450 2900
F 0 "C5" H 8460 2970 50  0000 L CNN
F 1 "100µF" H 8460 2820 50  0000 L CNN
F 2 "Capacitors_SMD:C_1210" H 8450 2900 50  0000 C CNN
F 3 "" H 8450 2900 50  0001 C CNN
	1    8450 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 2800 8450 2800
Connection ~ 8250 2800
Wire Wire Line
	8050 3000 8450 3000
Connection ~ 8250 3000
Wire Wire Line
	8250 2700 8250 2800
Connection ~ 8250 2750
$Comp
L C_Small C2
U 1 1 59ACB046
P 5300 4250
F 0 "C2" H 5310 4320 50  0000 L CNN
F 1 "10µF" H 5310 4170 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 5300 4250 50  0000 C CNN
F 3 "" H 5300 4250 50  0001 C CNN
	1    5300 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 4150 5300 4100
Connection ~ 5300 4100
Wire Wire Line
	5300 4350 5300 4650
Connection ~ 5300 4600
$Comp
L R_Small R19
U 1 1 59B317D0
P 6950 3100
F 0 "R19" H 6980 3120 50  0000 L CNN
F 1 "10k" H 6980 3060 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" V 6850 3100 50  0000 C CNN
F 3 "" H 6950 3100 50  0001 C CNN
	1    6950 3100
	0    1    -1   0   
$EndComp
$Comp
L GND #PWR10
U 1 1 59B317D6
P 7100 3650
F 0 "#PWR10" H 7100 3400 50  0001 C CNN
F 1 "GND" H 7100 3500 50  0000 C CNN
F 2 "" H 7100 3650 50  0001 C CNN
F 3 "" H 7100 3650 50  0001 C CNN
	1    7100 3650
	1    0    0    -1  
$EndComp
$Comp
L C_Small C15
U 1 1 59BC6D20
P 7650 3550
F 0 "C15" H 7660 3620 50  0000 L CNN
F 1 "100µF" H 7660 3470 50  0000 L CNN
F 2 "Capacitors_SMD:C_1210" H 7650 3550 50  0000 C CNN
F 3 "" H 7650 3550 50  0001 C CNN
	1    7650 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7650 3400 7650 3450
Connection ~ 7800 3400
Wire Wire Line
	7650 3650 7650 4050
Wire Wire Line
	7650 4050 8250 4050
Connection ~ 8250 4050
$Comp
L TEST_1P J11
U 1 1 59BCA548
P 1650 2000
F 0 "J11" H 1650 2270 50  0000 C CNN
F 1 "TEST_1P" H 1650 2200 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 1850 2000 50  0001 C CNN
F 3 "" H 1850 2000 50  0001 C CNN
	1    1650 2000
	1    0    0    -1  
$EndComp
$Comp
L TEST_1P J10
U 1 1 59BCAC0B
P 1550 950
F 0 "J10" H 1550 1220 50  0000 C CNN
F 1 "TEST_1P" H 1550 1150 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 1750 950 50  0001 C CNN
F 3 "" H 1750 950 50  0001 C CNN
	1    1550 950 
	1    0    0    -1  
$EndComp
$Comp
L TEST_1P J12
U 1 1 59BCB311
P 4550 1400
F 0 "J12" H 4550 1670 50  0000 C CNN
F 1 "TEST_1P" H 4550 1600 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 4750 1400 50  0001 C CNN
F 3 "" H 4750 1400 50  0001 C CNN
	1    4550 1400
	1    0    0    -1  
$EndComp
$Comp
L TEST_1P J13
U 1 1 59BCBDE9
P 5300 1850
F 0 "J13" H 5300 2120 50  0000 C CNN
F 1 "TEST_1P" H 5300 2050 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 5500 1850 50  0001 C CNN
F 3 "" H 5500 1850 50  0001 C CNN
	1    5300 1850
	0    1    1    0   
$EndComp
$Comp
L TEST_1P J14
U 1 1 59BCC6AF
P 5800 2700
F 0 "J14" H 5800 2970 50  0000 C CNN
F 1 "TEST_1P" H 5800 2900 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 6000 2700 50  0001 C CNN
F 3 "" H 6000 2700 50  0001 C CNN
	1    5800 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 2700 5800 2750
Connection ~ 5800 2750
$Comp
L TEST_1P J15
U 1 1 59BCD1B8
P 6850 2650
F 0 "J15" H 6850 2920 50  0000 C CNN
F 1 "TEST_1P" H 6850 2850 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 7050 2650 50  0001 C CNN
F 3 "" H 7050 2650 50  0001 C CNN
	1    6850 2650
	1    0    0    -1  
$EndComp
$Comp
L TEST_1P J16
U 1 1 59BCD9B6
P 8250 2700
F 0 "J16" H 8250 2970 50  0000 C CNN
F 1 "TEST_1P" H 8250 2900 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 8450 2700 50  0001 C CNN
F 3 "" H 8450 2700 50  0001 C CNN
	1    8250 2700
	1    0    0    -1  
$EndComp
$Comp
L SI3443DDV Q3
U 1 1 5A944E97
P 7100 2600
F 0 "Q3" H 7100 2700 60  0000 C CNN
F 1 "SI3443DDV" H 7100 2600 60  0000 C CNN
F 2 "nodetech_custom:TSOP-6" H 7100 2600 60  0000 C CNN
F 3 "" H 7100 2600 60  0001 C CNN
	1    7100 2600
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6700 2750 6900 2750
Wire Wire Line
	7300 2700 7300 2850
Connection ~ 7300 2750
Connection ~ 7300 2800
Wire Wire Line
	7300 2750 7450 2750
$Comp
L R_Small R20
U 1 1 5A945B9F
P 6650 3400
F 0 "R20" H 6680 3420 50  0000 L CNN
F 1 "1k" H 6680 3360 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" V 6800 3300 50  0000 C CNN
F 3 "" H 6650 3400 50  0001 C CNN
	1    6650 3400
	0    1    1    0   
$EndComp
$Comp
L Q_NPN_BEC Q4
U 1 1 5A9461A1
P 7000 3400
F 0 "Q4" H 7200 3450 50  0000 L CNN
F 1 "Q_NPN_BEC" H 7200 3350 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 7000 3150 50  0000 C CNN
F 3 "" H 7000 3400 50  0001 C CNN
	1    7000 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 3400 6800 3400
Text GLabel 6500 3400 0    60   Input ~ 0
Ctrl
Wire Wire Line
	6500 3400 6550 3400
Wire Wire Line
	6850 2650 6850 3100
Connection ~ 6850 2750
Wire Wire Line
	7100 3600 7100 3650
Wire Wire Line
	7100 3000 7100 3200
Connection ~ 7100 3100
Wire Wire Line
	7050 3100 7100 3100
$EndSCHEMATC
