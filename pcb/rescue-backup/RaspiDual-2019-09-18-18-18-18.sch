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
Sheet 5 5
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
L Raspberry_Pi_2_3 J9
U 1 1 59A5DC3B
P 6550 3200
F 0 "J9" H 7250 1950 50  0000 C CNN
F 1 "Raspberry_Pi_2_3" H 6150 4100 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_2x20_Pitch2.54mm_SMD" H 6550 4850 50  0000 C CNN
F 3 "" H 6600 3050 50  0001 C CNN
	1    6550 3200
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR22
U 1 1 59A5DE57
P 4200 4600
F 0 "#PWR22" H 4200 4350 50  0001 C CNN
F 1 "GND" H 4200 4450 50  0000 C CNN
F 2 "" H 4200 4600 50  0001 C CNN
F 3 "" H 4200 4600 50  0001 C CNN
	1    4200 4600
	1    0    0    -1  
$EndComp
$Comp
L R_Small R17
U 1 1 59A5DE5D
P 2700 5250
F 0 "R17" H 2730 5270 50  0000 L CNN
F 1 "5.6k" H 2730 5210 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 2700 5250 50  0000 C CNN
F 3 "" H 2700 5250 50  0001 C CNN
	1    2700 5250
	0    1    1    0   
$EndComp
$Comp
L R_Small R18
U 1 1 59A5DE64
P 2950 5450
F 0 "R18" H 2980 5470 50  0000 L CNN
F 1 "10k" H 2980 5410 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 2950 5450 50  0000 C CNN
F 3 "" H 2950 5450 50  0001 C CNN
	1    2950 5450
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR21
U 1 1 59A5DE6B
P 2950 5600
F 0 "#PWR21" H 2950 5350 50  0001 C CNN
F 1 "GND" H 2950 5450 50  0000 C CNN
F 2 "" H 2950 5600 50  0001 C CNN
F 3 "" H 2950 5600 50  0001 C CNN
	1    2950 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 4600 4200 4500
Wire Wire Line
	2950 5350 2950 5250
Connection ~ 2950 5250
Wire Wire Line
	2950 5550 2950 5600
Wire Wire Line
	2450 5250 2600 5250
$Comp
L Raspberry_Pi_2_3 J8
U 1 1 59A5DE80
P 4600 3200
F 0 "J8" H 5300 1950 50  0000 C CNN
F 1 "Raspberry_Pi_2_3" H 4200 4100 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x20_Pitch2.54mm_SMD" H 4600 4550 50  0000 C CNN
F 3 "" H 4650 3050 50  0001 C CNN
	1    4600 3200
	1    0    0    -1  
$EndComp
Text HLabel 4350 1650 0    60   Input ~ 0
Vout
Text HLabel 3400 3200 0    60   Output ~ 0
PIon
Text HLabel 2450 5250 0    60   Input ~ 0
PIctrl
Wire Wire Line
	5500 2300 5650 2300
Wire Wire Line
	5500 2500 5650 2500
Wire Wire Line
	5500 2400 5650 2400
Wire Wire Line
	5500 3000 5650 3000
Wire Wire Line
	5500 3200 5650 3200
Wire Wire Line
	5500 3100 5650 3100
Wire Wire Line
	5500 2700 5650 2700
Wire Wire Line
	5650 2800 5500 2800
Wire Wire Line
	5500 3300 5650 3300
Wire Wire Line
	5650 3400 5500 3400
Wire Wire Line
	5500 3600 5650 3600
Wire Wire Line
	5650 3700 5500 3700
Wire Wire Line
	5500 3900 5650 3900
Wire Wire Line
	5650 4000 5500 4000
Wire Wire Line
	4350 1650 6750 1650
Wire Wire Line
	6750 1650 6750 1900
Wire Wire Line
	6650 1700 6650 1900
Wire Wire Line
	4400 1700 6650 1700
Wire Wire Line
	4700 1900 4700 1750
Wire Wire Line
	4700 1750 6450 1750
Wire Wire Line
	6450 1750 6450 1900
Wire Wire Line
	6350 1900 6350 1800
Wire Wire Line
	6350 1800 4800 1800
Wire Wire Line
	4800 1800 4800 1900
Wire Wire Line
	4900 4500 4900 4550
Wire Wire Line
	4200 4550 6950 4550
Wire Wire Line
	6250 4550 6250 4500
Wire Wire Line
	6350 4500 6350 4600
Wire Wire Line
	6350 4600 4800 4600
Wire Wire Line
	4800 4600 4800 4500
Wire Wire Line
	4700 4500 4700 4650
Wire Wire Line
	4700 4650 6450 4650
Wire Wire Line
	6450 4650 6450 4500
Wire Wire Line
	6550 4500 6550 4700
Wire Wire Line
	6550 4700 4600 4700
Wire Wire Line
	4600 4700 4600 4500
Wire Wire Line
	4500 4500 4500 4750
Wire Wire Line
	4500 4750 6650 4750
Wire Wire Line
	6650 4750 6650 4500
Wire Wire Line
	6750 4800 6750 4500
Wire Wire Line
	4400 4800 6750 4800
Wire Wire Line
	4400 4800 4400 4500
Wire Wire Line
	4300 4500 4300 4900
Wire Wire Line
	4300 4900 6850 4900
Wire Wire Line
	6850 4900 6850 4500
Wire Wire Line
	6950 4550 6950 4500
Connection ~ 6850 4550
Wire Wire Line
	3700 4000 3600 4000
Wire Wire Line
	3600 4000 3600 5000
Wire Wire Line
	3600 5000 7550 5000
Wire Wire Line
	7550 5000 7550 4000
Wire Wire Line
	7550 4000 7450 4000
Wire Wire Line
	7450 3900 7600 3900
Wire Wire Line
	7600 3900 7600 5050
Wire Wire Line
	7600 5050 3550 5050
Wire Wire Line
	3550 5050 3550 3900
Wire Wire Line
	3550 3900 3700 3900
Wire Wire Line
	3700 3600 3500 3600
Wire Wire Line
	3500 3600 3500 5100
Wire Wire Line
	3500 5100 7650 5100
Wire Wire Line
	7650 5100 7650 3600
Wire Wire Line
	7650 3600 7450 3600
Wire Wire Line
	7450 3500 7700 3500
Wire Wire Line
	7700 3500 7700 5150
Wire Wire Line
	7700 5150 3450 5150
Wire Wire Line
	3450 5150 3450 3500
Wire Wire Line
	3450 3500 3700 3500
Wire Wire Line
	3700 3400 3400 3400
Wire Wire Line
	3400 3400 3400 5200
Wire Wire Line
	3400 5200 7750 5200
Wire Wire Line
	7750 5200 7750 3400
Wire Wire Line
	7750 3400 7450 3400
Wire Wire Line
	7450 3300 7800 3300
Wire Wire Line
	7800 3300 7800 5250
Wire Wire Line
	7800 5250 2800 5250
Wire Wire Line
	3350 5250 3350 3300
Wire Wire Line
	3700 2500 3650 2500
Wire Wire Line
	3650 2500 3650 1400
Wire Wire Line
	3650 1400 7500 1400
Wire Wire Line
	7500 1400 7500 2500
Wire Wire Line
	7500 2500 7450 2500
Wire Wire Line
	7450 2600 7550 2600
Wire Wire Line
	7550 2600 7550 1350
Wire Wire Line
	7550 1350 3600 1350
Wire Wire Line
	3600 1350 3600 2600
Wire Wire Line
	3600 2600 3700 2600
Wire Wire Line
	3700 2700 3550 2700
Wire Wire Line
	3550 2700 3550 1300
Wire Wire Line
	3550 1300 7600 1300
Wire Wire Line
	7600 1300 7600 2700
Wire Wire Line
	7600 2700 7450 2700
Wire Wire Line
	7450 2800 7650 2800
Wire Wire Line
	7650 2800 7650 1250
Wire Wire Line
	7650 1250 3500 1250
Wire Wire Line
	3500 1250 3500 2800
Wire Wire Line
	3500 2800 3700 2800
Wire Wire Line
	3700 2900 3450 2900
Wire Wire Line
	3450 2900 3450 1200
Wire Wire Line
	3450 1200 7700 1200
Wire Wire Line
	7700 1200 7700 2900
Wire Wire Line
	7700 2900 7450 2900
Wire Wire Line
	7450 3000 7750 3000
Wire Wire Line
	7750 3000 7750 1150
Wire Wire Line
	7750 1150 3400 1150
Wire Wire Line
	3400 1150 3400 3000
Wire Wire Line
	3400 3000 3700 3000
Wire Wire Line
	3700 3100 3350 3100
Wire Wire Line
	3350 3100 3350 1100
Wire Wire Line
	3350 1100 7800 1100
Wire Wire Line
	7800 1100 7800 3100
Wire Wire Line
	7800 3100 7450 3100
Wire Wire Line
	7450 3200 7750 3200
Wire Wire Line
	3700 3200 3400 3200
Text HLabel 7750 3200 2    60   Output ~ 0
PIon
Wire Wire Line
	4500 1700 4500 1900
Wire Wire Line
	4400 1650 4400 1900
Connection ~ 4400 1650
Connection ~ 4500 1700
Connection ~ 4400 1700
Connection ~ 4200 4550
Connection ~ 4300 4550
Connection ~ 3350 5250
Wire Wire Line
	3350 3300 3700 3300
Connection ~ 4400 4550
Connection ~ 4500 4550
Connection ~ 4600 4550
Connection ~ 4700 4550
Connection ~ 4800 4550
Connection ~ 4900 4550
Connection ~ 6350 4550
Connection ~ 6250 4550
Connection ~ 6450 4550
Connection ~ 6550 4550
Connection ~ 6650 4550
Connection ~ 6750 4550
$EndSCHEMATC
