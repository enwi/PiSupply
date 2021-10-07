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
Sheet 2 5
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 1050 1400 0    60   Input ~ 0
Vin
Text HLabel 3650 1000 2    60   Output ~ 0
5Vout
$Comp
L MT3608 U2
U 1 1 59A0AE20
P 2200 1250
F 0 "U2" H 2200 1350 60  0000 C CNN
F 1 "MT3608" H 2200 650 60  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-6" H 2200 1250 60  0000 C CNN
F 3 "" H 2200 1250 60  0001 C CNN
	1    2200 1250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR13
U 1 1 59A0AE27
P 1450 1750
F 0 "#PWR13" H 1450 1500 50  0001 C CNN
F 1 "GND" H 1450 1600 50  0000 C CNN
F 2 "" H 1450 1750 50  0001 C CNN
F 3 "" H 1450 1750 50  0001 C CNN
	1    1450 1750
	1    0    0    -1  
$EndComp
$Comp
L L L1
U 1 1 59A0AE2D
P 2200 1000
F 0 "L1" V 2150 1000 50  0000 C CNN
F 1 "4.7µH - CDRH104R" V 2275 1000 50  0000 C CNN
F 2 "nodetech_custom:CDRH10RT_footprint" H 2200 1000 50  0000 C CNN
F 3 "" H 2200 1000 50  0001 C CNN
	1    2200 1000
	0    -1   -1   0   
$EndComp
$Comp
L D_Schottky_ALT D5
U 1 1 59A0AE34
P 2950 1000
F 0 "D5" H 2950 1100 50  0000 C CNN
F 1 "SS34" H 2950 900 50  0000 C CNN
F 2 "Diodes_SMD:D_SMA" H 2950 1000 50  0000 C CNN
F 3 "" H 2950 1000 50  0001 C CNN
	1    2950 1000
	-1   0    0    1   
$EndComp
$Comp
L R_Small R6
U 1 1 59A0AE3B
P 3200 1500
F 0 "R6" H 3230 1520 50  0000 L CNN
F 1 "47k" H 3230 1460 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 3200 1500 50  0000 C CNN
F 3 "" H 3200 1500 50  0001 C CNN
	1    3200 1500
	1    0    0    -1  
$EndComp
$Comp
L R_Small R7
U 1 1 59A0AE42
P 3200 1900
F 0 "R7" H 3230 1920 50  0000 L CNN
F 1 "6.2k" H 3230 1860 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 3200 1900 50  0000 C CNN
F 3 "" H 3200 1900 50  0001 C CNN
	1    3200 1900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR14
U 1 1 59A0AE49
P 3200 2200
F 0 "#PWR14" H 3200 1950 50  0001 C CNN
F 1 "GND" H 3200 2050 50  0000 C CNN
F 2 "" H 3200 2200 50  0001 C CNN
F 3 "" H 3200 2200 50  0001 C CNN
	1    3200 2200
	1    0    0    -1  
$EndComp
$Comp
L C_Small C7
U 1 1 59A0AE4F
P 3550 1700
F 0 "C7" H 3560 1770 50  0000 L CNN
F 1 "10µF" H 3560 1620 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 3550 1700 50  0000 C CNN
F 3 "" H 3550 1700 50  0001 C CNN
	1    3550 1700
	1    0    0    -1  
$EndComp
$Comp
L C_Small C6
U 1 1 59A0AE56
P 1250 1550
F 0 "C6" H 1260 1620 50  0000 L CNN
F 1 "10µF" H 1260 1470 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 1250 1550 50  0000 C CNN
F 3 "" H 1250 1550 50  0001 C CNN
	1    1250 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1050 1400 1800 1400
Wire Wire Line
	1450 1000 2050 1000
Wire Wire Line
	2700 1400 2600 1400
Wire Wire Line
	2350 1000 2800 1000
Wire Wire Line
	2700 1400 2700 1000
Connection ~ 2700 1000
Wire Wire Line
	3100 1000 3650 1000
Wire Wire Line
	3200 1600 3200 1800
Wire Wire Line
	2600 1700 3200 1700
Connection ~ 3200 1700
Wire Wire Line
	3200 2000 3200 2200
Wire Wire Line
	3200 2100 3550 2100
Wire Wire Line
	3550 2100 3550 1800
Connection ~ 3200 2100
Wire Wire Line
	3550 1000 3550 1600
Connection ~ 3200 1000
Connection ~ 3550 1000
Wire Wire Line
	1250 1400 1250 1450
Connection ~ 1250 1400
Wire Wire Line
	1250 1650 1250 1700
Wire Wire Line
	1250 1700 1800 1700
Connection ~ 1450 1700
Wire Wire Line
	1450 1700 1450 1750
Wire Wire Line
	3200 1000 3200 1400
Text HLabel 1750 1550 0    60   Input ~ 0
EN
Wire Wire Line
	1450 1000 1450 1400
Connection ~ 1450 1400
Wire Wire Line
	1800 1550 1750 1550
$EndSCHEMATC
