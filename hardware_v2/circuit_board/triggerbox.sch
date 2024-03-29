EESchema Schematic File Version 4
LIBS:triggerbox-cache
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date "1 dec 2013"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	3750 2850 3950 2850
Wire Wire Line
	3950 2850 3950 3050
Wire Wire Line
	5050 2750 5050 3050
Wire Wire Line
	1900 3150 1900 3050
Wire Wire Line
	1900 3050 2050 3050
NoConn ~ 3750 2350
NoConn ~ 3750 1750
NoConn ~ 3750 2150
NoConn ~ 3750 2450
NoConn ~ 3750 2550
NoConn ~ 3750 2650
NoConn ~ 3750 2950
NoConn ~ 3750 3050
NoConn ~ 3750 3150
NoConn ~ 2050 3150
NoConn ~ 2050 2950
NoConn ~ 2050 2750
NoConn ~ 2050 2650
NoConn ~ 2050 2550
NoConn ~ 2050 2450
NoConn ~ 2050 2350
NoConn ~ 2050 2250
NoConn ~ 2050 1950
NoConn ~ 2050 1850
$Comp
L Device:LED D1
U 1 1 529B8B36
P 4200 2750
F 0 "D1" H 4200 2850 50  0000 C CNN
F 1 "LED" H 4200 2650 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 4200 2750 50  0001 C CNN
F 3 "" H 4200 2750 50  0001 C CNN
	1    4200 2750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 529B8ADF
P 4750 2750
F 0 "R1" V 4830 2750 50  0000 C CNN
F 1 "220" V 4750 2750 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 4750 2750 50  0001 C CNN
F 3 "" H 4750 2750 50  0001 C CNN
	1    4750 2750
	0    1    1    0   
$EndComp
$Comp
L triggerbox:ARDUINO_NANO U2
U 1 1 529B84F2
P 2900 2400
F 0 "U2" H 2450 3350 60  0000 C CNN
F 1 "ARDUINO_NANO" H 2900 1550 60  0000 C CNN
F 2 "Module:Arduino_Nano" H 2900 2400 50  0001 C CNN
F 3 "" H 2900 2400 50  0001 C CNN
	1    2900 2400
	-1   0    0    1   
$EndComp
Wire Wire Line
	3750 2750 4050 2750
Wire Wire Line
	4350 2750 4600 2750
Wire Wire Line
	4900 2750 5050 2750
$Comp
L Connector:Screw_Terminal_01x04 J1
U 1 1 5F64EBA4
P 1700 5500
F 0 "J1" H 1780 5492 50  0000 L CNN
F 1 "Trigger1" H 1780 5401 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_bornier-4_P5.08mm" H 1700 5500 50  0001 C CNN
F 3 "~" H 1700 5500 50  0001 C CNN
	1    1700 5500
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x04 J2
U 1 1 5F65163C
P 1700 6100
F 0 "J2" H 1780 6092 50  0000 L CNN
F 1 "Trigger2" H 1780 6001 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_bornier-4_P5.08mm" H 1700 6100 50  0001 C CNN
F 3 "~" H 1700 6100 50  0001 C CNN
	1    1700 6100
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x04 J3
U 1 1 5F651B4C
P 1700 6700
F 0 "J3" H 1780 6692 50  0000 L CNN
F 1 "Trigger3" H 1780 6601 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_bornier-4_P5.08mm" H 1700 6700 50  0001 C CNN
F 3 "~" H 1700 6700 50  0001 C CNN
	1    1700 6700
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x04 J4
U 1 1 5F6524FF
P 1700 7300
F 0 "J4" H 1780 7292 50  0000 L CNN
F 1 "Trigger4" H 1780 7201 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_bornier-4_P5.08mm" H 1700 7300 50  0001 C CNN
F 3 "~" H 1700 7300 50  0001 C CNN
	1    1700 7300
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x04 J5
U 1 1 5F652E0B
P 3800 5500
F 0 "J5" H 3880 5492 50  0000 L CNN
F 1 "Trigger5" H 3880 5401 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_bornier-4_P5.08mm" H 3800 5500 50  0001 C CNN
F 3 "~" H 3800 5500 50  0001 C CNN
	1    3800 5500
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x04 J6
U 1 1 5F6538F3
P 3800 6100
F 0 "J6" H 3880 6092 50  0000 L CNN
F 1 "Trigger6" H 3880 6001 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_bornier-4_P5.08mm" H 3800 6100 50  0001 C CNN
F 3 "~" H 3800 6100 50  0001 C CNN
	1    3800 6100
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x04 J7
U 1 1 5F653DE0
P 3800 6700
F 0 "J7" H 3880 6692 50  0000 L CNN
F 1 "Analog_Out" H 3880 6601 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_bornier-4_P5.08mm" H 3800 6700 50  0001 C CNN
F 3 "~" H 3800 6700 50  0001 C CNN
	1    3800 6700
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x04 J8
U 1 1 5F654312
P 3800 7300
F 0 "J8" H 3880 7292 50  0000 L CNN
F 1 "Analog_In" H 3880 7201 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_bornier-4_P5.08mm" H 3800 7300 50  0001 C CNN
F 3 "~" H 3800 7300 50  0001 C CNN
	1    3800 7300
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0101
U 1 1 5F66C9A3
P 1500 5400
F 0 "#PWR0101" H 1500 5250 50  0001 C CNN
F 1 "+12V" H 1515 5573 50  0000 C CNN
F 2 "" H 1500 5400 50  0001 C CNN
F 3 "" H 1500 5400 50  0001 C CNN
	1    1500 5400
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0102
U 1 1 5F66D446
P 1500 6000
F 0 "#PWR0102" H 1500 5850 50  0001 C CNN
F 1 "+12V" H 1515 6173 50  0000 C CNN
F 2 "" H 1500 6000 50  0001 C CNN
F 3 "" H 1500 6000 50  0001 C CNN
	1    1500 6000
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0103
U 1 1 5F66D804
P 1500 6600
F 0 "#PWR0103" H 1500 6450 50  0001 C CNN
F 1 "+12V" H 1515 6773 50  0000 C CNN
F 2 "" H 1500 6600 50  0001 C CNN
F 3 "" H 1500 6600 50  0001 C CNN
	1    1500 6600
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0104
U 1 1 5F66DD6F
P 1500 7200
F 0 "#PWR0104" H 1500 7050 50  0001 C CNN
F 1 "+12V" H 1515 7373 50  0000 C CNN
F 2 "" H 1500 7200 50  0001 C CNN
F 3 "" H 1500 7200 50  0001 C CNN
	1    1500 7200
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0105
U 1 1 5F66ECCF
P 3600 5400
F 0 "#PWR0105" H 3600 5250 50  0001 C CNN
F 1 "+12V" H 3615 5573 50  0000 C CNN
F 2 "" H 3600 5400 50  0001 C CNN
F 3 "" H 3600 5400 50  0001 C CNN
	1    3600 5400
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0106
U 1 1 5F66F0FF
P 3600 6000
F 0 "#PWR0106" H 3600 5850 50  0001 C CNN
F 1 "+12V" H 3615 6173 50  0000 C CNN
F 2 "" H 3600 6000 50  0001 C CNN
F 3 "" H 3600 6000 50  0001 C CNN
	1    3600 6000
	1    0    0    -1  
$EndComp
$Comp
L power:GNDPWR #PWR0107
U 1 1 5F66F66D
P 1200 5500
F 0 "#PWR0107" H 1200 5300 50  0001 C CNN
F 1 "GNDPWR" H 1200 5550 50  0000 C CNN
F 2 "" H 1200 5450 50  0001 C CNN
F 3 "" H 1200 5450 50  0001 C CNN
	1    1200 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 5500 1500 5500
Text GLabel 1500 6200 0    50   Input ~ 0
Tr1
Text GLabel 1500 6800 0    50   Input ~ 0
Tr2
Text GLabel 1500 7400 0    50   Input ~ 0
Tr2
Text GLabel 3600 5600 0    50   Input ~ 0
Tr3
Text GLabel 3600 6200 0    50   Input ~ 0
Tr4
Text GLabel 1500 5600 0    50   Input ~ 0
Tr1
$Comp
L power:GNDPWR #PWR0108
U 1 1 5F687530
P 1200 6100
F 0 "#PWR0108" H 1200 5900 50  0001 C CNN
F 1 "GNDPWR" H 1200 6150 50  0000 C CNN
F 2 "" H 1200 6050 50  0001 C CNN
F 3 "" H 1200 6050 50  0001 C CNN
	1    1200 6100
	1    0    0    -1  
$EndComp
$Comp
L power:GNDPWR #PWR0109
U 1 1 5F687923
P 1200 6700
F 0 "#PWR0109" H 1200 6500 50  0001 C CNN
F 1 "GNDPWR" H 1200 6750 50  0000 C CNN
F 2 "" H 1200 6650 50  0001 C CNN
F 3 "" H 1200 6650 50  0001 C CNN
	1    1200 6700
	1    0    0    -1  
$EndComp
$Comp
L power:GNDPWR #PWR0110
U 1 1 5F687EB5
P 1200 7300
F 0 "#PWR0110" H 1200 7100 50  0001 C CNN
F 1 "GNDPWR" H 1200 7350 50  0000 C CNN
F 2 "" H 1200 7250 50  0001 C CNN
F 3 "" H 1200 7250 50  0001 C CNN
	1    1200 7300
	1    0    0    -1  
$EndComp
$Comp
L power:GNDPWR #PWR0111
U 1 1 5F68844A
P 3250 6100
F 0 "#PWR0111" H 3250 5900 50  0001 C CNN
F 1 "GNDPWR" H 3250 6150 50  0000 C CNN
F 2 "" H 3250 6050 50  0001 C CNN
F 3 "" H 3250 6050 50  0001 C CNN
	1    3250 6100
	1    0    0    -1  
$EndComp
$Comp
L power:GNDPWR #PWR0112
U 1 1 5F688E55
P 3250 5500
F 0 "#PWR0112" H 3250 5300 50  0001 C CNN
F 1 "GNDPWR" H 3250 5550 50  0000 C CNN
F 2 "" H 3250 5450 50  0001 C CNN
F 3 "" H 3250 5450 50  0001 C CNN
	1    3250 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 5500 3250 5500
Wire Wire Line
	3600 6100 3250 6100
Wire Wire Line
	1500 6100 1200 6100
Wire Wire Line
	1500 6700 1200 6700
Wire Wire Line
	1500 7300 1200 7300
$Comp
L power:GND #PWR0113
U 1 1 5F695490
P 1200 5700
F 0 "#PWR0113" H 1200 5450 50  0001 C CNN
F 1 "GND" H 1050 5650 50  0000 C CNN
F 2 "" H 1200 5700 50  0001 C CNN
F 3 "" H 1200 5700 50  0001 C CNN
	1    1200 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 5700 1200 5700
$Comp
L power:GND #PWR0114
U 1 1 5F69895C
P 1200 6300
F 0 "#PWR0114" H 1200 6050 50  0001 C CNN
F 1 "GND" H 1050 6250 50  0000 C CNN
F 2 "" H 1200 6300 50  0001 C CNN
F 3 "" H 1200 6300 50  0001 C CNN
	1    1200 6300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 5F698DC6
P 1200 6900
F 0 "#PWR0115" H 1200 6650 50  0001 C CNN
F 1 "GND" H 1050 6850 50  0000 C CNN
F 2 "" H 1200 6900 50  0001 C CNN
F 3 "" H 1200 6900 50  0001 C CNN
	1    1200 6900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 5F698FD1
P 1200 7500
F 0 "#PWR0116" H 1200 7250 50  0001 C CNN
F 1 "GND" H 1050 7450 50  0000 C CNN
F 2 "" H 1200 7500 50  0001 C CNN
F 3 "" H 1200 7500 50  0001 C CNN
	1    1200 7500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0117
U 1 1 5F6994E6
P 3250 5700
F 0 "#PWR0117" H 3250 5450 50  0001 C CNN
F 1 "GND" H 3100 5650 50  0000 C CNN
F 2 "" H 3250 5700 50  0001 C CNN
F 3 "" H 3250 5700 50  0001 C CNN
	1    3250 5700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0118
U 1 1 5F699B71
P 3250 6300
F 0 "#PWR0118" H 3250 6050 50  0001 C CNN
F 1 "GND" H 3100 6250 50  0000 C CNN
F 2 "" H 3250 6300 50  0001 C CNN
F 3 "" H 3250 6300 50  0001 C CNN
	1    3250 6300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 6300 3250 6300
Wire Wire Line
	3600 5700 3250 5700
Wire Wire Line
	1500 6900 1200 6900
Wire Wire Line
	1500 7500 1200 7500
Wire Wire Line
	1500 6300 1200 6300
Text GLabel 3600 6600 0    50   Input ~ 0
AO1
Text GLabel 3600 6800 0    50   Input ~ 0
AO2
Text GLabel 3600 7200 0    50   Input ~ 0
AI1
Text GLabel 3600 7400 0    50   Input ~ 0
AI2
$Comp
L power:GND #PWR0119
U 1 1 5F6A8366
P 3250 6900
F 0 "#PWR0119" H 3250 6650 50  0001 C CNN
F 1 "GND" H 3100 6850 50  0000 C CNN
F 2 "" H 3250 6900 50  0001 C CNN
F 3 "" H 3250 6900 50  0001 C CNN
	1    3250 6900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0120
U 1 1 5F6A8735
P 3250 7500
F 0 "#PWR0120" H 3250 7250 50  0001 C CNN
F 1 "GND" H 3100 7450 50  0000 C CNN
F 2 "" H 3250 7500 50  0001 C CNN
F 3 "" H 3250 7500 50  0001 C CNN
	1    3250 7500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 6700 3350 6700
Wire Wire Line
	3600 6900 3350 6900
Wire Wire Line
	3600 7300 3350 7300
Wire Wire Line
	3600 7500 3350 7500
Wire Wire Line
	3350 6700 3350 6900
Connection ~ 3350 6900
Wire Wire Line
	3350 6900 3250 6900
Wire Wire Line
	3350 7300 3350 7500
Connection ~ 3350 7500
Wire Wire Line
	3350 7500 3250 7500
Text GLabel 2050 2050 0    50   Input ~ 0
AI1
Text GLabel 2050 2150 0    50   Input ~ 0
AI2
$Comp
L power:+5V #PWR0121
U 1 1 5F6C00B6
P 6550 3500
F 0 "#PWR0121" H 6550 3350 50  0001 C CNN
F 1 "+5V" H 6565 3673 50  0000 C CNN
F 2 "" H 6550 3500 50  0001 C CNN
F 3 "" H 6550 3500 50  0001 C CNN
	1    6550 3500
	1    0    0    -1  
$EndComp
Text GLabel 7650 3500 2    50   Input ~ 0
AO1
$Comp
L triggerbox:MCP4822 U3
U 1 1 5F6BF4C7
P 7100 3500
F 0 "U3" H 7100 3750 60  0000 C CNN
F 1 "MCP4822" H 7100 3650 60  0000 C CNN
F 2 "Package_SO:SOIC-8-1EP_3.9x4.9mm_P1.27mm_EP2.29x3mm" H 7100 3500 50  0001 C CNN
F 3 "" H 7100 3500 50  0001 C CNN
	1    7100 3500
	1    0    0    -1  
$EndComp
Text GLabel 7650 3700 2    50   Input ~ 0
AO2
$Comp
L power:GND #PWR0122
U 1 1 5F6C3CDE
P 8000 3600
F 0 "#PWR0122" H 8000 3350 50  0001 C CNN
F 1 "GND" H 8000 3450 50  0000 C CNN
F 2 "" H 8000 3600 50  0001 C CNN
F 3 "" H 8000 3600 50  0001 C CNN
	1    8000 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 3600 7650 3600
Text GLabel 6550 3600 0    50   Input ~ 0
CS
Text GLabel 6550 3700 0    50   Input ~ 0
SCK
Text GLabel 6550 3800 0    50   Input ~ 0
SDI
Text GLabel 7650 3800 2    50   Input ~ 0
LDAC
Text GLabel 3750 2250 2    50   Input ~ 0
LDAC
Text GLabel 3750 1950 2    50   Input ~ 0
CS
Text GLabel 2050 1750 0    50   Input ~ 0
SCK
Text GLabel 3750 1850 2    50   Input ~ 0
SDI
$Comp
L power:GND #PWR0123
U 1 1 5F6CD79D
P 3950 3050
F 0 "#PWR0123" H 3950 2800 50  0001 C CNN
F 1 "GND" H 3950 2900 50  0000 C CNN
F 2 "" H 3950 3050 50  0001 C CNN
F 3 "" H 3950 3050 50  0001 C CNN
	1    3950 3050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0124
U 1 1 5F6CE186
P 5050 3050
F 0 "#PWR0124" H 5050 2800 50  0001 C CNN
F 1 "GND" H 5050 2900 50  0000 C CNN
F 2 "" H 5050 3050 50  0001 C CNN
F 3 "" H 5050 3050 50  0001 C CNN
	1    5050 3050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0125
U 1 1 5F6CE5DE
P 1900 3150
F 0 "#PWR0125" H 1900 2900 50  0001 C CNN
F 1 "GND" H 1900 3000 50  0000 C CNN
F 2 "" H 1900 3150 50  0001 C CNN
F 3 "" H 1900 3150 50  0001 C CNN
	1    1900 3150
	1    0    0    -1  
$EndComp
$Comp
L triggerbox:SN74HC125 U1
U 1 1 5F6CEAA7
P 7050 2000
F 0 "U1" H 6900 2200 60  0000 C CNN
F 1 "SN74HC125" H 7450 2200 60  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 7050 2000 50  0001 C CNN
F 3 "" H 7050 2000 50  0001 C CNN
	1    7050 2000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0126
U 1 1 5F6D103D
P 6300 2600
F 0 "#PWR0126" H 6300 2350 50  0001 C CNN
F 1 "GND" H 6300 2450 50  0000 C CNN
F 2 "" H 6300 2600 50  0001 C CNN
F 3 "" H 6300 2600 50  0001 C CNN
	1    6300 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 2550 6300 2550
Wire Wire Line
	6300 2550 6300 2600
Wire Wire Line
	6300 2550 6300 2250
Wire Wire Line
	6300 2250 6450 2250
Connection ~ 6300 2550
Wire Wire Line
	6300 2250 6300 1950
Wire Wire Line
	6300 1950 6450 1950
Connection ~ 6300 2250
Text GLabel 6100 2050 0    50   Input ~ 0
Trig
Text GLabel 3750 2050 2    50   Input ~ 0
Trig
Wire Wire Line
	6450 2050 6200 2050
Wire Wire Line
	6200 2050 6200 2350
Wire Wire Line
	6200 2350 6450 2350
Wire Wire Line
	7900 2050 8050 2050
Wire Wire Line
	8050 2050 8050 2350
Wire Wire Line
	8050 2350 7900 2350
$Comp
L power:GND #PWR0127
U 1 1 5F6DF289
P 8050 2650
F 0 "#PWR0127" H 8050 2400 50  0001 C CNN
F 1 "GND" H 8050 2500 50  0000 C CNN
F 2 "" H 8050 2650 50  0001 C CNN
F 3 "" H 8050 2650 50  0001 C CNN
	1    8050 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 2350 8050 2650
Connection ~ 8050 2350
Wire Wire Line
	6200 2050 6100 2050
Connection ~ 6200 2050
Text GLabel 6100 2150 0    50   Input ~ 0
Tr4
Wire Wire Line
	6450 2150 6100 2150
Text GLabel 6100 2450 0    50   Input ~ 0
Tr3
Wire Wire Line
	6450 2450 6100 2450
Text GLabel 8250 2250 2    50   Input ~ 0
Tr2
Text GLabel 8250 2550 2    50   Input ~ 0
Tr1
Wire Wire Line
	8250 2550 7900 2550
Wire Wire Line
	8250 2250 7900 2250
Text GLabel 8250 2150 2    50   Input ~ 0
Trig
Wire Wire Line
	8250 2150 8150 2150
Wire Wire Line
	7900 2450 8150 2450
Wire Wire Line
	8150 2450 8150 2150
Connection ~ 8150 2150
Wire Wire Line
	8150 2150 7900 2150
$Comp
L power:+5V #PWR0128
U 1 1 5F6F5926
P 7900 1950
F 0 "#PWR0128" H 7900 1800 50  0001 C CNN
F 1 "+5V" H 7915 2123 50  0000 C CNN
F 2 "" H 7900 1950 50  0001 C CNN
F 3 "" H 7900 1950 50  0001 C CNN
	1    7900 1950
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0129
U 1 1 5F6F633B
P 1850 2850
F 0 "#PWR0129" H 1850 2700 50  0001 C CNN
F 1 "+5V" H 1865 3023 50  0000 C CNN
F 2 "" H 1850 2850 50  0001 C CNN
F 3 "" H 1850 2850 50  0001 C CNN
	1    1850 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 2850 2050 2850
$Comp
L Connector:Barrel_Jack_Switch J9
U 1 1 5F71C5F1
P 5000 5550
F 0 "J9" H 5057 5867 50  0000 C CNN
F 1 "CamPW" H 5057 5776 50  0000 C CNN
F 2 "Connector_BarrelJack:BarrelJack_Horizontal" H 5050 5510 50  0001 C CNN
F 3 "~" H 5050 5510 50  0001 C CNN
	1    5000 5550
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0130
U 1 1 5F71CECD
P 5300 5450
F 0 "#PWR0130" H 5300 5300 50  0001 C CNN
F 1 "+12V" H 5315 5623 50  0000 C CNN
F 2 "" H 5300 5450 50  0001 C CNN
F 3 "" H 5300 5450 50  0001 C CNN
	1    5300 5450
	1    0    0    -1  
$EndComp
$Comp
L power:GNDPWR #PWR0131
U 1 1 5F721278
P 5300 5650
F 0 "#PWR0131" H 5300 5450 50  0001 C CNN
F 1 "GNDPWR" H 5300 5500 50  0000 C CNN
F 2 "" H 5300 5600 50  0001 C CNN
F 3 "" H 5300 5600 50  0001 C CNN
	1    5300 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 5550 5300 5650
Connection ~ 5300 5650
$EndSCHEMATC
