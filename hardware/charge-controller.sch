EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
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
L Interface_CAN_LIN:SN65HVD230 U4
U 1 1 6175EF02
P 10000 4400
F 0 "U4" H 10200 4750 50  0000 C CNN
F 1 "SN65HVD231" H 10250 4650 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 10000 3900 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn65hvd230.pdf" H 9900 4800 50  0001 C CNN
F 4 "C129257" H 10000 4400 50  0001 C CNN "LCSC"
	1    10000 4400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR018
U 1 1 61755FCF
P 9200 1200
F 0 "#PWR018" H 9200 950 50  0001 C CNN
F 1 "GND" V 9205 1072 50  0000 R CNN
F 2 "" H 9200 1200 50  0001 C CNN
F 3 "" H 9200 1200 50  0001 C CNN
	1    9200 1200
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR021
U 1 1 617541B7
P 9200 2700
F 0 "#PWR021" H 9200 2450 50  0001 C CNN
F 1 "GND" V 9205 2572 50  0000 R CNN
F 2 "" H 9200 2700 50  0001 C CNN
F 3 "" H 9200 2700 50  0001 C CNN
	1    9200 2700
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR028
U 1 1 61752AB9
P 10600 2700
F 0 "#PWR028" H 10600 2450 50  0001 C CNN
F 1 "GND" V 10605 2572 50  0000 R CNN
F 2 "" H 10600 2700 50  0001 C CNN
F 3 "" H 10600 2700 50  0001 C CNN
	1    10600 2700
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR024
U 1 1 61751895
P 10600 1200
F 0 "#PWR024" H 10600 950 50  0001 C CNN
F 1 "GND" V 10605 1072 50  0000 R CNN
F 2 "" H 10600 1200 50  0001 C CNN
F 3 "" H 10600 1200 50  0001 C CNN
	1    10600 1200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	10900 1100 10600 1100
$Comp
L power:+3V3 #PWR025
U 1 1 61756F32
P 10600 1400
F 0 "#PWR025" H 10600 1250 50  0001 C CNN
F 1 "+3V3" V 10600 1500 50  0000 L CNN
F 2 "" H 10600 1400 50  0001 C CNN
F 3 "" H 10600 1400 50  0001 C CNN
	1    10600 1400
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR027
U 1 1 61752145
P 10600 2200
F 0 "#PWR027" H 10600 1950 50  0001 C CNN
F 1 "GND" V 10605 2072 50  0000 R CNN
F 2 "" H 10600 2200 50  0001 C CNN
F 3 "" H 10600 2200 50  0001 C CNN
	1    10600 2200
	0    -1   -1   0   
$EndComp
Text GLabel 10600 2500 2    50   Output ~ 0
SPI_MOSI
Text GLabel 10600 2600 2    50   Output ~ 0
SPI_CLK
Text GLabel 10600 2800 2    50   Output ~ 0
SPI_CS
Text GLabel 10600 2900 2    50   Input ~ 0
SPI_MISO
Text GLabel 10400 4500 2    50   BiDi ~ 0
CANL
Text GLabel 10400 4400 2    50   BiDi ~ 0
CANH
Text GLabel 10600 2300 2    50   Output ~ 0
8Mhz
$Comp
L power:GND #PWR022
U 1 1 6179D267
P 9900 3100
F 0 "#PWR022" H 9900 2850 50  0001 C CNN
F 1 "GND" H 9905 2927 50  0000 C CNN
F 2 "" H 9900 3100 50  0001 C CNN
F 3 "" H 9900 3100 50  0001 C CNN
	1    9900 3100
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR033
U 1 1 617A3AEF
P 10000 4100
F 0 "#PWR033" H 10000 3950 50  0001 C CNN
F 1 "+3V3" H 10015 4273 50  0000 C CNN
F 2 "" H 10000 4100 50  0001 C CNN
F 3 "" H 10000 4100 50  0001 C CNN
	1    10000 4100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR034
U 1 1 617A47A7
P 10000 4800
F 0 "#PWR034" H 10000 4550 50  0001 C CNN
F 1 "GND" H 10005 4627 50  0000 C CNN
F 2 "" H 10000 4800 50  0001 C CNN
F 3 "" H 10000 4800 50  0001 C CNN
	1    10000 4800
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR029
U 1 1 61749E96
P 10900 800
F 0 "#PWR029" H 10900 650 50  0001 C CNN
F 1 "+5V" H 10915 973 50  0000 C CNN
F 2 "" H 10900 800 50  0001 C CNN
F 3 "" H 10900 800 50  0001 C CNN
	1    10900 800 
	1    0    0    -1  
$EndComp
Text GLabel 10600 2400 2    50   Input ~ 0
CAN_INT
$Comp
L Device:C C8
U 1 1 617802D1
P 10350 5250
F 0 "C8" H 10465 5296 50  0000 L CNN
F 1 "100n" H 10465 5205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 10388 5100 50  0001 C CNN
F 3 "~" H 10350 5250 50  0001 C CNN
F 4 "C14663" H 10350 5250 50  0001 C CNN "LCSC"
	1    10350 5250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C9
U 1 1 61780656
P 10750 5250
F 0 "C9" H 10865 5296 50  0000 L CNN
F 1 "100n" H 10865 5205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 10788 5100 50  0001 C CNN
F 3 "~" H 10750 5250 50  0001 C CNN
F 4 "C14663" H 10750 5250 50  0001 C CNN "LCSC"
	1    10750 5250
	1    0    0    -1  
$EndComp
Connection ~ 10350 5400
Wire Wire Line
	10350 5400 10750 5400
Connection ~ 10350 5100
Wire Wire Line
	10350 5100 10750 5100
$Comp
L power:+3V3 #PWR035
U 1 1 6178B9C2
P 10350 5000
F 0 "#PWR035" H 10350 4850 50  0001 C CNN
F 1 "+3V3" H 10365 5173 50  0000 C CNN
F 2 "" H 10350 5000 50  0001 C CNN
F 3 "" H 10350 5000 50  0001 C CNN
	1    10350 5000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR036
U 1 1 6178C12D
P 10350 5500
F 0 "#PWR036" H 10350 5250 50  0001 C CNN
F 1 "GND" H 10355 5327 50  0000 C CNN
F 2 "" H 10350 5500 50  0001 C CNN
F 3 "" H 10350 5500 50  0001 C CNN
	1    10350 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	10350 5000 10350 5100
Wire Wire Line
	10350 5400 10350 5500
$Comp
L Device:D_Schottky D6
U 1 1 61756641
P 10900 950
F 0 "D6" V 10854 1030 50  0000 L CNN
F 1 "SS54" V 10945 1030 50  0000 L CNN
F 2 "Diode_SMD:D_SMA" H 10900 950 50  0001 C CNN
F 3 "~" H 10900 950 50  0001 C CNN
F 4 "C22452" V 10900 950 50  0001 C CNN "LCSC"
	1    10900 950 
	0    -1   -1   0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H1
U 1 1 6199AFD3
P 9400 6800
F 0 "H1" H 9500 6803 50  0000 L CNN
F 1 "MountingHole" H 9500 6758 50  0001 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad" H 9400 6800 50  0001 C CNN
F 3 "~" H 9400 6800 50  0001 C CNN
	1    9400 6800
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H2
U 1 1 6199C740
P 9700 6800
F 0 "H2" H 9800 6803 50  0000 L CNN
F 1 "MountingHole" H 9800 6758 50  0001 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad" H 9700 6800 50  0001 C CNN
F 3 "~" H 9700 6800 50  0001 C CNN
	1    9700 6800
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H3
U 1 1 6199C8D6
P 10000 6800
F 0 "H3" H 10100 6803 50  0000 L CNN
F 1 "MountingHole" H 10100 6755 50  0001 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad" H 10000 6800 50  0001 C CNN
F 3 "~" H 10000 6800 50  0001 C CNN
	1    10000 6800
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H4
U 1 1 6199CA74
P 10300 6800
F 0 "H4" H 10400 6803 50  0000 L CNN
F 1 "MountingHole" H 10400 6755 50  0001 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad" H 10300 6800 50  0001 C CNN
F 3 "~" H 10300 6800 50  0001 C CNN
	1    10300 6800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR023
U 1 1 619B5460
P 10300 6900
F 0 "#PWR023" H 10300 6650 50  0001 C CNN
F 1 "GND" H 10305 6727 50  0000 C CNN
F 2 "" H 10300 6900 50  0001 C CNN
F 3 "" H 10300 6900 50  0001 C CNN
	1    10300 6900
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Switching:TPS62160DGK U1
U 1 1 6174E98B
P 7100 1850
F 0 "U1" H 7100 2417 50  0000 C CNN
F 1 "TPS62160DGK" H 7100 2326 50  0000 C CNN
F 2 "Package_SO:MSOP-8_3x3mm_P0.65mm" H 7250 1500 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/tps62160.pdf" H 7100 2400 50  0001 C CNN
F 4 "C60726" H 7100 1850 50  0001 C CNN "LCSC"
	1    7100 1850
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR015
U 1 1 6174FDA2
P 6500 1450
F 0 "#PWR015" H 6500 1300 50  0001 C CNN
F 1 "+12V" H 6515 1623 50  0000 C CNN
F 2 "" H 6500 1450 50  0001 C CNN
F 3 "" H 6500 1450 50  0001 C CNN
	1    6500 1450
	1    0    0    -1  
$EndComp
$Comp
L Device:L L1
U 1 1 61754A5C
P 7750 1550
F 0 "L1" V 7569 1550 50  0000 C CNN
F 1 "2.2u" V 7660 1550 50  0000 C CNN
F 2 "Inductor_SMD:L_Neosid_SMS-ME3015" H 7750 1550 50  0001 C CNN
F 3 "~" H 7750 1550 50  0001 C CNN
F 4 "C43389" V 7750 1550 50  0001 C CNN "LCSC"
	1    7750 1550
	0    1    1    0   
$EndComp
$Comp
L Device:C C6
U 1 1 61755D56
P 6500 1800
F 0 "C6" H 6615 1846 50  0000 L CNN
F 1 "10u" H 6615 1755 50  0000 L CNN
F 2 "Capacitor_SMD:C_1210_3225Metric" H 6538 1650 50  0001 C CNN
F 3 "~" H 6500 1800 50  0001 C CNN
F 4 "C77102" H 6500 1800 50  0001 C CNN "LCSC"
	1    6500 1800
	1    0    0    -1  
$EndComp
$Comp
L Device:C C7
U 1 1 617571F5
P 8450 1700
F 0 "C7" H 8565 1746 50  0000 L CNN
F 1 "22u" H 8565 1655 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 8488 1550 50  0001 C CNN
F 3 "~" H 8450 1700 50  0001 C CNN
F 4 "C5672" H 8450 1700 50  0001 C CNN "LCSC"
	1    8450 1700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR016
U 1 1 6175E11C
P 7150 2350
F 0 "#PWR016" H 7150 2100 50  0001 C CNN
F 1 "GND" H 7155 2177 50  0000 C CNN
F 2 "" H 7150 2350 50  0001 C CNN
F 3 "" H 7150 2350 50  0001 C CNN
	1    7150 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 2350 7150 2300
Wire Wire Line
	7150 2300 7100 2300
Wire Wire Line
	7100 2300 7100 2250
Wire Wire Line
	7200 2250 7200 2300
Wire Wire Line
	7200 2300 7150 2300
Connection ~ 7150 2300
Wire Wire Line
	6500 1450 6500 1550
Wire Wire Line
	6500 1550 6700 1550
Wire Wire Line
	6500 1650 6500 1550
Connection ~ 6500 1550
Wire Wire Line
	6700 1650 6500 1650
Connection ~ 6500 1650
Wire Wire Line
	6500 1950 6500 2300
Wire Wire Line
	6500 2300 7100 2300
Connection ~ 7100 2300
Wire Wire Line
	7500 1550 7600 1550
$Comp
L Device:R R14
U 1 1 6177B994
P 8050 1700
F 0 "R14" H 8120 1746 50  0000 L CNN
F 1 "220k" H 8120 1655 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7980 1700 50  0001 C CNN
F 3 "~" H 8050 1700 50  0001 C CNN
F 4 "C22961" H 8050 1700 50  0001 C CNN "LCSC"
	1    8050 1700
	1    0    0    -1  
$EndComp
$Comp
L Device:R R15
U 1 1 6177C2E2
P 8050 2100
F 0 "R15" H 8120 2146 50  0000 L CNN
F 1 "47k" H 8120 2055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7980 2100 50  0001 C CNN
F 3 "~" H 8050 2100 50  0001 C CNN
F 4 "C25819" H 8050 2100 50  0001 C CNN "LCSC"
	1    8050 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 2300 8050 2300
Connection ~ 7200 2300
Wire Wire Line
	8050 2300 8050 2250
Wire Wire Line
	7500 1750 7900 1750
Wire Wire Line
	7900 1750 7900 1900
Wire Wire Line
	7900 1900 8050 1900
Wire Wire Line
	8050 1900 8050 1850
Wire Wire Line
	8050 1900 8050 1950
Connection ~ 8050 1900
Wire Wire Line
	7900 1550 7950 1550
Wire Wire Line
	7950 1550 7950 1650
Connection ~ 7950 1550
Wire Wire Line
	7950 1550 8050 1550
Wire Wire Line
	7500 1650 7950 1650
Wire Wire Line
	8050 2300 8450 2300
Wire Wire Line
	8450 2300 8450 1850
Connection ~ 8050 2300
Wire Wire Line
	8050 1550 8450 1550
Connection ~ 8050 1550
$Comp
L power:+5V #PWR017
U 1 1 6179C7AD
P 8450 1450
F 0 "#PWR017" H 8450 1300 50  0001 C CNN
F 1 "+5V" H 8465 1623 50  0000 C CNN
F 2 "" H 8450 1450 50  0001 C CNN
F 3 "" H 8450 1450 50  0001 C CNN
	1    8450 1450
	1    0    0    -1  
$EndComp
Connection ~ 8450 1550
Wire Wire Line
	8450 1450 8450 1550
$Comp
L Device:R R13
U 1 1 617A40C5
P 6300 2100
F 0 "R13" V 6093 2100 50  0000 C CNN
F 1 "100k" V 6184 2100 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6230 2100 50  0001 C CNN
F 3 "~" H 6300 2100 50  0001 C CNN
F 4 "C25803" V 6300 2100 50  0001 C CNN "LCSC"
	1    6300 2100
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR014
U 1 1 617A550B
P 6050 2000
F 0 "#PWR014" H 6050 1850 50  0001 C CNN
F 1 "+5V" H 6065 2173 50  0000 C CNN
F 2 "" H 6050 2000 50  0001 C CNN
F 3 "" H 6050 2000 50  0001 C CNN
	1    6050 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 2000 6050 2100
Wire Wire Line
	6050 2100 6150 2100
Wire Wire Line
	6450 2100 6600 2100
Wire Wire Line
	6600 2100 6600 1950
Wire Wire Line
	6600 1950 6700 1950
Text GLabel 9700 5250 2    50   BiDi ~ 0
CANL
Text GLabel 9700 5150 2    50   BiDi ~ 0
CANH
$Comp
L Connector_Generic:Conn_01x02 J3
U 1 1 61783CE3
P 9650 5550
F 0 "J3" V 9522 5630 50  0000 L CNN
F 1 "CAN_TERM" V 9613 5630 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 9650 5550 50  0001 C CNN
F 3 "~" H 9650 5550 50  0001 C CNN
	1    9650 5550
	0    1    1    0   
$EndComp
$Comp
L Device:R R16
U 1 1 61784F42
P 9500 5150
F 0 "R16" V 9707 5150 50  0000 C CNN
F 1 "120" V 9616 5150 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 9430 5150 50  0001 C CNN
F 3 "~" H 9500 5150 50  0001 C CNN
F 4 "C17909" V 9500 5150 50  0001 C CNN "LCSC"
	1    9500 5150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9700 5250 9650 5250
Wire Wire Line
	9650 5250 9650 5350
Wire Wire Line
	9550 5350 9350 5350
Wire Wire Line
	9350 5350 9350 5150
Wire Wire Line
	9650 5150 9700 5150
$Comp
L Interface_CAN_LIN:MCP2515-xST U3
U 1 1 61732E24
P 8150 4900
F 0 "U3" H 8400 5750 50  0000 C CNN
F 1 "MCP2515-xST" H 8450 5650 50  0000 C CNN
F 2 "Package_SO:TSSOP-20_4.4x6.5mm_P0.65mm" H 8150 4000 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21801e.pdf" H 8250 4100 50  0001 C CNN
F 4 "C15193" H 8150 4900 50  0001 C CNN "LCSC"
	1    8150 4900
	1    0    0    -1  
$EndComp
Text GLabel 8750 4900 2    50   Output ~ 0
CAN_INT
$Comp
L power:+3V3 #PWR030
U 1 1 617A14DE
P 8150 4100
F 0 "#PWR030" H 8150 3950 50  0001 C CNN
F 1 "+3V3" H 8165 4273 50  0000 C CNN
F 2 "" H 8150 4100 50  0001 C CNN
F 3 "" H 8150 4100 50  0001 C CNN
	1    8150 4100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR031
U 1 1 6179DF89
P 8150 5700
F 0 "#PWR031" H 8150 5450 50  0001 C CNN
F 1 "GND" H 8155 5527 50  0000 C CNN
F 2 "" H 8150 5700 50  0001 C CNN
F 3 "" H 8150 5700 50  0001 C CNN
	1    8150 5700
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR032
U 1 1 6179A3A6
P 8750 5500
F 0 "#PWR032" H 8750 5350 50  0001 C CNN
F 1 "+3V3" V 8750 5600 50  0000 L CNN
F 2 "" H 8750 5500 50  0001 C CNN
F 3 "" H 8750 5500 50  0001 C CNN
	1    8750 5500
	0    1    1    0   
$EndComp
Text GLabel 7550 4400 0    50   Output ~ 0
SPI_MISO
Text GLabel 7550 4300 0    50   Input ~ 0
SPI_MOSI
Text GLabel 7550 4600 0    50   Input ~ 0
SPI_CLK
Text GLabel 7550 4500 0    50   Input ~ 0
SPI_CS
Text GLabel 7550 5200 0    50   Input ~ 0
8Mhz
Text GLabel 9600 4300 0    50   Input ~ 0
TXCAN
Text GLabel 9600 4400 0    50   Output ~ 0
RXCAN
Text GLabel 8750 4400 2    50   Output ~ 0
TXCAN
Text GLabel 8750 4300 2    50   Input ~ 0
RXCAN
Text Notes 6900 950  0    50   ~ 0
5-15v power supply
Text Notes 9050 4000 0    50   ~ 0
CAN
$Comp
L Connector_Generic:Conn_02x03_Top_Bottom J2
U 1 1 61C657A9
P 5100 1450
F 0 "J2" H 5150 1767 50  0000 C CNN
F 1 "Conn_02x03_Odd_Even" H 5150 1676 50  0000 C CNN
F 2 "Connector_Molex:Molex_Micro-Fit_3.0_43045-0600_2x03_P3.00mm_Horizontal" H 5100 1450 50  0001 C CNN
F 3 "~" H 5100 1450 50  0001 C CNN
	1    5100 1450
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR020
U 1 1 61C70B0A
P 9200 2200
F 0 "#PWR020" H 9200 1950 50  0001 C CNN
F 1 "GND" V 9205 2072 50  0000 R CNN
F 2 "" H 9200 2200 50  0001 C CNN
F 3 "" H 9200 2200 50  0001 C CNN
	1    9200 2200
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR019
U 1 1 61C70CF7
P 9200 1700
F 0 "#PWR019" H 9200 1450 50  0001 C CNN
F 1 "GND" V 9205 1572 50  0000 R CNN
F 2 "" H 9200 1700 50  0001 C CNN
F 3 "" H 9200 1700 50  0001 C CNN
	1    9200 1700
	0    1    1    0   
$EndComp
$Comp
L power:+3V3 #PWR010
U 1 1 61C1A7F9
P 6100 4150
F 0 "#PWR010" H 6100 4000 50  0001 C CNN
F 1 "+3V3" H 6100 4300 50  0000 C CNN
F 2 "" H 6100 4150 50  0001 C CNN
F 3 "" H 6100 4150 50  0001 C CNN
	1    6100 4150
	1    0    0    -1  
$EndComp
Text GLabel 6000 4550 0    50   BiDi ~ 0
EVSE_PP
Wire Wire Line
	6100 4150 6100 4200
$Comp
L power:GND #PWR011
U 1 1 6196710F
P 6100 4950
F 0 "#PWR011" H 6100 4700 50  0001 C CNN
F 1 "GND" H 6105 4777 50  0000 C CNN
F 2 "" H 6100 4950 50  0001 C CNN
F 3 "" H 6100 4950 50  0001 C CNN
	1    6100 4950
	1    0    0    -1  
$EndComp
Text GLabel 6200 4550 2    50   Output ~ 0
MCU_PP
$Comp
L Device:R R12
U 1 1 6195715F
P 6100 4350
F 0 "R12" H 6170 4396 50  0000 L CNN
F 1 "330" H 6170 4305 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6030 4350 50  0001 C CNN
F 3 "~" H 6100 4350 50  0001 C CNN
F 4 "C23138" H 6100 4350 50  0001 C CNN "LCSC"
	1    6100 4350
	1    0    0    -1  
$EndComp
Text GLabel 1750 950  2    50   Output ~ 0
OUT12_1
Wire Wire Line
	1750 950  1700 950 
Wire Wire Line
	1700 950  1700 1000
$Comp
L power:GND #PWR05
U 1 1 61DB4ECA
P 1700 1800
F 0 "#PWR05" H 1700 1550 50  0001 C CNN
F 1 "GND" H 1705 1627 50  0000 C CNN
F 2 "" H 1700 1800 50  0001 C CNN
F 3 "" H 1700 1800 50  0001 C CNN
	1    1700 1800
	1    0    0    -1  
$EndComp
$Comp
L Driver_FET:NCV8402xST Q1
U 1 1 61DB1F18
P 1400 1400
F 0 "Q1" H 1828 1446 50  0000 L CNN
F 1 "NCV8402xST" H 1828 1355 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-223" H 1400 1120 50  0001 C CNN
F 3 "https://www.onsemi.com/pub/Collateral/NCV8402-D.PDF" H 1700 1400 50  0001 C CNN
F 4 "C77755" H 1400 1400 50  0001 C CNN "LCSC"
	1    1400 1400
	1    0    0    -1  
$EndComp
Text GLabel 900  1400 0    50   Input ~ 0
OUT1
Text GLabel 4800 1350 0    50   BiDi ~ 0
EVSE_CP
Text GLabel 4800 1450 0    50   BiDi ~ 0
EVSE_PP
Text GLabel 5300 1550 2    50   BiDi ~ 0
CANL
Text GLabel 4800 1550 0    50   BiDi ~ 0
CANH
$Comp
L power:+12V #PWR012
U 1 1 61CAE183
P 5300 1350
F 0 "#PWR012" H 5300 1200 50  0001 C CNN
F 1 "+12V" V 5300 1550 50  0000 C CNN
F 2 "" H 5300 1350 50  0001 C CNN
F 3 "" H 5300 1350 50  0001 C CNN
	1    5300 1350
	0    1    1    0   
$EndComp
Text GLabel 10600 1900 2    50   Input ~ 0
MCU_PP
Text GLabel 10600 1600 2    50   Output ~ 0
MCU_SW
Text GLabel 10600 1800 2    50   Input ~ 0
MCU_CP
$Comp
L power:GND #PWR026
U 1 1 61CC664C
P 10600 1700
F 0 "#PWR026" H 10600 1450 50  0001 C CNN
F 1 "GND" V 10605 1572 50  0000 R CNN
F 2 "" H 10600 1700 50  0001 C CNN
F 3 "" H 10600 1700 50  0001 C CNN
	1    10600 1700
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR013
U 1 1 61D1A4B3
P 5300 1450
F 0 "#PWR013" H 5300 1200 50  0001 C CNN
F 1 "GND" V 5305 1322 50  0000 R CNN
F 2 "" H 5300 1450 50  0001 C CNN
F 3 "" H 5300 1450 50  0001 C CNN
	1    5300 1450
	0    -1   -1   0   
$EndComp
Text GLabel 4800 2150 0    50   Output ~ 0
IN12_1
Text GLabel 4800 2050 0    50   Output ~ 0
IN12_2
Text GLabel 4800 1950 0    50   Input ~ 0
OUT12_3
Text GLabel 4800 1850 0    50   Input ~ 0
OUT12_4
Text GLabel 9200 2100 0    50   Output ~ 0
OUT1
Text GLabel 9200 2800 0    50   Input ~ 0
IN1
$Comp
L Connector_Generic:Conn_02x04_Top_Bottom J1
U 1 1 61C6623C
P 5000 2050
F 0 "J1" H 5050 2367 50  0000 C CNN
F 1 "Conn_02x04_Odd_Even" H 5050 2276 50  0000 C CNN
F 2 "Connector_Molex:Molex_Micro-Fit_3.0_43045-0800_2x04_P3.00mm_Horizontal" H 5000 2050 50  0001 C CNN
F 3 "~" H 5000 2050 50  0001 C CNN
	1    5000 2050
	1    0    0    1   
$EndComp
Text GLabel 9200 2600 0    50   Input ~ 0
IN2
Text GLabel 9200 2500 0    50   Input ~ 0
IN3
Text GLabel 9200 2300 0    50   Input ~ 0
IN4
Text GLabel 9200 1800 0    50   Output ~ 0
OUT2
Text GLabel 9200 1500 0    50   Output ~ 0
OUT3
Text GLabel 9200 1300 0    50   Output ~ 0
OUT4
Text GLabel 3600 1050 2    50   Output ~ 0
OUT12_2
Wire Wire Line
	3600 1050 3550 1050
Wire Wire Line
	3550 1050 3550 1100
$Comp
L power:GND #PWR06
U 1 1 61D2D34B
P 3550 1900
F 0 "#PWR06" H 3550 1650 50  0001 C CNN
F 1 "GND" H 3555 1727 50  0000 C CNN
F 2 "" H 3550 1900 50  0001 C CNN
F 3 "" H 3550 1900 50  0001 C CNN
	1    3550 1900
	1    0    0    -1  
$EndComp
$Comp
L Driver_FET:NCV8402xST Q2
U 1 1 61D2D355
P 3250 1500
F 0 "Q2" H 3678 1546 50  0000 L CNN
F 1 "NCV8402xST" H 3678 1455 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-223" H 3250 1220 50  0001 C CNN
F 3 "https://www.onsemi.com/pub/Collateral/NCV8402-D.PDF" H 3550 1500 50  0001 C CNN
F 4 "C77755" H 3250 1500 50  0001 C CNN "LCSC"
	1    3250 1500
	1    0    0    -1  
$EndComp
Text GLabel 2750 1500 0    50   Input ~ 0
OUT2
Text GLabel 1750 2450 2    50   Output ~ 0
OUT12_3
Wire Wire Line
	1750 2450 1700 2450
Wire Wire Line
	1700 2450 1700 2500
$Comp
L power:GND #PWR07
U 1 1 61D36551
P 1700 3300
F 0 "#PWR07" H 1700 3050 50  0001 C CNN
F 1 "GND" H 1705 3127 50  0000 C CNN
F 2 "" H 1700 3300 50  0001 C CNN
F 3 "" H 1700 3300 50  0001 C CNN
	1    1700 3300
	1    0    0    -1  
$EndComp
$Comp
L Driver_FET:NCV8402xST Q3
U 1 1 61D3655B
P 1400 2900
F 0 "Q3" H 1828 2946 50  0000 L CNN
F 1 "NCV8402xST" H 1828 2855 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-223" H 1400 2620 50  0001 C CNN
F 3 "https://www.onsemi.com/pub/Collateral/NCV8402-D.PDF" H 1700 2900 50  0001 C CNN
F 4 "C77755" H 1400 2900 50  0001 C CNN "LCSC"
	1    1400 2900
	1    0    0    -1  
$EndComp
Text GLabel 900  2900 0    50   Input ~ 0
OUT3
Text GLabel 3450 2450 2    50   Output ~ 0
OUT12_4
Wire Wire Line
	3450 2450 3400 2450
Wire Wire Line
	3400 2450 3400 2500
$Comp
L power:GND #PWR08
U 1 1 61D43D91
P 3400 3300
F 0 "#PWR08" H 3400 3050 50  0001 C CNN
F 1 "GND" H 3405 3127 50  0000 C CNN
F 2 "" H 3400 3300 50  0001 C CNN
F 3 "" H 3400 3300 50  0001 C CNN
	1    3400 3300
	1    0    0    -1  
$EndComp
$Comp
L Driver_FET:NCV8402xST Q4
U 1 1 61D43D9B
P 3100 2900
F 0 "Q4" H 3528 2946 50  0000 L CNN
F 1 "NCV8402xST" H 3528 2855 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-223" H 3100 2620 50  0001 C CNN
F 3 "https://www.onsemi.com/pub/Collateral/NCV8402-D.PDF" H 3400 2900 50  0001 C CNN
F 4 "C77755" H 3100 2900 50  0001 C CNN "LCSC"
	1    3100 2900
	1    0    0    -1  
$EndComp
Text GLabel 2600 2900 0    50   Input ~ 0
OUT4
Text GLabel 5300 2150 2    50   Output ~ 0
IN12_3
Text GLabel 5300 2050 2    50   Output ~ 0
IN12_4
Text GLabel 5300 1950 2    50   Input ~ 0
OUT12_1
Text GLabel 5300 1850 2    50   Input ~ 0
OUT12_2
$Comp
L MCU_RaspberryPi_and_Boards:Pico U2
U 1 1 6171CBAA
P 9900 1950
F 0 "U2" H 9900 3165 50  0000 C CNN
F 1 "Pico" H 9900 3074 50  0000 C CNN
F 2 "footprints:RPi_Pico_SMD_TH" V 9900 1950 50  0001 C CNN
F 3 "" H 9900 1950 50  0001 C CNN
	1    9900 1950
	1    0    0    -1  
$EndComp
Text GLabel 9600 4600 0    50   Input ~ 0
Rs
Text GLabel 10600 2100 2    50   Output ~ 0
Rs
$Comp
L Device:C C5
U 1 1 61DCAF11
P 6100 4750
F 0 "C5" H 6215 4796 50  0000 L CNN
F 1 "100n" H 6215 4705 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6138 4600 50  0001 C CNN
F 3 "~" H 6100 4750 50  0001 C CNN
F 4 "C14663" H 6100 4750 50  0001 C CNN "LCSC"
	1    6100 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 4600 6100 4550
Wire Wire Line
	6100 4900 6100 4950
$Comp
L Transistor_BJT:MMBT3904 Q5
U 1 1 61CE5FA1
P 4000 4850
F 0 "Q5" H 4191 4896 50  0000 L CNN
F 1 "MMBT3904" H 4191 4805 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4200 4775 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/2N3903-D.PDF" H 4000 4850 50  0001 L CNN
F 4 "C20526" H 4000 4850 50  0001 C CNN "LCSC"
	1    4000 4850
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Schottky D5
U 1 1 61C8CA43
P 3850 4200
F 0 "D5" H 3850 4000 50  0000 C CNN
F 1 "SS54" H 3850 4100 50  0000 C CNN
F 2 "Diode_SMD:D_SMA" H 3850 4200 50  0001 C CNN
F 3 "~" H 3850 4200 50  0001 C CNN
F 4 "C22452" V 3850 4200 50  0001 C CNN "LCSC"
	1    3850 4200
	-1   0    0    1   
$EndComp
Wire Wire Line
	3400 4850 3500 4850
Text GLabel 3400 4850 0    50   Input ~ 0
MCU_SW
Wire Wire Line
	4100 5050 4100 5150
Wire Wire Line
	4100 4600 4100 4650
Wire Wire Line
	3650 4200 3700 4200
Text GLabel 3650 4200 0    50   BiDi ~ 0
EVSE_CP
Wire Wire Line
	4100 4300 4100 4200
Wire Wire Line
	4000 4200 4100 4200
$Comp
L Device:R R10
U 1 1 61977052
P 4100 4450
F 0 "R10" H 4030 4404 50  0000 R CNN
F 1 "1k5" H 4030 4495 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4030 4450 50  0001 C CNN
F 3 "~" H 4100 4450 50  0001 C CNN
F 4 "C22843" H 4100 4450 50  0001 C CNN "LCSC"
	1    4100 4450
	-1   0    0    1   
$EndComp
Connection ~ 4100 4200
Wire Wire Line
	4150 4200 4100 4200
$Comp
L Device:R R11
U 1 1 61CC475B
P 4300 4200
F 0 "R11" V 4500 4200 50  0000 C CNN
F 1 "2k7" V 4400 4200 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4230 4200 50  0001 C CNN
F 3 "~" H 4300 4200 50  0001 C CNN
F 4 "C13167" H 4300 4200 50  0001 C CNN "LCSC"
	1    4300 4200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4100 5150 4750 5150
Connection ~ 4750 5150
Wire Wire Line
	4750 4400 4750 5150
Wire Wire Line
	4750 3900 4750 4000
Wire Wire Line
	4800 3900 4750 3900
$Comp
L Transistor_BJT:MMBT3904 Q6
U 1 1 61DF0915
P 4650 4200
F 0 "Q6" H 4841 4246 50  0000 L CNN
F 1 "MMBT3904" H 4841 4155 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4850 4125 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/2N3903-D.PDF" H 4650 4200 50  0001 L CNN
F 4 "C20526" H 4650 4200 50  0001 C CNN "LCSC"
	1    4650 4200
	1    0    0    -1  
$EndComp
Text GLabel 4800 3900 2    50   Output ~ 0
MCU_CP
$Comp
L power:GND #PWR09
U 1 1 6199D4AB
P 4750 5150
F 0 "#PWR09" H 4750 4900 50  0001 C CNN
F 1 "GND" H 4755 4977 50  0000 C CNN
F 2 "" H 4750 5150 50  0001 C CNN
F 3 "" H 4750 5150 50  0001 C CNN
	1    4750 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 4550 6100 4550
Connection ~ 6100 4550
Wire Wire Line
	6100 4550 6200 4550
Wire Wire Line
	6100 4500 6100 4550
$Comp
L Device:R R9
U 1 1 61E8A9AC
P 3650 4850
F 0 "R9" V 3850 4850 50  0000 C CNN
F 1 "1k5" V 3750 4850 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3580 4850 50  0001 C CNN
F 3 "~" H 3650 4850 50  0001 C CNN
F 4 "C22843" H 3650 4850 50  0001 C CNN "LCSC"
	1    3650 4850
	0    -1   -1   0   
$EndComp
Text GLabel 4100 6550 0    50   Input ~ 0
IN12_3
$Comp
L power:GND #PWR03
U 1 1 61E01A48
P 4450 7150
F 0 "#PWR03" H 4450 6900 50  0001 C CNN
F 1 "GND" H 4455 6977 50  0000 C CNN
F 2 "" H 4450 7150 50  0001 C CNN
F 3 "" H 4450 7150 50  0001 C CNN
	1    4450 7150
	1    0    0    -1  
$EndComp
Text GLabel 4900 6850 2    50   Output ~ 0
IN3
$Comp
L Device:R R6
U 1 1 61E01A5F
P 4450 6700
F 0 "R6" H 4520 6746 50  0000 L CNN
F 1 "5.6k" H 4520 6655 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4380 6700 50  0001 C CNN
F 3 "~" H 4450 6700 50  0001 C CNN
F 4 "C23189" H 4450 6700 50  0001 C CNN "LCSC"
	1    4450 6700
	1    0    0    -1  
$EndComp
Connection ~ 4450 7150
$Comp
L Device:D_Zener D3
U 1 1 61E01A6D
P 4450 7000
F 0 "D3" V 4404 7080 50  0000 L CNN
F 1 "3.3v" V 4495 7080 50  0000 L CNN
F 2 "Diode_SMD:D_MiniMELF" H 4450 7000 50  0001 C CNN
F 3 "~" H 4450 7000 50  0001 C CNN
F 4 "C8056" V 4450 7000 50  0001 C CNN "LCSC"
	1    4450 7000
	0    1    1    0   
$EndComp
Wire Wire Line
	4100 6550 4150 6550
Wire Wire Line
	4150 6700 4150 6550
Connection ~ 4150 6550
Wire Wire Line
	4150 6550 4450 6550
Wire Wire Line
	4150 7000 4150 7150
Wire Wire Line
	4150 7150 4450 7150
Connection ~ 4450 6850
Text GLabel 5600 6550 0    50   Input ~ 0
IN12_4
$Comp
L power:GND #PWR04
U 1 1 61E06304
P 5950 7150
F 0 "#PWR04" H 5950 6900 50  0001 C CNN
F 1 "GND" H 5955 6977 50  0000 C CNN
F 2 "" H 5950 7150 50  0001 C CNN
F 3 "" H 5950 7150 50  0001 C CNN
	1    5950 7150
	1    0    0    -1  
$EndComp
Text GLabel 6400 6850 2    50   Output ~ 0
IN4
$Comp
L Device:R R8
U 1 1 61E0631B
P 5950 6700
F 0 "R8" H 6020 6746 50  0000 L CNN
F 1 "5.6k" H 6020 6655 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5880 6700 50  0001 C CNN
F 3 "~" H 5950 6700 50  0001 C CNN
F 4 "C23189" H 5950 6700 50  0001 C CNN "LCSC"
	1    5950 6700
	1    0    0    -1  
$EndComp
Connection ~ 5950 7150
Wire Wire Line
	5950 7150 6300 7150
$Comp
L Device:D_Zener D4
U 1 1 61E06329
P 5950 7000
F 0 "D4" V 5904 7080 50  0000 L CNN
F 1 "3.3v" V 5995 7080 50  0000 L CNN
F 2 "Diode_SMD:D_MiniMELF" H 5950 7000 50  0001 C CNN
F 3 "~" H 5950 7000 50  0001 C CNN
F 4 "C8056" V 5950 7000 50  0001 C CNN "LCSC"
	1    5950 7000
	0    1    1    0   
$EndComp
Wire Wire Line
	5600 6550 5650 6550
Wire Wire Line
	5650 6700 5650 6550
Connection ~ 5650 6550
Wire Wire Line
	5650 6550 5950 6550
Wire Wire Line
	5650 7000 5650 7150
Wire Wire Line
	5650 7150 5950 7150
Connection ~ 5950 6850
$Comp
L Device:R R5
U 1 1 61E78A41
P 4150 6850
F 0 "R5" H 4220 6896 50  0000 L CNN
F 1 "47k" H 4220 6805 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4080 6850 50  0001 C CNN
F 3 "~" H 4150 6850 50  0001 C CNN
F 4 "C25819" H 4150 6850 50  0001 C CNN "LCSC"
	1    4150 6850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 61E78FFA
P 5650 6850
F 0 "R7" H 5720 6896 50  0000 L CNN
F 1 "47k" H 5720 6805 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5580 6850 50  0001 C CNN
F 3 "~" H 5650 6850 50  0001 C CNN
F 4 "C25819" H 5650 6850 50  0001 C CNN "LCSC"
	1    5650 6850
	1    0    0    -1  
$EndComp
Connection ~ 4800 6850
Wire Wire Line
	4800 6850 4900 6850
Wire Wire Line
	5950 6850 6300 6850
Connection ~ 6300 6850
Wire Wire Line
	6300 6850 6400 6850
Wire Wire Line
	4450 7150 4800 7150
Wire Wire Line
	4450 6850 4800 6850
$Comp
L Device:C C4
U 1 1 61E92658
P 6300 7000
F 0 "C4" H 6415 7046 50  0000 L CNN
F 1 "100n" H 6415 6955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6338 6850 50  0001 C CNN
F 3 "~" H 6300 7000 50  0001 C CNN
F 4 "C14663" H 6300 7000 50  0001 C CNN "LCSC"
	1    6300 7000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 61E8EB55
P 4800 7000
F 0 "C3" H 4915 7046 50  0000 L CNN
F 1 "100n" H 4915 6955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4838 6850 50  0001 C CNN
F 3 "~" H 4800 7000 50  0001 C CNN
F 4 "C14663" H 4800 7000 50  0001 C CNN "LCSC"
	1    4800 7000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 61E883F5
P 3300 7000
F 0 "C2" H 3415 7046 50  0000 L CNN
F 1 "100n" H 3415 6955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3338 6850 50  0001 C CNN
F 3 "~" H 3300 7000 50  0001 C CNN
F 4 "C14663" H 3300 7000 50  0001 C CNN "LCSC"
	1    3300 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 6850 3400 6850
Connection ~ 3300 6850
Wire Wire Line
	2950 6850 3300 6850
$Comp
L Device:R R3
U 1 1 61E78568
P 2650 6850
F 0 "R3" H 2720 6896 50  0000 L CNN
F 1 "47k" H 2720 6805 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2580 6850 50  0001 C CNN
F 3 "~" H 2650 6850 50  0001 C CNN
F 4 "C25819" H 2650 6850 50  0001 C CNN "LCSC"
	1    2650 6850
	1    0    0    -1  
$EndComp
Connection ~ 2950 6850
Wire Wire Line
	2650 7150 2950 7150
Wire Wire Line
	2650 7000 2650 7150
Wire Wire Line
	2650 6550 2950 6550
Connection ~ 2650 6550
Wire Wire Line
	2650 6700 2650 6550
Wire Wire Line
	2600 6550 2650 6550
$Comp
L Device:D_Zener D2
U 1 1 61DFDF31
P 2950 7000
F 0 "D2" V 2904 7080 50  0000 L CNN
F 1 "3.3v" V 2995 7080 50  0000 L CNN
F 2 "Diode_SMD:D_MiniMELF" H 2950 7000 50  0001 C CNN
F 3 "~" H 2950 7000 50  0001 C CNN
F 4 "C8056" V 2950 7000 50  0001 C CNN "LCSC"
	1    2950 7000
	0    1    1    0   
$EndComp
Wire Wire Line
	2950 7150 3300 7150
Connection ~ 2950 7150
$Comp
L Device:R R4
U 1 1 61DFDF23
P 2950 6700
F 0 "R4" H 3020 6746 50  0000 L CNN
F 1 "5.6k" H 3020 6655 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2880 6700 50  0001 C CNN
F 3 "~" H 2950 6700 50  0001 C CNN
F 4 "C23189" H 2950 6700 50  0001 C CNN "LCSC"
	1    2950 6700
	1    0    0    -1  
$EndComp
Text GLabel 3400 6850 2    50   Output ~ 0
IN2
$Comp
L power:GND #PWR02
U 1 1 61DFDF0C
P 2950 7150
F 0 "#PWR02" H 2950 6900 50  0001 C CNN
F 1 "GND" H 2955 6977 50  0000 C CNN
F 2 "" H 2950 7150 50  0001 C CNN
F 3 "" H 2950 7150 50  0001 C CNN
	1    2950 7150
	1    0    0    -1  
$EndComp
Text GLabel 2600 6550 0    50   Input ~ 0
IN12_2
$Comp
L Device:C C1
U 1 1 61EA6266
P 1800 7000
F 0 "C1" H 1915 7046 50  0000 L CNN
F 1 "100n" H 1915 6955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1838 6850 50  0001 C CNN
F 3 "~" H 1800 7000 50  0001 C CNN
F 4 "C14663" H 1800 7000 50  0001 C CNN "LCSC"
	1    1800 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 6850 1900 6850
Connection ~ 1800 6850
Wire Wire Line
	1450 6850 1800 6850
$Comp
L Device:R R1
U 1 1 61E742FC
P 1150 6850
F 0 "R1" H 1220 6896 50  0000 L CNN
F 1 "47k" H 1220 6805 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1080 6850 50  0001 C CNN
F 3 "~" H 1150 6850 50  0001 C CNN
F 4 "C25819" H 1150 6850 50  0001 C CNN "LCSC"
	1    1150 6850
	1    0    0    -1  
$EndComp
Connection ~ 1450 6850
Wire Wire Line
	1150 7150 1450 7150
Wire Wire Line
	1150 7000 1150 7150
Wire Wire Line
	1150 6550 1450 6550
Connection ~ 1150 6550
Wire Wire Line
	1150 6700 1150 6550
Wire Wire Line
	1100 6550 1150 6550
$Comp
L Device:D_Zener D1
U 1 1 61EA8BF4
P 1450 7000
F 0 "D1" V 1404 7080 50  0000 L CNN
F 1 "3.3v" V 1495 7080 50  0000 L CNN
F 2 "Diode_SMD:D_MiniMELF" H 1450 7000 50  0001 C CNN
F 3 "~" H 1450 7000 50  0001 C CNN
F 4 "C8056" V 1450 7000 50  0001 C CNN "LCSC"
	1    1450 7000
	0    1    1    0   
$EndComp
Wire Wire Line
	1450 7150 1800 7150
Connection ~ 1450 7150
$Comp
L Device:R R2
U 1 1 61E6F64A
P 1450 6700
F 0 "R2" H 1520 6746 50  0000 L CNN
F 1 "5.6k" H 1520 6655 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1380 6700 50  0001 C CNN
F 3 "~" H 1450 6700 50  0001 C CNN
F 4 "C23189" H 1450 6700 50  0001 C CNN "LCSC"
	1    1450 6700
	1    0    0    -1  
$EndComp
Text GLabel 1900 6850 2    50   Output ~ 0
IN1
$Comp
L power:GND #PWR01
U 1 1 61D4702E
P 1450 7150
F 0 "#PWR01" H 1450 6900 50  0001 C CNN
F 1 "GND" H 1455 6977 50  0000 C CNN
F 2 "" H 1450 7150 50  0001 C CNN
F 3 "" H 1450 7150 50  0001 C CNN
	1    1450 7150
	1    0    0    -1  
$EndComp
Text GLabel 1100 6550 0    50   Input ~ 0
IN12_1
Wire Wire Line
	9400 6900 9700 6900
Connection ~ 9700 6900
Wire Wire Line
	9700 6900 10000 6900
Connection ~ 10000 6900
Wire Wire Line
	10000 6900 10300 6900
Connection ~ 10300 6900
$EndSCHEMATC
