EESchema Schematic File Version 4
LIBS:laser_range_PCB-cache
EELAYER 29 0
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
L RS_Downloads:VL53L0X VL53L0X1
U 1 1 5DD074CF
P 3950 2750
F 0 "VL53L0X1" H 4650 3015 50  0000 C CNN
F 1 "VL53L0X" H 4650 2924 50  0000 C CNN
F 2 "RS_Downloads:VL53L0X" H 5200 2850 50  0001 L CNN
F 3 "" H 5200 2750 50  0001 L CNN
F 4 "World smallest Time-of-Flight ranging and gesture detection sensor" H 5200 2650 50  0001 L CNN "Description"
F 5 "1" H 5200 2550 50  0001 L CNN "Height"
F 6 "" H 5200 2450 50  0001 L CNN "RS Part Number"
F 7 "" H 5200 2350 50  0001 L CNN "RS Price/Stock"
F 8 "STMicroelectronics" H 5200 2250 50  0001 L CNN "Manufacturer_Name"
F 9 "VL53L0X" H 5200 2150 50  0001 L CNN "Manufacturer_Part_Number"
	1    3950 2750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5DD0BB9D
P 3600 2950
F 0 "R1" H 3530 2904 50  0000 R CNN
F 1 "10k" H 3530 2995 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3530 2950 50  0001 C CNN
F 3 "~" H 3600 2950 50  0001 C CNN
	1    3600 2950
	-1   0    0    1   
$EndComp
$Comp
L Device:R R2
U 1 1 5DD0C8A8
P 5850 2950
F 0 "R2" H 5780 2904 50  0000 R CNN
F 1 "10k" H 5780 2995 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5780 2950 50  0001 C CNN
F 3 "~" H 5850 2950 50  0001 C CNN
	1    5850 2950
	-1   0    0    1   
$EndComp
$Comp
L Device:C C2
U 1 1 5DD0D455
P 5450 3950
F 0 "C2" V 5198 3950 50  0000 C CNN
F 1 "10u" V 5289 3950 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5488 3800 50  0001 C CNN
F 3 "~" H 5450 3950 50  0001 C CNN
	1    5450 3950
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5DD0E820
P 3850 3450
F 0 "#PWR0101" H 3850 3200 50  0001 C CNN
F 1 "GND" H 3855 3277 50  0000 C CNN
F 2 "" H 3850 3450 50  0001 C CNN
F 3 "" H 3850 3450 50  0001 C CNN
	1    3850 3450
	-1   0    0    -1  
$EndComp
Text Label 3400 2750 0    50   ~ 0
3V3
Text Label 6100 3150 0    50   ~ 0
3V3
Wire Wire Line
	5350 3050 5550 3050
Wire Wire Line
	5350 2950 5550 2950
Text Label 5550 2950 0    50   ~ 0
SDA
Text Label 5550 3050 0    50   ~ 0
SCL
Wire Wire Line
	3850 2950 3850 3050
Wire Wire Line
	3950 3250 3850 3250
Connection ~ 3850 3250
Wire Wire Line
	3850 3250 3850 3400
Wire Wire Line
	3950 3050 3850 3050
Connection ~ 3850 3050
Wire Wire Line
	3850 3050 3850 3250
Wire Wire Line
	3950 2950 3850 2950
Wire Wire Line
	5350 3250 5350 3400
Wire Wire Line
	5350 3400 5200 3400
Connection ~ 3850 3400
Wire Wire Line
	3850 3400 3850 3450
Wire Wire Line
	3600 3150 3600 3100
Wire Wire Line
	3600 2750 3600 2800
Wire Wire Line
	3600 2750 3400 2750
Connection ~ 3600 2750
Wire Wire Line
	5850 3100 5850 3150
Connection ~ 5850 3150
Wire Wire Line
	5850 3150 6100 3150
Wire Wire Line
	5350 2750 5850 2750
Wire Wire Line
	5850 2750 5850 2800
Wire Wire Line
	5850 2750 6100 2750
Connection ~ 5850 2750
Text Label 6100 2750 0    50   ~ 0
INT
Wire Wire Line
	3950 2850 3850 2850
Wire Wire Line
	3850 2850 3850 2950
Connection ~ 3850 2950
Wire Wire Line
	3950 3150 3600 3150
Wire Wire Line
	3600 2750 3950 2750
Wire Wire Line
	2000 2750 2250 2750
Wire Wire Line
	2750 2750 3000 2750
Wire Wire Line
	2000 2850 2250 2850
Wire Wire Line
	2750 2850 3000 2850
Wire Wire Line
	2000 2950 2250 2950
Wire Wire Line
	5350 3150 5700 3150
$Comp
L Device:C C1
U 1 1 5DD0CB6A
P 5450 3550
F 0 "C1" V 5198 3550 50  0000 C CNN
F 1 "100n" V 5289 3550 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5488 3400 50  0001 C CNN
F 3 "~" H 5450 3550 50  0001 C CNN
	1    5450 3550
	0    1    1    0   
$EndComp
Wire Wire Line
	5300 3950 5200 3950
Wire Wire Line
	5200 3950 5200 3550
Connection ~ 5200 3400
Wire Wire Line
	5200 3400 3850 3400
Wire Wire Line
	5300 3550 5200 3550
Connection ~ 5200 3550
Wire Wire Line
	5200 3550 5200 3400
Wire Wire Line
	5600 3950 5700 3950
Wire Wire Line
	5700 3950 5700 3550
Connection ~ 5700 3150
Wire Wire Line
	5700 3150 5850 3150
Wire Wire Line
	5600 3550 5700 3550
Connection ~ 5700 3550
Wire Wire Line
	5700 3550 5700 3150
Text Label 2000 2750 0    50   ~ 0
3V3
Text Label 3000 2750 0    50   ~ 0
GND
Text Label 2000 2850 0    50   ~ 0
SDA
Text Label 3000 2850 0    50   ~ 0
SCL
Text Label 2000 2950 0    50   ~ 0
GND
$Comp
L Connector_Generic:Conn_02x03_Odd_Even J1
U 1 1 5DD29773
P 2450 2850
F 0 "J1" H 2500 3167 50  0000 C CNN
F 1 "Conn_02x03_Odd_Even" H 2500 3076 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x03_P2.54mm_Vertical" H 2450 2850 50  0001 C CNN
F 3 "~" H 2450 2850 50  0001 C CNN
	1    2450 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 2950 3000 2950
Text Label 3000 2950 0    50   ~ 0
INT
$EndSCHEMATC
