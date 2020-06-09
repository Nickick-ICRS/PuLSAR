EESchema Schematic File Version 4
LIBS:main_pcb-cache
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
L RS_Downloads:BMX055 IMU1
U 1 1 5DD3E5EC
P 3250 2650
F 0 "IMU1" H 3850 2915 50  0000 C CNN
F 1 "BMX055" H 3850 2824 50  0000 C CNN
F 2 "RS_Downloads:LGA_PACKAGE_20_PINS" H 4300 2750 50  0001 L CNN
F 3 "" H 4300 2650 50  0001 L CNN
F 4 "Small versatile 9-axis sensor module" H 4300 2550 50  0001 L CNN "Description"
F 5 "" H 4300 2450 50  0001 L CNN "Height"
F 6 "" H 4300 2350 50  0001 L CNN "RS Part Number"
F 7 "" H 4300 2250 50  0001 L CNN "RS Price/Stock"
F 8 "Bosch Sensortec" H 4300 2150 50  0001 L CNN "Manufacturer_Name"
F 9 "BMX055" H 4300 2050 50  0001 L CNN "Manufacturer_Part_Number"
	1    3250 2650
	1    0    0    -1  
$EndComp
Text Label 2900 2850 0    50   ~ 0
IMU_VDD
Text Label 4550 2850 0    50   ~ 0
IMU_VDDIO
Wire Wire Line
	3250 3450 3000 3450
Wire Wire Line
	4450 2650 4700 2650
Text Label 3000 3450 0    50   ~ 0
SCL
Text Label 4700 2650 0    50   ~ 0
SDA
$Comp
L power:GND #PWR0101
U 1 1 5DD43857
P 2650 3650
F 0 "#PWR0101" H 2650 3400 50  0001 C CNN
F 1 "GND" H 2655 3477 50  0000 C CNN
F 2 "" H 2650 3650 50  0001 C CNN
F 3 "" H 2650 3650 50  0001 C CNN
	1    2650 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 3150 3250 3150
Wire Wire Line
	2650 2950 2650 3150
Wire Wire Line
	2650 2950 3250 2950
Connection ~ 2650 3150
Wire Wire Line
	4450 2750 4950 2750
Wire Wire Line
	4450 3250 4950 3250
Connection ~ 4950 3250
Wire Wire Line
	4950 3250 4950 3550
$Comp
L power:GND #PWR0102
U 1 1 5DD44916
P 4950 3650
F 0 "#PWR0102" H 4950 3400 50  0001 C CNN
F 1 "GND" H 4955 3477 50  0000 C CNN
F 2 "" H 4950 3650 50  0001 C CNN
F 3 "" H 4950 3650 50  0001 C CNN
	1    4950 3650
	1    0    0    -1  
$EndComp
Text Notes 2800 2300 0    50   Italic 0
I2C Addresses: 0x18 (accel), 0x68 (gyro), 0x10 (magnet)
Wire Wire Line
	4450 3550 4950 3550
Connection ~ 4950 3550
Wire Wire Line
	4950 3550 4950 3650
$Comp
L Device:C C1
U 1 1 5DD45A12
P 2050 2500
F 0 "C1" H 1935 2546 50  0000 R CNN
F 1 "100n" H 1935 2455 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2088 2350 50  0001 C CNN
F 3 "~" H 2050 2500 50  0001 C CNN
	1    2050 2500
	-1   0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5DD4712D
P 2050 3400
F 0 "C2" H 1935 3446 50  0000 R CNN
F 1 "100n" H 1935 3355 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2088 3250 50  0001 C CNN
F 3 "~" H 2050 3400 50  0001 C CNN
	1    2050 3400
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2650 3150 2650 3650
Text Label 2850 3250 0    50   Italic 0
IMU_VDDIO
Wire Wire Line
	2050 2350 2050 2250
Wire Wire Line
	2050 2250 1850 2250
Text Label 1850 2250 0    50   Italic 0
GND
Wire Wire Line
	2050 2650 2050 2750
Text Label 2050 2850 0    50   Italic 0
IMU_VDD
Wire Wire Line
	4450 2850 4550 2850
Wire Wire Line
	4950 2750 4950 3250
Wire Wire Line
	2050 3550 2050 3750
Text Label 2050 3750 0    50   Italic 0
GND
Wire Wire Line
	2050 3250 2050 3150
Wire Wire Line
	2050 3150 2050 3000
Connection ~ 2050 3150
Text Label 2050 3000 0    50   Italic 0
IMU_VDDIO
Text Label 1850 3150 0    50   Italic 0
3V3
Wire Wire Line
	1850 3150 2050 3150
Wire Wire Line
	2050 2750 1850 2750
Connection ~ 2050 2750
Wire Wire Line
	2050 2750 2050 2850
Text Label 1850 2750 0    50   Italic 0
3V3
Wire Wire Line
	2900 2850 3250 2850
Wire Wire Line
	2850 3250 3250 3250
Wire Wire Line
	8350 1600 8850 1600
Wire Wire Line
	8350 1700 8850 1700
Wire Wire Line
	8350 1800 8850 1800
Text Label 8850 1600 0    50   ~ 0
IO0
Text Label 8850 1700 0    50   ~ 0
TX
Text Label 8850 1800 0    50   ~ 0
IO2
Wire Wire Line
	8350 1900 8850 1900
Wire Wire Line
	8350 2000 8850 2000
Wire Wire Line
	8350 2100 8850 2100
Text Label 8850 1900 0    50   ~ 0
RX
Text Label 8850 2000 0    50   ~ 0
IO4
Text Label 8850 2100 0    50   ~ 0
IO5
Wire Wire Line
	8350 2200 8850 2200
Wire Wire Line
	8350 2300 8850 2300
Wire Wire Line
	8350 2400 8850 2400
Wire Wire Line
	8350 2500 8850 2500
Wire Wire Line
	8350 2600 8850 2600
Wire Wire Line
	8350 2700 8850 2700
Wire Wire Line
	8350 2800 8850 2800
Wire Wire Line
	8350 2900 8850 2900
Wire Wire Line
	8350 3000 8850 3000
Wire Wire Line
	8350 3100 8850 3100
Wire Wire Line
	8350 3200 8850 3200
Wire Wire Line
	8350 3300 8850 3300
Wire Wire Line
	8350 3400 8850 3400
Wire Wire Line
	8350 3500 8850 3500
Wire Wire Line
	8350 3600 8850 3600
Wire Wire Line
	8350 3700 8850 3700
Wire Wire Line
	8350 3800 8850 3800
Wire Wire Line
	8350 3900 8850 3900
Wire Wire Line
	7150 2800 6650 2800
Wire Wire Line
	7150 2900 6650 2900
Wire Wire Line
	7150 3000 6650 3000
Wire Wire Line
	7150 3100 6650 3100
Wire Wire Line
	7150 3200 6650 3200
Wire Wire Line
	7150 3300 6650 3300
Wire Wire Line
	7150 1800 6650 1800
Wire Wire Line
	7150 1900 6650 1900
Wire Wire Line
	7750 4200 7750 4400
$Comp
L power:GND #PWR0103
U 1 1 5DE2B911
P 7750 4400
F 0 "#PWR0103" H 7750 4150 50  0001 C CNN
F 1 "GND" H 7755 4227 50  0000 C CNN
F 2 "" H 7750 4400 50  0001 C CNN
F 3 "" H 7750 4400 50  0001 C CNN
	1    7750 4400
	1    0    0    -1  
$EndComp
Text Label 6650 1300 0    50   ~ 0
3V3
Text Label 8850 2200 0    50   ~ 0
MTDI
Text Label 8850 2300 0    50   ~ 0
IO13
Text Label 8850 2400 0    50   ~ 0
MTMS
Text Label 8850 2500 0    50   ~ 0
MTDO
Text Label 8850 2600 0    50   ~ 0
IO16
Text Label 8850 2700 0    50   ~ 0
MTCK
Text Label 8850 2800 0    50   ~ 0
IO18
Text Label 8850 2900 0    50   ~ 0
IO19
Text Label 8850 3000 0    50   ~ 0
IO21
Text Label 8850 3100 0    50   ~ 0
IO22
Text Label 8850 3200 0    50   ~ 0
IO23
Text Label 8850 3300 0    50   ~ 0
IO25
Text Label 8850 3400 0    50   ~ 0
IO26
Text Label 8850 3500 0    50   ~ 0
IO27
Text Label 8850 3600 0    50   ~ 0
IO32
Text Label 8850 3700 0    50   ~ 0
IO33
Text Label 8850 3800 0    50   ~ 0
IO34
Text Label 8850 3900 0    50   ~ 0
IO35
Text Label 6650 2800 0    50   ~ 0
SDO
Text Label 6650 2900 0    50   ~ 0
SDI
Text Label 6650 3000 0    50   ~ 0
SHD
Text Label 6650 3100 0    50   ~ 0
SWP
Text Label 6650 3200 0    50   ~ 0
SCK
Text Label 6650 3300 0    50   ~ 0
SCS
Text Label 6650 1900 0    50   ~ 0
IO39
Text Label 6650 1800 0    50   ~ 0
IO36
Text Label 7050 1600 0    50   ~ 0
EN
$Comp
L RF_Module:ESP32-WROOM-32D ESP32
U 1 1 5DE34F84
P 7750 2800
F 0 "ESP32" H 7750 4381 50  0000 C CNN
F 1 "ESP32-WROOM-32D" H 7750 4290 50  0000 C CNN
F 2 "RF_Module:ESP32-WROOM-32" H 7750 1300 50  0001 C CNN
F 3 "https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32d_esp32-wroom-32u_datasheet_en.pdf" H 7450 2850 50  0001 C CNN
	1    7750 2800
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5DE363BB
P 8450 1400
F 0 "C3" V 8198 1400 50  0000 C CNN
F 1 "100n" V 8289 1400 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 8488 1250 50  0001 C CNN
F 3 "~" H 8450 1400 50  0001 C CNN
	1    8450 1400
	0    1    1    0   
$EndComp
Wire Wire Line
	7750 1400 8200 1400
Wire Wire Line
	8600 1400 8700 1400
$Comp
L Device:R R1
U 1 1 5DE3C8F1
P 6950 1450
F 0 "R1" H 7020 1496 50  0000 L CNN
F 1 "10k" H 7020 1405 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 6880 1450 50  0001 C CNN
F 3 "~" H 6950 1450 50  0001 C CNN
	1    6950 1450
	1    0    0    -1  
$EndComp
Text Label 9650 1800 0    50   ~ 0
MTDO
$Comp
L Device:R R2
U 1 1 5DE4234D
P 9850 1950
F 0 "R2" H 9920 1996 50  0000 L CNN
F 1 "10k" H 9920 1905 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9780 1950 50  0001 C CNN
F 3 "~" H 9850 1950 50  0001 C CNN
	1    9850 1950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5DE450D9
P 9850 2200
F 0 "#PWR0104" H 9850 1950 50  0001 C CNN
F 1 "GND" H 9855 2027 50  0000 C CNN
F 2 "" H 9850 2200 50  0001 C CNN
F 3 "" H 9850 2200 50  0001 C CNN
	1    9850 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9650 1400 10350 1400
Text Label 9650 1400 0    50   ~ 0
IO0
Wire Wire Line
	9650 1300 10350 1300
Wire Wire Line
	9650 1200 10350 1200
Text Label 9650 1200 0    50   ~ 0
TX
Text Label 9650 1300 0    50   ~ 0
RX
$Comp
L power:GND #PWR0105
U 1 1 5DE3BC32
P 9150 1400
F 0 "#PWR0105" H 9150 1150 50  0001 C CNN
F 1 "GND" H 9155 1227 50  0000 C CNN
F 2 "" H 9150 1400 50  0001 C CNN
F 3 "" H 9150 1400 50  0001 C CNN
	1    9150 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5DE5351E
P 8450 1000
F 0 "C4" V 8198 1000 50  0000 C CNN
F 1 "10u" V 8289 1000 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 8488 850 50  0001 C CNN
F 3 "~" H 8450 1000 50  0001 C CNN
	1    8450 1000
	0    1    1    0   
$EndComp
Wire Wire Line
	8200 1400 8200 1000
Wire Wire Line
	8200 1000 8300 1000
Connection ~ 8200 1400
Wire Wire Line
	8200 1400 8300 1400
Wire Wire Line
	8600 1000 8700 1000
Wire Wire Line
	8700 1000 8700 1400
Connection ~ 8700 1400
Wire Wire Line
	8700 1400 9150 1400
Wire Wire Line
	9850 2100 9850 2150
Wire Wire Line
	10100 1900 10100 2150
Wire Wire Line
	10100 2150 9850 2150
Connection ~ 9850 2150
Wire Wire Line
	9850 2150 9850 2200
Wire Wire Line
	10350 1700 10200 1700
Wire Wire Line
	9650 1600 9900 1600
Wire Wire Line
	9650 1500 9900 1500
Text Label 9650 1700 0    50   ~ 0
MTCK
Text Label 9650 1600 0    50   ~ 0
MTDI
Text Label 9650 1500 0    50   ~ 0
MTMS
$Comp
L Device:R R4
U 1 1 5DE7D12B
P 10050 1700
F 0 "R4" V 10000 1700 50  0000 C CNN
F 1 "100" V 10050 1700 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9980 1700 50  0001 C CNN
F 3 "~" H 10050 1700 50  0001 C CNN
	1    10050 1700
	0    1    1    0   
$EndComp
Wire Wire Line
	9900 1700 9650 1700
$Comp
L Device:R R5
U 1 1 5DE802D3
P 10050 1600
F 0 "R5" V 10000 1600 50  0000 C CNN
F 1 "100" V 10050 1600 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9980 1600 50  0001 C CNN
F 3 "~" H 10050 1600 50  0001 C CNN
	1    10050 1600
	0    1    1    0   
$EndComp
Wire Wire Line
	10200 1600 10350 1600
$Comp
L Device:R R6
U 1 1 5DE8061E
P 10050 1500
F 0 "R6" V 10000 1500 50  0000 C CNN
F 1 "100" V 10050 1500 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9980 1500 50  0001 C CNN
F 3 "~" H 10050 1500 50  0001 C CNN
	1    10050 1500
	0    1    1    0   
$EndComp
Wire Wire Line
	10200 1500 10350 1500
Wire Wire Line
	9650 1800 9850 1800
$Comp
L Device:R R3
U 1 1 5DE84E51
P 10050 1800
F 0 "R3" V 10000 1800 50  0000 C CNN
F 1 "100" V 10050 1800 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9980 1800 50  0001 C CNN
F 3 "~" H 10050 1800 50  0001 C CNN
	1    10050 1800
	0    1    1    0   
$EndComp
Wire Wire Line
	10200 1800 10350 1800
Connection ~ 9850 1800
Wire Wire Line
	9850 1800 9900 1800
Wire Wire Line
	10100 1900 10350 1900
$Comp
L Connector_Generic:Conn_01x08 UART_PROG1
U 1 1 5DE8B943
P 10550 1500
F 0 "UART_PROG1" H 10630 1492 50  0000 L CNN
F 1 "Conn_01x08" H 10630 1401 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical" H 10550 1500 50  0001 C CNN
F 3 "~" H 10550 1500 50  0001 C CNN
	1    10550 1500
	1    0    0    -1  
$EndComp
Text Label 10200 1900 0    50   ~ 0
GND
Wire Wire Line
	9600 3000 9850 3000
Wire Wire Line
	9600 3100 9850 3100
$Comp
L Device:R R7
U 1 1 5DE9EC5B
P 10100 3250
F 0 "R7" V 10000 3250 50  0000 C CNN
F 1 "4k7" V 10100 3250 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 10030 3250 50  0001 C CNN
F 3 "~" H 10100 3250 50  0001 C CNN
	1    10100 3250
	0    1    1    0   
$EndComp
$Comp
L Device:R R8
U 1 1 5DE9FF52
P 10100 2850
F 0 "R8" V 10000 2850 50  0000 C CNN
F 1 "4k7" V 10100 2850 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 10030 2850 50  0001 C CNN
F 3 "~" H 10100 2850 50  0001 C CNN
	1    10100 2850
	0    1    1    0   
$EndComp
Text Label 10100 3100 0    50   ~ 0
SDA
Text Label 10100 3000 0    50   ~ 0
SCL
Text Label 9600 3000 0    50   ~ 0
IO21
Text Label 9600 3100 0    50   ~ 0
IO22
Wire Wire Line
	10250 2850 10400 2850
Wire Wire Line
	10400 2850 10400 3250
Wire Wire Line
	10400 3250 10250 3250
Wire Wire Line
	10400 3250 10400 3350
Connection ~ 10400 3250
$Comp
L power:GND #PWR0106
U 1 1 5DEA64AE
P 10400 3350
F 0 "#PWR0106" H 10400 3100 50  0001 C CNN
F 1 "GND" H 10405 3177 50  0000 C CNN
F 2 "" H 10400 3350 50  0001 C CNN
F 3 "" H 10400 3350 50  0001 C CNN
	1    10400 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 3100 9850 3250
Wire Wire Line
	9850 3250 9950 3250
Connection ~ 9850 3100
Wire Wire Line
	9850 3100 10100 3100
Wire Wire Line
	9950 2850 9850 2850
Wire Wire Line
	9850 2850 9850 3000
Connection ~ 9850 3000
Wire Wire Line
	9850 3000 10100 3000
Text Notes 9800 2700 0    100  ~ 0
I2C Bus
Text Notes 9400 1000 0    100  ~ 0
Programming Circuit
Wire Wire Line
	6950 1600 7150 1600
Wire Wire Line
	7750 1400 7250 1400
Wire Wire Line
	7250 1400 7250 1300
Wire Wire Line
	7250 1300 6950 1300
Connection ~ 7750 1400
Connection ~ 6950 1300
Wire Wire Line
	6950 1300 6650 1300
$Comp
L RS_Downloads:DRV8847PWR H-Bridge1
U 1 1 5DEC3BC6
P 8800 4850
F 0 "H-Bridge1" H 9400 5115 50  0000 C CNN
F 1 "DRV8847PWR" H 9400 5024 50  0000 C CNN
F 2 "RS_Downloads:SOP65P640X120-16N" H 9850 4950 50  0001 L CNN
F 3 "http://www.ti.com/lit/gpn/DRV8847" H 9850 4850 50  0001 L CNN
F 4 "2A Dual H-Bridge Motor Driver" H 9850 4750 50  0001 L CNN "Description"
F 5 "1.2" H 9850 4650 50  0001 L CNN "Height"
F 6 "" H 9850 4550 50  0001 L CNN "RS Part Number"
F 7 "" H 9850 4450 50  0001 L CNN "RS Price/Stock"
F 8 "Texas Instruments" H 9850 4350 50  0001 L CNN "Manufacturer_Name"
F 9 "DRV8847PWR" H 9850 4250 50  0001 L CNN "Manufacturer_Part_Number"
	1    8800 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	10000 4850 10350 4850
Wire Wire Line
	10000 4950 10350 4950
Wire Wire Line
	10000 5450 10350 5450
Wire Wire Line
	10000 5550 10350 5550
Text Label 10350 4850 0    50   ~ 0
IO23
Text Label 10350 4950 0    50   ~ 0
IO25
Text Label 10350 5450 0    50   ~ 0
IO26
Text Label 10350 5550 0    50   ~ 0
IO27
Wire Wire Line
	8800 4950 8500 4950
Wire Wire Line
	8800 5150 8500 5150
Wire Wire Line
	8800 5250 8500 5250
Wire Wire Line
	8800 5450 8500 5450
Text Label 8500 5450 0    50   ~ 0
MTR_2A
Text Label 8500 5250 0    50   ~ 0
MTR_2B
Text Label 8500 5150 0    50   ~ 0
MTR_1B
Text Label 8500 4950 0    50   ~ 0
MTR_1A
Wire Wire Line
	8800 5050 8350 5050
Wire Wire Line
	8350 5050 8350 5200
Wire Wire Line
	8350 5350 8800 5350
Wire Wire Line
	8350 5200 8200 5200
Wire Wire Line
	8200 5200 8200 5250
Connection ~ 8350 5200
Wire Wire Line
	8350 5200 8350 5350
$Comp
L power:GND #PWR0107
U 1 1 5DEF04F4
P 8200 5250
F 0 "#PWR0107" H 8200 5000 50  0001 C CNN
F 1 "GND" H 8205 5077 50  0000 C CNN
F 2 "" H 8200 5250 50  0001 C CNN
F 3 "" H 8200 5250 50  0001 C CNN
	1    8200 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	10000 5150 10250 5150
Wire Wire Line
	10750 5150 10750 5200
$Comp
L power:GND #PWR0108
U 1 1 5DEF4DC2
P 10750 5200
F 0 "#PWR0108" H 10750 4950 50  0001 C CNN
F 1 "GND" H 10755 5027 50  0000 C CNN
F 2 "" H 10750 5200 50  0001 C CNN
F 3 "" H 10750 5200 50  0001 C CNN
	1    10750 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8800 5550 8650 5550
Text Label 8500 5550 0    50   ~ 0
H-FAULT
Wire Wire Line
	8650 5550 8650 5650
Connection ~ 8650 5550
Wire Wire Line
	8650 5550 8500 5550
$Comp
L Device:R R9
U 1 1 5DEFDF50
P 8650 5800
F 0 "R9" H 8720 5846 50  0000 L CNN
F 1 "10k" H 8720 5755 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 8580 5800 50  0001 C CNN
F 3 "~" H 8650 5800 50  0001 C CNN
	1    8650 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 5950 8650 6150
Text Label 8650 6150 0    50   ~ 0
3V3
Wire Wire Line
	10000 5250 10100 5250
Text Label 10350 5250 0    50   ~ 0
VBAT
Wire Wire Line
	10100 5250 10100 5750
Connection ~ 10100 5250
Wire Wire Line
	10100 5250 10350 5250
$Comp
L Device:C C5
U 1 1 5DF10A23
P 9950 5950
F 0 "C5" H 9750 6000 50  0000 L CNN
F 1 "100n" H 9650 5900 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 9988 5800 50  0001 C CNN
F 3 "~" H 9950 5950 50  0001 C CNN
	1    9950 5950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 5DF117F5
P 10250 5950
F 0 "C6" H 10365 5996 50  0000 L CNN
F 1 "10u" H 10365 5905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 10288 5800 50  0001 C CNN
F 3 "~" H 10250 5950 50  0001 C CNN
	1    10250 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	10100 5750 9950 5750
Wire Wire Line
	9950 5750 9950 5800
Wire Wire Line
	10100 5750 10250 5750
Wire Wire Line
	10250 5750 10250 5800
Connection ~ 10100 5750
Wire Wire Line
	9950 6100 10100 6100
Wire Wire Line
	10100 6100 10100 6150
Connection ~ 10100 6100
Wire Wire Line
	10100 6100 10250 6100
$Comp
L power:GND #PWR0109
U 1 1 5DF2495E
P 10100 6150
F 0 "#PWR0109" H 10100 5900 50  0001 C CNN
F 1 "GND" H 10105 5977 50  0000 C CNN
F 2 "" H 10100 6150 50  0001 C CNN
F 3 "" H 10100 6150 50  0001 C CNN
	1    10100 6150
	1    0    0    -1  
$EndComp
Wire Wire Line
	8800 4850 8500 4850
Text Label 8500 4850 0    50   ~ 0
3V3
Wire Wire Line
	10000 5350 10250 5350
Wire Wire Line
	10250 5350 10250 5150
Connection ~ 10250 5150
Wire Wire Line
	10250 5150 10750 5150
Wire Wire Line
	10250 5050 10250 5150
Wire Wire Line
	10000 5050 10250 5050
$Comp
L s-1170b33uc-otstfg:S-1170B33UC-OTSTFG REG_3v3
U 1 1 5DF41391
P 1900 1250
F 0 "REG_3v3" H 1900 1592 50  0000 C CNN
F 1 "S-1170B33UC-OTSTFG" H 1900 1501 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-89-5" H 1900 1550 50  0001 C CNN
F 3 "https://www.mouser.co.uk/datasheet/2/360/S1170_E-1365746.pdf" H 1900 1250 50  0001 C CNN
	1    1900 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 1150 2400 1150
Wire Wire Line
	2300 1350 2400 1350
Wire Wire Line
	1500 1150 1250 1150
$Comp
L power:GND #PWR0110
U 1 1 5DF53B32
P 1900 1550
F 0 "#PWR0110" H 1900 1300 50  0001 C CNN
F 1 "GND" H 1905 1377 50  0000 C CNN
F 2 "" H 1900 1550 50  0001 C CNN
F 3 "" H 1900 1550 50  0001 C CNN
	1    1900 1550
	1    0    0    -1  
$EndComp
Text Label 1250 1150 0    50   ~ 0
3V3
Wire Wire Line
	2400 1150 2400 1350
Connection ~ 2400 1350
Wire Wire Line
	2400 1350 2550 1350
Text Label 2550 1350 0    50   ~ 0
VBAT
Text Notes 1350 800  0    100  ~ 0
Power Regulator
$Comp
L Connector_Generic:Conn_01x02 JST1
U 1 1 5DF6A296
P 3650 1100
F 0 "JST1" H 3730 1092 50  0000 L CNN
F 1 "Conn_01x02" H 3730 1001 50  0000 L CNN
F 2 "Connector_JST:JST_EH_B2B-EH-A_1x02_P2.50mm_Vertical" H 3650 1100 50  0001 C CNN
F 3 "~" H 3650 1100 50  0001 C CNN
	1    3650 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 1100 3200 1100
Wire Wire Line
	3450 1200 3200 1200
Text Label 3200 1200 0    50   ~ 0
GND
Text Label 3200 1100 0    50   ~ 0
VBAT
Text Notes 3050 950  0    100  ~ 0
Battery Connector
Text Notes 8950 4450 0    100  ~ 0
Motor Driver
Text Notes 3750 2100 0    100  ~ 0
IMU
Text Notes 7600 1000 0    100  ~ 0
MCU
$Comp
L Connector_Generic:Conn_01x04 ENC1
U 1 1 5DF99960
P 6200 5500
F 0 "ENC1" H 6150 5800 50  0000 L CNN
F 1 "Conn_01x04" H 5950 5700 50  0000 L CNN
F 2 "RS_Downloads:Connector_Pad_4x2.0x2.0mm" H 6200 5500 50  0001 C CNN
F 3 "~" H 6200 5500 50  0001 C CNN
	1    6200 5500
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 ENC2
U 1 1 5DF9AB4E
P 6700 5600
F 0 "ENC2" H 6700 5200 50  0000 C CNN
F 1 "Conn_01x04" H 6700 5300 50  0000 C CNN
F 2 "RS_Downloads:Connector_Pad_4x2.0x2.0mm" H 6700 5600 50  0001 C CNN
F 3 "~" H 6700 5600 50  0001 C CNN
	1    6700 5600
	-1   0    0    1   
$EndComp
Wire Wire Line
	6900 5400 7150 5400
Wire Wire Line
	6900 5500 7150 5500
Wire Wire Line
	6900 5600 7150 5600
Wire Wire Line
	6900 5700 7150 5700
Wire Wire Line
	6000 5700 5750 5700
Wire Wire Line
	6000 5600 5750 5600
Wire Wire Line
	6000 5500 5750 5500
Wire Wire Line
	6000 5400 5750 5400
Text Label 5750 5400 0    50   ~ 0
3V3
Text Label 5750 5500 0    50   ~ 0
GND
Text Label 7150 5600 0    50   ~ 0
GND
Text Label 7150 5700 0    50   ~ 0
3V3
Text Label 7150 5500 0    50   ~ 0
A2
Text Label 7150 5400 0    50   ~ 0
B2
Text Label 5750 5600 0    50   ~ 0
A1
Text Label 5750 5700 0    50   ~ 0
B1
Text Notes 6100 5050 0    100  ~ 0
Encoders
Wire Wire Line
	5900 5950 6200 5950
Wire Wire Line
	5900 6050 6200 6050
Wire Wire Line
	6700 5950 7000 5950
Wire Wire Line
	7000 6050 6700 6050
Text Label 5900 5950 0    50   ~ 0
A1
Text Label 5900 6050 0    50   ~ 0
B1
Text Label 7000 5950 0    50   ~ 0
A2
Text Label 7000 6050 0    50   ~ 0
B2
Text Notes 9200 2600 1    50   ~ 0
IO0, 2, 4, 5 used at boot
Text Label 6200 5950 0    50   ~ 0
IO36
Text Label 6200 6050 0    50   ~ 0
IO39
Text Label 6700 5950 0    50   ~ 0
IO34
Text Label 6700 6050 0    50   ~ 0
IO35
$Comp
L Connector_Generic:Conn_02x03_Odd_Even LASER1
U 1 1 5E0140FB
P 4350 5500
F 0 "LASER1" H 4400 5817 50  0000 C CNN
F 1 "Conn_02x03_Odd_Even" H 4400 5726 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Horizontal" H 4350 5500 50  0001 C CNN
F 3 "~" H 4350 5500 50  0001 C CNN
	1    4350 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 5400 3900 5400
Wire Wire Line
	4150 5500 3900 5500
Wire Wire Line
	4150 5600 3900 5600
Wire Wire Line
	4650 5600 4900 5600
Wire Wire Line
	4650 5500 4900 5500
Text Label 3900 5400 0    50   ~ 0
3V3
Text Label 4900 5500 0    50   ~ 0
SC1
Text Label 4900 5600 0    50   ~ 0
INT1
Text Label 3900 5500 0    50   ~ 0
SD1
Wire Wire Line
	3900 5600 3900 5700
Wire Wire Line
	5200 5400 5200 5500
Wire Wire Line
	4650 5400 5200 5400
$Comp
L power:GND #PWR0111
U 1 1 5E05C9C4
P 3900 5700
F 0 "#PWR0111" H 3900 5450 50  0001 C CNN
F 1 "GND" H 3905 5527 50  0000 C CNN
F 2 "" H 3900 5700 50  0001 C CNN
F 3 "" H 3900 5700 50  0001 C CNN
	1    3900 5700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 5E05D259
P 5200 5500
F 0 "#PWR0112" H 5200 5250 50  0001 C CNN
F 1 "GND" H 5205 5327 50  0000 C CNN
F 2 "" H 5200 5500 50  0001 C CNN
F 3 "" H 5200 5500 50  0001 C CNN
	1    5200 5500
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x03_Odd_Even LASER2
U 1 1 5E05FA6D
P 4350 6250
F 0 "LASER2" H 4400 6567 50  0000 C CNN
F 1 "Conn_02x03_Odd_Even" H 4400 6476 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Horizontal" H 4350 6250 50  0001 C CNN
F 3 "~" H 4350 6250 50  0001 C CNN
	1    4350 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 6150 3900 6150
Wire Wire Line
	4150 6250 3900 6250
Wire Wire Line
	4150 6350 3900 6350
Wire Wire Line
	4650 6350 4900 6350
Wire Wire Line
	4650 6250 4900 6250
Text Label 3900 6150 0    50   ~ 0
3V3
Text Label 4900 6250 0    50   ~ 0
SC2
Text Label 4900 6350 0    50   ~ 0
INT2
Text Label 3900 6250 0    50   ~ 0
SD2
Wire Wire Line
	3900 6350 3900 6450
Wire Wire Line
	5200 6150 5200 6250
Wire Wire Line
	4650 6150 5200 6150
$Comp
L power:GND #PWR0113
U 1 1 5E05FA7F
P 3900 6450
F 0 "#PWR0113" H 3900 6200 50  0001 C CNN
F 1 "GND" H 3905 6277 50  0000 C CNN
F 2 "" H 3900 6450 50  0001 C CNN
F 3 "" H 3900 6450 50  0001 C CNN
	1    3900 6450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0114
U 1 1 5E05FA85
P 5200 6250
F 0 "#PWR0114" H 5200 6000 50  0001 C CNN
F 1 "GND" H 5205 6077 50  0000 C CNN
F 2 "" H 5200 6250 50  0001 C CNN
F 3 "" H 5200 6250 50  0001 C CNN
	1    5200 6250
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x03_Odd_Even LASER3
U 1 1 5E06A5ED
P 4350 7000
F 0 "LASER3" H 4400 7317 50  0000 C CNN
F 1 "Conn_02x03_Odd_Even" H 4400 7226 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Horizontal" H 4350 7000 50  0001 C CNN
F 3 "~" H 4350 7000 50  0001 C CNN
	1    4350 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 6900 3900 6900
Wire Wire Line
	4150 7000 3900 7000
Wire Wire Line
	4150 7100 3900 7100
Wire Wire Line
	4650 7100 4900 7100
Wire Wire Line
	4650 7000 4900 7000
Text Label 3900 6900 0    50   ~ 0
3V3
Text Label 4900 7000 0    50   ~ 0
SC3
Text Label 4900 7100 0    50   ~ 0
INT3
Text Label 3900 7000 0    50   ~ 0
SD3
Wire Wire Line
	3900 7100 3900 7200
Wire Wire Line
	5200 6900 5200 7000
Wire Wire Line
	4650 6900 5200 6900
$Comp
L power:GND #PWR0115
U 1 1 5E06A5FF
P 3900 7200
F 0 "#PWR0115" H 3900 6950 50  0001 C CNN
F 1 "GND" H 3905 7027 50  0000 C CNN
F 2 "" H 3900 7200 50  0001 C CNN
F 3 "" H 3900 7200 50  0001 C CNN
	1    3900 7200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 5E06A605
P 5200 7000
F 0 "#PWR0116" H 5200 6750 50  0001 C CNN
F 1 "GND" H 5205 6827 50  0000 C CNN
F 2 "" H 5200 7000 50  0001 C CNN
F 3 "" H 5200 7000 50  0001 C CNN
	1    5200 7000
	1    0    0    -1  
$EndComp
Text Notes 3800 5050 0    100  ~ 0
Laser Rangefinders
$Comp
L Interface_Expansion:TCA9544A MPLEX1
U 1 1 5E082F55
P 1900 6050
F 0 "MPLEX1" H 1950 7131 50  0000 C CNN
F 1 "TCA9544A" H 1950 7040 50  0000 C CNN
F 2 "Package_SO:TSSOP-20_4.4x6.5mm_P0.65mm" H 2900 5150 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tca9544a.pdf" H 1950 6300 50  0001 C CNN
	1    1900 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 5350 1250 5350
Wire Wire Line
	1500 5450 1250 5450
Text Label 1250 5450 0    50   ~ 0
SDA
Text Label 1250 5350 0    50   ~ 0
SCL
Text Label 2450 6650 0    50   ~ 0
SD3
Text Label 2450 6550 0    50   ~ 0
SC3
Text Label 2450 6250 0    50   ~ 0
SD2
Text Label 2450 6150 0    50   ~ 0
SC2
Text Label 2450 5850 0    50   ~ 0
SD1
Text Label 2450 5750 0    50   ~ 0
SC1
Wire Wire Line
	1400 6550 1400 6650
Connection ~ 1400 6650
Wire Wire Line
	1400 6650 1400 6750
Wire Wire Line
	1400 6750 1400 7150
Wire Wire Line
	1900 7150 1900 7050
Connection ~ 1400 6750
$Comp
L power:GND #PWR0117
U 1 1 5E153B41
P 1650 7350
F 0 "#PWR0117" H 1650 7100 50  0001 C CNN
F 1 "GND" H 1655 7177 50  0000 C CNN
F 2 "" H 1650 7350 50  0001 C CNN
F 3 "" H 1650 7350 50  0001 C CNN
	1    1650 7350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 7150 1650 7350
Wire Wire Line
	1650 7150 1900 7150
Wire Wire Line
	1400 6550 1500 6550
Wire Wire Line
	1400 6650 1500 6650
Wire Wire Line
	1400 6750 1500 6750
Wire Wire Line
	1400 7150 1650 7150
Connection ~ 1650 7150
Text Label 2650 5150 0    50   ~ 0
3V3
$Comp
L Device:R R10
U 1 1 5E1F7197
P 2950 5750
F 0 "R10" V 2900 5900 50  0000 C CNN
F 1 "4k7" V 2950 5750 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2880 5750 50  0001 C CNN
F 3 "~" H 2950 5750 50  0001 C CNN
	1    2950 5750
	0    1    1    0   
$EndComp
$Comp
L Device:R R11
U 1 1 5E1F8558
P 2950 5850
F 0 "R11" V 2900 6000 50  0000 C CNN
F 1 "4k7" V 2950 5850 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2880 5850 50  0001 C CNN
F 3 "~" H 2950 5850 50  0001 C CNN
	1    2950 5850
	0    1    1    0   
$EndComp
$Comp
L Device:R R12
U 1 1 5E1F87A0
P 2950 6150
F 0 "R12" V 2900 6300 50  0000 C CNN
F 1 "4k7" V 2950 6150 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2880 6150 50  0001 C CNN
F 3 "~" H 2950 6150 50  0001 C CNN
	1    2950 6150
	0    1    1    0   
$EndComp
$Comp
L Device:R R13
U 1 1 5E1F8A1C
P 2950 6250
F 0 "R13" V 2900 6400 50  0000 C CNN
F 1 "4k7" V 2950 6250 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2880 6250 50  0001 C CNN
F 3 "~" H 2950 6250 50  0001 C CNN
	1    2950 6250
	0    1    1    0   
$EndComp
$Comp
L Device:R R14
U 1 1 5E1F8BA0
P 2950 6550
F 0 "R14" V 2900 6700 50  0000 C CNN
F 1 "4k7" V 2950 6550 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2880 6550 50  0001 C CNN
F 3 "~" H 2950 6550 50  0001 C CNN
	1    2950 6550
	0    1    1    0   
$EndComp
$Comp
L Device:R R15
U 1 1 5E1F900A
P 2950 6650
F 0 "R15" V 2900 6800 50  0000 C CNN
F 1 "4k7" V 2950 6650 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2880 6650 50  0001 C CNN
F 3 "~" H 2950 6650 50  0001 C CNN
	1    2950 6650
	0    1    1    0   
$EndComp
Wire Wire Line
	2400 5750 2800 5750
Wire Wire Line
	2400 5850 2800 5850
Wire Wire Line
	2400 6150 2800 6150
Wire Wire Line
	2400 6250 2800 6250
Wire Wire Line
	2400 6550 2800 6550
Wire Wire Line
	2400 6650 2800 6650
Wire Wire Line
	3100 6650 3200 6650
Wire Wire Line
	3200 6650 3200 6550
Wire Wire Line
	3200 6550 3100 6550
Wire Wire Line
	3200 6550 3200 6350
Wire Wire Line
	3200 6250 3100 6250
Connection ~ 3200 6550
Wire Wire Line
	3200 6250 3200 6150
Wire Wire Line
	3200 6150 3100 6150
Connection ~ 3200 6250
Wire Wire Line
	3200 6150 3200 5950
Wire Wire Line
	3200 5850 3100 5850
Connection ~ 3200 6150
Wire Wire Line
	3200 5850 3200 5750
Wire Wire Line
	3200 5750 3100 5750
Connection ~ 3200 5850
Wire Wire Line
	1900 5150 3200 5150
Wire Wire Line
	3200 5150 3200 5550
Connection ~ 3200 5750
Wire Wire Line
	2400 5550 3200 5550
Connection ~ 3200 5550
Wire Wire Line
	3200 5550 3200 5750
Wire Wire Line
	2400 5950 3200 5950
Connection ~ 3200 5950
Wire Wire Line
	3200 5950 3200 5850
Wire Wire Line
	2400 6350 3200 6350
Connection ~ 3200 6350
Wire Wire Line
	3200 6350 3200 6250
Wire Wire Line
	3200 6750 3200 6650
Wire Wire Line
	2400 6750 3200 6750
Connection ~ 3200 6650
Text Notes 1650 4850 0    100  ~ 0
I2C Multiplexer
$Comp
L Connector_Generic:Conn_01x01 MTR_1A1
U 1 1 5E301F50
P 5650 950
F 0 "MTR_1A1" H 5730 992 50  0000 L CNN
F 1 "Conn_01x01" H 5730 901 50  0000 L CNN
F 2 "Connector_PinHeader_1.00mm:PinHeader_1x01_P1.00mm_Vertical" H 5650 950 50  0001 C CNN
F 3 "~" H 5650 950 50  0001 C CNN
	1    5650 950 
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 MTR_1B1
U 1 1 5E3033D7
P 5650 1150
F 0 "MTR_1B1" H 5730 1192 50  0000 L CNN
F 1 "Conn_01x01" H 5730 1101 50  0000 L CNN
F 2 "Connector_PinHeader_1.00mm:PinHeader_1x01_P1.00mm_Vertical" H 5650 1150 50  0001 C CNN
F 3 "~" H 5650 1150 50  0001 C CNN
	1    5650 1150
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 MTR_2A1
U 1 1 5E303671
P 5650 1350
F 0 "MTR_2A1" H 5730 1392 50  0000 L CNN
F 1 "Conn_01x01" H 5730 1301 50  0000 L CNN
F 2 "Connector_PinHeader_1.00mm:PinHeader_1x01_P1.00mm_Vertical" H 5650 1350 50  0001 C CNN
F 3 "~" H 5650 1350 50  0001 C CNN
	1    5650 1350
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 MTR_2B1
U 1 1 5E30380F
P 5650 1550
F 0 "MTR_2B1" H 5730 1592 50  0000 L CNN
F 1 "Conn_01x01" H 5730 1501 50  0000 L CNN
F 2 "Connector_PinHeader_1.00mm:PinHeader_1x01_P1.00mm_Vertical" H 5650 1550 50  0001 C CNN
F 3 "~" H 5650 1550 50  0001 C CNN
	1    5650 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 950  5150 950 
Text Label 5150 950  0    50   ~ 0
MTR_1A
Wire Wire Line
	5450 1150 5150 1150
Wire Wire Line
	5450 1350 5150 1350
Wire Wire Line
	5450 1550 5150 1550
Text Label 5150 1150 0    50   ~ 0
MTR_1B
Text Label 5150 1350 0    50   ~ 0
MTR_2A
Text Label 5150 1550 0    50   ~ 0
MTR_2B
Text Notes 5300 800  0    100  ~ 0
Motors
$Comp
L Device:LED D1
U 1 1 5DF15D31
P 6200 3750
F 0 "D1" H 6193 3966 50  0000 C CNN
F 1 "LED" H 6193 3875 50  0000 C CNN
F 2 "Diode_SMD:D_0805_2012Metric" H 6200 3750 50  0001 C CNN
F 3 "~" H 6200 3750 50  0001 C CNN
	1    6200 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D2
U 1 1 5DF16766
P 6200 4100
F 0 "D2" H 6193 4316 50  0000 C CNN
F 1 "LED" H 6193 4225 50  0000 C CNN
F 2 "Diode_SMD:D_0805_2012Metric" H 6200 4100 50  0001 C CNN
F 3 "~" H 6200 4100 50  0001 C CNN
	1    6200 4100
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D3
U 1 1 5DF16A06
P 6200 4450
F 0 "D3" H 6193 4666 50  0000 C CNN
F 1 "LED" H 6193 4575 50  0000 C CNN
F 2 "Diode_SMD:D_0805_2012Metric" H 6200 4450 50  0001 C CNN
F 3 "~" H 6200 4450 50  0001 C CNN
	1    6200 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 4450 6700 4450
Wire Wire Line
	6350 4100 6700 4100
Wire Wire Line
	6350 3750 6700 3750
$Comp
L Device:R R16
U 1 1 5DF4E8F5
P 5750 3750
F 0 "R16" V 5700 3750 50  0000 C CNN
F 1 "470" V 5750 3750 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5680 3750 50  0001 C CNN
F 3 "~" H 5750 3750 50  0001 C CNN
	1    5750 3750
	0    1    1    0   
$EndComp
$Comp
L Device:R R17
U 1 1 5DF4F45B
P 5750 4100
F 0 "R17" V 5700 4100 50  0000 C CNN
F 1 "470" V 5750 4100 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5680 4100 50  0001 C CNN
F 3 "~" H 5750 4100 50  0001 C CNN
	1    5750 4100
	0    1    1    0   
$EndComp
$Comp
L Device:R R18
U 1 1 5DF4F7D3
P 5750 4450
F 0 "R18" V 5700 4450 50  0000 C CNN
F 1 "470" V 5750 4450 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5680 4450 50  0001 C CNN
F 3 "~" H 5750 4450 50  0001 C CNN
	1    5750 4450
	0    1    1    0   
$EndComp
Wire Wire Line
	5900 3750 6050 3750
Wire Wire Line
	5900 4100 6050 4100
Wire Wire Line
	6050 4450 5900 4450
Wire Wire Line
	5600 4450 5500 4450
Wire Wire Line
	5500 4450 5500 4100
Wire Wire Line
	5500 4100 5600 4100
Wire Wire Line
	5600 3750 5500 3750
Wire Wire Line
	5500 3750 5500 4100
Connection ~ 5500 4100
Wire Wire Line
	5500 4450 5500 4550
Connection ~ 5500 4450
$Comp
L power:GND #PWR0118
U 1 1 5DFD8530
P 5500 4550
F 0 "#PWR0118" H 5500 4300 50  0001 C CNN
F 1 "GND" H 5505 4377 50  0000 C CNN
F 2 "" H 5500 4550 50  0001 C CNN
F 3 "" H 5500 4550 50  0001 C CNN
	1    5500 4550
	1    0    0    -1  
$EndComp
Text Label 6700 3750 0    50   ~ 0
IO16
Text Label 6700 4100 0    50   ~ 0
IO19
Text Label 6700 4450 0    50   ~ 0
IO33
$EndSCHEMATC
