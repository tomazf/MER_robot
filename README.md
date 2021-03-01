# MER robot

![alt text](https://ict.aiju.info/mer/pluginfile.php/1/theme_academic/logo/1614160915/mer_logo_def.png)

The project "Modelling educational robot" is meeting the necessity of the 21st Digital Century Society to implement as much as possible in the regular and open curriculum the subject programming and robotics. Robotics helps address the growing demand for teaching science, technology, engineering and maths in schools. 

--------------------------------------------

 Simple init sketch for robot (MER project - https://ict.aiju.info/mer)

 version: v7 - 28.2.2021

 Sensors/defined HW:
  - 4x VL53L0x (range senzor) with non-blocking library
  - 1x TCS34725 (RGB) with interrupt
  - 1x GY-271 (HMC5883L module) or MPU6050 (gyro)

 Added onboard OLED - oct. 2020
 Constructed final chassis - dec. 2020

 Added listed HW - jan. 2021
  - 1x HM-10 (BT module)
  - 1x nRF24L01 module  (TODO)

 Added listed HW - feb. 2021
  - 1x ESP-01 module    (SW only simple web-to-serial for now with TCP socket) - modified RoboRemo code
                         WEB: https://github.com/roboremo/ESP8266-WiFi-UART-Bridge
                         APP: Android App to connect - RoboRemo Free
  - 1x GPS module (only Rx used) - parsing GPGGA messages

 Added listed HW - mar. 2021
  - 1x Pololu gripper (Micro Gripper Kit with Position Feedback Servo)
  - 1x onboard ESP32 camera module
  - 1x gipper and camera mount 3D printed parts
  - 1x LDR sensor
  - 1x DS1820s temp sensor

 Added listed SW - mar. 2021
  - neotimer lib
  - freemem lib
  - state machine for gripper
  - a few robot functions
  - sender object handler

 libs:
 - compass https://github.com/helscream/HMC5883L_Header_Arduino_Auto_calibration/tree/master/Core/Compass_header_example_ver_0_2
 - softI2C https://github.com/Fire7/Adafruit_TCS34725_SoftI2C for TCS34725
 - TCS_int https://github.com/adafruit/Adafruit_TCS34725/blob/master/examples/interrupt/interrupt.ino
 - command https://github.com/ppedro74/Arduino-SerialCommands
 - VL53L1x local lib, edited based on - https://github.com/badVibes--/vl53l0x-arduino (added INT code)
 - StateMAchine https://github.com/jrullan/StateMachine
 - LinkedList https://github.com/ivanseidel/LinkedList - delete test.cpp file!
 - GPS https://github.com/stevemarple/MicroNMEA
 - freeMEM https://github.com/maniacbug/MemoryFree
 - neotimer https://github.com/jrullan/neotimer

 Description:
  - added LED class
  - added MOTOR class
  - added VL53L0X class
  - added few simple robot functions
  - added robot turning functions (absolute, relative with direction)
  - added state-machine logic for gripper
  - added neotimer class - for easy delay management
  - added serial command parser (F, L, G)
  - added ACK reply status for serial
