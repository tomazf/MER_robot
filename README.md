# MER robot

![alt text](https://ict.aiju.info/mer/pluginfile.php/1/theme_academic/logo/1614160915/mer_logo_def.png)

The project "Modelling educational robot" is meeting the necessity of the 21st Digital Century Society to implement as much as possible in the regular and open curriculum the subject programming and robotics. Robotics helps address the growing demand for teaching science, technology, engineering and maths in schools. 

--------------------------------------------

 Simple init sketch for robot (MER project - https://ict.aiju.info/mer)

 version: v8 - 8.3.2021

 Sensors/defined HW:
  - 4x VL53L0x (range senzor) with non-blocking library
  - 1x TCS34725 (RGB) with interrupt
  - 1x GY-271 (HMC5883L module) or MPU6050 (gyro)

 Added onboard OLED - oct. 2020
 Constructed final chassis - dec. 2020

 Added listed HW - jan. 2021
  - 1x HM-10 (BT module)
  - 1x nRF24L01 module  (TODO or optional)

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
  - added IR/VL53x/RGB/compass data for GET command

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
  - added EEPROM storage function (for calibration data)
  
# COMMAND reference
	
This section briefly describes serial command parser for MER robot. Commands for controlling or getting status from robot are processed on Arduino UART with 115200 baud. Serial ports for communication are defined in SW (can be multiple). 

All commands must be sent with CR and LF line endings (this can be modified in source code).
All commands receive ACK messages. If unknown command is sent/received, sender is notified.
Commands can have multiple parameters, as described.

Command are divided in 3 groups:
 - L (LED commands) for manipulating onboard LEDs
 - F (FUNCTION mode commands) for setting different modes and variables of a robot
 - G (GET data commands) for getting sensor data and updates if defined

Command description (L):
 - L 1 1     // LED 1 on
 - L 1 0     // LED 1 off
 - L 0 2     // LED 0 blink mode
 - L 0 2 100 // LED 0 set blink 100ms only!
 - L 20      // all off
 - L 21      // all on
  
Command description (F):
 - F 1     // set command = 1  -> see execute_command() in **functions.ino**
 - F 2     // set command = 2  -> see execute_command() in **functions.ino**
 - F .
 - F 11    // stop gripper
 - F 12    // run gripper
 - F 13    // release gripper
 - F 20 80   // set motor speed to 80
 - F 21 50   // set motor speed_turn increment to 50
 - F 22 10   // set motor speed_offset_l to 10 and right to 0
 - F 22 -10  // set motor speed_offset_r to 10 and left to 0
 - F 23 55   // set arm angle 55°
 - F 24 60   // robot relative angle turn RIGHT -> command 14
  
Command description (G):
 - G         // complete array is sent (no parameter)
 - G 0       // only first value from array is sent
 - G 1       // only second value from array is sent
 - G 3       // only fourth value from array is sent
 - G 54      // send GRIPPER state -> 0 = stop, 1 = gripping, 2 = hold_state
 - G 55      // only GPS data is sent to caller (if available)
 - G 56      // only IR data is sent to caller
 - G 57      // only VLx data is sent to caller
 - G 58      // only RGB data is sent to caller
 - G 59      // only COMPASS data is sent to caller


## TODO - add GET command for periodically send sensor data to client
