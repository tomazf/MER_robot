//
// Simple init sketch for robot (MER project - https://ict.aiju.info/mer)
//
// version: v7 - 28.2.2021
//
// Sensors/defined HW:
//  - 4x VL53L0x (range senzor) with non-blocking library
//  - 1x TCS34725 (RGB) with interrupt
//  - 1x GY-271 (HMC5883L module) or MPU6050 (gyro)
//
// Added onboard OLED - oct. 2020
// Constructed final chassis - dec. 2020
//
// Added listed HW - jan. 2021
//  - 1x HM-10 (BT module)
//  - 1x nRF24L01 module  (TODO)
//
// Added listed HW - feb. 2021
//  - 1x ESP-01 module    (SW only simple web-to-serial for now with TCP socket) - modified RoboRemo code
//                         WEB: https://github.com/roboremo/ESP8266-WiFi-UART-Bridge         
//                         APP: Android App to connect - RoboRemo Free
//  - 1x GPS module (only Rx used) - parsing GPGGA messages
//
// Added listed HW - mar. 2021
//  - 1x Pololu gripper (Micro Gripper Kit with Position Feedback Servo)
//  - 1x onboard ESP32 camera module
//  - 1x gipper and camera mount 3D printed parts
//  - 1x LDR sensor
//
// Added listed SW - mar. 2021
//  - neotimer lib
//  - freemem lib
//  - state machine for gripper
//  - a few robot functions
//  - sender object handler
//
// libs:
//      compass: https://github.com/helscream/HMC5883L_Header_Arduino_Auto_calibration/tree/master/Core/Compass_header_example_ver_0_2
//      softI2C: https://github.com/Fire7/Adafruit_TCS34725_SoftI2C for TCS34725
//      TCS_int: https://github.com/adafruit/Adafruit_TCS34725/blob/master/examples/interrupt/interrupt.ino
//      command: https://github.com/ppedro74/Arduino-SerialCommands
//      VL53L1x: local lib, edited based on - https://github.com/badVibes--/vl53l0x-arduino (added INT code)
// StateMAchine: https://github.com/jrullan/StateMachine
//   LinkedList: https://github.com/ivanseidel/LinkedList - delete test.cpp file!
//          GPS: https://github.com/stevemarple/MicroNMEA
//      freeMEM: https://github.com/maniacbug/MemoryFree
//     neotimer: https://github.com/jrullan/neotimer
//
// Description:
//  - added LED class
//  - added MOTOR class
//  - added VL53L0X class
//  - added few simple robot functions
//  - added robot turning functions (absolute, relative with direction)
//  - added state-machine logic for gripper
//  - added neotimer class - for easy delay management
//  - added serial command parser (F, L, G)
//  - added ACK reply status for serial
//

// ---------------------------------------------------------------------------
// INLCUDES
//
#include "./config.h"
#include "./src/ledBlink.h"
#include "./src/motor.h"
#include "./src/VL53L0X.h"                // Install Pololu modified lib by Ferbi - read comments above

#include <Wire.h>
#include "./src/image.h"
#include <Streaming.h>
#include "Adafruit_TCS34725softi2c.h"     // https://github.com/Fire7/Adafruit_TCS34725_SoftI2C
//#include "Adafruit_TCS34725.h"          // HW I2C - not used due to same address of VL53X senzor at startup (RGB senzor addr: 0x29 HEX (fixed) + LED pin)
#include <Adafruit_GFX.h>                 // Install Adafruit SSD1306 v2.4.x lib or higher in Arduino IDE
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <SerialCommands.h>
#include <MemoryFree.h>
#include <neotimer.h>

#if defined(USE_GRIPPER)
#include <StateMachine.h>
#include <Servo.h>
#endif

#if defined(USE_GPS)
#include <MicroNMEA.h>
#endif

#if defined(USE_COMPASS)
#include <compass.h>
#endif

// do better - use timer class
unsigned long timeG_1 = 0;          // for state 1
unsigned long timeG_2 = 0;          // for state 3

char serial_command_buffer_[32];    // for serial parser

// ---------------------------------------------------------------------------
// OBJECTS
//
#ifdef OLED_I2C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif
#ifdef OLED_SPI
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS);
#endif

VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;
VL53L0X sensor4;
motorMER motor = motorMER(RB_MOTOR_1_1, RB_MOTOR_1_2, RB_MOTOR_2_1, RB_MOTOR_2_2);                    // create motor object
//Adafruit_TCS34725 rgb_senzor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);                                 // HW I2C (not used)
Adafruit_TCS34725softi2c rgb_sensor = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X, SDApin, SCLpin);     // SW I2C
// serial parser
SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\r\n", " ");
SerialCommands serial_commands2_(&Serial2, serial_command_buffer_, sizeof(serial_command_buffer_), "\r\n", " ");

#if defined(USE_GRIPPER)
Servo gripper;
Servo arm;
// create state-machine object for gripper action
StateMachine machine_gripper = StateMachine();

// ---------------------------------------------------------------------------
// STATES for gripper
//

State* S0 = machine_gripper.addState(&state0);    // reset servo to 0
State* S1 = machine_gripper.addState(&state1);    // delay_1
State* S2 = machine_gripper.addState(&state2);    // increment value
State* S3 = machine_gripper.addState(&state3);    // delay_2
State* S4 = machine_gripper.addState(&state4);    // read feedback and compare
State* S5 = machine_gripper.addState(&state5);    // hold nad wait for release command - grip_release
#endif

#if defined(USE_GPS)
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
#endif

Neotimer timer_COMPASS = Neotimer(INTERVAL_COMPASS);    // Set timer's preset nad start
Neotimer timer_BATT = Neotimer(INTERVAL_BATT);          // Set timer's preset nad start
Neotimer timer_RGB = Neotimer(INTERVAL_RGB);            // Set timer's preset nad start
Neotimer timer_OLED = Neotimer(INTERVAL_OLED);          // Set timer's preset nad start
Neotimer timer_GPS = Neotimer(INTERVAL_GPS);            // Set timer's preset nad start
Neotimer timer_ARM = Neotimer(INTERVAL_ARM);            // Set timer's preset nad start
Neotimer timer_GRIP = Neotimer(INTERVAL_GRIP);          // Set timer's preset nad start
Neotimer timer_CUSTOM;                                  // Set timer's preset nad start - custom delay

// array of objects for onboard LEDs
//
ledBlink LED[] = {ledBlink(LED_pin1), ledBlink(LED_pin2), ledBlink(LED_pin3), ledBlink(LED_pin4), ledBlink(rgbLED_pin)};

// ---------------------------------------------------------------------------
// COMMAND parser
//
void cmd_unrecognized(SerialCommands* sender, const char* cmd);
void set_mode(SerialCommands* sender);
void set_led(SerialCommands* sender);
void get_data(SerialCommands* sender);

// note: Commands are case sensitive
SerialCommand set_mode_("F", set_mode);       // F - function/mode commands - description in command.ino file
SerialCommand set_led_("L", set_led);         // L - led commands
SerialCommand get_data_("G", get_data);       // G - get commands

// ---------------------------------------------------------------------------
// VARIABLES
//
// for display:      r  g  b  c s1 s2 s3 s4  x  y  z  B  V  L
//                   0  1  2  3  4  5  6  7  8  9 10 11 12 13
int OLED_data[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* OLED_data values:
 *  
 *  0 - red
 *  1 - green
 *  2 - blue
 *  3 - clear
 *  4 - VL53x_s1
 *  5 - VL53x_s2
 *  6 - VL53x_s3
 *  7 - VL53x_s4
 *  8 - compass X
 *  9 - compass Y
 *  10 - compass Z
 *  11 - bearing
 *  12 - battery voltage
 *  13 - light detector
 *  
 */


// for i2c devices and status
byte i2c_device[2][7] = { {0x29, 0x30, 0x31, 0x32, 0x33, 0x1e, 0x3c},
  {0, 0, 0, 0, 0, 0, 0}
};

// size collect
int velikost = sizeof(i2c_device) / 2;


// ---------------------------------------------------------------------------
// I2C devices - must be in that order for init check!
//
//  0x29 - RGB senzor
//  0x30 - VL53 s1
//  0x31 - VL53 s2
//  0x32 - VL53 s3
//  0x33 - VL53 s4
//  0x1e - GY-271
//  0x3c - OLED


// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// FUNCTION DECLARATIONS
//
void execute_command();                               // declaration for compiler
void send_ack(SerialCommands*, bool err = false);     // declaration for compiler
SerialCommands* senderG;                              // global sender variable

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// SETUP
//
void setup() {

  // serial baud setup
  Serial.begin(115200);
  Serial2.begin(9600);                // BT module
  delay(10);                          // wait until serial port opens for native USB devices

  // start I2C
  Wire.begin();

  Serial.println("INIT...");

  // set ANALOG_ref
  //
  analogReference(DEFAULT);           // use AREF for reference voltage
  pinMode(readBATT_pin, INPUT);

  // LED init
  //
  if (init_LED()) {
    Serial.println("LED init OK!");
  }
  else {
    Serial.println("LED init FAILED!");
    for (int i = 0; i < 4; i++)
    {
      LED[i].blink();
      LED[i].update();
    }
  }

  // set VL53 sensor addresses
  //
  if (setID_VL53()) {
    Serial.println("VL53 init OK!");
  }
  else
  {
    Serial.println("VL53 init FAILED!");
    LEDerror();
    for (;;);
  }

  // for RGB sensor
  //
  pinMode(rgbLED_pin, OUTPUT);      // LED pin is seperated from module
  for (int i = 0; i < 10; i++) {
    digitalWrite(rgbLED_pin, HIGH);  // turn white led ON - just init debug
    delay(200 - i * 20);
    digitalWrite(rgbLED_pin, LOW);  // turn white led ON - just init debug
    delay(200 - i * 20);
  }

  if (!rgb_sensor.begin()) {
    Serial.println("No TCS34725 found ... check your connections!");
    LEDerror();
    for (;;);
  }
  else {
    Serial.println("RGB init OK!");;
    i2c_device[1][0] = 1;
  }

  // check i2c device presence
  //
  i2c_presence();
  Serial.println("I2C init OK!");

  // for OLED display
  //
  if (i2c_device[1][6] == 1) {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {               // Address 0x3C for 128x64
      Serial.println(F("No SSD1306 display found ... check your connections!"));
      LEDerror();
      for (;;);
    }
  }

  // logo
  //
  if (SHOW_LOGO)
  {
    display.clearDisplay();
    display.drawBitmap(0, 0,  MER_IMAGE, 128, 64, WHITE);
    display.display();
    delay(3000);
  }

  // for compass
  //
  if (i2c_device[1][5] == 1) {
    init_compass();
  }
  else
  {
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    LEDerror();
  }
  Serial.println("MAG init OK!");

  // motor init
  //
  motor.init();
  Serial.println("MOTOR init OK!");

  // write some text
  //
  init_OLED();
  Serial.println("OLED init OK!");

  // serial parser
  //
  init_serialParser();
  Serial.println("PARSER init OK!");

  // gripper init
  //
#if defined(USE_GRIPPER)
  gripper_init();
  arm_test();
  gripper_test();
  Serial.println("GRIPPER init OK!");
#endif

  Serial.println("done!");
}

// ---------------------------------------------------------------------------
// MAIN LOOP
//
void loop() {
  if (I2C_scanner) i2c_scanner();
  else
  {

    // commands parse
    serial_commands_.ReadSerial();

    // for debug time-loop only - not for production
    //Serial.println(millis());

    // we must do better here!
    if (i2c_device[1][5] == 1) read_Compass();
    read_Batt();
    read_RGB();
    read_VL53x();

#if defined(USE_GRIPPER)
    // do gripper and arm stuff
    gripper_run();
    arm_run();
#endif

#if defined(USE_GPS)
    // parse GPS data
    parse_gps();
#endif

    // it has delay, only once every second (one write takes 60ms)
    // so loop can work faster - measurments still get their data!!
    write_OLED();

    // updale LED on PCB
    update_LED();

    // commands execute
    execute_command();

  }
}
