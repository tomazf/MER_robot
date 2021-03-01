//
// Simple init sketch for robot (MER project)
//
// version: v5 - 13.2.2021
//
// Sensors/defined HW:
//  - 4x VL53L0x (range senzor) with non-blocking library
//  - 1x TCS34725 (RGB) with interrupt
//  - 1x GY-271 or MPU6050 (gyro) - TODO fix
//
// Added onboard OLED - oct. 2020
//
// Added listed HW - jan. 2021
//  - 1x HM-10 (BT module)
//  - 1x nRF24L01 module
//
//
// libs:
//      compass: https://github.com/adafruit/Adafruit_HMC5883_Unified
//      softI2C: https://github.com/Fire7/Adafruit_TCS34725_SoftI2C for TCS34725
//      TCS_int: https://github.com/adafruit/Adafruit_TCS34725/blob/master/examples/interrupt/interrupt.ino
//      command: https://github.com/ppedro74/Arduino-SerialCommands
//      VL53L1x: local lib, edited based on - https://github.com/badVibes--/vl53l0x-arduino (added INT code)
//
// added LED class
// added MOTOR class
// added VL53L0X class
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
#include <Adafruit_HMC5883_U.h>
#include <Arduino.h>
#include <SerialCommands.h>

unsigned long time_1 = 0;
unsigned long time_2 = 0;
unsigned long time_3 = 0;
unsigned long time_4 = 0;
unsigned long time_5 = 0;

char serial_command_buffer_[32];

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
motorMER motor = motorMER(RB_MOTOR_1_1, RB_MOTOR_1_2, RB_MOTOR_2_1,RB_MOTOR_2_2);
//Adafruit_TCS34725 rgb_senzor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);                                 // HW I2C
Adafruit_TCS34725softi2c rgb_sensor = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X, SDApin, SCLpin);     // SW I2C
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(COMPASS_ID);
// serial parser
SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\r\n", " ");
SerialCommands serial_commands2_(&Serial2, serial_command_buffer_, sizeof(serial_command_buffer_), "\r\n", " ");


// array of objects for onboard LEDs
//
ledBlink LED[] = {ledBlink(LED_pin1), ledBlink(LED_pin2), ledBlink(LED_pin3), ledBlink(LED_pin4), ledBlink(rgbLED_pin)};

// ---------------------------------------------------------------------------
// COMMAND parser
//
void cmd_unrecognized(SerialCommands* sender, const char* cmd);
void set_mode(SerialCommands* sender);
void set_led(SerialCommands* sender);

// Note: Commands are case sensitive
SerialCommand set_mode_("F", set_mode);
SerialCommand set_led_("L", set_led);

// ---------------------------------------------------------------------------
// VARIABLES
//
// for display:      r  g  b  c s1 s2 s3 s4  x  y  z  h  B
int OLED_data[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

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
//void RB_MotorInit();
void execute_command();

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
    if (!mag.begin())
    {
      Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
      //while (1);
    }
  }
  else Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
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
  serial_commands_.SetDefaultHandler(cmd_unrecognized);
  serial_commands_.AddCommand(&set_mode_);
  serial_commands_.AddCommand(&set_led_);
  serial_commands2_.SetDefaultHandler(cmd_unrecognized);
  serial_commands2_.AddCommand(&set_mode_);
  serial_commands2_.AddCommand(&set_led_);

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

    // you can do better!
    if (i2c_device[1][5] == 1) read_Compass();
    read_Batt();
    read_RGB();
    read_VL53x();

    // it has delay, only once every second (one write takes 60ms)
    // so loop can work faster - measurments still get their data!!
    write_OLED();

    if (millis() >= time_1 + INTERVAL_MAIN_LOOP)
    {
      time_1 += INTERVAL_MAIN_LOOP;

      // updale LED on PCB
      update_LED();

      // commands execute
      execute_command();
    }

  }
}
