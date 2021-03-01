#ifndef CONFIG_H
#define CONFIG_H

// ---------------------------------------------------------------------------
// DEFINES
//
#define SDApin A5             // SW I2C - for RGB senzor
#define SCLpin A4             // SW I2C - for RGB senzor
#define rgbLED_pin A7         // RGB white LED
#define readBATT_pin A15      // battery measure

// MOTOR pins
//
#define RB_MOTOR_1_1 13       // MOTOR 1+
#define RB_MOTOR_1_2 12       // MOTOR 1-
#define RB_MOTOR_2_1 11       // MOTOR 2+
#define RB_MOTOR_2_2 10       // MOTOR 2-

// LED pins
//
#define LED_pin1 39           // onboard LED 1
#define LED_pin2 37           // onboard LED 2
#define LED_pin3 35           // onboard LED 3
#define LED_pin4 33           // onboard LED 4

// vl53x addresses
//
#define VL53s1_ADDRESS 0x30   // VL53X sensor #1 address
#define VL53s2_ADDRESS 0x31   // VL53X sensor #2 address
#define VL53s3_ADDRESS 0x32   // VL53X sensor #3 address
#define VL53s4_ADDRESS 0x33   // VL53X sensor #4 address

// vl53x XSHUT pins
//
#define sensor1_pin A0        // XSHUT VL53 s1
#define sensor2_pin A1        // XSHUT VL53 s2
#define sensor3_pin 29        // XSHUT VL53 s3
#define sensor4_pin 27        // XSHUT VL53 s4

// vl53x interrupt pins
//
#define VL53s1_INTpin 18      // VL53X sensor #1 interrupt pin  -> set to -1 if not used - read internal register when done - little bit slower though
#define VL53s2_INTpin 19      // VL53X sensor #2 interrupt pin  -> set to -1 if not used
#define VL53s3_INTpin A14     // VL53X sensor #3 interrupt pin  -> set to -1 if not used
#define VL53s4_INTpin A13     // VL53X sensor #4 interrupt pin  -> set to -1 if not used

// IR INPUTS pins
//
#define IR_SENSOR_RIGHT2  47
#define IR_SENSOR_RIGHT1  45
#define IR_SENSOR_CENTER  43
#define IR_SENSOR_LEFT1 41
#define IR_SENSOR_LEFT2 46

// set LOOP intervals
//
#define INTERVAL_MAIN_LOOP 50
#define INTERVAL_COMPASS 100
#define INTERVAL_BATT 30000
#define INTERVAL_RGB 100    // don't go below 55, because integration time is set to 50ms! - read-ahead mode
#define INTERVAL_OLED 1000

#define LED_MIN_BLINK 30      // min off/on delay
#define GROUND_DISTANCE 60    // min ground distance in mm

#define I2C_scanner false     // I2C scanner
#define SERIAL_DEBUG false    // SERIAL debug mode
#define SCREEN_WIDTH 128      // OLED display width, in pixels
#define SCREEN_HEIGHT 64      // OLED display height, in pixels
#define OLED_RESET -1         // Reset pin # (or -1 if sharing Arduino reset pin - I2C only 2 wires!)
#define SHOW_LOGO true        // boot logo show
#define FREEMEM_LIB true      // use freeMemory lib? - just for indication

#define OLED_I2C              // define only one OLED type
//#define OLED_SPI
#ifdef OLED_SPI
#define OLED_DC     48        // set DC pin
#define OLED_CS     49        // set CS pin
#endif

#define DECLINATION 4.3       // Novo mesto 4° 20' Find yours here: http://www.magnetic-declination.com/
#define COMPASS_ID 12345      // compass ID
#define REF_VOLTAGE 4.974     // voltage reference in V

// todo variables
//
int command = 0;                // hold case command DATA
bool vl53_present = true;

#endif
