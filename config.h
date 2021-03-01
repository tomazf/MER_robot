#ifndef CONFIG_H
#define CONFIG_H

// ---------------------------------------------------------------------------
// DEFINES
//
#define SDApin A5             // SW I2C - for RGB senzor
#define SCLpin A4             // SW I2C - for RGB senzor
#define rgbLED_pin A7         // RGB white LED
#define readBATT_pin A15      // battery measure
#define LDR_pin A9            // simple analog light detector

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
#define VL53s2_INTpin A2      // VL53X sensor #2 interrupt pin  -> set to -1 if not used
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
#define INTERVAL_MOVE 1000
#define INTERVAL_GPS 1000
#define INTERVAL_COMPASS 100
#define INTERVAL_BATT 30000
#define INTERVAL_RGB 100        // don't go below 55, because integration time is set to 50ms! - read-ahead mode (without interrupt)
#define INTERVAL_OLED 1000      // for OLED and GPS
#define INTERVAL_ARM 1500       // for OLED and GPS
#define INTERVAL_GRIP 2000      // for OLED and GPS
#define SERVO_HOME 2500         // delay for home-position
#define SERVO_GRIPP 180         // delay for increment (could be less)

// GRIPP MEASUREMENT and control
//
#define USE_GRIPPER           // is gripper mounted?
//#define USE_GPS             // is GPS mounted?
#define USE_COMPASS           // is compass mounted?
#if defined(USE_GPS)
#define GPS_PORT Serial1      // HW serial
#endif

#define LED_MIN_BLINK 30      // min off/on delay
#define GROUND_DISTANCE 60    // min ground distance in mm

#define I2C_scanner false     // I2C scanner
#define SERIAL_DEBUG false    // SERIAL debug mode
#define SCREEN_WIDTH 128      // OLED display width, in pixels
#define SCREEN_HEIGHT 64      // OLED display height, in pixels
#define OLED_RESET -1         // Reset pin # (or -1 if sharing Arduino reset pin - I2C only 2 wires!)
#define SHOW_LOGO true        // boot logo show
#define REF_VOLTAGE 4.974     // voltage reference in V (also for analog inputs)

#define OLED_I2C              // define only one OLED type
//#define OLED_SPI
#ifdef OLED_SPI
#define OLED_DC     48        // set DC pin
#define OLED_CS     49        // set CS pin
#endif


// todo variables
//
int command = 0;                // hold case command DATA
bool vl53_present = true;       // for init state

bool _turn_init = true;              // could be used as local-only variablesbool 
bool _edgeAvoidance_init = true;     // could be used as local-only variablesbool
bool _timer_custom_init = true;      // could be used as local-only variable
int _angle_rel;                 // for function sends
int _dir;                       // for function sends
int rotation_calibrate = 0;     // stop rotating motors at diff - calibration

// control variables and feedback pins for GRIPPER
//
#if defined(USE_GRIPPER)
#define gripperPin 8
#define armPin 9
#define feedbackPin A8      // gripper feedback pin 

#define ARM_MAX 150         // max arm angle
#define GRIPP_START 5       // initial value for gripper - 5 is for best results
#define GRIPP_INCREMENT 5   // increment of continous gripper
#define GRIPP_DIFF 3        // diff between pre an post measurement
#define GRIPP_BLK 60        // feedback lock - keep it above 50
#endif

// compass calibration data
//
//  calibration: https://github.com/helscream/HMC5883L_Header_Arduino_Auto_calibration/blob/master/Core/Compass_header_example_ver_0_2/Compass_header_example_ver_0_2.ino
//  note 1: use this code to get calibration values (upload sketch and monitor serial)
//  note 2: a compass in 2D plane (i.e x and y axis)only
//  note 3: compass points to the magnetic north and NOT the true north (which is OK, we need relative movement)
//
#if defined(USE_COMPASS)
#define Dcompass_x_offset 204.93
#define Dcompass_y_offset 366.89
#define Dcompass_z_offset 352.01
#define Dcompass_x_gainError 0.93
#define Dcompass_y_gainError 0.98
#define Dcompass_z_gainError 0.88
#endif

#endif
