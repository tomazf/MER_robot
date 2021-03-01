//
// robot v3 - init and HW routines
//
// functions.ino -> put this file in same folder
//
// TODO: add proper .h file includes and function prototypes
//


// detect all I2C devices
//
void i2c_presence() {
  for (int i = 1; i < velikost; i++)
  {
    Wire.beginTransmission(i2c_device[0][i]);
    if (Wire.endTransmission() == 0)
    {
      i2c_device[1][i] = 1;
      delay(1);
    }
  }
}

// Adafruit OLED init
//
void init_OLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(0, 5);
  display.println("Setup init...");
  display.setCursor(0, 20);
  display.println("    -- ROBO v3 --");

  display.setCursor(0, 40);
  display.print(" devices:");
    for (int i = 0; i < velikost; i++) if (i2c_device[1][i] == 1) display.print("+"); else display.print("-");

  display.display();
  delay(4000);

  display.clearDisplay();
  display.display();
}

// init COMPASS
//
void init_compass()
{
  compass_x_offset = Dcompass_x_offset;
  compass_y_offset = Dcompass_y_offset;
  compass_z_offset = Dcompass_z_offset;
  compass_x_gainError = Dcompass_x_gainError;
  compass_y_gainError = Dcompass_y_gainError;
  compass_z_gainError = Dcompass_z_gainError;
  compass_init(2);
}

// init LEDs
//
bool init_LED()
{
  delay(50);
  LEDshow();                      // just small demo
  for (int i = 0; i < sizeof(LED) / sizeof(ledBlink); i++)
  {
    LED[i].set_blink(100, 100);
    LED[i].off();
  }
  return true;
}

// serial parser INIT
//
void init_serialParser() {
  serial_commands_.SetDefaultHandler(cmd_unrecognized);
  serial_commands_.AddCommand(&set_mode_);
  serial_commands_.AddCommand(&set_led_);
  serial_commands_.AddCommand(&get_data_);
  serial_commands2_.SetDefaultHandler(cmd_unrecognized);
  serial_commands2_.AddCommand(&set_mode_);
  serial_commands2_.AddCommand(&set_led_);
  serial_commands2_.AddCommand(&get_data_);
}

// write some OLED debug data
//
void write_OLED() {

  if (timer_OLED.repeat())
  {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);

    display.setCursor(0, 0);
    display << "RGB R:" << OLED_data[0] << "\n";
    display << "    G:" << OLED_data[1];
    display.setCursor(60, 0);
    display << " B:" << OLED_data[2];
    display.setCursor(60, 8);
    display << " C:" << OLED_data[3];
    display.println();

    display.setCursor(104, 0);
    display << " MER";
    display.setCursor(104, 8);
    display << "SCNM";

    display << "VLx 1:" << OLED_data[4];
    display.setCursor(60, 16);
    display << " 3:" << OLED_data[6] << "\n";
    display << "    2:" << OLED_data[5];
    display.setCursor(60, 24);
    display << " 4:" << OLED_data[7];

#ifdef LDR_pin
    display.setCursor(0, 32);
    display << "L: " << OLED_data[13];
#endif
#ifdef DS_pin
    display.setCursor(0, 40);
    display << "T: " << _FLOAT(OLED_data[14] / 10.0, 1) << "C";    // one decimal only
#endif

#ifdef USE_GPS
    display.setCursor(60, 32);
    display << "H:" << GPS_data.h << ":" << GPS_data.m << "." << GPS_data.s;
    display.setCursor(60, 40);
    display << "F:" << GPS_data.fix;
#endif

    display.drawLine(0, 54, 127, 54, 1);
    display.setCursor(4, 56);
    display << "B:" << (float)OLED_data[12] / 1000 << "V ";
#ifdef MEMORY_FREE_H
    display << "M:" << freeMemory();
#endif
#ifdef USE_COMPASS
    display << " H:" << OLED_data[11];
#endif

    // update display (refresh)
    display.display();
  }
}

// read range sensor data
//
void read_VL53x()
{

  if (sensor1.available())
  {
    // save readout to table
    OLED_data[4] = sensor1.readRangeContinuousMillimeters();

    if (SERIAL_DEBUG) {
      Serial.println(OLED_data[4]);
      if (sensor1.timeoutOccurred()) {
        Serial.print(" TIMEOUT");
      }
    }
  }

  if (sensor2.available())
  {
    // save readout to table
    OLED_data[5] = sensor2.readRangeContinuousMillimeters();

    if (SERIAL_DEBUG) {
      Serial.println(OLED_data[5]);
      if (sensor2.timeoutOccurred()) {
        Serial.print(" TIMEOUT");
      }
    }
  }

  if (sensor3.available())
  {
    // save readout to table
    OLED_data[6] = sensor3.readRangeContinuousMillimeters();;

    if (SERIAL_DEBUG) {
      Serial.println(OLED_data[6]);
      if (sensor3.timeoutOccurred()) {
        Serial.print(" TIMEOUT");
      }
    }
  }

  if (sensor4.available())
  {
    // save readout to table
    OLED_data[7] = sensor4.readRangeContinuousMillimeters();

    if (SERIAL_DEBUG) {
      Serial.println(OLED_data[7]);
      if (sensor4.timeoutOccurred()) {
        Serial.print(" TIMEOUT");
      }
    }
  }

}


// read DS data - async
// read LDR analog light intensity
//
void read_DS_LDR()
{
#ifdef DS_pin
  if (_ds_init)
  {
    DStemp.setWaitForConversion(false);        // makes it async
    DStemp.requestTemperatures();
    DStemp.setWaitForConversion(true);
    _ds_init = false;
  }
#endif

  if (timer_DS.repeat())                      // waited long enoguh?
  {
#ifdef DS_pin
    OLED_data[14] = (int)(DStemp.getTempCByIndex(0) * 10);
#endif
#ifdef LDR_pin
    OLED_data[13] = analogRead(LDR_pin);
#endif
    _ds_init = true;
  }
}

// read RGB data
//
void read_RGB()
{

  if (timer_RGB.repeat())
  {

    uint16_t clear, red, green, blue;
    //rgb_sensor.setInterrupt(false);      // turn on LED
    //delay(150);  // takes 50ms to read
    //rgb_sensor.getRawData(&red, &green, &blue, &clear);

    clear = rgb_sensor.read16(TCS34725_CDATAL);
    red = rgb_sensor.read16(TCS34725_RDATAL);
    green = rgb_sensor.read16(TCS34725_GDATAL);
    blue = rgb_sensor.read16(TCS34725_BDATAL);

    //rgb_sensor.setInterrupt(true);      // turn off LED

    if (SERIAL_DEBUG)
    {
      Serial.print("C:\t"); Serial.print(clear);
      Serial.print("\tR:\t"); Serial.print(red);
      Serial.print("\tG:\t"); Serial.print(green);
      Serial.print("\tB:\t"); Serial.print(blue);
      Serial.println();
    }

    // save readout to table
    OLED_data[0] = red;
    OLED_data[1] = green;
    OLED_data[2] = blue;
    OLED_data[3] = clear;
  }
}

// read COMPASS
//
void read_Compass()
{
  if (timer_COMPASS.repeat())
  {

    compass_scalled_reading();
    compass_heading();

    // save readout to table
    OLED_data[8] = (int)compass_x_scalled;
    OLED_data[9] = (int)compass_y_scalled;
    OLED_data[10] = (int)compass_z_scalled;
    OLED_data[11] = (int)bearing;


    if (SERIAL_DEBUG)
    {
      /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
      Serial.print("X: "); Serial.print(compass_x_scalled); Serial.print("  ");
      Serial.print("Y: "); Serial.print(compass_y_scalled); Serial.print("  ");
      Serial.print("Z: "); Serial.print(compass_z_scalled); Serial.print("  ");
      Serial.print("H: "); Serial.print(bearing); Serial.print("  "); Serial.println();
    }
  }
}

// read BATT
//
void read_Batt()
{

  if (timer_BATT.repeat())
  {
    int battValue = analogRead(readBATT_pin);
    float voltage = (float)(battValue * REF_VOLTAGE / 1023) * 3.356;     // resistor divider 100k + (470k || 47k)
    // exact: 42.484k and 100.100k = 3.356 factor

    OLED_data[12] = voltage * 1000;           // save readout to table

    if (SERIAL_DEBUG)
    {
      Serial << "AD: " << battValue << " " << voltage << "V\n";
    }
  }
}

// update LED
//
void update_LED()
{

  // different functions can be implemented based on LED status
  //
  // currently:
  //
  //   LED[3] - gripper HOLD state
  //       ON - gripper_hold
  //      OFF - gripper_off
  //    BLINK - gripping (or trying to)
  //

  // update all LEDs
  //
  for (int i = 0; i < sizeof(LED) / sizeof(ledBlink); i++)
  {
    LED[i].update();
  }
}


// i2c scanner
//
void i2c_scanner()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknow error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}

// set i2c address for VL53x
//
bool setID_VL53() {

  // all reset
  pinMode(sensor1_pin, OUTPUT);
  pinMode(sensor2_pin, OUTPUT);
  pinMode(sensor3_pin, OUTPUT);
  pinMode(sensor4_pin, OUTPUT);
  digitalWrite(sensor1_pin, LOW);
  digitalWrite(sensor2_pin, LOW);
  digitalWrite(sensor3_pin, LOW);
  digitalWrite(sensor4_pin, LOW);
  delay(10);

  // activating sensor1
  digitalWrite(sensor1_pin, HIGH);
  delay(10);
  sensor1.setAddress(VL53s1_ADDRESS);
  delay(10);
  sensor1.setTimeout(200);
  if (!sensor1.init(VL53s1_INTpin)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    vl53_present = false;
  }

  // activating sensor2
  digitalWrite(sensor2_pin, HIGH);
  delay(10);
  sensor2.setAddress(VL53s2_ADDRESS);
  delay(10);
  sensor2.setTimeout(200);
  if (!sensor2.init(VL53s2_INTpin)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    vl53_present = false;
  }

  // activating sensor3
  digitalWrite(sensor3_pin, HIGH);
  delay(10);
  sensor3.setAddress(VL53s3_ADDRESS);
  delay(10);
  sensor3.setTimeout(200);
  if (!sensor3.init(VL53s3_INTpin)) {
    Serial.println(F("Failed to boot third VL53L0X"));
    vl53_present = false;
  }

  // activating sensor4
  digitalWrite(sensor4_pin, HIGH);
  delay(10);
  sensor4.setAddress(VL53s4_ADDRESS);
  delay(10);
  sensor4.setTimeout(200);
  if (!sensor4.init(VL53s4_INTpin)) {
    Serial.println(F("Failed to boot fourth VL53L0X"));
    vl53_present = false;
  }

  if (vl53_present) {
    // start continuous mode
    sensor1.startContinuous();
    sensor2.startContinuous();
    sensor3.startContinuous();
    sensor4.startContinuous();
    return true;
  }
  else return false;
}

// do LED show
//
void LEDshow() {
  for (int i = 0; i < 4; i++)
  {
    LED[i].on();
    LED[i].update();
    delay(300);
    LED[i].off();
    LED[i].update();
  }
  for (int i = 0; i < 4; i++)
  {
    LED[3 - i].on();
    LED[3 - i].update();
    delay(300);
    LED[3 - i].off();
    LED[3 - i].update();
  }
}

// do LED ERROR
//
void LEDerror() {
  LED[0].on();
  LED[0].update();
}



// FUNCTIONS for robot MODES
//
// defined based on robot level
//
// current supported modes:
//
// robotAvoidance_VL - optical avoidance based on obstacle distance
// robotLineFollow  - line follower mode
// robotEdgeAvoidance_IR - edge avoidance based on IR sensors
// robotRelTurn - relative angle Turn - need compass
// robotAbsTurn - absolute angle Turn - need compass
//
//


// optical avoidance - FRONT sensors
//
// command: 7 and 8
//
void robotAvoidance_VL(uint8_t dir, int obstacle_range_mm) {
  if (OLED_data[5] <= GROUND_DISTANCE) {                    // front ground distance
    if (OLED_data[4] <= obstacle_range_mm) {                // front length distance
      motor.setDirection(dir);
    }
    else {
      motor.setDirection(FORWARD);
    }
  }
  else motor.setDirection(STOP);
}

// line follower
//
// command: 9
//
void robotLineFollow()
{
  if (digitalRead(IR_SENSOR_RIGHT2) == 1 && digitalRead(IR_SENSOR_RIGHT1) == 0 && digitalRead(IR_SENSOR_CENTER) == 0 && digitalRead(IR_SENSOR_LEFT1) == 0 && digitalRead(IR_SENSOR_LEFT1) == 0) motor.setDirection(LEFT );
  else if (digitalRead(IR_SENSOR_RIGHT2) == 1 && digitalRead(IR_SENSOR_RIGHT1) == 1 && digitalRead(IR_SENSOR_CENTER) == 0 && digitalRead(IR_SENSOR_LEFT1) == 0 && digitalRead(IR_SENSOR_LEFT1) == 0) motor.setDirection(LEFT);
  else if (digitalRead(IR_SENSOR_RIGHT2) == 0 && digitalRead(IR_SENSOR_RIGHT1) == 1 && digitalRead(IR_SENSOR_CENTER) == 0 && digitalRead(IR_SENSOR_LEFT1) == 0 && digitalRead(IR_SENSOR_LEFT1) == 0) motor.setDirection(TURN_LEFT);
  else if (digitalRead(IR_SENSOR_RIGHT2) == 0 && digitalRead(IR_SENSOR_RIGHT1) == 1 && digitalRead(IR_SENSOR_CENTER) == 1 && digitalRead(IR_SENSOR_LEFT1) == 0 && digitalRead(IR_SENSOR_LEFT1) == 0) motor.setDirection(TURN_LEFT);
  else if (digitalRead(IR_SENSOR_RIGHT2) == 0 && digitalRead(IR_SENSOR_RIGHT1) == 0 && digitalRead(IR_SENSOR_CENTER) == 1 && digitalRead(IR_SENSOR_LEFT1) == 0 && digitalRead(IR_SENSOR_LEFT2) == 0) motor.setDirection(FORWARD);
  else if (digitalRead(IR_SENSOR_RIGHT2) == 0 && digitalRead(IR_SENSOR_RIGHT1) == 0 && digitalRead(IR_SENSOR_CENTER) == 0 && digitalRead(IR_SENSOR_LEFT1) == 1 && digitalRead(IR_SENSOR_LEFT2) == 0) motor.setDirection(TURN_RIGHT);
  else if (digitalRead(IR_SENSOR_RIGHT2) == 0 && digitalRead(IR_SENSOR_RIGHT1) == 0 && digitalRead(IR_SENSOR_CENTER) == 1 && digitalRead(IR_SENSOR_LEFT1) == 1 && digitalRead(IR_SENSOR_LEFT2) == 0) motor.setDirection(TURN_RIGHT);
  else if (digitalRead(IR_SENSOR_RIGHT2) == 0 && digitalRead(IR_SENSOR_RIGHT1) == 0 && digitalRead(IR_SENSOR_CENTER) == 0 && digitalRead(IR_SENSOR_LEFT1) == 1 && digitalRead(IR_SENSOR_LEFT2) == 1) motor.setDirection(RIGHT);
  else if (digitalRead(IR_SENSOR_RIGHT2) == 0 && digitalRead(IR_SENSOR_RIGHT1) == 0 && digitalRead(IR_SENSOR_CENTER) == 0 && digitalRead(IR_SENSOR_LEFT1) == 0 && digitalRead(IR_SENSOR_LEFT2) == 1) motor.setDirection(RIGHT);
  else if (digitalRead(IR_SENSOR_RIGHT2) == 1 && digitalRead(IR_SENSOR_RIGHT1) == 1 && digitalRead(IR_SENSOR_CENTER) == 1 && digitalRead(IR_SENSOR_LEFT1) == 1 && digitalRead(IR_SENSOR_LEFT2) == 1) motor.setDirection(STOP);
  else if (digitalRead(IR_SENSOR_RIGHT2) == 0 && digitalRead(IR_SENSOR_RIGHT1) == 0 && digitalRead(IR_SENSOR_CENTER) == 0 && digitalRead(IR_SENSOR_LEFT1) == 0 && digitalRead(IR_SENSOR_LEFT2) == 0) motor.setDirection(FORWARD);
  else motor.setDirection(FORWARD);
}

// optical avoidance - FRONT sensors
//
// command: 10
//
void robotEdgeAvoidance_IR()
{
  static uint8_t state;
  static bool edge_problem;
  static bool edge_direction;
  if (_edgeAvoidance_init)
  {
    state = 1;
    edge_problem = false;
    _edgeAvoidance_init = false;
    motor.setSpeed(140);              // set some speed
    motor.setDirection(FORWARD);
  }

  if (!edge_problem)
  {
    if (digitalRead(IR_SENSOR_RIGHT2) == 1 || digitalRead(IR_SENSOR_LEFT2) == 1)
    {
      edge_problem = true;
      edge_direction = (digitalRead(IR_SENSOR_RIGHT2) == 1) ? 0 : 1;    // 0 - left, 1-right
    }
  }
  else
  {
    switch (state) {
      case 1:
        {
          motor.setDirection(STOP);
          state++;
          break;
        }
      case 2: if (timerC(1000, state) > state) state++; break;
      case 3:
        {
          motor.setSpeed(120);
          motor.setDirection(BACKWARD);
          state++;
          break;
        }
      case 4: if (timerC(2000, state) > state) state++; break;
      case 5:
        {
          motor.setDirection(STOP);
          state++;
          break;
        }
      case 6: if (timerC(1000, state) > state) state++; break;
      case 7:
        {
          if (edge_direction) motor.setDirection(TURN_RIGHT);
          else motor.setDirection(TURN_LEFT);
          state++;
          break;
          //rel_angleTurn(90);

        }
      case 8: if (timerC(1500, state) > state) state++; break;
      case 9: edge_problem = false; _edgeAvoidance_init = true; break;      // final case
    }
  }
}

// CUSTOM timer - input delay, returns incremental number when done
//
// - used for delay between switch states
// - that way we can use the same timer multiple times
//
uint8_t timerC(int timer_delay, uint8_t number)
{
  if (_timer_custom_init)                 // set delay and start
  {
    timer_CUSTOM.set(timer_delay);
    _timer_custom_init = false;
  }
  if (timer_CUSTOM.repeat())              // execute when done
  {
    _timer_custom_init = true;
    return ++number;
  }
  else return number;
}

// relative angle Turn - need compass
//
// command: 14 - calculates abs angle and goes to next command 15 (abs_angleTurn)
//
void robotRelTurn(int angle)
{
  _angle_rel = (rotation_calibrate + OLED_data[11] + angle) % 360;
  command = 15;                                // quit current command -> goto relative movement
}

// absolute angle Turn - need compass
//
// - TODO: return value when done! - to specific serial port
//
// command: 15
//
bool robotAbsTurn(int angle)
{
  static bool angle_done;
  static int init_angle;
  static int end_angle;

  if (_turn_init)                              // just for initial calculation
  {
    init_angle = OLED_data[11];
    end_angle = angle + rotation_calibrate;
    _turn_init = false;
    angle_done = false;
  }

  if (end_angle - init_angle > 0 && _dir == 1)       // 1 = right  0 = left
  {
    if (end_angle - OLED_data[11] > 0 ) motor.setDirection(RIGHT);
    else angle_done = true;
  }
  else if (end_angle - init_angle < 0 && _dir == 0)
  {
    if (end_angle - OLED_data[11] < 0 ) motor.setDirection(LEFT);
    else angle_done = true;
  }
  else if (end_angle - init_angle < 0 && _dir == 1)
  {
    // first go over zero degrees
    if (end_angle - OLED_data[11] < 0) motor.setDirection(RIGHT);
    // we're over zero
    if (end_angle - OLED_data[11] > 0 ) angle_done = true;
  }
  else if (end_angle - init_angle > 0 && _dir == 0)
  {
    // first go over zero degrees
    if (end_angle - OLED_data[11] > 0 ) motor.setDirection(LEFT);
    // we're over zero
    if (end_angle - OLED_data[11] < 0 ) angle_done = true;
  }

  if (angle_done)
  {
    motor.setDirection(STOP);
    _turn_init = true;                 // ready for next turn
    command = 99;                      // quit current command
    senderG->GetSerial()->println("TURN DONE!");     // report turn done
    return true;                       // turn done
  }
  else return false;                   // turn not finished yet
}

//  Send ACK for status report
//
//  - upgraded for source command port
//
void send_ack(SerialCommands * sender, bool err = false)
{
  if (!err) sender->GetSerial()->println("ACK");
  else sender->GetSerial()->println("Unknown command parameter!");
}


// COMMAND execute
//
// based on function requested
void execute_command()
{
  switch (command)
  {
    case 0: motor.setDirection(STOP); break;                  // all stop
    case 1: motor.setDirection(LEFT); break;
    case 2: motor.setDirection(RIGHT); break;
    case 3: motor.setDirection(FORWARD); break;
    case 4: motor.setDirection(BACKWARD); break;
    case 5: motor.setDirection(TURN_LEFT); break;
    case 6: motor.setDirection(TURN_RIGHT); break;
    case 7: robotAvoidance_VL(LEFT, 150); break;              // optical avoidance to left
    case 8: robotAvoidance_VL(RIGHT, 150); break;             // optical avoidance to right
    case 9: robotLineFollow(); break;                         // line follower
    case 10: robotEdgeAvoidance_IR(); break;                  // edge avoidance - IR sensors only

#if defined(USE_GRIPPER)
    case 11: gripper_stop(); command = 99; break;
    case 12: gripper_start(); command = 99; break;
    case 13: gripper_release(); command = 99; break;
#endif

    case 14: robotRelTurn(_angle_rel); break;                 // relative angle turn
    case 15: robotAbsTurn(_angle_rel); break;                 // absolute angle turn

    case 99: break;                                           // dummy command for exit one-timers
    default: Serial << ("Unknown command: [") << command << "]";
  }
}
