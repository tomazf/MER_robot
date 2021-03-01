//
// robot v2 - init and HW routines
//
// functions.ino -> put this file in same folder
//
// TODO: add proper .h file includes and function prototypes
//

// Adafruit OLED init
//
void init_OLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(0, 5);
  display.println("Setup init...");
  display.setCursor(0, 20);
  display.println("    -- ROBO v2 --");

  display.setCursor(0, 40);
  display.print(" devices:");
    for (int i = 0; i < velikost; i++) if (i2c_device[1][i] == 1) display.print("+"); else display.print("-");

  display.display();
  delay(4000);

  display.clearDisplay();
  display.display();
}

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

// write some OLED debug data
//
void write_OLED() {

  if (millis() >= time_5 + INTERVAL_OLED)
  {
    time_5 += INTERVAL_OLED;

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
    display << "VL53X s1 = " << OLED_data[4] << "\n";
    display << "      s2 = " << OLED_data[5] << "\n";
    display << "      s3 = " << OLED_data[6] << "\n";
    display << "      s4 = " << OLED_data[7];

    display.setCursor(0, 50);
    display << "BATT " << (float)OLED_data[12] / 1000 << "V "  << OLED_data[11];

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

//read RGB data
//
void read_RGB()
{
  if (millis() >= time_4 + INTERVAL_RGB)
  {
    time_4 += INTERVAL_RGB;

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
  if (millis() >= time_2 + INTERVAL_COMPASS)
  {
    {
      time_2 += INTERVAL_COMPASS;

      /* Get a new sensor event */
      sensors_event_t event;
      mag.getEvent(&event);

      // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
      // Calculate heading when the magnetometer is level, then correct for signs of axis.
      float heading = atan2(event.magnetic.y, event.magnetic.x);

      // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
      // Find yours here: http://www.magnetic-declination.com/
      // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
      // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
      float declinationAngle = DECLINATION * PI / 180; // converted to RADIANS
      heading += declinationAngle;

      // Correct for when signs are reversed.
      if (heading < 0)
        heading += 2 * PI;

      // Check for wrap due to addition of declination.
      if (heading > 2 * PI)
        heading -= 2 * PI;

      // Convert radians to degrees for readability.
      float headingDegrees = heading * 180 / M_PI;

      // save readout to table
      OLED_data[8] = event.magnetic.x;
      OLED_data[9] = event.magnetic.y;
      OLED_data[10] = event.magnetic.z;
      OLED_data[11] = headingDegrees;
    }

    if (SERIAL_DEBUG)
    {
      sensors_event_t event;
      mag.getEvent(&event);
      /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
      Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
      Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
      Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  "); Serial.println("uT");
    }
  }
  //  delay(250);
}

// read BATT
//
void read_Batt()
{

  if (millis() >= time_3 + INTERVAL_BATT)
  {
    time_3 += INTERVAL_BATT;

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

// init LED
//
bool init_LED()
{
  delay(50);
  LEDshow();                      // just small demo
  for (int i = 0; i < sizeof(LED)/sizeof(ledBlink); i++)
  {
    LED[i].set_blink(100, 100);
    LED[i].off();
  }
  return true;
}

// update LED
//
void update_LED()
{
  for (int i = 0; i < sizeof(LED)/sizeof(ledBlink); i++)
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


// serial parser INIT
//
void init_serialParser() {
  serial_commands_.SetDefaultHandler(cmd_unrecognized);
  serial_commands_.AddCommand(&set_mode_);
  serial_commands_.AddCommand(&set_led_);
  serial_commands2_.SetDefaultHandler(cmd_unrecognized);
  serial_commands2_.AddCommand(&set_mode_);
  serial_commands2_.AddCommand(&set_led_);
}


// FUNCTIONS for robot MODES
//
// defined based on robot level
//
// current supported modes:
//
// robotAvoidance_1 - optical avoidance based on distance
// robotLineFollow  - line follower mode
//
//


// optical avoidance - FRONT sensors
//
void robotAvoidance_1(int dir, int obstacle_range_mm) {
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
    case 7: robotAvoidance_1(LEFT, 150); break;               // optical avoidance to left
    case 8: robotAvoidance_1(RIGHT, 150); break;              // optical avoidance to right
    case 9: robotLineFollow(); break;                         // line follower

    default: Serial << ("Unknown command: [") << command << "]";
  }
}
