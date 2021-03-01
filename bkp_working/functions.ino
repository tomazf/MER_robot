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

// read range senzor data
//
void read_VL53x()
{

  senzor1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  senzor2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
  senzor3.rangingTest(&measure3, false); // pass in 'true' to get debug data printout!
  senzor4.rangingTest(&measure4, false); // pass in 'true' to get debug data printout!

  if (SERIAL_DEBUG)
  {
    // print sensor one reading
    Serial.print(F("1: "));
    if (measure1.RangeStatus != 4) {    // if not out of range
      Serial.print(measure1.RangeMilliMeter);
    } else {
      Serial.print(F("Out of range"));
    }

    Serial.print(F(" "));

    // print sensor two reading
    Serial.print(F("2: "));
    if (measure2.RangeStatus != 4) {
      Serial.print(measure2.RangeMilliMeter);
    } else {
      Serial.print(F("Out of range"));
    }

    // print sensor two reading
    Serial.print(F("3: "));
    if (measure3.RangeStatus != 4) {
      Serial.print(measure3.RangeMilliMeter);
    } else {
      Serial.print(F("Out of range"));
    }

    // print sensor two reading
    Serial.print(F("4: "));
    if (measure4.RangeStatus != 4) {
      Serial.print(measure4.RangeMilliMeter);
    } else {
      Serial.print(F("Out of range"));
    }
    Serial.println();
  }

  // save readout to table
  if (measure1.RangeStatus != 4) OLED_data[4] = measure1.RangeMilliMeter;
  if (measure2.RangeStatus != 4) OLED_data[5] = measure2.RangeMilliMeter;
  if (measure3.RangeStatus != 4) OLED_data[6] = measure3.RangeMilliMeter;
  if (measure4.RangeStatus != 4) OLED_data[7] = measure4.RangeMilliMeter;

}

//read RGB data
//
void read_RGB()
{
  if (millis() >= time_4 + INTERVAL_RGB)
  {
    time_4 += INTERVAL_RGB;

    uint16_t clear, red, green, blue;
    //rgb_senzor.setInterrupt(false);      // turn on LED
    //delay(150);  // takes 50ms to read
    //rgb_senzor.getRawData(&red, &green, &blue, &clear);

    clear = rgb_senzor.read16(TCS34725_CDATAL);
    red = rgb_senzor.read16(TCS34725_RDATAL);
    green = rgb_senzor.read16(TCS34725_GDATAL);
    blue = rgb_senzor.read16(TCS34725_BDATAL);

    //rgb_senzor.setInterrupt(true);      // turn off LED

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
    float voltage = (float)(battValue * 5.0 / 1023) * (142727) / 42727;     // resistor divider 100k + (470k || 47k)

    // save readout to table
    OLED_data[12] = voltage * 1000;

    if (SERIAL_DEBUG)
    {
      Serial << "AD: " << battValue << " " << voltage << "V\n";
    }
  }
}

// init LED
//
void init_LED()
{
  delay(500);
  for (int i = 0; i < 4; i++)
  {
    LED[i].set_blink(100, 100);
    LED[i].off();
  }
}

// update LED
//
void update_LED()
{
  for (int i = 0; i < 4; i++)
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
void setID_VL53() {
  // all reset
  digitalWrite(senzor1_pin, LOW);
  digitalWrite(senzor2_pin, LOW);
  digitalWrite(senzor3_pin, LOW);
  digitalWrite(senzor4_pin, LOW);
  delay(10);
  // all unreset
  digitalWrite(senzor1_pin, HIGH);
  digitalWrite(senzor2_pin, HIGH);
  digitalWrite(senzor3_pin, HIGH);
  digitalWrite(senzor4_pin, HIGH);
  delay(10);

  // activating senzor1 and reseting senzor2
  digitalWrite(senzor1_pin, HIGH);
  digitalWrite(senzor2_pin, LOW);
  digitalWrite(senzor3_pin, LOW);
  digitalWrite(senzor4_pin, LOW);

  // init senzor1
  if (!senzor1.begin(VL53s1_ADDRESS, VL53_DEBUG)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    //while (1);
  }
  delay(10);

  // activating senzor2
  digitalWrite(senzor2_pin, HIGH);
  delay(10);

  //init senzor2
  if (!senzor2.begin(VL53s2_ADDRESS, VL53_DEBUG)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    //while (1);
  }

  // activating senzor3
  digitalWrite(senzor3_pin, HIGH);
  delay(10);

  //init senzor3
  if (!senzor3.begin(VL53s3_ADDRESS, VL53_DEBUG)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    //while (1);
  }

  // activating senzor4
  digitalWrite(senzor4_pin, HIGH);
  delay(10);

  //init senzor4
  if (!senzor4.begin(VL53s4_ADDRESS, VL53_DEBUG)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    //while (1);
  }
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
