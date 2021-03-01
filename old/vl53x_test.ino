/* This example shows how to use continuous mode to take
range measurements with the VL53L0X. It is based on
vl53l0x_ContinuousRanging_Example.c from the VL53L0X API.

The range readings are in units of mm. */

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;
VL53L0X sensor4;

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(29, OUTPUT);
  pinMode(27, OUTPUT);
  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);
  digitalWrite(29, LOW);
  digitalWrite(27, LOW);
  delay(10);
  
  digitalWrite(A0, HIGH);
  delay(10);
  
  sensor1.setAddress(0x30);
  delay(10);
  
  sensor1.setTimeout(500);
  
  if (!sensor1.init(18))
  {
    Serial.println("Failed to detect and initialize sensor1!");
    while (1) {}
  }

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor1.startContinuous();


  digitalWrite(A1, HIGH);
  delay(10);
  sensor2.setAddress(0x31);
  delay(10);
  sensor2.setTimeout(500);
  if (!sensor2.init(19))
  {
    Serial.println("Failed to detect and initialize sensor2!");
    while (1) {}
  }
  sensor2.startContinuous();

  digitalWrite(29, HIGH);
  delay(10);
  sensor3.setAddress(0x32);
  delay(10);
  sensor3.setTimeout(500);
  if (!sensor3.init(-1))
  {
    Serial.println("Failed to detect and initialize sensor3!");
    while (1) {}
  }
  sensor3.startContinuous();

  digitalWrite(27, HIGH);
  delay(10);
  sensor4.setAddress(0x33);
  delay(10);
  sensor4.setTimeout(500);
  if (!sensor4.init(-1))
  {
    Serial.println("Failed to detect and initialize sensor4!");
    while (1) {}
  }
  sensor4.startContinuous();

/*
  Serial.print("A1: ");
  Serial.println(sensor1.getAddress(),HEX);
Serial.print("A2: ");
  Serial.println(sensor2.getAddress(),HEX);
  */

  delay(2000);
  
}

void loop()
{
  //i2c_scanner();
  
    if (sensor1.available())
  {
  Serial.print("S1: ");
  Serial.println(sensor1.readRangeContinuousMillimeters());
  }
  if (sensor2.available())
  {
  Serial.print("S2: ");
  Serial.println(sensor2.readRangeContinuousMillimeters());
  }

    if (sensor3.available())
  {
  Serial.print("S3: ");
  Serial.println(sensor3.readRangeContinuousMillimeters());
  }

    if (sensor4.available())
  {
  Serial.print("S4: ");
  Serial.println(sensor4.readRangeContinuousMillimeters());
  }
Serial.println(millis());
 
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
