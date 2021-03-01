/*
  Analog Feedback Servo Calibration Demo
  feedback_servo_calib.ino
  Uses S1213 Analog Feedback Servo Motor
  Results displayed on Serial Monitor

  DroneBot Workshop 2019
  https://dronebotworkshop.com
*/
#include <Servo.h>

// Control and feedback pins
int servoPin = 9;
int feedbackPin = A0;

// Value from feedback signal
int feedbackValue;

int value = 0;
bool state = true;

int readout[] = { 123, 128, 140, 152, 165, 176, 188, 199, 209, 220, 232, 242,
                  252, 264, 274, 285, 295, 306, 316, 326, 336, 346, 357, 367,
                  377, 386, 396, 406, 416, 426, 435, 445, 455, 464, 473, 485, 493
                };

#define SERVO_HOME 3000
#define SERVO_GRIPP 3500

unsigned long time_1 = 0;
unsigned long time_2 = 0;

bool servo_home = true;
bool servo_gripp = false;
bool servo_loop = true;

// Create a servo object
Servo myservo;

void setup()
{
  Serial.begin(115200);
  pinMode(feedbackValue, INPUT);

  myservo.attach(servoPin);
  myservo.write(150);
  delay(2000);

  Serial.println("Gooo....");

  calibrate();
  delay(10000);
}

void loop()
{
  
  if (servo_home)
  {
    // home gripper
    if (value == 0) {
      myservo.write(value);
      if (millis() >= time_1 + SERVO_HOME) {
        time_1 += SERVO_HOME;
        servo_home = false;
        servo_gripp = true;
        servo_loop = true;

        value = 5;
        //      delay(3000);
        Serial.println("zero...3s");
        delay(2000);
      }
    }
  }

  if (servo_gripp)
  {
    myservo.write(value);
    if (servo_loop) {
      if (millis() >= time_2 + SERVO_GRIPP) {
        time_2 += SERVO_GRIPP;
        servo_loop = false;
        //delay(350);
      }
    }
    else {
      feedbackValue = analogRead(feedbackPin);

      //     if (value != 0 )
      //     {
      /*
        Serial.print("   R: ");
        Serial.println(readout[value / 5]);
        Serial.println();
      */
      Serial.print("   V: ");
      Serial.println(value);
      Serial.print("   R: ");
      Serial.println(readout[value / 5]);
      Serial.print("   F: ");
      Serial.println(feedbackValue);
      //if (feedbackValue_post > feedbackValue_pre + 4)
      if ((readout[value / 5] + 5 > feedbackValue) && (readout[value / 5] - 5 < feedbackValue))
      {
        if (!state) {
          value = -5;
        }
        state = true;
      }
      else {
        Serial.println("HOLD");
        state = false;
      }


      if (state) value += 5;        // still no object to gripp
      if (value == 185)
      {
        value = 0;  // try again
        servo_home = true;
        servo_gripp = false;
        //     }
      }
    }
  }


}

void calibrate() {

  // Step through servo positions
  // Increment by 5 degrees
  for (int servoPos = 0; servoPos <= 180; servoPos = servoPos + 5) {

    // Position servo motor
    myservo.write(servoPos);
    // Allow time to get there
    delay(600);

    // Read value from feedback signal
    feedbackValue = analogRead(feedbackPin);

    // Write value to serial monitor
    Serial.print("Position = ");
    Serial.print(servoPos);
    Serial.print("\t");
    Serial.println(feedbackValue);
readout[servoPos / 5] = feedbackValue;
  }

  // Move back to home position
  myservo.write(0);

  // Print to serial monitor when done
  Serial.println("Finished!");

}
