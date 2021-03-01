
#include <StateMachine.h>
#include <Servo.h>

// GRIPP MEASUREMENT and DELAYS
//
#define GRIPP_INCREMENT 5   // increment of continous gripper
#define GRIPP_DIFF 3    // diff between pre an post measurement
#define GRIPP_BLK 50    // feedback lock - keep it above 50
#define SERVO_HOME 2500
#define SERVO_GRIPP 200

// control variables and feedback pins
//
int servoPin = 9;
int feedbackPin = A0;
bool grip_release = false;
bool grip_hold_state = false;
int gripp_value = 0;


// local vars
//
int feedbackValue;
int feedbackValue_pre;
unsigned long time_1 = 0;
unsigned long time_2 = 0;

// create a servo object
Servo myservo;
// create state-machine objest for gripper action
StateMachine machine = StateMachine();

State* S0 = machine.addState(&state0);    // reset servo to 0
State* S1 = machine.addState(&state1);    // delay_1
State* S2 = machine.addState(&state2);    // increment value
State* S3 = machine.addState(&state3);    // delay_2
State* S4 = machine.addState(&state4);    // read feedback and compare
State* S5 = machine.addState(&state5);    // hold nad wait for release command - grip_release

void setup() {
  Serial.begin(115200);

  // state transitions
  S0->addTransition(&transitionS0S1, S1);
  S1->addTransition(&transitionS1S2, S2);
  S2->addTransition(&transitionS2S3, S3);
  S2->addTransition(&transitionS2S0, S0);
  S3->addTransition(&transitionS3S4, S4);
  S4->addTransition(&transitionS4S2, S2);
  S4->addTransition(&transitionS4S5, S5);
  S5->addTransition(&transitionS5S0, S0);

  pinMode(feedbackValue, INPUT);
  myservo.attach(servoPin);

  Serial.println("Gooo....");
}

void loop() {
  machine.run();
}


//=======================================

void state0() {
  //Serial.println("State 0");
  gripp_value = 0;
  myservo.write(gripp_value);
}

bool transitionS0S1() {
  time_1 = millis();            // set timer_1 for 1st loop
  return true;
}

void state1() {
  // just a delay - NOP
}

bool transitionS1S2() {
  if (millis() >= time_1 + SERVO_HOME) {
    time_1 += SERVO_HOME;
    return true;
  }
  return false;
}

//-------------------------
void state2() {
  gripp_value += GRIPP_INCREMENT;
  feedbackValue_pre = analogRead(feedbackPin);    // doing 2 readouts for certainty
  feedbackValue_pre = analogRead(feedbackPin);    // doing 2 readouts for certainty
  if (gripp_value <= 180)
  {
    myservo.write(gripp_value);
  }
}

bool transitionS2S3() {
  if (gripp_value <= 180)
  {
    time_2 = millis();        // set timer_2 for 2nd loop
    return true;
  }
  else return false;
}

bool transitionS2S0() {
  if (gripp_value > 180) return true;
  else return false;
}

//------------------------
void state3() {
  // just a delay - NOP
}

bool transitionS3S4() {
  if (millis() >= time_2 + SERVO_GRIPP) {
    time_2 += SERVO_GRIPP;
    //Serial.println("State 3 goto 4");
    return true;
  }
  else return false;
}

//-------------------------
void state4() {
  feedbackValue = analogRead(feedbackPin);      // doing 2 readouts for certainty
  feedbackValue = analogRead(feedbackPin);      // doing 2 readouts for certainty
  //Serial.println("State 4");
  //Serial.print(" fdb: ");
  //Serial.println(feedbackValue);
  //Serial.print("val: ");
  //Serial.println(value);
  //Serial.println();
}

bool transitionS4S2() {
  if ( (feedbackValue < feedbackValue_pre) || abs(feedbackValue - feedbackValue_pre) > GRIPP_BLK || (feedbackValue - feedbackValue_pre) <= GRIPP_DIFF )
    return false;
  else {
    return true;
  }
}

bool transitionS4S5() {
  //Serial.println("GOTO 5...");
  return true;
}

//------------------------
void state5() {
  grip_hold_state = true;
  // hold state - NOP - external varible controls exit
}

bool transitionS5S0() {
  if (grip_release)
  {
    grip_hold_state = false;
    return true;
  }
  else return false;
}
