//
// robot v3 - init and HW routines
//
// gripper.ino -> put this file in same folder
//
// for gripper state-machine logic:
//
//    - for gripper manipulation
//    - for arm manipulation
//

#if defined(USE_GRIPPER)

// local vars
//
int feedbackValue;
int feedbackValue_pre;

bool grip_hold_state = false;   // hold indicator
bool grip_run = false;          // run gripper
bool grip_release = false;      // to reset state 5
int grip_value = 0;             // gripper value
int arm_value = 0;              // robotic arm value

void gripper_stop() {
  gripper.write(0);
  grip_release = true;
  grip_run = false;
  timer_GRIP.start();           // wait to move
}

void arm_move(uint8_t angle) {
  arm.attach(armPin);
  if (angle > ARM_MAX) angle = ARM_MAX;
  arm.write(angle);
  timer_ARM.start();            // wait to move
}

void arm_run() {

  if (timer_ARM.done())
  {
    arm.detach();
    timer_ARM.reset();
  }
}

void gripper_start() {
  gripper.attach(gripperPin);
  grip_run = true;
}

void gripper_release() {
  grip_release = true;
}

void gripper_test() {
  gripper.attach(gripperPin);
  gripper.write(50);
  delay(1000);
  gripper.write(170);
  delay(1000);
  gripper.write(5);
  delay(1000);
  gripper.detach();
}

void arm_test() {
  arm.attach(armPin);
  arm.write(50);
  delay(1000);
  arm.write(130);
  delay(1000);
  arm.write(5);
  delay(1000);
  arm.detach();
}

void gripper_init() {

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
}

void gripper_run() {
  if (grip_run) {
    machine_gripper.run();

    // set LED
    if (grip_hold_state)
    {
      LED[3].on();      // indicate grip HOLD
      senderG->GetSerial()->println("Gripper HOLD!");     // report HOLD state
    }
    else LED[3].blink();
  }
  else if (timer_GRIP.done())
  {
    gripper.detach();
    LED[3].off();
    timer_GRIP.reset();
  }
}


//=======================================
// machine states - do not edit
//
//  S0 -> S1 -> S2 -> S3 -> S4 - S5
//  
//  S0 - reset to 0   -> goto S1
//  S1 - delay        -> goto S2
//  S2 - increment +5 -> goto S3 if <180 else goto S0
//  S3 - delay        -> goto S4
//  S4 - read&compare -> goto S2 if !grip else S5
//  S5 - hold         -> goto S0 if release
//
//
void state0() {
  //Serial.println("State 0");
  grip_value = GRIPP_START;
  gripper.write(grip_value);
}

bool transitionS0S1() {
  timeG_1 = millis();            // set timer_1 for 1st loop
  return true;
}

void state1() {
  // just a delay - NOP
}

bool transitionS1S2() {
  if (millis() >= timeG_1 + SERVO_HOME) {
    timeG_1 += SERVO_HOME;
    return true;
  }
  return false;
}

//-------------------------
void state2() {
  grip_value += GRIPP_INCREMENT;
  feedbackValue_pre = analogRead(feedbackPin);       // doing 2 readouts for certainty
  feedbackValue_pre = analogRead(feedbackPin);       // doing 2 readouts for certainty
  if (grip_value <= 180)
  {
    gripper.write(grip_value);
  }
}

bool transitionS2S3() {
  if (grip_value <= 180)
  {
    timeG_2 = millis();        // set timer_2 for 2nd loop
    return true;
  }
  else return false;
}

bool transitionS2S0() {
  if (grip_value > 180) return true;
  else return false;
}

//------------------------
void state3() {
  // just a delay - NOP
}

bool transitionS3S4() {
  if (millis() >= timeG_2 + SERVO_GRIPP) {
    timeG_2 += SERVO_GRIPP;
    //Serial.println("State 3 goto 4");
    return true;
  }
  else return false;
}

//-------------------------
void state4() {
  feedbackValue = analogRead(feedbackPin);       // doing 2 readouts for certainty
  feedbackValue = analogRead(feedbackPin);       // doing 2 readouts for certainty
  /*
    Serial.println(" 4ka...");
    Serial.print(" F_pre: ");
    Serial.println(feedbackValue_pre);
    Serial.print("     F: ");
    Serial.println(feedbackValue);
    Serial.print("   val: ");
    Serial.println(grip_value);
  */
}

bool transitionS4S2() {
  //if ( (feedbackValue < feedbackValue_pre) || abs(feedbackValue - feedbackValue_pre) > GRIPP_BLK || (feedbackValue - feedbackValue_pre) <= GRIPP_DIFF )
  if ( abs(feedbackValue - feedbackValue_pre) > GRIPP_BLK || (feedbackValue - feedbackValue_pre) <= GRIPP_DIFF )
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
    // relesease HOLD state
    grip_hold_state = false;
    grip_release = false;
    return true;
  }
  else
  {
    // HOLD state - stay in 5
    return false;
  }
}

#endif
