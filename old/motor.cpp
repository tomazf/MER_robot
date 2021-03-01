/*
  motor.ino
    object oriented version of the motor code
*/

#if (ARDUINO <  100)
#include <WProgram.h>
#else
#include <Arduino.h>
#endif
#include <inttypes.h>

#include "motor.h"

// CONSTRUCTORS
// ---------------------------------------------------------------------------

motorMER::motorMER()
{
  _speed = 100;
  _speed_turn = 50;
  _speed_offset_l = 0;
  _speed_offset_r = 0;
}


// PUBLIC METHODS
// ---------------------------------------------------------------------------

void motorMER::init()
{
  pinMode(RB_MOTOR_1_1, OUTPUT);
  pinMode(RB_MOTOR_1_2, OUTPUT);
  pinMode(RB_MOTOR_2_1, OUTPUT);
  pinMode(RB_MOTOR_2_2, OUTPUT);
  setDirection(0);
}

void motorMER::setDirection(uint8_t dir) {
  if (dir == 0) {
    analogWrite(RB_MOTOR_1_1, 0);
    analogWrite(RB_MOTOR_1_2, 0);
    analogWrite(RB_MOTOR_2_1, 0);
    analogWrite(RB_MOTOR_2_2, 0);
  }
  else if (dir == 1) {
    analogWrite(RB_MOTOR_1_1, _speed - _speed_offset_l);
    analogWrite(RB_MOTOR_1_2, 0);
    analogWrite(RB_MOTOR_2_1, _speed - _speed_offset_r);
    analogWrite(RB_MOTOR_2_2, 0);
  }
  else if (dir == 2) {
    analogWrite(RB_MOTOR_1_1, 0);
    analogWrite(RB_MOTOR_1_2, _speed - _speed_offset_l);
    analogWrite(RB_MOTOR_2_1, 0);
    analogWrite(RB_MOTOR_2_2, _speed - _speed_offset_r);
  }
  else if (dir == 3) {
    analogWrite(RB_MOTOR_1_1, _speed + _speed_turn - _speed_offset_l);
    analogWrite(RB_MOTOR_1_2, 0);
    analogWrite(RB_MOTOR_2_1, _speed - _speed_offset_r);
    analogWrite(RB_MOTOR_2_2, 0);
  }
  else if (dir == 4) {
    analogWrite(RB_MOTOR_1_1, _speed - _speed_offset_l);
    analogWrite(RB_MOTOR_1_2, 0);
    analogWrite(RB_MOTOR_2_1, _speed + _speed_turn - _speed_offset_r);
    analogWrite(RB_MOTOR_2_2, 0);
  }
  else if (dir == 5) {
    analogWrite(RB_MOTOR_1_1, _speed - _speed_offset_l);
    analogWrite(RB_MOTOR_1_2, 0);
    analogWrite(RB_MOTOR_2_1, 0);
    analogWrite(RB_MOTOR_2_2, _speed - _speed_offset_r);
  }
  else if (dir == 6) {
    analogWrite(RB_MOTOR_1_1, 0);
    analogWrite(RB_MOTOR_1_2, _speed - _speed_offset_l);
    analogWrite(RB_MOTOR_2_1, _speed - _speed_offset_r);
    analogWrite(RB_MOTOR_2_2, 0);
  }
  else {
    analogWrite(RB_MOTOR_1_1, 0);
    analogWrite(RB_MOTOR_1_2, 0);
    analogWrite(RB_MOTOR_2_1, 0);
    analogWrite(RB_MOTOR_2_2, 0);
  }
}

void motorMER::setSpeed(uint8_t spd) {
  _speed = spd;
}

void motorMER::setSpeedTurn(uint8_t spd) {
  _speed_turn = spd;
}

void motorMER::setSpeedOffset(uint8_t spd, bool side) {
  if (!side) {
    _speed_offset_l = spd;
    _speed_offset_r = 0;
  }
  else {
    _speed_offset_r = spd;
    _speed_offset_l = 0;
  }
}
