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

motorMER::motorMER(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
  _speed = 100;
  _speed_turn = 50;
  _speed_offset_l = 0;
  _speed_offset_r = 0;
  
  _RB_MOTOR_1_1 = a;
  _RB_MOTOR_1_2 = b;
  _RB_MOTOR_2_1 = c;
  _RB_MOTOR_2_2 = d;
}


// PUBLIC METHODS
// ---------------------------------------------------------------------------

void motorMER::init()
{
  pinMode(_RB_MOTOR_1_1, OUTPUT);
  pinMode(_RB_MOTOR_1_2, OUTPUT);
  pinMode(_RB_MOTOR_2_1, OUTPUT);
  pinMode(_RB_MOTOR_2_2, OUTPUT);
  setDirection(0);
}

void motorMER::setDirection(uint8_t dir) {
  if (dir == 0) {
    analogWrite(_RB_MOTOR_1_1, 0);
    analogWrite(_RB_MOTOR_1_2, 0);
    analogWrite(_RB_MOTOR_2_1, 0);
    analogWrite(_RB_MOTOR_2_2, 0);
  }
  else if (dir == 1) {
    analogWrite(_RB_MOTOR_1_1, _speed - _speed_offset_l);
    analogWrite(_RB_MOTOR_1_2, 0);
    analogWrite(_RB_MOTOR_2_1, _speed - _speed_offset_r);
    analogWrite(_RB_MOTOR_2_2, 0);
  }
  else if (dir == 2) {
    analogWrite(_RB_MOTOR_1_1, 0);
    analogWrite(_RB_MOTOR_1_2, _speed - _speed_offset_l);
    analogWrite(_RB_MOTOR_2_1, 0);
    analogWrite(_RB_MOTOR_2_2, _speed - _speed_offset_r);
  }
  else if (dir == 3) {
    analogWrite(_RB_MOTOR_1_1, _speed + _speed_turn - _speed_offset_l);
    analogWrite(_RB_MOTOR_1_2, 0);
    analogWrite(_RB_MOTOR_2_1, _speed - _speed_offset_r);
    analogWrite(_RB_MOTOR_2_2, 0);
  }
  else if (dir == 4) {
    analogWrite(_RB_MOTOR_1_1, _speed - _speed_offset_l);
    analogWrite(_RB_MOTOR_1_2, 0);
    analogWrite(_RB_MOTOR_2_1, _speed + _speed_turn - _speed_offset_r);
    analogWrite(_RB_MOTOR_2_2, 0);
  }
  else if (dir == 5) {
    analogWrite(_RB_MOTOR_1_1, _speed - _speed_offset_l);
    analogWrite(_RB_MOTOR_1_2, 0);
    analogWrite(_RB_MOTOR_2_1, 0);
    analogWrite(_RB_MOTOR_2_2, _speed - _speed_offset_r);
  }
  else if (dir == 6) {
    analogWrite(_RB_MOTOR_1_1, 0);
    analogWrite(_RB_MOTOR_1_2, _speed - _speed_offset_l);
    analogWrite(_RB_MOTOR_2_1, _speed - _speed_offset_r);
    analogWrite(_RB_MOTOR_2_2, 0);
  }
  else {
    analogWrite(_RB_MOTOR_1_1, 0);
    analogWrite(_RB_MOTOR_1_2, 0);
    analogWrite(_RB_MOTOR_2_1, 0);
    analogWrite(_RB_MOTOR_2_2, 0);
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
