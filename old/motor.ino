//
// robot v2 - motor init and HW routines
//
// motor.ino -> put this file in same folder
//

#include "./config.h"

/*
  void motorTurn(int degrees, char pos, char next, int speed = SPEED) {
  if (!motor_turn)
  {
    angle = OLED_data[11];      // read current angle
    motor_turn = !motor_turn;
  }
  else
  {
    if ((angle + degrees - rotation_calibrate) > OLED_data[11]) {
      RB_Motor_setDirrection(pos, speed);
    }
    else
    {
      motor_turn = false;
      angle = OLED_data[11];
      RB_Motor_setDirrection(next, speed);
      delay(5000);
    }
  }
  }
*/


void RB_MotorInit() {
  pinMode(RB_MOTOR_1_1, OUTPUT);
  pinMode(RB_MOTOR_1_2, OUTPUT);
  pinMode(RB_MOTOR_2_1, OUTPUT);
  pinMode(RB_MOTOR_2_2, OUTPUT);
}

void RB_Motor_setDirrection(char dir, char robot_speed) {
  if (dir == 0) {
    analogWrite(RB_MOTOR_1_1, 0);
    analogWrite(RB_MOTOR_1_2, 0);
    analogWrite(RB_MOTOR_2_1, 0);
    analogWrite(RB_MOTOR_2_2, 0);
  }
  else if (dir == 1) {
    analogWrite(RB_MOTOR_1_1, robot_speed);
    analogWrite(RB_MOTOR_1_2, 0);
    analogWrite(RB_MOTOR_2_1, robot_speed);
    analogWrite(RB_MOTOR_2_2, 0);
  }
  else if (dir == 2) {
    analogWrite(RB_MOTOR_1_1, 0);
    analogWrite(RB_MOTOR_1_2, robot_speed);
    analogWrite(RB_MOTOR_2_1, 0);
    analogWrite(RB_MOTOR_2_2, robot_speed);
  }
  else if (dir == 3) {
    analogWrite(RB_MOTOR_1_1, robot_speed + SPEED_TURN);
    analogWrite(RB_MOTOR_1_2, 0);
    analogWrite(RB_MOTOR_2_1, robot_speed);
    analogWrite(RB_MOTOR_2_2, 0);
  }
  else if (dir == 4) {
    analogWrite(RB_MOTOR_1_1, robot_speed);
    analogWrite(RB_MOTOR_1_2, 0);
    analogWrite(RB_MOTOR_2_1, robot_speed + SPEED_TURN);
    analogWrite(RB_MOTOR_2_2, 0);
  }
  else if (dir == 5) {
    analogWrite(RB_MOTOR_1_1, robot_speed);
    analogWrite(RB_MOTOR_1_2, 0);
    analogWrite(RB_MOTOR_2_1, 0);
    analogWrite(RB_MOTOR_2_2, robot_speed);
  }
  else if (dir == 6) {
    analogWrite(RB_MOTOR_1_1, 0);
    analogWrite(RB_MOTOR_1_2, robot_speed);
    analogWrite(RB_MOTOR_2_1, robot_speed);
    analogWrite(RB_MOTOR_2_2, 0);
  }
  else {
    analogWrite(RB_MOTOR_1_1, 0);
    analogWrite(RB_MOTOR_1_2, 0);
    analogWrite(RB_MOTOR_2_1, 0);
    analogWrite(RB_MOTOR_2_2, 0);
  }
}
