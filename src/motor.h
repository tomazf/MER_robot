// ---------------------------------------------------------------------------
// Copyright 2021 - Under creative commons license 3.0:
//
// MOTOR class
//  v0.2 - 1.2.2021 (basic functionalities)
//
// @author T. Ferbezar - tomaz.ferbezar@sc-nm.si
// ---------------------------------------------------------------------------

#ifndef mnotor_h
#define motor_h

#include <inttypes.h>

// MOTOR DIRECTIONS
//
#define STOP        0
#define FORWARD     1
#define BACKWARD    2
#define TURN_LEFT   3
#define TURN_RIGHT  4
#define LEFT        5
#define RIGHT       6

// start of class
//
class motorMER
{
  public:

    /*!
      @method
      @abstract   Class constructor.
      @discussion Initializes class variables

      @param     4 motor pins - m1+, m1-, m2+, m2-
    */
    motorMER (uint8_t, uint8_t, uint8_t, uint8_t);

    /*!
      @methods
      @abstract   Class constructor.
      @discussion Initializes class variables - update method
    */
    void init();

    /*!
      @methods
      @abstract   Class constructor.
      @discussion Initializes class variables - turn ON LED

      @param:     set motor direction - defined values
    */
    void setDirection(uint8_t);

    /*!
      @methods
      @abstract   Class constructor.
      @discussion set motor speed

      @param:     speed
    */
    void setSpeed(uint8_t);

    /*!
      @methods
      @abstract   Class constructor.
      @discussion set motor speed on turn

      @param:     speed_turn
    */
    void setSpeedTurn(uint8_t);
	
	/*!
      @methods
      @abstract   Class constructor.
      @discussion set motor speed offset

      @param:     speed_offset, L (0) or R (1)
    */
    void setSpeedOffset(uint8_t, bool);

  private:

    /*!
      @variables
    */

    uint8_t _speed;                 // motor speed
    uint8_t _speed_turn;            // motor speed on turn
	uint8_t _speed_offset_l;		// motor speed offset R
	uint8_t _speed_offset_r;		// motor speed offset L
	uint8_t _RB_MOTOR_1_1;			// motor M1+
	uint8_t _RB_MOTOR_1_2;			// motor M1-
	uint8_t _RB_MOTOR_2_1;			// motor M2+
	uint8_t _RB_MOTOR_2_2;			// motor M2-
	
};

#endif
