//
// robot v2 - default includes
//
//

#include "./config.h"


//This is the default handler, and gets called when no other command matches.
void cmd_unrecognized(SerialCommands* sender, const char* cmd)
{
  sender->GetSerial()->print("Unrecognized command [");
  sender->GetSerial()->print(cmd);
  sender->GetSerial()->println("]");
}

// First parameter pwm value is required
// Optional parameters: led colors
// e.g. F 1     // set command = 1  -> case
//      F 2     // set command = 3  -> case
//      F 20 80 // set motor speed to 80
//      F 21 50 // set motor speed_turn increment to 50
//      F 22 10 // set motor speed_offset_l to 10
//      F 23 20 // set motor speed_offset_r to 20
//
void set_mode(SerialCommands* sender)
{
  //Note: Every call to Next moves the pointer to next parameter

  char* st_str = sender->Next();
  if (st_str == NULL)
  {
    if (SERIAL_DEBUG) sender->GetSerial()->println("ERROR NO_1st_PARAM");
    return;
  }
  if ( atoi(st_str) < 20 ) {
    command = atoi(st_str);    // lower defines motor modes
    return;
  }

  char* nd_str = sender->Next();
  if (nd_str == NULL)
  {
    if (SERIAL_DEBUG) sender->GetSerial()->println("ERROR NO_2nd_PARAM");
    return;
  }
  if (atoi(st_str) == 20) motor.setSpeed(atoi(nd_str));
  if (atoi(st_str) == 21) motor.setSpeedTurn(atoi(nd_str));
  if (atoi(st_str) == 22) motor.setSpeedOffset(atoi(nd_str), 0);
  if (atoi(st_str) == 23) motor.setSpeedOffset(atoi(nd_str), 1);

  if (SERIAL_DEBUG) Serial << "param 1: " << atoi(st_str) << " * param 2: " << atoi(nd_str);
}

// First parameter LED value is required and second parameter state
// Optional parameters: blink delay
//
// e.g. L 1 1     // LED 1 on
//      L 1 0     // LED 1 off
//      L 0 2     // LED 0 blink mode
//      L 0 2 100 // LED 0 set blink 100ms only!
//      L 11 1    //
//      L 20      // all off
//      L 21      // all on
//
void set_led(SerialCommands* sender)
{
  //Note: Every call to Next moves the pointer to next parameter

  bool switchD = false;

  char* st_str = sender->Next();
  if (st_str == NULL)
  {
    if (SERIAL_DEBUG) sender->GetSerial()->println("ERROR NO_1st_PARAM");
    return;
  }

  if (atoi(st_str) == 20 || atoi(st_str) == 21) {
    switchD = true;
    for (int i = 0; i < 4; i++)
    {
      if (atoi(st_str) == 20) LED[i].off();
      else LED[i].on();
    }
  }
  if (switchD) return;

  char* nd_str = sender->Next();
  if (nd_str == NULL)
  {
    if (SERIAL_DEBUG) sender->GetSerial()->println("ERROR NO_2nd_PARAM");
    return;
  }
  // OK we got at least 2 parameters so far

  // Check if third parameter is sent
  char* rd_str = sender->Next();

  if (atoi(nd_str) == 0) {
    LED[atoi(st_str)].off();
  }
  else if (atoi(nd_str) == 1) {
    LED[atoi(st_str)].on();
  }
  else if (atoi(nd_str) == 2)
  {
    if (rd_str == NULL) {
      LED[atoi(st_str)].blink();
    }
    else if (atoi(rd_str) > LED_MIN_BLINK ) {
      LED[atoi(st_str)].set_blink(atoi(rd_str), atoi(rd_str));
    }
  }

  if (SERIAL_DEBUG) Serial << "param 1: " << atoi(st_str) << " * param 2: " << atoi(nd_str) << " * param 3: " << atoi(rd_str) << endl;
}
