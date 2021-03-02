//
// robot v3 - default includes
//
//

#include "./config.h"


//This is the default handler, and gets called when no other command matches.
void cmd_unrecognized(SerialCommands* sender, const char* cmd)
{
  sender->GetSerial()->print("Unrecognized command [");
  sender->GetSerial()->print(cmd);
  sender->GetSerial()->println("]");
  send_ack(sender, true);
}

// First parameter pwm value is required
// Optional parameters: led colors
// e.g. F 1     // set command = 1  -> case
//      F 2     // set command = 3  -> case
//      .
//      F 11    // stop gripper
//      F 12    // run gripper
//      F 13    // release gripper
//
//      F 20 80   // set motor speed to 80
//      F 21 50   // set motor speed_turn increment to 50
//      F 22 10   // set motor speed_offset_l to 10 and right to 0
//      F 22 -10  // set motor speed_offset_r to 10 and left to 0
//      F 23 55   // set arm angle 55°
//      F 24 60   // robot relative angle turn RIGHT -> command 14
//      F 25 -60  // robot absolute angle turn LEFT -> command 15
//
void set_mode(SerialCommands* sender)
{
  // we must do better here!
  senderG = sender;       // to save caller

  //Note: Every call to Next moves the pointer to next parameter
  char* st_str = sender->Next();
  if (st_str == NULL)
  {
    if (SERIAL_DEBUG) sender->GetSerial()->println("ERROR NO_1st_PARAM");
    send_ack(sender, true);
    return;
  }
  if ( atoi(st_str) < 20 ) {
    command = atoi(st_str);    // lower defines motor modes
    send_ack(sender);
    return;
  }

  char* nd_str = sender->Next();
  if (nd_str == NULL)
  {
    if (SERIAL_DEBUG) sender->GetSerial()->println("ERROR NO_2nd_PARAM");
    return;
  }
  else
  {
    // special command parser
    //
    if (atoi(st_str) == 20) motor.setSpeed(atoi(nd_str));
    if (atoi(st_str) == 21) motor.setSpeedTurn(atoi(nd_str));
    if (atoi(st_str) == 22)
    {
      motor.setSpeedOffset(atoi(nd_str), (atoi(nd_str) > 0) ? 1 : 0);
      write_EEPROM((int8_t)atoi(nd_str));                                     // save to EEPROM
    }
#if defined(USE_GRIPPER)
    if (atoi(st_str) == 23) arm_move(atoi(nd_str));
#endif
#if defined(USE_COMPASS)
    if (atoi(st_str) == 24) {
      _angle_rel = atoi(nd_str);
      if (atoi(nd_str) < 0) _dir = 0;
      else _dir = 1;
      command = 14;         // for relative turn
    }
    if (atoi(st_str) == 25) {
      _angle_rel = atoi(nd_str);
      if (atoi(nd_str) < 0) _dir = 0;
      else _dir = 1;
      command = 15;         // for absolute turn
    }
#endif
    if (atoi(st_str) < 26 ) send_ack(sender);     // we must do better here!
    else send_ack(sender, true);
    return;
  }

  if (SERIAL_DEBUG) Serial << "param 1: " << atoi(st_str) << " * param 2: " << atoi(nd_str);
}


// First parameter LED value is required and second parameter state
// Optional parameters: blink delay
//
// e.g. L 1 1     // LED 1 on
//      L 1 0     // LED 1 off
//      L 0 2     // LED 0 blink mode
//      L 0 2 100 // LED 0 set blink 100ms only!
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
    send_ack(sender, true);
    return;
  }

  if (atoi(st_str) == 20 || atoi(st_str) == 21) {
    switchD = true;
    for (int i = 0; i < 4; i++)
    {
      if (atoi(st_str) == 20) LED[i].off();
      else LED[i].on();
    }
    send_ack(sender);
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
  if (atoi(nd_str) < (sizeof(LED) / sizeof(ledBlink)) ) send_ack(sender);
  else send_ack(sender, true);                                                // true means command not found

  if (SERIAL_DEBUG) Serial << "param 1: " << atoi(st_str) << " * param 2: " << atoi(nd_str) << " * param 3: " << atoi(rd_str) << endl;
}


// First parameter GET value is optional, otherwise specific parameter is sent only
//
// source: OLED_data array
// Optional parameters: custom data
//
// e.g. G         // complete array is sent (no parameter)
//      G 0       // only first value from array is sent
//      G 1       // only second value from array is sent
//      G 3       // only fourth value from array is sent
//      G 55      // GPS data is sent
//
void get_data(SerialCommands* sender)
{
  //Note: Every call to Next moves the pointer to next parameter

  char* st_str = sender->Next();

  if (st_str == NULL)
  {
    sender->GetSerial()->print("D");
    for (int i = 0; i < (sizeof(OLED_data) / sizeof(int)); i++)
    {
      sender->GetSerial()->print(";");
      sender->GetSerial()->print(OLED_data[i]);
    }
    sender->GetSerial()->println(";ACK");
    return;
  }
#ifdef USE_GPS
  else if (atoi(st_str) == 55)
  {
    sender->GetSerial()->print("D-GPS;");
    sender->GetSerial()->print(GPS_data.lat);
    sender->GetSerial()->print(";");
    sender->GetSerial()->print(GPS_data.lon);
    sender->GetSerial()->print(";");
    sender->GetSerial()->print(GPS_data.h);
    sender->GetSerial()->print(";");
    sender->GetSerial()->print(GPS_data.m);
    sender->GetSerial()->print(";");
    sender->GetSerial()->print(GPS_data.s);
    sender->GetSerial()->print(";");
    sender->GetSerial()->print(GPS_data.fix);
    sender->GetSerial()->print(";");
    sender->GetSerial()->print(GPS_data.num);
    sender->GetSerial()->print(";");
    sender->GetSerial()->println("ACK");
    return;
  }
#endif
  else
  {
    if (abs(atoi(st_str)) < (sizeof(OLED_data) / sizeof(int)))
    {
      sender->GetSerial()->print("D;");
      sender->GetSerial()->print(OLED_data[atoi(st_str)]);        // we start with 0 as in array
      sender->GetSerial()->println(";ACK");
      return;
    }
    else
    {
      sender->GetSerial()->println("D;data unavailable;ACK");
    }
  }
}
