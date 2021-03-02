//
// robot v3 - init and HW routines
//
// gps.ino -> put this file in same folder
//
// for GPS - receive and store to object data
//
// - not final CODE - should be edited and put in parser object - for now is OK
//     - LAT
//     - LON
//     - time - h:m:s
//     - fix
//     - num
//

#if defined(USE_GPS)

// save to predefined struct
//
void save_GPS_data()
{
  //Serial.print("Valid fix: ");
  GPS_data.fix = (bool)nmea.isValid();

  //Serial.print("Num. satellites: ");
  GPS_data.num = (uint8_t)nmea.getNumSatellites();

  GPS_data.h = (uint8_t)nmea.getHour();
  GPS_data.m = (uint8_t)nmea.getMinute();
  GPS_data.s = (uint8_t)nmea.getSecond();

  GPS_data.lat = nmea.getLatitude() / 1000000.;
  GPS_data.lon = nmea.getLongitude() / 1000000.;

  // clear for next read
  nmea.clear();
}

void parse_gps()
{
  while ( GPS_PORT.available())
  {
    char c = GPS_PORT.read();
    nmea.process(c);
  }
  if (timer_GPS.repeat())
  {
    save_GPS_data();
  }
}

/* functions available

  Serial.print("Valid fix: ");
  Serial.println(nmea.isValid() ? "yes" : "no");

  Serial.print("Num. satellites: ");
  Serial.println(nmea.getNumSatellites());

  Serial.print("Date/time: ");
  Serial.print(nmea.getYear());
  Serial.print('-');
  Serial.print(int(nmea.getMonth()));
  Serial.print('-');
  Serial.print(int(nmea.getDay()));
  Serial.print('T');
  Serial.print(int(nmea.getHour()));
  Serial.print(':');
  Serial.print(int(nmea.getMinute()));
  Serial.print(':');
  Serial.println(int(nmea.getSecond()));

  long latitude_mdeg = nmea.getLatitude();
  long longitude_mdeg = nmea.getLongitude();
  Serial.print("Latitude (deg): ");
  Serial.println(latitude_mdeg / 1000000., 6);
  Serial.print("Longitude (deg): ");
  Serial.println(longitude_mdeg / 1000000., 6);

  long alt;
  Serial.print("Altitude (m): ");
  if (nmea.getAltitude(alt))
    Serial.println(alt / 1000., 3);
  else
    Serial.println("not available");

  nmea.clear();
*/

#endif
