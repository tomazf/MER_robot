//
// robot v3 - init and HW routines
//
// gps.ino -> put this file in same folder
//
// for GPS - receive and store to object data
//
//
// - not final CODE - should be edited and put in parser object
//     - LAT
//     - LON
//     - time data - h:m:s
// - better coding needed
//

#if defined(USE_GPS)

void show_gps() {

  // Output GPS information from previous second
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
}

void parse_gps()
{
  if (timer_GPS.repeat())
  {
    while ( GPS_PORT.available())
    {
      char c = GPS_PORT.read();
      nmea.process(c);
    }
  }
}

#endif
