#include <ESP8266WiFi.h>
#include <WiFiClient.h>

#define UART_BAUD 115200
#define packTimeout 5                 // ms (if nothing more on UART, then send packet)
#define bufferSize 8192


const char *ssid = "MER";             // You will connect your phone to this Access Point
const char *pw = "novomesto";         // and this is the password
IPAddress ip(192, 168, 4, 1);         // From RoboRemo app, connect to this IP
IPAddress netmask(255, 255, 255, 0);
const int port = 9876;                // and this port

WiFiServer server(port);
WiFiClient client;


uint8_t buf1[bufferSize];
uint16_t i1 = 0;

uint8_t buf2[bufferSize];
uint16_t i2 = 0;


void setup() {

  delay(5000);

  Serial.begin(UART_BAUD);
  Serial.println("Go...");
  delay(5000);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ip, ip, netmask);         // configure ip address for softAP
  WiFi.softAP(ssid, pw);                      // configure ssid and password for softAP

  Serial.println("AP done!");
  delay(5000);

  Serial.println("Starting TCP Server");
  server.begin();                             // start TCP server
  Serial.println("Server done!");
  delay(5000);
}


void loop() 
{
  if (!client.connected()) { // if client not connected
    client = server.available(); // wait for it to connect
    return;
  }
  //Serial.println("Loop TCP with client!");
  // here we have a connected client

  if (client.available()) {
    while (client.available()) {
      buf1[i1] = (uint8_t)client.read(); // read char from client (RoboRemo app)
      if (i1 < bufferSize - 1) i1++;
    }
    // now send to UART:
    Serial.write(buf1, i1);
    i1 = 0;
  }

  if (Serial.available()) {
    // read the data until pause:
    while (1) {
      if (Serial.available()) {
        buf2[i2] = (char)Serial.read(); // read char from UART
        if (i2 < bufferSize - 1) i2++;
      } else {
        //delayMicroseconds(packTimeoutMicros);
        delay(packTimeout);
        if (!Serial.available()) {
          break;
        }
      }
    }

    // now send to WiFi:
    client.write((char*)buf2, i2);
    i2 = 0;
  }
}
