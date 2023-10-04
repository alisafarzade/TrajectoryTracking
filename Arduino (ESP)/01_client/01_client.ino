/*
 *  This sketch sends a message to a TCP server
 *
 */

#include <WiFi.h>
#include <WiFiMulti.h>

WiFiMulti WiFiMulti;
const uint16_t port = 3000;
const char * host = "192.168.127.148";
WiFiClient client;
char rec;

void setup()
{
  pinMode(2, OUTPUT);
  digitalWrite(2, 0);
  Serial.begin(115200);
  delay(10);

  WiFiMulti.addAP("FDMX", "123454321");


  while(WiFiMulti.run() != WL_CONNECTED) {
      delay(500);
  }
  digitalWrite(2, 1);
  delay(500);
}


void loop()
{
  while (!client.connected()) {
    digitalWrite(2,0);
    delay(100);
    client.connect(host, port);
  }
  digitalWrite(2,1);

  if (client.available() > 0)
  {
    digitalWrite(2,0);
    rec = client.read();
    Serial.print(rec);
    delay(1);
  }
}
