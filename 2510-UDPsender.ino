/*
 * broadcasts data packets to port 2510 
 * in ascii text strings with format #:####,####
 * where 1st number is robot ID, 2nd is X value, 3rd is y value
 * 
 */

#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid     = "TP-Link_05AF";
const char* password = "47543454";

WiFiUDP UDPTestServer;
unsigned int UDPPort = 2510; // port for cans is 1510, port for robots is 2510
IPAddress ipTarget(192, 168, 1, 255); // 255 is a broadcast address to everyone at 192.168.1.xxx
IPAddress ipLocal(192, 168, 1, 10);  // replace with your IP address

void fncUdpSend(char *datastr, int len)
{
  UDPTestServer.beginPacket(ipTarget, UDPPort);
  UDPTestServer.write((uint8_t *)datastr, len);
  UDPTestServer.endPacket();
}

void setup() {
  Serial.begin(115200);  
  Serial.print("Connecting to ");  Serial.println(ssid);
 
  WiFi.config(ipLocal, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.begin(ssid, password);

  UDPTestServer.begin(UDPPort); // strange bug needs to come after WiFi.begin but before connect

}
                                                
void loop() {  
  char s[13];
  int x, y; // some data, like  xy position
  
  // store into a string with format #:####,####, which is robotid, x, y
  sprintf(s,"%1d:%4d,%4d",4, x, y); 
  fncUdpSend(s,13);
  Serial.printf("sending data: %s",s);
  delay(100); 
}
