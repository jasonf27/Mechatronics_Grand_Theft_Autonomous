/*
 * receives packets  (plus any other broadcasters)
 * 
 */
#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid     = "TP-Link_05AF";
const char* password = "47543454";

WiFiUDP canUDPServer;
WiFiUDP robotUDPServer;


IPAddress myIPaddress(192, 168, 1, 9); // change to your IP

void handleCanMsg() {
  const int UDP_PACKET_SIZE = 14; // can be up to 65535
  uint8_t packetBuffer[UDP_PACKET_SIZE];
  
  int cb = canUDPServer.parsePacket();
  if (cb) {
    int x,y;
    packetBuffer[cb]=0; // null terminate string
    canUDPServer.read(packetBuffer, UDP_PACKET_SIZE);
    
    x = atoi((char *)packetBuffer+2);
    y = atoi((char *)packetBuffer+7);

    Serial.print("From Can ");
    Serial.println((char *)packetBuffer);
    Serial.println(x); 
    Serial.println(y);
    }
}

void handleRobotMsg() {
  const int UDP_PACKET_SIZE = 14; // can be up to 65535
  uint8_t packetBuffer[UDP_PACKET_SIZE];
  
  int cb = robotUDPServer.parsePacket();
  if (cb) {
    int x,y;
    packetBuffer[cb]=0; // null terminate string
    robotUDPServer.read(packetBuffer, UDP_PACKET_SIZE);
    
    x = atoi((char *)packetBuffer+2);
    y = atoi((char *)packetBuffer+7);
    Serial.print("From Robot ");
    Serial.println((char *)packetBuffer);
    Serial.println(x); 
    Serial.println(y);
    }
}

void setup() {
  Serial.begin(115200);
  Serial.print("Connecting to ");  Serial.println(ssid);
  
  WiFi.config(myIPaddress, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.begin(ssid, password);
  
  canUDPServer.begin(1510); // can port 1510
  robotUDPServer.begin(2510); // robot port 2510
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");  
}

void loop() {
   handleCanMsg();
   handleRobotMsg();
   delay(10);
}
