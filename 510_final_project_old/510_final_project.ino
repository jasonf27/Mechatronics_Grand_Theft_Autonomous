/*
 * Jason Friedman – Meam 510
 * 
 * Includes code to satisfy all checkoff tasks from Final Project, interfacing w/ hardware
 * Namely webpage interface and control, manual control of car, wall following,
 * Go to XY via vive reading, and (WIP) beacon tracking
 * 
 */

// Webpage control helper scripts
#include "sliderJS.h"  // contains string "body" html code
#include "html510.h"
HTML510Server h(80);

// Broadcast UDP helper scripts
#include <WiFi.h>
#include <WiFiUdp.h>

// Ultrasonic sensor readings - Define 4 pins
#define echoPinFront 18 // attach pin to pin Echo of RCWL-1601
#define trigPinFront 23 //attach pin to pin Trig of RCWL-1601
#define echoPinRight 10 // attach pin to pin Echo of RCWL-1601
#define trigPinRight 5 //attach pin to pin Trig of RCWL-1601

// Vive helper script and necessary pins
#include "vive510.h"
#define SIGNALPIN1 34 // pin receiving signal from Vive circuit
Vive510 vive1(SIGNALPIN1);

// Controlling 2 motors of car
#define LEDPIN_L 2     /*Dir determined using Pins 12 & 14*/
#define LEDPIN_R 32    /*Dir determined using Pins 12 & 14*/
/* define the ADC channel, resolution(10 bit), frequency of led (5Hz) */
#define channel    0     /*ADC Channl 0*/
#define Res        10   /*resolution of ADC set to 10 bits*/ 
int freq =  50;  

// Track which task is currently being performed, controlled by webpage buttons
static int botMode;

// UDP station mode
const char* ssid = "TP-Link_05AF"; // GM router
const char* password = "47543454";

// HTML web handler
void handleRoot() {
  h.sendhtml(body);
}

// UDP port and IP numbers
WiFiUDP UDPTestServer;
unsigned int UDPPort = 2510; // port for cans is 1510, port for robots is 2510
IPAddress ipTarget(192, 168, 1, 255); // 255 is a broadcast address to everyone at 192.168.1.xxx
IPAddress ipLocal(192, 168, 1, 194);  // Andy's IP address

// UDP function for sending data to 510-wide server
void fncUdpSend(char *datastr, int len)
{
  UDPTestServer.beginPacket(ipTarget, UDPPort);
  UDPTestServer.write((uint8_t *)datastr, len);
  UDPTestServer.endPacket();
}

// Handler for duty cycle Slider
void handleSlider(){                  /* slider subroutine for duty cycle */
  String p = "Duty Cycle = ";
  int v = h.getVal();
  int val = map(v,0,100,0,1023);
  /* reads analog values from slider */
  ledcWrite(channel, val); 
  p = p + v;                            /*prints duty cycle values*/
  
  h.sendplain(p);
}

// Variables tracking can position, continually updated (elsewhere in code)
static double canX;
static double canY;

// Handler for goToXY x coordinate slider
void Xslider(){
  String x = " X Coordinate = ";
  int xval=h.getVal();
  canX = xval;
  x = x + xval;
  h.sendplain(x);
  Serial.print("X Coordinate: ");
  Serial.println(x);
}

// Handler for goToXY y coordinate slider
void Yslider(){
  String y = " Y Coordinate = ";
  int yval=h.getVal();
  canY = yval;
  y = y + yval;
  h.sendplain(y);
  Serial.print("Y Coordinate: ");
  Serial.println(y);
}

// Motor variables, interfaced with webpage
int leftservo, rightservo;
int leftstate, rightstate;
int leftarm, rightarm;
long lastLeverMs;
static int RobotID;

// Handler for two levers which control wheel motion (if any)
void handleLever() {                  /* lever subroutine for motor directions */
  Serial.printf("handleLever Live");
   botMode = 0;
   leftarm = h.getVal();
   rightarm = h.getVal();
   leftstate = h.getVal();
   rightstate = h.getVal();
  String s = String(leftarm) + "," + String(rightarm) + "," +
             String(leftstate) + "," + String(rightstate);

  // Send corresponding command to motor
  if (leftstate>0)      { digitalWrite(4,LOW); digitalWrite(15,HIGH);}
  else if (leftstate<0) {digitalWrite(4,HIGH); digitalWrite(15,LOW);} 
  else                  {digitalWrite(4,LOW); digitalWrite(15,LOW);} 
  if (rightstate>0)      { digitalWrite(12,LOW); digitalWrite(14,HIGH);}
  else if (rightstate<0) {digitalWrite(12,HIGH); digitalWrite(14,LOW);} 
  else                  {digitalWrite(12,LOW); digitalWrite(14,LOW);} 
  
  h.sendplain(s);
}

// Ultrasonic distance reading for argued L or R sensor, in cm
double getDistance(int trigPin, int echoPin) {
  // defines variables
  long duration; // variable for the duration of sound wave travel
  int distance; // variable for the distance measurement
  
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

  return distance;
}

double getDistanceFront() {
  return getDistance(trigPinFront, echoPinFront);
}
double getDistanceRight() {
  return getDistance(trigPinRight, echoPinRight);
}

// Static variables to track progress during goToXY execution, updated at each time step
static double lastRobX; // Robot X coordinate from 2nd to last time step (m)
static double lastRobY; // Robot Y coordinate from 2nd to last time step (m)
static double distToCan; // Distance between robot and can
static double thOffset; // Angle (degrees) between robot forward and robot-to-can vectors
static double robX; // Robot X coordinate from last time step (m)
static double robY; // Robot Y coordinate from last time step (m)
static int robViveX; // Raw vive x reading (not yet converted to meters)
static int robViveY; // Raw vive x reading (not yet converted to meters)
static double duty; // Duty cycle
static double deltX; // Change in X from between last and 2nd to last time steps
static double deltY; // Change in Y from between last and 2nd to last time steps
static double diffX; // x distance from robot to can
static double diffY; // y distance from robot to can
static double thBot; // angle (degrees) of robot forward vector, relative to XY plane
static double thToCan; // angle (degrees) of robot-to-can vector, relative to XY plane
static boolean swap; // boolean indicating if vive XY readings must be swapped
static int timeLast; // time of most recent UDP broadcast

// Print certain state variables, only when necessary for debugging
// @param 'first' is true iff these Vals correspond to very first time step of goToXY execution
void printVals(boolean first) {
  Serial.print("Vive reading in ~m: (");
  Serial.print(robX);
  Serial.print(", ");
  Serial.print(robY);
  Serial.println(")");
  Serial.print("Distance to can: ");
  Serial.print(distToCan);
  Serial.println(" ~m");
  if(!first) { // Exclude delta position if first time step i.e. no previous data to compare
    Serial.print("Change in position over time: (");
    Serial.print(deltX);
    Serial.print(", ");
    Serial.print(deltY);
    Serial.println(")");
  }
  Serial.print("Can relative to robot: (");
  Serial.print(diffX);
  Serial.print(", ");
  Serial.print(diffY);
  Serial.println(")");
  if(!first) { // Exclude angle of bot bc no change in bot position to compute orientation data
    Serial.print("thBot: ");
    Serial.print(thBot);
    Serial.println(" degrees");
  }
  Serial.print("thToCan: ");
  Serial.print(thToCan);
  Serial.println(" degrees");
  if(!first) { // Excude offset angle b/c no valid thBot for computation
    Serial.print("thOffset: ");
    Serial.print(thOffset);
    Serial.println(" degrees");
  }
}

// Send @param duty percent to motor as analog input, and update global variable too
void setDuty(double val) {
  duty = val;
  int dutyAnalog = map(duty,0,100,0,1023);
  /* reads analog values from slider */
  ledcWrite(channel, dutyAnalog);
}

// Helper functions to send each motor in desired direction (if any)
void leftForward() { digitalWrite(4,HIGH); digitalWrite(15,LOW);}
void leftBack() {digitalWrite(4,LOW); digitalWrite(15,HIGH);}
void leftStill() {digitalWrite(4,LOW); digitalWrite(15,LOW);}
void rightForward() { digitalWrite(12,HIGH); digitalWrite(14,LOW);}
void rightBack() {digitalWrite(12,LOW); digitalWrite(14,HIGH);} 
void rightStill() {digitalWrite(12,LOW); digitalWrite(14,LOW);} 

// Helper function to encompass all 6 functions above into 1 function
// For L and R (separately), 1 means forward, 0 means rest, -1 means backward
void moveWheels(int left, int right) {
  if (left > 0) { leftForward(); }
  else if (left < 0) { leftBack(); }
  else { leftStill(); }
  if (right > 0) { rightForward(); }
  else if (right < 0) { rightBack(); }
  else { rightStill(); }
}

// Car control commands, use above helpers to control macroscopic car motion, transl or rotat
void CW() { moveWheels(1,0);  }
void CCW() { moveWheels(0,1); }
void forward() { moveWheels(1,1); }
void rest() {moveWheels(0,0); }
void back() { moveWheels(-1,-1); }

// Arduino loop() for wall follow task
void loopWallFollow() {
  // Continue straight while sufficiently far from walls (3-10cm from right, >15 cm from front)
  while(getDistanceFront() > 15 && getDistanceRight() >= 3 && getDistanceRight() <= 10) {
    forward();
  }
  // Individually handle 3 cases which could have occurred to break out of above while()
  // (1) If too close to front wall, then instead turn left 90*
  if (getDistanceFront() <= 15) {
    rest();
    delay(1000);
    CCW();
    delay(750); // Empirically measured time required for 90* rotation
  }
  // (2) If too close to wall on the right, rotate left (i.e. CCW) a bit
  else if (getDistanceRight() < 3) {
    CCW();
    delay(83); // Empirically measured time required for 10* rotation
  }
  // (3) If too close to wall on the left, rotate right (i.e. CW) a bit
  else if (getDistanceRight() > 10) {
    CW();
    delay(83); // Empirically measured time required for 10* rotation
  }
}

// Input can arguments in vive coordinates (if necessary) convert to (m) and RHR obeying coord
void setCan(double viveCanX, double viveCanY) {
  canX = viveCanX / 2000.0; // Empirically observed conversion of 2000 vive units : 1 m
  canY = viveCanY / 2000.0;
  // Swap occurs if vive XY violate RHR i.e. create coordinate system w Z oriented downward
  if(swap) {
    double temp = canX;
    canX = canY;
    canY = temp;
  }
}

// Handler for Robot ID 1 button
void id1(){
  RobotID=1;
  Serial.println("RobotID=");
  Serial.print(RobotID);
}
// Handler for Robot ID 2 button
void id2(){
  RobotID=2;
  Serial.println("RobotID=");
  Serial.print(RobotID);
}
// Handler for Robot ID 3 button
void id3(){
  RobotID=3;
  Serial.println("RobotID=");
  Serial.print(RobotID);
}
// Handler for Robot ID 4 button
void id4(){
  RobotID=4;
  Serial.println("RobotID=");
  Serial.print(RobotID);
}

// Arduino setup(), runs first upon sending script to board
void setup() { 

  Serial.begin(115200);
  
  // Ultrasonic sensor pin modes
  pinMode(trigPinFront, OUTPUT); // Sets the front trigPin as an OUTPUT
  pinMode(trigPinRight, OUTPUT); // Sets the  right trigPin as an OUTPUT
  pinMode(echoPinFront, INPUT); // Sets the front echoPin as an INPUT
  pinMode(echoPinRight, INPUT); // Sets the right echoPin as an INPUT

  // Initialize variables
  timeLast = 0;
  botMode = 0;
  RobotID = 1;
  
// AP Mode for html
// We used AP mode at first, before implementing UDP broadcast and thus shifting to Station mode
//  WiFi.softAP(ssid);
//  WiFi.softAPConfig(IPAddress(192, 168, 1, 194),  IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0)); 
//  IPAddress myIP = WiFi.softAPIP();
//  Serial.print("AP IP address: ");  Serial.println(myIP); 

//  Station mode for html
    WiFi.mode(WIFI_MODE_STA);
    WiFi.begin(ssid,password);
    WiFi.config(IPAddress(192, 168, 1, 194),  IPAddress(192, 168, 1, 1), IPAddress(255, 255, 254, 0));

  // Wait until WiFi is connected 
   while (WiFi.status() != WL_CONNECTED){
     delay(500);
     Serial.print(".");
    }
    Serial.println("Wifi connected!");
  
  //UDPTestServer.begin(UDPPort); // strange bug needs to come after WiFi.begin but before connect

  // Begin HTML user interface watchdog, and attach handlers for various buttons
  h.begin();
  h.attachHandler("/ ",handleRoot);  /*web handler*/
  h.attachHandler("/slider?val=",handleSlider); /*slider*/
  h.attachHandler("/lever?val=",handleLever); /*levers*/
  h.attachHandler("/wallfollow",setWallFollow); /*wall follow*/
  h.attachHandler("/XY",setGoToXY); /*wall follow*/
  //h.attachHandler("/XY",setGoToBeacon); /*wall follow*/
  h.attachHandler("/Xslider?val=",Xslider); /*slider*/
  h.attachHandler("/Yslider?val=",Yslider); /*slider*/
  h.attachHandler("/id1",id1); /*id1*/
  h.attachHandler("/id2",id2); /*id2*/
  h.attachHandler("/id3",id3); /*id3*/
  h.attachHandler("/id4",id4); /*id4*/

  // Analog output readings for motor control
  ledcAttachPin(LEDPIN_L, channel); /*PWM setup*/
  ledcAttachPin(LEDPIN_R, channel); /*PWM setup*/
  ledcSetup(channel, freq, Res);

  // Left motor integration, pin modes
  pinMode(LEDPIN_L, OUTPUT);  /*PWM pin*/
  pinMode(4, OUTPUT);       /*direction control pins*/
  pinMode(15, OUTPUT);

  // Right motor integration, pin modes
  pinMode(LEDPIN_R, OUTPUT);  /*PWM pin*/
  pinMode(14, OUTPUT);       /*direction control pins*/
  pinMode(12, OUTPUT);

  // Vive: Being readings
  vive1.begin();
  Serial.println("Vive trackers started");

  // Initialize, purely to avoid compile errors
  lastRobX = 0;
  lastRobY = 0;

  swap = true; // Since x and y are non-right hand rule i.e. z points down in world
  // As of testing on Fri 12/17 on M81 vive setup

  // Test values for vive XY, although these are reset later by webpage interface
  setCan(3690, 5480); 

  // Det duty to 50% always, to allow more precise motions than 100%
  setDuty(50);

}

// Handler functions, update global "botMode" for purpose of casing in loop() below
void setTank() {
  botMode = 0;
}
void setWallFollow() {
  botMode = 1;
}
void setGoToXY() {
  botMode = 2;
  setCan(canX, canY); // Read in XY values from sliders, translate them to (m) in RHR frame
}
void setGoToBeacon() {
  botMode = 3;
}

// Arduino loop()
void loop(){

   // Send XY vive coordinate location via UDP broadcast if >=1 second has passed since last send
   h.serve();
//    int mil = millis();
//  if (mil - timeLast >= 1000){
//    timeLast = mil;
//    char s[13];
//    updateRobotXY();
//    
//    // store into a string with format #:####,####, which is robotid, x, y
//    sprintf(s,"%1d:%4d,%4d",RobotID, robViveX, robViveY); 
//    fncUdpSend(s,13);
//    Serial.printf("sending data: %s",s);
//    Serial.print('\n');
//  }

  // Case on which botMode is active (i.e. which mode selected on HTML), run corresponding loop()
  if (botMode == 1) {  
    loopWallFollow();
  } else if (botMode == 2) {
    loopGoToXY();
  } else if (botMode == 3) {
    loopGoToBeacon();
  }
}

// Arduino loop() if in Beacon mode, WIP
void loopGoToBeacon() {
  
}

// Arduino loop() if in goToXY mode
void loopGoToXY() {
  updateVals(); // Update tracker variables before straight forward motion
  int delayVal = 0; 
  /* Case on how far bot to can, take smaller / more precise steps the closer you get.
  Start by moving forward, getting nearer to can and also gaining orientational data by comparison */
  if (distToCan > 0.50) {
    Serial.println("Distance Case 1");
    delayVal = 1000;
    forward();
    delay(delayVal);
  } else if (distToCan > 0.05) {
    Serial.println("Distance Case 2");
    delayVal = 300;
    forward();
    delay(delayVal);
  } else { // Once bot is within 5cm, finish the job then stop forever
    Serial.println("Distance Case 3");
    double offsetY = 0;
    delayVal = (distToCan - offsetY) / 0.186 * 1000;
    forward();
    delay(delayVal);
    rest();
    while(true);
  }
  updateVals(); /* Update tracker variables after straight forward motion
  Notably including thBot, orientation known by comparing past 2 time step vive readings */
  reorient(thOffset); // thOffset = thToCan - thBot; angle bot towards can (if necessary)
  rest();
  delay(3000); // Rest for 3 seconds between time steps, to give everyone evaluation time
}

// Rotate robot such that it faces towards the can, or at least closer than before
// @param theta is desired angular offset
void reorient(double theta) {
  double T = 0; // Time of rotation
  if (duty == 100) {
    T = map(theta, 0, 360, 0, 2000); /* Hardcoded, empirically observed mapping
    from rotation desired (degrees) to time rotation required (ms) */
  } else if (duty == 50) {
    T = map(theta, 0, 360, 0, 3600);
  }
  if(T > 0) { CCW(); delay(T); }
  else if(T < 0) { CW(); delay(-T); } /* Negative CCW angle (and proportionally time)
  means positive CW angle */
  else { } // If no angle of rotation necessary, then do nothing!
}

// Update tracker variables
void updateVals() {
  updateRobotXY(); // Updates robX, robY, robViveX, robViveY static variables
  deltX = robX - lastRobX;
  deltY = robY - lastRobY;
  diffX = canX - robX;
  diffY = canY - robY;
  distToCan = sqrt(diffX * diffX + diffY * diffY);
  thBot = atan2(deltY, deltX) * 180 / PI;
  thToCan = atan2(diffY, diffX) * 180 / PI;
  thOffset = thToCan - thBot;
  while (thOffset < -180) {thOffset += 360;} // Return angles in (-180,180) range for efficiency
  while (thOffset > 180) {thOffset -= 360;}
  lastRobX = robX;
  lastRobY = robY;
}

// Updates robX, robY, robViveX, robViveY static variables
void updateRobotXY() {
  while (vive1.status() != VIVE_LOCKEDON) { // sync() until getting a good vive reading
    Serial.println("Vive1 not working");
    vive1.sync(15); // try to resync (nonblocking);
  }
  robViveY = 0;
  robViveX = 0;
  int count = 0;
  while(robViveY == 0 && robViveX == 0) { // Ignore (0, 0) readings because  they're buggy
    if (count > 0) { Serial.println("Vive reading (0, 0)"); } /* Print only from undesirable
    behavior as indicated by the while loop executing more than once i.e. reading (0,0) */
    robViveX = vive1.xCoord();
    robViveY = vive1.yCoord();
    count += 1;
  }
  robX = robViveX / 2000.0; // Convert to (m) from vive coordinates
  robY = robViveY / 2000.0;
  if(swap) { // Swap if necessary, to get coordinates which obey RHR
    double temp = robX;
    robX = robY;
    robY = temp;
  }
}
