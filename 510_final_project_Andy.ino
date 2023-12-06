/*
 * Jason Friedman – Meam 510
 * 
 * drives motors to follow walls
 * Adapted from Niko's code to drive motor using user input from webpage
 * 
 */

// Make sure the following files are all in the same "510_final_project" folder:
// html510.cpp
// html510.h
// sliderJS.h
// vive510.cpp
// vive510.h

//webpage stuff
#include "sliderJS.h"  // contains string "body" html code
#include "html510.h"
HTML510Server h(80);

// Define 4 pins for ultrasonic sensor readings
#define echoPinFront 18 // attach pin to pin Echo of RCWL-1601
#define trigPinFront 23 //attach pin to pin Trig of RCWL-1601
#define echoPinRight 10 // attach pin to pin Echo of RCWL-1601
#define trigPinRight 5 //attach pin to pin Trig of RCWL-1601

// VIVE STUFF, copied from sample code
#include "vive510.h"
#define SIGNALPIN1 34 // pin receiving signal from Vive circuit
Vive510 vive1(SIGNALPIN1);

/* setup pins for output, analog read */
#define LEDPIN_L 2     /*Dir determined using Pins 12 & 14*/
#define LEDPIN_R 32    /*Dir determined using Pins 12 & 14*/
/* define the ADC channel, resolution(10 bit), frequency of led (5Hz) */
#define channel    0     /*ADC Channl 0*/
#define Res        10   /*resolution of ADC set to 10 bits*/ 
int freq =  50; 
static int botMode;

const char* ssid  = "LightningMcQueen";

/*****************/
/* web handler   */
void handleRoot() {
  h.sendhtml(body);
}

void handleSlider(){                  /* slider subroutine for duty cycle */
  String p = "Duty Cycle = ";
  int v = h.getVal();
  int val = map(v,0,100,0,1023);
  /* reads analog values from slider */
  ledcWrite(channel, val); 
  p = p + v;                            /*prints duty cycle values*/
  
  h.sendplain(p);
}

void Xslider(){
  String x = " X Coordinate = ";
  int xval=h.getVal();
  x = x + xval;
  h.sendplain(x);
  Serial.print("X Coordinate: ");
  Serial.println(x);
}

void Yslider(){
  String y = " Y Coordinate = ";
  int yval=h.getVal();
  y = y + yval;
  h.sendplain(y);
  Serial.print("Y Coordinate: ");
  Serial.println(y);
}

/* initializes motor variables */
int leftservo, rightservo;
int leftstate, rightstate;
int leftarm, rightarm;
long lastLeverMs;
static int RobotID;

void handleLever() {                  /* lever subroutine for motor directions */
  Serial.printf("handleLever Live");
   botMode = 0;
   leftarm = h.getVal();
   rightarm = h.getVal();
   leftstate = h.getVal();
   rightstate = h.getVal();
  String s = String(leftarm) + "," + String(rightarm) + "," +
             String(leftstate) + "," + String(rightstate);

  /*L/R arm handler (not applicable)*/
  //  if (leftarm) do something?
  //  if (rightarm) do something?

  /*L/R lever handler*/
  if (leftstate>0)      { digitalWrite(4,LOW); digitalWrite(15,HIGH);}
  else if (leftstate<0) {digitalWrite(4,HIGH); digitalWrite(15,LOW);} 
  else                  {digitalWrite(4,LOW); digitalWrite(15,LOW);} 

  if (rightstate>0)      { digitalWrite(12,LOW); digitalWrite(14,HIGH);}
  else if (rightstate<0) {digitalWrite(12,HIGH); digitalWrite(14,LOW);} 
  else                  {digitalWrite(12,LOW); digitalWrite(14,LOW);} 
  
  //lastLeverMs = millis(); //timestamp command
  h.sendplain(s);
  //Serial.printf("received %d %d %d %d \n",leftarm, rightarm, leftstate, rightstate); // move bot
}


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

// Blind Can XY grapping code
static double lastRobX;
static double lastRobY;
static double lastTime;
//static double velX; // May not be necessary actually
//static double velY;
static double distToCan;
static double canX;
static double canY;
static double thOffset;
static double robX;
static double robY;
static double duty;
static double deltX;
static double deltY;
static double diffX;
static double diffY;
static double thBot;
static double thToCan;

void printVals(bool first) {
  Serial.print("Vive reading: (");
  Serial.print(robX * 8000.0);
  Serial.print(", ");
  Serial.print(robY * 8000.0);
  Serial.println(")");
  Serial.print("Distance to can: ");
  Serial.print(distToCan);
  Serial.println(" ~m");
  if(!first) {
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
  if(!first) {
    Serial.print("thBot: ");
    Serial.print(thBot);
    Serial.println(" degrees");
    Serial.print("thToCan: ");
  }
  Serial.print(thToCan);
  Serial.println(" degrees");
  Serial.print("thOffset: ");
  if(!first) {
    Serial.print(thOffset);
    Serial.println(" degrees");
  }
}

// Jason
// Send @param duty percent to motor as analog input
void setDuty(double val) {
  duty = val;
  int dutyAnalog = map(duty,0,100,0,1023);
  /* reads analog values from slider */
  ledcWrite(channel, dutyAnalog);
}

// Jason
// Motor control commands

void CW() { moveWheels(1,0);  }
void CCW() { moveWheels(0,1); }
void forward() { moveWheels(1,1); }
void rest() {moveWheels(0,0); }
void back() { moveWheels(-1,-1); }

void moveWheels(int left, int right) {
  if (left > 0) { leftForward(); }
  else if (left < 0) { leftBack(); }
  else { leftStill(); }
  if (right > 0) { rightForward(); }
  else if (right < 0) { rightBack(); }
  else { rightStill(); }
}

void leftForward() { digitalWrite(4,HIGH); digitalWrite(15,LOW);}
void leftBack() {digitalWrite(4,LOW); digitalWrite(15,HIGH);}
void leftStill() {digitalWrite(4,LOW); digitalWrite(15,LOW);}
void rightForward() { digitalWrite(12,HIGH); digitalWrite(14,LOW);}
void rightBack() {digitalWrite(12,LOW); digitalWrite(14,HIGH);} 
void rightStill() {digitalWrite(12,LOW); digitalWrite(14,LOW);} 

void id1(){
  RobotID=1;
  Serial.println("RobotID=");
  Serial.print(RobotID);
}
void id2(){
  RobotID=2;
  Serial.println("RobotID=");
  Serial.print(RobotID);
}
void id3(){
  RobotID=3;
  Serial.println("RobotID=");
  Serial.print(RobotID);
}
void id4(){
  RobotID=4;
  Serial.println("RobotID=");
  Serial.print(RobotID);
}


void setup() { 

  //Serial.begin(9600); // 9600 from ultrasonic, was different for Vive and Motor webpage
  // Pin modes for ultrasonic sensor readings
  pinMode(trigPinFront, OUTPUT); // Sets the front trigPin as an OUTPUT
  pinMode(trigPinRight, OUTPUT); // Sets the  right trigPin as an OUTPUT
  pinMode(echoPinFront, INPUT); // Sets the front echoPin as an INPUT
  pinMode(echoPinRight, INPUT); // Sets the right echoPin as an INPUT

  botMode = 0;

  Serial.begin(115200);     
  WiFi.softAP(ssid);
  WiFi.softAPConfig(IPAddress(192, 168, 1, 194),  IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0)); 
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");  Serial.println(myIP); 

  h.begin();
  h.attachHandler("/ ",handleRoot);  /*web handler*/
  h.attachHandler("/slider?val=",handleSlider); /*slider*/
  h.attachHandler("/lever?val=",handleLever); /*levers*/
  h.attachHandler("/wallfollow",setWallFollow); /*wall follow*/
  h.attachHandler("/XY",setGoToXY); /*wall follow*/
  //h.attachHandler("/XY",setGoToXY); /*wall follow*/
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

  /*left motor integration*/
  pinMode(LEDPIN_L, OUTPUT);  /*PWM pin*/
  pinMode(4, OUTPUT);       /*direction control pins*/
  pinMode(15, OUTPUT);

  /*right motor integration*/
  pinMode(LEDPIN_R, OUTPUT);  /*PWM pin*/
  pinMode(14, OUTPUT);       /*direction control pins*/
  pinMode(12, OUTPUT);

  // Start up vive
  vive1.begin();
  Serial.println("Vive trackers started");
  
  // Assigned simply so first updateVals() call compiles, but reset before first practical usage
  lastRobX = 0;
  lastRobY = 0;
  lastTime = 0;
//  velX = 0;
//  velY = 0;

  // Input these can XY locations if known i.e. provided by a TA
  canX = 5000.0 / 8000;
  canY = 4000.0 / 8000;

  // Initializes all static variables
  updateVals();
  printVals(true);

  // Set duty cycle at 100%
  // TO-DO Maybe set duty at 50% always, just for slow and finer reorientation
  setDuty(50);

  // If robot was moving, then stop it 
//  rest();
 
}

void setTank() {
  botMode = 0;
}

void setWallFollow() {
  botMode = 1;
}
void setGoToXY() {
  botMode = 2;
}
void setGoToBeacon() {
  botMode = 3;
}

void loop(){
   h.serve();
  if (botMode == 1) {  
    loopWallFollow();
  } else if (botMode == 2) {
    loopGoToXY();
  } else if (botMode == 3) {
    loopGoToBeacon();
  }
 // loopGoToXY();
}

void loopGoToBeacon() {
  
}

void loopGoToXY() {
  int delayVal = 0;
  if (distToCan > 0.50) {
    Serial.println("Distance Case 1");
    delayVal = 1000;
    setDuty(50);
  } else if (distToCan > 0.10) {
    Serial.println("Distance Case 2");
    delayVal = 300; // Maybe scale it proportionally to distance away?
    setDuty(50);
  } else {
    Serial.println("Distance Case 3");
    double offsetY = 0;
    delayVal = (distToCan - offsetY) / 0.186 * 1000;
    setDuty(50);
    forward();
    delay(delayVal);
    rest();
    while(true);
  }
  forward();
  delay(delayVal);
  updateVals();
  printVals(false);
  reorient(thOffset); // Behavior depends on 'duty' value
  rest();
  delay(20000);
}

// TO-DO: Edit to account for duty cycle, lower it when theta is smaller
void reorient(double theta) {
  double T = 0;
//  if (duty == 100) {
//    T = map(theta, 0, 360, 0, 3000); // in milliseconds
//  } else if (duty == 50) {
//    T = map(theta, 0, 360, 0, 5400); // in milliseconds
//  }
  if (duty == 100) {
    T = map(theta, 0, 360, 0, 2000); // in milliseconds
  } else if (duty == 50) {
    T = map(theta, 0, 360, 0, 3600); // in milliseconds
  }
  if(T > 0) { CCW(); delay(T); }
  else if(T < 0) { CW(); delay(-T); }
  else { }
}


// EVERYTHING BELOW HERE IS FAIRLY PERMANENT, NO NEED TO CHANGE MUCH MORE

void updateVals() {
  updateRobotXY(); // Updates robX and robY static variables
//  double timeRead = millis();
//  double deltT = timeRead - lastTime;
  deltX = robX - lastRobX;
  deltY = robY - lastRobY;
//  if (deltT != 0) {
//    velX = deltX / (deltT / 1000.0); // in m/s
//    velY = deltY / (deltT / 1000.0);
//  }
//  canX = viveStuff(); Update these once we read in Can Vive UDP's, until then assume they're fixed
//  canY = viveStuff();
  diffX = canX - robX;
  diffY = canY - robY;
  distToCan = sqrt(diffX * diffX + diffY * diffY);
  thBot = atan2(deltY, deltX) * 180 / PI; // Changed this during first debugging
  thToCan = atan2(diffY, diffX) * 180 / PI;
  thOffset = thToCan - thBot;
  lastRobX = robX;
  lastRobY = robY;
//  lastTime = timeRead;
}

// Convert units here, since 8000.0 is just a ballpark
void updateRobotXY() {
  while (vive1.status() != VIVE_LOCKEDON) {
    vive1.sync(15); // try to resync (nonblocking);
  }
  robX = vive1.xCoord() / 8000.0;
  robY = vive1.yCoord() / 8000.0;
}

void loopWallFollow() {
  while(getDistanceFront() > 15 && getDistanceRight() >= 3 && getDistanceRight() <= 10) {
    forward();
  }
  if (getDistanceFront() <= 15) {
    rest();
    delay(1000);
    CCW();
    delay(750);
  }
  // Jason changed this from 2 to 3, to avoid hitting wall
  else if (getDistanceRight() < 3) {
    CCW();
    delay(83); // Measures full car 360* rotation takes 3sec, hence linearly 83ms for 10* rotation
  }
//  else
  else if (getDistanceRight() > 10) {
    CW();
    delay(83);
  }
  
}

  // AVERAGING OUT NUM_ITER DISTANCE READINGS, TAKEN FROM FRONT AND RIGHT IN ALTERNATION
  // Seems unnecessary currently, but if necessary, copy paste into whichever function uses averages
//  double sumFront = 0;
//  double sumRight = 0;
//  int NUM_ITER = 50;
//  for (int i = 0; i < NUM_ITER; i++) {
//    double distIFront = getDistanceFront();
//    double distIRight = getDistanceRight();
//    sumFront = sumFront + distIFront;
//    sumRight = sumRight + distIRight;
//  }
//  double avgDistFront = sumFront / NUM_ITER;
//  double avgDistRight = sumRight / NUM_ITER;
