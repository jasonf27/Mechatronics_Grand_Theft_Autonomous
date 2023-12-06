/*
 * vive510
 *   class for tracking multiple diodes under a vive basestation (1.0)
 *   
 * use timer interrupt to measure pulse width and between pulses, 
 * updates vive.xCoord and vive.yCoord return x,y coordinates that are updated asynchronously 60 times per sec
 * range X (up/down): 1400 to 7000 may vary unit to unit - roughly 120 deg
 * 
 * range Y (left/right): 1000 to 6700 may vary unit to unit - roughly 120 deg
 * distance reliable to 10'
 */

#include "vive510.h"

#define SIGNALPIN1 34 // pin receiving signal from Vive circuit

Vive510 vive1(SIGNALPIN1);

void setup(){
  Serial.begin(9600);
  vive1.begin();
  Serial.println("Vive trackers started");
}

void loop() {

  if (vive1.status() == VIVE_LOCKEDON) {
    Serial.printf("X %d, Y %d\n",vive1.xCoord()/10,vive1.yCoord()/10);
  }
  else 
     vive1.sync(15); // try to resync (nonblocking);

  delay(100);
}
