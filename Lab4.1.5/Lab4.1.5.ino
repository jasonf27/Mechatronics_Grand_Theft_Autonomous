/*
 * Niko Simpkins – Meam 510
 * 
 * drives motors for esp32 based on webpage interface
 * 
 */

#include "sliderJS.h"  // contains string "body" html code
#include "html510.h"
HTML510Server h(80);

/* define the ADC channel, resolution(10 bit), frequency of led (5Hz) */
#define channel    0     /*ADC Channl 0*/
#define Res        10   /*resolution of ADC set to 10 bits*/ 
int freq =  50;  

/* setup pins for output, analog read */
#define LEDPIN_L 2     /*Dir determined using Pins 12 & 14*/
#define LEDPIN_R 32    /*Dir determined using Pins 12 & 14*/

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

/* initializes motor variables */
int leftservo, rightservo;
int leftstate, rightstate;
int leftarm, rightarm;
long lastLeverMs;


void handleLever() {                  /* lever subroutine for motor directions */
  Serial.printf("handleLever Live");
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

void setup() { 
  Serial.begin(115200);     
  WiFi.softAP(ssid);
  WiFi.softAPConfig(IPAddress(192, 168, 1, 194),  IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0)); 
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");  Serial.println(myIP);      
             
  h.begin();
  h.attachHandler("/ ",handleRoot);  /*web handler*/
  h.attachHandler("/slider?val=",handleSlider); /*slider*/
  h.attachHandler("/lever?val=",handleLever); /*levers*/
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
 
}

void loop(){
  
  h.serve();
}
