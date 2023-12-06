#define echoPinFront 18 // attach pin to pin Echo of RCWL-1601
#define trigPinFront 23 //attach pin to pin Trig of RCWL-1601

#define echoPinRight 10 // attach pin to pin Echo of RCWL-1601
#define trigPinRight 5 //attach pin to pin Trig of RCWL-1601

// defines variables
//long duration; // variable for the duration of sound wave travel
//int distance; // variable for the distance measurement

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(trigPinFront, OUTPUT); // Sets the front trigPin as an OUTPUT
  pinMode(trigPinRight, OUTPUT); // Sets the  right trigPin as an OUTPUT
  pinMode(echoPinFront, INPUT); // Sets the front echoPin as an INPUT
  pinMode(echoPinRight, INPUT); // Sets the right echoPin as an INPUT
}

void loop() {
  // put your main code here, to run repeatedly:


  // AVERAGING OUT NUM_ITER DISTANCE READINGS, TAKEN FROM FRONT AND RIGHT IN ALTERNATION
  double sumFront = 0;
  double sumRight = 0;
  int NUM_ITER = 50;
  for (int i = 0; i < NUM_ITER; i++) {
    double distIFront = getDistanceFront();
    double distIRight = getDistanceRight();
    sumFront = sumFront + distIFront;
    sumRight = sumRight + distIRight;
  }
  double avgDistFront = sumFront / NUM_ITER;
  double avgDistRight = sumRight / NUM_ITER;
  
//  Serial.print("Distance Front: ");
//  Serial.print(avgDistFront);
//  Serial.println(" cm");
//  Serial.print("Distance Right: ");
//  Serial.print(avgDistRight);
//  Serial.println(" cm");
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


//double * avgDistCombined() {
//  double sumFront = 0;
//  double sumRight =0;
//  int NUM_ITER = 50;
//  for (int i = 0; i < NUM_ITER; i++) {
//
//    double* bothDist = getDistanceCombined();
//    double distIFront = bothDist[1];
//    double distIRight = bothDist[0];
//    sumFront = sumFront + distIFront;
//    sumRight = sumRight + distIRight;
//    
//  }
//  double avgDistFront = sumFront / NUM_ITER;
//  double avgDistRight = sumRight / NUM_ITER;
//
//  double avgDistArr[2] = {avgDistRight, avgDistFront};
//  return avgDistArr;
//  
//}

// Jason's failed attempt at returning both distances in one
// We could also return a single string with all the info, then just parse that string into two ints
// But not worth the effort, let's just stick with doing them both separately
//double * getDistanceCombined() {
//  // defines variables
//  long durationRight; // variable for the duration of sound wave travel
//  long durationFront; // variable for the duration of sound wave travel
//  double distanceRight; // variable for the distance measurement
//  double distanceFront; // variable for the distance measurement
//
//  // Clears the trigPin condition
//  digitalWrite(trigPinRight, LOW);
//  digitalWrite(trigPinFront, LOW);
//  delayMicroseconds(2);
//  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
//  digitalWrite(trigPinRight, HIGH);
//  digitalWrite(trigPinFront, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(trigPinRight, LOW);
//  digitalWrite(trigPinFront, LOW);
//  // Reads the echoPin, returns the sound wave travel time in microseconds
//  durationRight = pulseIn(echoPinRight, HIGH);
//  durationFront = pulseIn(echoPinFront, HIGH);
//  // Calculating the distance
//  distanceRight = durationRight * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
//  distanceFront = durationFront * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
//
//  double distArr[2] = {distanceRight, distanceFront};
//
//  return distArr;
//
//}
