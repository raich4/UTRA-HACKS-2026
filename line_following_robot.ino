// part 1

// INCLUDES
#include <Servo.h>
Servo myservo1; // create servo object to control a servo
Servo myservo2;

// STATE VARIABLES
int state = 1;
int colour = 0; // 0 = black, 1 = red, 2 = blue, 3 = green, 4 = white
int S1pos = 65; // for servo 1
int S2pos = 145; // for servo 2


// PIN SETUP
// colour sensor
#define CS0 6 
#define CS1 5
#define CS2 3
#define CS3 4
#define CSO 2
// motor
int rightMotor1 = A5; 
int rightMotor2 = A4;
int leftMotor1  = A3;
int leftMotor2  = A2;
int ENA = 11;   // PWM pin for right motor speed
int ENB = 10;  // PWM pin for left motor speed
// ultrasound
const int trigPin = A1;
const int echoPin = A0;

// OTHER SETUP VARIABLES
// colour sensing variables
const int redMin = 15;    // Red minimum value
const int redMax = 217;   // Red maximum value
const int greenMin = 20;  // Green minimum value
const int greenMax = 297; // Green maximum value
const int blueMin = 20;   // Blue minimum value
const int blueMax = 273;  // Blue maximum value
// variables for colour pulse width modulation
int redPW;
int greenPW;
int bluePW;
// Variables for final Color values 
int redValue;
int greenValue;
int blueValue;

void setup() {
  // PIN SETUP
  // colour sensor
  pinMode(CS0, OUTPUT);
  pinMode(CS1, OUTPUT);
  pinMode(CS2, OUTPUT);
  pinMode(CS3, OUTPUT);
  pinMode(CSO, INPUT);
  digitalWrite(CS0,HIGH);
  digitalWrite(CS1,LOW);
  // ultrasound
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  // motors
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  // servo
  myservo1.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo1.write(S1pos); //set position where ever first,
  myservo2.write(S2pos);

  Serial.begin(9600);
}

void loop() {
  // state 0: before the intersection
  // state 1: before pick up box → look for blue
  // state 2: before obstacle 1 → look for obstacle
  // state 3: before obstacle 2 → look for obstacle
  // state 4: before drop off box → look for blue
  // state 5: before red, green, black intersection again
  // state 6: before box pickup/dropoff on green section
  // state 7: go to the start


  // go forward
  Serial.print("state: ");
  Serial.println(state);
  // if (state == 0 || state == 7) { // follow the black line
  //   colour = detect_colour();
  //   Serial.println(colour);
  //   if (colour == 0) {
  //     onColour();
  //   }
  //   else {
  //     returnToColour(0);
  //   }
  // }
  // else if (state >= 1 && state <= 5) { // follow red line
  //   colour = detect_colour();
  //   Serial.print("colour: ");
  //   Serial.println(colour);
  //   if (colour == 1) {
  //     onColour();
  //   }
  //   else {
  //     returnToColour(1);
  //   }
  // }

  if ((colour == 1) || (colour == 0) || (colour == 3)) {
      onColour();
    }
    else {
      returnToColour();
    }

  // check for activity
  if (state == 0) {
    colour = detect_colour();
    if (colour == 1 || colour == 3) {
      state++;
    }
  }
  else if (state == 1 || state == 4) {
    colour = detect_colour();
    Serial.println(colour);
    if (colour == 2) {
      stopMotors();
      if (state == 1) { // box pickup
        boxPickup();
      }
      else { // box dropoff
        boxDropoff();
      }
      state++;
    }
  }
  checkAvoidObject();
}

// FUNCTION CODE ------------------------------------------------------
void checkAvoidObject() {
  float trueDis = medianDistance(7); //7-sample median
  Serial.println(trueDis);

  if (trueDis <= 15){ // avoid the object
    right(800,80,80);
    forward(3000,90,150);
    right(400,80,80);
    state++;
  }
}

void onColour() {
  Serial.println("im running onColour!");
  forwardDrive(60, 60);
}

void returnToColour() {
  Serial.println("Searching for line...");
  unsigned long startTime = millis();
  int turnspeed = 6; // Increased: 55 is often too weak to turn wheels on carpet
  
  // Timing variables
  unsigned long halfDuration = 500; 
  unsigned long fullDuration = (3 * halfDuration) + 700;

  // Use (millis() - startTime) to track how much time has passed SINCE the start
  while (millis() - startTime < fullDuration) {
    unsigned long elapsed = millis() - startTime;

    // Phase 1: Turn one way
    if (elapsed < halfDuration) {
      Serial.println("Sweep Right");
      rightDrive(turnspeed, turnspeed);
    } 
    // Phase 2: Turn the other way (wider sweep)
    else {
      Serial.println("Sweep Left");
      leftDrive(turnspeed, turnspeed);
    }

    // Check if we found the line during the sweep
    readColor(); 
    bool found = false;
    if (detectBlack()) found = true;
    if (isRed())      found = true;
    if (isGreen())    found = true;

    if (found) {
      Serial.println("Line Found!");
      delay(50);
      stopMotors();
      return; // Exit the function immediately
    }
  }
  
  stopMotors(); // Stop if we finished the whole sweep without finding it
}

void boxPickup() {
  right(300, 90, 100); // rotate 90 degrees

  lower_claw();
  open_claw();

  while (detect_colour() != 2) { // go forward till box
    forward(100, 100, 100);
  }
  forward(200, 100, 100);
  stopMotors();

  close_claw();
  raise_claw();

  while (detect_colour() != 1) { // go back till red line
    back(100, 100, 100);
  }

  stopMotors();

  left(300, 100, 90); // rotate left 90
}

void boxDropoff() {
  left(300, 100, 90); // rotate left 90
  lower_claw();
  while (detect_colour() != 2) { // go forward till box
    forward(100, 100, 100);
  }
  forward(200, 100, 100);
  
  stopMotors();
  open_claw();

  while (detect_colour() != 1) { // go back till red line
    back(100, 100, 100);
  } 
  close_claw();
  raise_claw();
  right(300, 90, 100); // rotate right 90
}

// SERVO CODE ------------------------------------------------------
void raise_claw(){
  for (S1pos = 65; S1pos <= 100; S1pos += 1){
  myservo1.write(S1pos);
  delay(20);
  }
  Serial.println("raise");
}

void lower_claw(){
  for (S1pos = 100; S1pos >= 65; S1pos -= 1){
  myservo1.write(S1pos);
  delay(20);
  }
  Serial.println("lower");
}

void open_claw(){
  for (S2pos = 145; S2pos >= 110; S2pos -= 1){
  myservo2.write(S2pos);
  delay(20);

  Serial.println(S2pos);
  }
}

void close_claw(){
  for (S2pos = 110; S2pos <= 145; S2pos += 1){
  myservo2.write(S2pos);
  delay(20);
  }
}

// COLOUR SENSING CODE ------------------------------------------------------
int detect_colour() {
  readColor();
    bool whiteDetected = detectWhite();

    if (!whiteDetected) { // ONLY LOOK FOR OTHER COLORS IF NOT WHITE
      if (detectBlack()) {
        colour = 0;
      }
      else if (isRed()) {
        colour = 1;
      }
      else if (isBlue()) {
        colour = 2;
      }
      else if (isGreen()) {
        colour = 3;
      }
    }
    else {
      colour = 4;
    }
  return colour;
}

void readColor() {
  redValue = readRed();
  delay(20);
  greenValue = readGreen();
  delay(20);
  blueValue  = readBlue();
  delay(20);
}

int readRed() {
  long sum = 0;

  digitalWrite(CS2,LOW); // SET TO RED
  digitalWrite(CS3,LOW);

  for (int i = 0; i < 5; i++) { // Read the average pulse width out of 5 iterations. 
    sum += pulseIn(CSO, LOW, 25000);
  }
  return map(sum/5, redMin,redMax,255,0);
}

int readGreen() {
  long sum = 0;

  digitalWrite(CS2, HIGH); // SET TO GREEN
  digitalWrite(CS3, HIGH);

  for (int i = 0; i < 5; i++) { 
    sum += pulseIn(CSO, LOW, 25000);
  }
  return map(sum/5, greenMin,greenMax,255,0);
}

int readBlue() {
  long sum = 0;

  digitalWrite(CS2, LOW); // SET TO BLUE
  digitalWrite(CS3, HIGH);

  for (int i = 0; i < 5; i++) {
    sum += pulseIn(CSO, LOW, 25000);
  }
  return map(sum/5, blueMin,blueMax,255,0);
}

bool detectBlack() {
  return (redValue < 200 && greenValue < 200 && blueValue < 200);
}

bool detectWhite() {
  return (redValue > 200 && greenValue > 200 && blueValue > 200);
}

bool isRed() {
  return (redValue > greenValue && redValue > blueValue);
}

bool isBlue() {
  return (blueValue > redValue && blueValue > greenValue);
}

bool isGreen() {
  return (greenValue > redValue && greenValue > blueValue);
}

// MOTOR CODE ------------------------------------------------------
void setSpeed(int rightSpeed, int leftSpeed) {
  // speed: 0..255
  rightSpeed = constrain(rightSpeed, 0, 255);
  leftSpeed  = constrain(leftSpeed, 0, 255);
  analogWrite(ENA, rightSpeed);
  analogWrite(ENB, leftSpeed);
}

// Stays in motion until you call stopMotors() ---
void forwardDrive(int speedleft, int speedright) {
  Serial.println("forwardDRIVE");
  setSpeed(speedright, speedleft); // right, left

  digitalWrite(rightMotor1, HIGH); digitalWrite(rightMotor2, LOW);
  digitalWrite(leftMotor1, HIGH);  digitalWrite(leftMotor2, LOW);
}

void backDrive(int speedleft, int speedright) {
  Serial.println("backDRIVE");
  setSpeed(speedright, speedleft); // right, left

  digitalWrite(rightMotor1, LOW);  digitalWrite(rightMotor2, HIGH);
  digitalWrite(leftMotor1, LOW);   digitalWrite(leftMotor2, HIGH);
}

void leftDrive(int speedleft, int speedright) {
  Serial.println("leftDRIVE");
  setSpeed(speedright, speedleft); // right, left

  digitalWrite(rightMotor1, LOW);  digitalWrite(rightMotor2, HIGH); // right clockwise
  digitalWrite(leftMotor1, HIGH);  digitalWrite(leftMotor2, LOW);   // left clockwise
}

void rightDrive(int speedleft, int speedright) {
  Serial.println("rightDRIVE");
  setSpeed(speedright, speedleft); // right, left

  digitalWrite(rightMotor1, HIGH); digitalWrite(rightMotor2, LOW);  // right counterclockwise
  digitalWrite(leftMotor1, LOW);   digitalWrite(leftMotor2, HIGH);  // left counterclockwise
}

void stopMotors()
{
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
}

//  Movement commands that run for time (ms), then stop ---

void back(unsigned long time, int speedleft, int speedright) {
  Serial.println("back");
  setSpeed(speedright, speedleft); //right, left

  digitalWrite(rightMotor1, LOW);  digitalWrite(rightMotor2, HIGH);
  digitalWrite(leftMotor1, LOW);   digitalWrite(leftMotor2, HIGH);

  delay(time);
  stopMotors();
}

void forward(unsigned long time, int speedleft, int speedright) {
  Serial.println("forward");
  setSpeed(speedright, speedleft); //right, left

  digitalWrite(rightMotor1, HIGH); digitalWrite(rightMotor2, LOW);
  digitalWrite(leftMotor1, HIGH);  digitalWrite(leftMotor2, LOW);

  delay(time);
  stopMotors();
}

void left(unsigned long time, int speedleft, int speedright)
{
  Serial.println("left");
  setSpeed(speedright, speedleft); //right, left

  // Right clockwise
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);

  // Left clockwise
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);

  delay(time);
  stopMotors();
}

void right(unsigned long time, int speedleft, int speedright)
{
  Serial.println("right");
  setSpeed(speedright, speedleft); //right, left

  // Right counterclockwise
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);

  // Left counterclockwise
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);

  delay(time);
  stopMotors();
}

// ULTRASONICS ------------------------------------------------------
float sonicMeasure(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) {
    Serial.println("No echo");
  }

  float distance = duration * 0.0343 / 2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  return distance;
}

float medianDistance(int samples) {
  const int MAX_SAMPLES = 9;
  if (samples > MAX_SAMPLES) samples = MAX_SAMPLES;
  if (samples < 3) samples = 3;

  float vals[MAX_SAMPLES];
  int n = 0;

  // collect valid readings
  while (n < samples) {
    float d = sonicMeasure();          // your function
    if (d > 2 && d < 400) {            // HC-SR04-ish valid range, tweak if needed
      vals[n++] = d;
    }
    delay(20);                         // small gap between pings
  }

  // simple sort (n is small)
  for (int i = 0; i < n - 1; i++) {
    for (int j = i + 1; j < n; j++) {
      if (vals[j] < vals[i]) {
        float tmp = vals[i];
        vals[i] = vals[j];
        vals[j] = tmp;
      }
    }
  }

  return vals[n / 2]; // median
}