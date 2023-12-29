//Initialisation for motion command
int IN1 = A1;  //forward (m1)
int IN2 = A3;  //backwards (m1)
int IN3 = 12;  // forward(m2)
int IN4 = 13;  //  (m2)backwards
int enA = 3;
int enB = 11;
int Right = 1;
int Left = 2;
int R;
int L;
int flg = 0;
int flg2 = 1;

//Initialisation for rotary encoder 
long int pulses = 0;
double distance;
int EncR = A0;
int EncL = A2;
int firstEncR;
int firstEncL;
int lastEncR;
int lastEncL;
long int changeR = 0;
long int changeL = 0;
char wheelDirection[1] = { 'N' };


//Initialisation for MPU
#include "Wire.h"
#include <MPU6050_light.h>
MPU6050 mpu(Wire);
unsigned long lastUpdateTime = 0;
unsigned long updateInterval = 100;
float finalAngle;

//Initialisation for LCD
#include <LiquidCrystal.h>
//select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// Define PID constants
double kp = 0.1;  // Proportional constant
double ki = 0.01; // Integral constant
double kd = 0.01; // Derivative constant

// Define PID variables
double error = 0;
double integral = 0;
double derivative = 0;
double lastError = 0;
double output = 0;

// Define target distance
double targetDistance = 150.0; 

// Define time variables for PID control
unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long elapsedTime = 0;

void pidControl() {
  // Calculate error
  error = targetDistance - distance;

  // Calculate integral and derivative
  integral += error * elapsedTime;
  derivative = (error - lastError) / elapsedTime;

  // Calculate PID output
  output = kp * error + ki * integral + kd * derivative;

  // Adjust motor speed based on PID output
  analogWrite(enA, 65 + output);
  analogWrite(enB, 65 + output);

  // Update last error and time variables
  lastError = error;
  previousTime = currentTime;
}

void measureDistance() {// measure distance of track 

  if (analogRead(EncR) > 100) {
    lastEncR = 1;
  } else {
    lastEncR = 0;
  }
  if (analogRead(EncL) > 100) {
    lastEncL = 1;
  } else {
    lastEncL = 0;
  }


  if (*wheelDirection == 'L' || *wheelDirection == 'D') { //only left side of encoder change will be calculated when turning left and driving straight
    if (lastEncR != firstEncR) {// if last encoder value is not equal to first encoder value, a change is calculated 
      changeR++;
      firstEncR = lastEncR;
    }
  }
  if (*wheelDirection == 'R') {//only right side of encoder change will be calculated when turning right 
    if (lastEncL != firstEncL) {// if last encoder value is not equal to first encoder value, a change is calculated 
      changeL++;
      firstEncL = lastEncL;
    }
  }
  pulses = changeR + changeL;
  distance = ((pulses / 40.0) * 3.14159 * 3 * 2);//40 changes in one revolution
}



void Neutral() {//stop 
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void Drive() { //drive straight
  analogWrite(enA, 65);  //85
  analogWrite(enB, 65);  //85
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void driveFast() {
  analogWrite(enA, 255);
  analogWrite(enB, 255);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void driveRamp() {
  analogWrite(enA, 80);
  analogWrite(enB, 80);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void Reverse() {
  analogWrite(enA, 100);
  analogWrite(enB, 100);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {  //turn to the left
  analogWrite(enA, 255);
  analogWrite(enB, 255);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnRight() { // turn to the right 
  analogWrite(enA, 255);  //255
  analogWrite(enB, 255);  //100
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void rotateRight() {
  analogWrite(enA, 0);    //125
  analogWrite(enB, 245);  //255
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}


void displayAngle() {

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Angle:");
  //while (mpu.getAngleY()>finalAngle){
  //finalAngle=mpu.getAngleY();
  //}
  lcd.print(mpu.getAngleY() + 12);
}

void displayDT(float printTime) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Distance");
  lcd.print(distance);
  lcd.setCursor(0, 2);
  lcd.print("Time");
  lcd.print(printTime);
}



void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(Right, INPUT);  //IR sensor
  pinMode(Left, INPUT);   //IR sensor
  pinMode(EncL, INPUT);
  pinMode(EncR, INPUT);


  Wire.begin();
  mpu.begin();
  mpu.calcOffsets(true,true);
  mpu.setFilterGyroCoef(0.98);

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);

  //For encoder
  if (analogRead(EncR) > 100) {  //determine first tick of encoder
    firstEncR = 1;
  } else {
    firstEncR = 0;
  }
  if (analogRead(EncL) > 100) {
    firstEncL = 1;
  } else {
    firstEncL = 0;
  }
}

void loop() {

  //read data from IR sensor
  R = digitalRead(Right);
  L = digitalRead(Left);

  //initialisation for if loop to call mpu.update() every 100 milliseconds
  unsigned long currentTime = millis();
  unsigned long lastUpdateTime = 0;
  unsigned long updateInterval = 100;
  float printTime = currentTime / 1000;
  unsigned long stopTime;

  //for PID control to stop vehicle
  elapsedTime=currentTime-previousTime;


  if (currentTime - lastUpdateTime >= updateInterval) {
    mpu.update();
    lastUpdateTime = currentTime;
  }
  if (flg == 0 || flg2 == 0) {

    if (R == 0 && L == 0) {
      driveRamp();
      *wheelDirection = 'D';
    }
    if (R == 0 && L == 1) {
      turnLeft();
      *wheelDirection = 'L';
    }

    if (R == 1 && L == 0) {
      turnRight();
      *wheelDirection = 'R';
    }
    if (R == 1 && L == 1) {
      Neutral();
      *wheelDirection = 'N';
    }
    //measureDistance();
    //displayDT(printTime);
  }


  if (mpu.getAngleY() > 8 && flg == 0) {
    flg = 1;
  }

  if (flg == 1 && flg2 == 1) {
    displayAngle();
    driveFast();
    Neutral();
    delay(4000);
    rotateRight();
    delay(1725);//1750 initial
    Neutral();
    delay(3000);
    driveRamp();
    delay(1500);
    flg = 3;
    flg2 = 4;
  }
  if (flg2 == 4 && flg == 3) {

    if (R == 1 && L == 1) {
      Drive();
      *wheelDirection = 'D';
    }
    if (R == 1 && L == 0) {
      turnLeft();
      *wheelDirection = 'L';
    }

    if (R == 0 && L == 1) {
      turnRight();
      *wheelDirection = 'R';
    }
    if (R == 0 && L == 0) {
      Neutral();
      *wheelDirection = 'N';
    }
    measureDistance();
    displayDT(printTime);

    if ((distance > targetDistance-0.1) && (distance<targetDistance+0.1)) {
      if (elapsedTime<2000){
        Neutral();
      }
    }
    else{
      pidControl();
    }
  }
}