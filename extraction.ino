//Libraries 
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "src/MeSingleLineFollower.h"
#include "src/MeCollisionSensor.h"
#include "src/MeBarrierSensor.h"
#include "src/MeNewRGBLed.h"
#include <MeMegaPi.h>

//Constants and states the robot can be in
#define SPEED 100
#define NOSTATE 0
#define OBSTACLEAVOIDANCE 1
#define LINEFOLLOWER 2

int currentState;

//Creating object for each sensor/motor
MeNewRGBLed leftLED(A13,4);
MeNewRGBLed rightLED(A14, 4);
MeCollisionSensor leftCol(A11);
MeCollisionSensor rightCol(A12);
MeBarrierSensor leftBar(A6);
MeBarrierSensor middleBar(A7);
MeBarrierSensor rightBar(A8);
MeSingleLineFollower leftLine(A9);
MeSingleLineFollower rightLine(A10);
MeMegaPiDCMotor motor1(PORT1A);
MeMegaPiDCMotor motor2(PORT1B);
MeMegaPiDCMotor motor3(PORT2B);
MeMegaPiDCMotor motor4(PORT2A);


//Moves the robot forward
void forward() {
  motor1.run(SPEED);
  motor2.run(SPEED);
  motor3.run(-SPEED);
  motor4.run(-SPEED);
}

//Moves the robot backward
void backward() {
  motor1.run(-SPEED);
  motor2.run(-SPEED);
  motor3.run(SPEED);
  motor4.run(SPEED);
}

//Moves the robot right without changing orientation 
void right() {
  motor1.run(-SPEED);
  motor2.run(SPEED);
  motor3.run(-SPEED);
  motor4.run(SPEED);
}

//Moves the robot left without changing orientation
void left() {
  motor1.run(SPEED);
  motor2.run(-SPEED);
  motor3.run(SPEED);
  motor4.run(-SPEED);
}

//Rotates the robot counterclockwise
void rotateLeft() {
  motor1.run(SPEED);
  motor2.run(SPEED);
  motor3.run(SPEED);
  motor4.run(SPEED);
}

//Rotates the robot clockwise
void rotateRight() {
  motor1.run(-SPEED);
  motor2.run(-SPEED);
  motor3.run(-SPEED);
  motor4.run(-SPEED);
}

//Stops all motors from turning
void stop() {
  motor1.stop();
  motor2.stop();
  motor3.stop();
  motor4.stop();
}

//Code for obstacle avoidance
void obstacleAvoidance() {
   while(!leftBar.isBarried() && !rightBar.isBarried()) {
    forward();
  }

  if(leftBar.isBarried() && rightBar.isBarried()) {
    left();
  } else if(rightBar.isBarried()) {
    backward();
    delay(500);
    rotateLeft();
    delay(500);
  } else if(leftBar.isBarried()) {
    backward();
    delay(500);
    rotateRight();
    delay(500);
  }
}

//Code for following the line
void lineFollower() {
   while(leftLine.onLine() && rightLine.onLine()) {
    forward();
  }

  if(!leftLine.onLine() && !rightLine.onLine()) {
    rotateLeft();
  } else if(!leftLine.onLine()) {
    rotateRight();
  } else if(!rightLine.onLine()) {
    rotateLeft();
  }
}

void setup() {
  currentState = NOSTATE;
}

void loop() {

  /**
  Left Collision Sensor -- Runs the obstacle avoidance code (Magenta)
  Right Collision Sensor -- Runs the line following code (Blue)
  Both Sensors -- Stops the robot (White)
  **/

  if(leftCol.isCollision() && rightCol.isCollision()) {
    currentState = NOSTATE;
  } else if(leftCol.isCollision()) {
    currentState = OBSTACLEAVOIDANCE;
  } else if(rightCol.isCollision()) {
    currentState = LINEFOLLOWER;
  }

  if(currentState == OBSTACLEAVOIDANCE) {
    leftLED.setColor(255, 0, 255);
    leftLED.show();
    obstacleAvoidance();
  } else if(currentState == LINEFOLLOWER) {
    leftLED.setColor(0, 255, 255);
    leftLED.show();
    lineFollower();
  } else if(currentState == NOSTATE) {
    leftLED.setColor(255, 255, 255);
    leftLED.show();
    stop();
  }
}