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
#define SENTRY 3

char correctSequence[] = {'l', 'r', 'l', 'l'};

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

void yellow() {
  rightLED.setColor(0, 255, 255, 0);
  leftLED.setColor(0, 255, 255, 0);
  leftLED.show();
  rightLED.show();
}

void purple() {
  rightLED.setColor(0, 204, 0, 204);
  leftLED.setColor(0, 204, 0, 204);
  leftLED.show();
  rightLED.show();
}

void red() {
  rightLED.setColor(0, 255, 0, 0);
  leftLED.setColor(0, 255, 0, 0);
  leftLED .show();
  rightLED .show();
}

void green() {
  rightLED.setColor(0, 0, 255, 0);
  leftLED.setColor(0, 0, 255, 0);
  leftLED.show();
  rightLED.show();
}

void blue() {
  rightLED.setColor(0, 0, 0, 255);
  leftLED.setColor(0, 0, 0, 255);
  leftLED.show();
  rightLED.show();
}

void white() {
  rightLED.setColor(0, 255, 255, 255);
  leftLED.setColor(0, 255, 255, 255);
  leftLED.show();
  rightLED.show();
}

void off() {
  rightLED.setColor(0, 0, 0, 0);
  leftLED.setColor(0, 0, 0, 0);
  leftLED.show();
  rightLED.show();
}

void realRobot() {
  purple();
  delay(100);
  yellow();
  delay(100);
}

void flashSuccess() {
  int count = 0;
  while(count < 10) {
    green();
    delay(100);
    off();
    delay(100);
    count++;
  }
}

void flashFail() {
  int count = 0;
  while(count < 10) {
    red();
    delay(100);
    off();
    delay(100);
    count++;
  }
}



//Code for obstacle avoidance
void obstacleAvoidance() {
   if(!leftBar.isBarried() && !rightBar.isBarried()) {
    forward();
  } else if(leftBar.isBarried() && rightBar.isBarried()) {
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
  if(!leftLine.onLine()) {
    rotateRight();
  } else if(!rightLine.onLine()) {
    rotateLeft();
  } else {
    forward();
  }
}

bool pressedThreeTimes() {
  if(rightCol.isCollision()) {
    int currentTime = millis();
    int count = 1;
    while(millis() - currentTime < 500) {
      if(rightCol.isCollision() && millis() - currentTime > 200) {
        count++;
        currentTime = millis();
        if(count == 3) {
          return true;
        }
      }
    }

    return false;
  } else {
    return false;
  }
}

bool swipeLeft() {
  if(leftBar.isBarried()) {
    int currentTime = millis();
    while(millis() - currentTime < 150) {
      if(rightBar.isBarried()) {
        return true;
      }
    }
  }

  return false;
}

bool swipeRight() {
  if(rightBar.isBarried()) {
    int currentTime = millis();
    while(millis() - currentTime < 250) {
      if(leftBar.isBarried()) {
        return true;
      }
    }
  }

  return false;
}

bool checkSequence() {
  int count = 0;
  char sequence[4];

  while(count != 4) {
    if(swipeLeft()) {
      sequence[count] = 'l';
      count++;
    } else if(swipeRight()) {
      sequence[count] = 'r';
      count++;
    }
  }

  for(int i = 0; i < sizeof(sequence); i++) {
    if(sequence[i] != correctSequence[i]) {
      return false;
    }
  }

  return true;
}

void setup() {

}

void loop() {

  while(!pressedThreeTimes()) {
    lineFollower();
    purple();
  }

  while(!swipeLeft() && !swipeRight()) {
    obstacleAvoidance();
    yellow();
  }
  
  stop();
  white();
  delay(3000);
  realRobot();
  realRobot();
  realRobot();
  realRobot();
  realRobot();
  realRobot();
  realRobot();
  realRobot();
  realRobot();
  realRobot();
  realRobot();
  realRobot();
  realRobot();
  realRobot();
  realRobot();
  realRobot();
  realRobot();
  realRobot();
  off();
  delay(3000);

  bool correct = false;
  while(!correct) {
    white();
    if(checkSequence()) {
      correct = true;
      flashSuccess();
    } else {
      flashFail();
    }
  }


  while(!rightCol.isCollision()) {
    green();
  }
  
  off();
  delay(1000);

  while(!rightCol.isCollision()) {
    white();
  }

  off();
  delay(1000);

  while(!rightCol.isCollision()) {
    purple();
    lineFollower();
  }


}
