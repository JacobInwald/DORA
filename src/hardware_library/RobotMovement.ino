#include "RobotMovement.h"
#include "SDPArduino.h" // Include the header where motor control functions are defined
#include <Wire.h>

RobotMovement::RobotMovement(int motorLeft, int motorRight) {
  _motorLeft = motorLeft;
  _motorRight = motorRight;
  SDPsetup(); // Initialize motor setup if needed
}

void RobotMovement::forward(int speed) {
  setMotorSpeed(_motorLeft, speed, true);
  setMotorSpeed(_motorRight, speed, true);
}

void RobotMovement::backward(int speed) {
  setMotorSpeed(_motorLeft, speed, false);
  setMotorSpeed(_motorRight, speed, false);
}

void RobotMovement::stop() {
  motorStop(_motorLeft);
  motorStop(_motorRight);
}

void RobotMovement::turnLeftOnSpot(int speed) {
  motorBackward(_motorLeft, speed);
  motorForward(_motorRight, speed);
}

void RobotMovement::turnRightOnSpot(int speed) {
  motorForward(_motorLeft, speed);
  motorBackward(_motorRight, speed);
}

void RobotMovement::pivotLeft(int speed) {
  motorStop(_motorLeft);
  motorForward(_motorRight, speed);
}

void RobotMovement::pivotRight(int speed) {
  motorForward(_motorLeft, speed);
  motorStop(_motorRight);
}

void RobotMovement::setMotorSpeed(int motor, int speed, bool forward) {
  if (forward) {
    motorForward(motor, speed);
  } else {
    motorBackward(motor, speed);
  }
}
