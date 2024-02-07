#include "RobotMovement.h"
#include "SDPArduino.h"
#include <Wire.h>

// Initialize the RobotMovement class with motors 0 and 1

RobotMovement robot(0, 1);

int i = 0;

void setup(){
  SDPsetup();
  helloWorld();
}

void loop(){
  // Move forward at 50% speed
  robot.forward(50);
  delay(2000);
  // Move backward at 50% speed
  robot.backward(50);
  delay(2000);
  // Turn left on the spot
  robot.turnLeftOnSpot(100);
  delay(2000);
  // Turn right on the spot
  robot.turnRightOnSpot(100);
  delay(4000);
  // Move forward at 50% speed
  robot.forward(100);
  delay(2000);
  // Move backward at 50% speed
  robot.backward(100);
  delay(2000);
  // Turn right on the spot
  robot.turnRightOnSpot(100);
    delay(2000);
  robot.forward(100);
  delay(2000);
  robot.stop();
  delay(2000);
}
