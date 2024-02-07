#ifndef RobotMovement_h
#define RobotMovement_h

#include "SDPArduino.h"

class RobotMovement {
  public:
    RobotMovement(int motorLeft, int motorRight);
    void forward(int speed);
    void backward(int speed);
    void stop();
    void turnLeftOnSpot(int speed);
    void turnRightOnSpot(int speed);
    void pivotLeft(int speed);
    void pivotRight(int speed);

  private:
    int _motorLeft;
    int _motorRight;
    void setMotorSpeed(int motor, int speed, bool forward);
};

#endif
