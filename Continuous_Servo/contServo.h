#ifndef CONTSERVO_H
#define CONTSERVO_H

#include <Servo.h>
#include "encoder.h"

class contServo { 
  private:
    int servoPin; 
    int defaultSpd;
    int tolerance;
    int servoSpd;
    int rotateSpd;
    int totalSpd;
    // int driveDirection;

    Servo servo;
    Encoder& encoder;

  public:
    contServo(int pin, Encoder& encoder);
    void initialize();
    void setSpeed(int speed);
    void stop();
    void setZero();
    void goToAngle(int angle);
    int closestAngle(int target, int current);

};

#endif
