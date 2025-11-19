#include "contServo.h"
#include "Arduino.h" 
#include "math.h"

contServo::contServo(int pin, Encoder& encoder) : servoPin(pin), encoder(encoder), defaultSpd(1484), tolerance(5), servoSpd(defaultSpd), rotateSpd(300), totalSpd(1500) {
}

void contServo::setSpeed(int speed) {
  servo.writeMicroseconds(speed);
}

void contServo::initialize() {
  servo.attach(servoPin);
  setSpeed(defaultSpd); 
}

void contServo::stop() {
  setSpeed(defaultSpd); 
}

void contServo::setZero() {
  goToAngle(0);
}

int contServo::closestAngle(int target, int currentAngle) {
  return (target - currentAngle + 540) % 360 - 180;
}
    
void contServo::goToAngle(int angle) {
  int error = closestAngle(angle, encoder.readAngle());

  if (abs(error) <= tolerance) {
    stop();
    return;
  }

  if (error > 0) {
    totalSpd = defaultSpd - rotateSpd;
  } else {
    totalSpd = defaultSpd + rotateSpd;
  }

  setSpeed(totalSpd);
}
