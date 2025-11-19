#ifndef ENCODER_H
#define ENCODER_H
#include "spark.h"

class Encoder {
  private:
    int encoderPin;
    
    int currentAngle;

  public:
    Encoder(int pin);
    int readAngle();

};

#endif
