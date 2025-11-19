###STILL NEEDS MAJOR REWORKING

#include "spark.h"
#include "contServo.h"
#include "encoder.h"

bool isStopped = false;

contServo servoA(3); //module 1
contServo servoB(4); //module 2
contServo servoC(5);  //module 3
contServo servoD(6); //module 4

Encoder encoderA(A0, servoA); //module 1
Encoder encoderB(A1, servoB);
Encoder encoderC(A2, servoC);
Encoder encoderD(A3, servoD);

Spark sparkA(7); 
Spark sparkB(8);
Spark sparkC(9);
Spark sparkD(10);

float Vx = 0;    // X translation from joystick
float Vy = 0;    // Y translation from joystick
float omega = 0; // Rotational velocity (omega)

float L = 19;  // Length of robot in inches
float W = 19;  // Width of robot in inches

float lerpVal = 0.35;

float lerp(float startVal, float stopVal, float lerpVal) { //lerp to reduce "teleporting"
    return startVal + lerpVal * (stopVal - startVal);
}

void setup() {
    Serial.begin(115200); // Set baud rate

    sparkA.initialize();
    sparkB.initialize();
    sparkC.initialize();
    sparkD.initialize();
    
    encoderA.goToDefaultPos();
    encoderB.goToDefaultPos();
    encoderC.goToDefaultPos();
    encoderD.goToDefaultPos();

    servoA.initialize();
    servoB.initialize();
    servoC.initialize();
    servoD.initialize();

}

void loop() {
    if (Serial.available() > 0) {
        String data = Serial.readStringUntil('\n'); // Read incoming command
        commands(data);
    }
    
    if(isStopped){
      delay(10);
      return;
    }

    encoderA.readAngle();
    encoderB.readAngle();
    encoderC.readAngle();
    encoderD.readAngle();

    encoderA.setPID(encoderA.getError());
    encoderB.setPID(encoderA.getError());
    encoderC.setPID(encoderA.getError());
    encoderD.setPID(encoderA.getError());
    
    //Wheel speeds calculation
    float A = Vx - omega * (L / 2);
    float B = Vx + omega * (L / 2);
    float C = Vy - omega * (W / 2);
    float D = Vy + omega * (W / 2);

    float wheelSpeed1 = sqrt(B*B + C*C);
    float wheelAngle1 = atan2(B, C) * 180/PI;

    float wheelSpeed2 = sqrt(B*B + D*D);
    float wheelAngle2 = atan2(B, D) * 180/PI;

    float wheelSpeed3 = sqrt(A*A + D*D);
    float wheelAngle3 = atan2(A, D) * 180/PI;

    float wheelSpeed4 = sqrt(A*A + C*C);
    float wheelAngle4 = atan2(A, C) * 180/PI;

    encoderA.setTargetAngle(wheelAngle1);
    encoderB.setTargetAngle(wheelAngle2);
    encoderC.setTargetAngle(wheelAngle3);
    encoderD.setTargetAngle(wheelAngle4);

    encoderA.setPID(encoderA.getError());
    encoderB.setPID(encoderB.getError());
    encoderC.setPID(encoderC.getError());
    encoderD.setPID(encoderD.getError());

    sparkA.setSpeed(constrain(wheelSpeed1, 1000, 2000));
    sparkB.setSpeed(constrain(wheelSpeed2, 1000, 2000));
    sparkC.setSpeed(constrain(wheelSpeed3, 1000, 2000));
    sparkD.setSpeed(constrain(wheelSpeed4, 1000, 2000));

    delay(10);  // Small delay to prevent overwhelming the servo
}

void commands(String data){
  if (data == "BUTTON_A") {
            Serial.println("A button pressed!");
            Serial.println("Reset speed and angle!");
            Serial.println(encoderA.readAngle() + "!");
            isStopped = true;

            sparkA.stopMotor();
            sparkB.stopMotor();
            sparkC.stopMotor();
            sparkD.stopMotor();

            encoderA.goToDefaultPos();
            encoderB.goToDefaultPos();
            encoderC.goToDefaultPos();
            encoderD.goToDefaultPos();

            encoderA.setPID(encoderA.getError());
            encoderB.setPID(encoderB.getError());
            encoderC.setPID(encoderC.getError());
            encoderD.setPID(encoderD.getError());
        }
        else if (data == "BUTTON_B") {
            Serial.println("B button pressed!");
            Serial.println("Resume spin");
            isStopped = false;
        }
        else if (data == "BUTTON_Y") {
            Serial.println("Y button pressed!");
        }
        else if (data == "BUTTON_X") {
            Serial.println("X button pressed!");
        }

        //Drive joystick
        else if (data.startsWith("stickLX:")) { 
            String value = data.substring(8);  // Get the value after "stickLX:"
            Serial.println("Left joystick X value: " + value);
            Vx = atoi(value.c_str()); 
        } 
        else if (data.startsWith("stickLY:")) {
            String value = data.substring(8);  // Get the value after "stickLY:"
            Serial.println("Left joystick Y value: " + value);
            Vy = atoi(value.c_str());
        }

        //Steer joystick
        else if (data.startsWith("stickRX:")) {
            String value = data.substring(8);  // Get the value after "stickRY:"
            Serial.println("Right joystick X value: " + value);
            omega = atoi(value.c_str());
        }
        else if (data.startsWith("stickRY:")) {
            String value = data.substring(8);  // Get the value after "stickRY:"
            Serial.println("Right joystick Y value: " + value);
        }

        //Flippers
        else if (data.startsWith("triggerR:")) {
            String value = data.substring(9);  // Get the value after "trigger_R:"
            Serial.println("Right Trigger value: " + value);
        }
        else if (data.startsWith("triggerL:")) {
            String value = data.substring(9);  // Get the value after "trigger_L:"
            Serial.println("Left Trigger value: " + value);
        }
}
