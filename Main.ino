#include "contServo.h"
#include "encoder.h"
#include "spark.h"
#include <math.h>

//Function Prototypes
void setSparkSpeed(Spark& spark, bool reverse, int speedOffset);
void updateWheelPos(int ang, float spd);
void setRotation(int setting, int speedOffset);
int closestAngle(int target, int current);
bool getCameraData(String data);

Encoder encoderA(A1);
contServo servoA(3, encoderA);
Spark sparkA(7);
Encoder encoderB(A2);
contServo servoB(4, encoderB);
Spark sparkB(8);
Encoder encoderC(A3);
contServo servoC(5, encoderC);
Spark sparkC(9);
Encoder encoderD(A4);
contServo servoD(6, encoderD);
Spark sparkD(10);

const int MOD_1_OFFSET = 290;
const int MOD_2_OFFSET = 47;
const int MOD_3_OFFSET = 60;
const int MOD_4_OFFSET = 200;

int targetAngle1 = MOD_1_OFFSET;
int targetAngle2 = MOD_2_OFFSET;
int targetAngle3 = MOD_3_OFFSET;
int targetAngle4 = MOD_4_OFFSET;

int defaultSpd = 1500;
int inc = 500; //Base increment

float dist_setpoint = 0.2;  //0.2m away from object
float ang_setpoint  = 0.0;  //0.0 degrees (center)
float angle_tolerance = 5.0;
float dist_tolerance  = 0.05;

float Kp_ang  = 0.8;
float Kp_dist = 200.0;

float camera_dist = 0.0;
float camera_ang  = 0.0;
float camera_strafe = 0.0;

void setup() {
  Serial.begin(500000);

  servoA.initialize(); servoB.initialize();
  servoC.initialize(); servoD.initialize();

  sparkA.initialize(); sparkB.initialize();
  sparkC.initialize(); sparkD.initialize();

  Serial.println("Starting Simple Control");
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');

    bool detectObj = getCameraData(data);

    if (detectObj && !(camera_ang == 0.0 && camera_dist == 0.0)) { //testing only for rotation right now
      float ang_error  = camera_ang - ang_setpoint;
      float dist_error = camera_dist - dist_setpoint;
      
      if(fabs(camera_ang) <= angle_tolerance){
        setSparkSpeed(sparkA, false, 0);
        setSparkSpeed(sparkB, false, 0);
        setSparkSpeed(sparkC, false, 0);
        setSparkSpeed(sparkD, false, 0);
      }

      else if (fabs(ang_error) > angle_tolerance) {
        int rot_speed = (int)(Kp_ang * ang_error);
        rot_speed = constrain(rot_speed, -inc, inc);

        int mag = abs(rot_speed);     
        if (mag < 200) mag = 200;      

        if (ang_error > 0) {
          setRotation(-1,  mag);        //CCW
        } 
        else {
          setRotation(0, mag);        //CW
        }
      } 
      else { //stop motors at current alignment
        setSparkSpeed(sparkA, false, 0);
        setSparkSpeed(sparkB, false, 0);
        setSparkSpeed(sparkC, false, 0);
        setSparkSpeed(sparkD, false, 0);
      }

    } 
    else { //no data/not valid
      setRotation(0, 0);
    }
  }
  
  servoA.goToAngle(targetAngle1);
  servoB.goToAngle(targetAngle2);
  servoC.goToAngle(targetAngle3);
  servoD.goToAngle(targetAngle4);
}

bool getCameraData(String data) { //(angle, distance, strafe)
  int comma1 = data.indexOf(",");
  if (comma1 == -1) return false;

  int comma2 = data.indexOf(",", comma1 + 1);
  if (comma2 == -1) return false;

  String ang_str    = data.substring(0, comma1);
  String dist_str   = data.substring(comma1 + 1, comma2);
  String strafe_str = data.substring(comma2 + 1);

  if (ang_str == "" || dist_str == "" || strafe_str == "") return false;

  camera_ang   = ang_str.toFloat();
  camera_dist  = dist_str.toFloat();
  camera_strafe = strafe_str.toFloat();
  return true;
}

void setRotation(int setting, int speedOffset) {
  int ang1, ang2, ang3, ang4;

  if (setting == -1) {  //clockwise
    ang1 = 315; ang2 = 45; ang3 = 135; ang4 = 225;
  } 
  else {              //counter-clockwise
    ang1 = 135; ang2 = 225; ang3 = 315; ang4 = 45;
  }

  targetAngle1 = (ang1 + MOD_1_OFFSET) % 360;
  targetAngle2 = (ang2 + MOD_2_OFFSET) % 360;
  targetAngle4 = (ang4 + MOD_4_OFFSET) % 360;

  int steeringTarget = ang3;
  if (steeringTarget >= 0 && steeringTarget < 90) {
      //map(135, 0, 90, 60, 140) --> 45 equivalent to 100 since halfway
      targetAngle3 = map(steeringTarget, 0, 90, 60, 140);
  } 
  else if (steeringTarget >= 90 && steeringTarget < 180) {
      targetAngle3 = map(steeringTarget, 90, 180, 140, 220);
  } 
  else if (steeringTarget >= 180 && steeringTarget < 270) {
      //map(335, 180, 270, 220, 300) -> 225 equivalent to 260 since halfway
      targetAngle3 = map(steeringTarget, 180, 270, 220, 300);
  } 
  else {
      targetAngle3 = map(steeringTarget, 270, 360, 300, 420) % 360;
  }

  int currAng1 = encoderA.readAngle();
  int currAng2 = encoderB.readAngle();
  int currAng3 = encoderC.readAngle();
  int currAng4 = encoderD.readAngle();

  const int flipThresh = 90;
  bool rev1 = false;
  bool rev2 = false;
  bool rev3 = false;
  bool rev4 = false;

  int clos1 = closestAngle(targetAngle1, currAng1);
  int clos2 = closestAngle(targetAngle2, currAng2);
  int clos3 = closestAngle(targetAngle3, currAng3);
  int clos4 = closestAngle(targetAngle4, currAng4);
  
  if(abs(clos1) > flipThresh){
    targetAngle1 = (targetAngle1 + 180) % 360;
    rev1 = true;
  }
  if(abs(clos2) >= flipThresh){
    targetAngle2 = (targetAngle2 + 180) % 360;
    rev2 = true;
  }
  if(abs(clos3) >= flipThresh){
    targetAngle3 = (targetAngle3 + 180) % 360;
    rev3 = true;
  }
  if(abs(clos4) >= flipThresh){
    targetAngle4 = (targetAngle4 + 180) % 360;
    rev4 = true;
  }

  //Bias for module 4
  int bias = 0;
  if(setting != -1){
    bias += 100;
  }

  setSparkSpeed(sparkA, rev1, speedOffset + bias);
  setSparkSpeed(sparkB, rev2, speedOffset);
  setSparkSpeed(sparkC, rev3, speedOffset);
  setSparkSpeed(sparkD, rev4, speedOffset + bias);
}

void setSparkSpeed(Spark &spark, bool reverse, int speedOffset){
  if (reverse) spark.setSpeed(defaultSpd - speedOffset);
  else         spark.setSpeed(defaultSpd + speedOffset);
}

int closestAngle(int target, int current) {
  return (target - current + 540) % 360 - 180;
}
