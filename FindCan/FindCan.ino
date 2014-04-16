#include <Arduino.h>
#include <DualMC33926MotorShield.h>
#include <RobotBase.h>
#include <Encoder.h>
#include <Pins.h>

unsigned long basePeriod = 1;
unsigned long lastBase;
unsigned long curMillis;

//Navigation target distance 
#define DIST 2.0

#define BASE_P 20
#define BASE_I 5
#define BASE_D 0

#define TURN_THETA 0.1

Encoder m1Enc(m1EncA, m1EncB); 
Encoder m2Enc(m2EncB, m2EncA); // reversed so forward counts up

DualMC33926MotorShield md;

//x, y pairs. X is forward and 0 degree heading
double wayPts[] = {
  100.0,   0.0,
  100.0, 100.0,
  0.0,   100.0,
  0.0,     0.0
};

double turnThresh = 0.05;

//Number of coordinate pairs
int numPts = 4;
int curPt;

void setup() {
  md.init();

  //Establish PID gains
  RobotBase.setPID(BASE_P, BASE_I, BASE_D);

  //Set allowed accel for wheel velocity targets (cm/s/s)
  RobotBase.setAccel(0.1);

  //Set max velocity and turn rate
  RobotBase.setMax(50, 1.0); //30cm/s, 0.2 Rad/s

  //set motor output ranges - works both positive and negative
  //Max, dead zone, min
  // -deadZone > X < deadZone : X = 0
  // X < min : X = min
  // X > max : x = max
  RobotBase.setOutput(400, 10, 50);

  //set ticks per desired distance unit
  RobotBase.setTicksPerUnit(71.65267); //units of cm

  //set the wheelbase of in desired distance unit
  RobotBase.setWidth(17.4); //units of cm

  Serial.begin(115200);

  curPt = 0;

  delay(2000);

  lastBase = millis();
}

void loop() {
  if (curPt < numPts) {
    curMillis = millis();

    double x = RobotBase.getX(), y = RobotBase.getY(), t = RobotBase.getTheta();
    double dX = wayPts[curPt * 2] - x, dY = wayPts[curPt * 2 + 1] - y;
    double dist = hypot(dX, dY);

    /*Serial.print("D");
     Serial.print(dist);
     Serial.print("/");
     Serial.println(DIST);*/

    if (dist < DIST) {
      curPt++;
      RobotBase.reset();
      turnThresh = 0.05;
    } 
    else {
      long encL = m1Enc.read();
      long encR = m2Enc.read();

      double dTheta = atan2(dY, dX) - t;
      if (dTheta > PI) {
        dTheta -= TWO_PI;
      } 
      else if (dTheta < -PI) {
        dTheta += TWO_PI;
      }

      if (curMillis - lastBase >= basePeriod) {
        lastBase = curMillis;

        double targetVel = dist;
        double targetTurn = dTheta;

        if (abs(targetTurn) > TURN_THETA) {
          targetVel = 0;
        } 
        else {
          double tR = (PI - abs(targetTurn)) / PI;
          targetVel *= tR * tR;
          turnThresh = 0.3;
        }

        /*Serial.print(curPt);
         Serial.print(": ");
         
         Serial.print("d");
         Serial.print(dist);
         Serial.print(" v");
         Serial.print(targetVel);
         Serial.print(" t");
         Serial.print(t);
         Serial.print(" a");
         Serial.println(targetTurn);*/

        RobotBase.update(encL, encR, targetVel, targetTurn, 0.001 * basePeriod);

        md.setM1Speed(RobotBase.getLeftOut());
        md.setM2Speed(RobotBase.getRightOut());
      }
    }
  } 
  else {
    md.setM1Speed(0);
    md.setM2Speed(0);

    delay(100);
  }
}



