#include <Arduino.h>
#include <DualMC33926MotorShield.h>
#include <RobotBase.h>
#include <Encoder.h>
#include <SharpIR.h>
#include <Pins.h>

// (Sensor Pin, No. Samples, % Range, Model)
SharpIR irR(IR_F, 40, 93, 20150); 
SharpIR irFR(IR_F, 40, 93, 1080);
SharpIR irF(IR_F, 40, 93, 1080);
SharpIR irFL(IR_F, 40, 93, 1080);
SharpIR irL(IR_F, 40, 93, 20150);

int irRDist;
int irFRDist;
int irFDist;
int irFLDist;
int irLDist;

unsigned long basePeriod = 1;
unsigned long lastBase;
unsigned long curMillis;

//Navigation target distance 
#define DIST 2.0

#define BASE_P 20
#define BASE_I 5
#define BASE_D 0

#define TURN_THETA 0.1

/*
  There are three known arena sizes
  1 - official arena size 7'x12'
  2 - contigent arena size ?x?
  3 - home arena size 4'x8'
*/
#define ARENA 3

#if ARENA == 1
  #define MAX_X 
  #define MAX_Y 
  #define MIN_X 
  #define MIN_Y 
#endif

#if ARENA == 2
  #define MAX_X 
  #define MAX_Y 
  #define MIN_X 
  #define MIN_Y 
#endif

#if ARENA == 3
  #define MAX_X 50  
  #define MAX_Y 230
  #define MIN_X -50
  #define MIN_Y 0
#endif

Encoder m1Enc(m1EncA, m1EncB); 
Encoder m2Enc(m2EncB, m2EncA); // reversed so forward counts up

DualMC33926MotorShield md;

//x, y pairs. X is forward and 0 degree heading
double wayPts[4][2] = {
  {100.0,   0.0},
  {100.0, 100.0},
  {0.0,   100.0},
  {0.0,     0.0}
};

// x, y positions of cans
double canPts[10][2];
int canCnt = 0;

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
  RobotBase.setOutput(350, 10, 50);

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
    double dX = wayPts[curPt][1] - x, dY = wayPts[curPt][2] - y;
    double dist = hypot(dX, dY);

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



