#include <Arduino.h>
#include <DualMC33926MotorShield.h>
#include <RobotBase.h>
#include <Encoder.h>
#include <SharpIR.h>
#include <TimedAction.h>
#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>
#include <Pins.h>

#define DEBUG_USE_LCD true
#define DEBUG_USE_SERIAL false
#define DEBUG_IR false

int irSamples = 10;
// (Sensor Pin, No. Samples, % Range, Model)
SharpIR irL(IR_L, irSamples, 93, 20150);
SharpIR irFL(IR_FL, irSamples, 93, 1080);
SharpIR irF(IR_F, irSamples, 93, 1080);
SharpIR irFR(IR_FR, irSamples, 93, 1080);
SharpIR irR(IR_R, irSamples, 93, 20150); 

int irRDist;
int irFRDist;
int irFDist;
int irFLDist;
int irLDist;

TimedAction irAction = TimedAction(500,readIrSensors);  // Scan IR Sensors every 800ms
#if DEBUG_IR
  TimedAction debugIrAction = TimedAction(1000,debugIr);
#endif

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// These #defines make it easy to set the backlight color
#define OFF 0x0
#define RED 0x1
#define GREEN 0x2
#define YELLOW 0x3
#define BLUE 0x4
#define VIOLET 0x5
#define TEAL 0x6
#define WHITE 0x7

unsigned long basePeriod = 20;
unsigned long lastBase;
unsigned long curMillis;

//Navigation target distance 
#define DIST 2.0

#define BASE_P 20
#define BASE_I 5
#define BASE_D 0

#define TURN_THETA 0.15

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
  #define MAX_X 40  
  #define MAX_Y 230
  #define MIN_X -40
  #define MIN_Y 0
#endif

Encoder m1Enc(m1EncA, m1EncB); 
Encoder m2Enc(m2EncB, m2EncA); // reversed so forward counts up

DualMC33926MotorShield md;

//x, y pairs. X is forward and 0 degree heading
double wayPts[4][2] = {
  {200.0,  0.0},
//  {50.0, 50.0},
//  { 0.0, 50.0},
  { 0.0,  0.0}
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
  RobotBase.setAccel(0.5);

  //Set max velocity and turn rate
  RobotBase.setMax(20, 3.14); //30cm/s, 0.2 Rad/s

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
  lcdInit();
  delay(2000);

  lastBase = millis();
}

void loop() {
  irAction.check();
  #if DEBUG_IR
    debugIrAction.check();
  #endif
  
  if (curPt < numPts) {
    curMillis = millis();

    double x = RobotBase.getX(), y = RobotBase.getY(), t = RobotBase.getTheta();
    double dX = wayPts[curPt][0] - x, dY = wayPts[curPt][1] - y;
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

void readIrSensors() {
  long _start = millis();
  float _objX,_objY,sensTheta;
  
  irFLDist = irFL.distance();
//  Serial.println(millis() - _start);
  if (irFLDist > 10 && irFLDist < 80) {  // ignore values outside the sensors specs
    // calculate detected objects x,y position
    //sensTheta = RobotBase.getTheta() - HALF_PI;
    _objX = irFLDist * (0.573576436) + RobotBase.getX();
    _objY = irFLDist * (0.819152044) + RobotBase.getY();
    canPts[canCnt][0] = _objX;
    canPts[canCnt][1] = _objY;
    wayPts[curPt][0] = _objX;
    wayPts[curPt][1] = _objY;
    ++canCnt;
   
  }
  /*
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(RobotBase.getX());
    lcd.print(" ");
    lcd.print(RobotBase.getY());
    lcd.setCursor(0,1);
    lcd.print(canPts[0][0]);
    lcd.print(" ");
    lcd.print(canPts[0][1]);
 */   
  irFRDist = irFR.distance();
//  Serial.println(millis() - _start);
  if (irFRDist > 10 && irFRDist < 80) {  // ignore values outside the sensors specs
    _objX = irFRDist * (0.819152044) + RobotBase.getX();
    _objY = irFRDist * (0.573576436) + RobotBase.getY();
      
  }


  irFDist  = irF.distance();
//  Serial.println(millis() - _start);
//  if (irFDist > 10 && irFDist < 80) {  // ignore values outside the sensors specs
    
//  }


  irLDist  = irL.distance();
//  Serial.println(millis() - _start);

  irRDist  = irR.distance();
//  Serial.println(millis() - _start);
//  Serial.println("");    
  
}

void lcdInit() {

  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  lcd.setBacklight(WHITE);
}

void debugIr() {
  #if DEBUG_USE_LCD
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("");
    lcd.print(irFLDist);
    lcd.print(",");
    lcd.print(irFDist);
    lcd.print(",");
    lcd.print(irFRDist);
    
    lcd.print(canPts[canCnt][0]);
    lcd.print(" ");
    lcd.print(canPts[canCnt][1]);
 /*   
    lcd.setCursor(0,1);
    lcd.print("");
    lcd.print(irLDist);
    lcd.print(",");
    lcd.print(irRDist);
*/
  #endif
  #if DEBUG_USE_SERIAL
  Serial.print("Right      : ");  
  Serial.println(irRDist);
  Serial.print("Front Right: ");  
  Serial.println(irFRDist);
  Serial.print("Front      : ");  
  Serial.println(irFDist);
  Serial.print("Front Left : ");  
  Serial.println(irFLDist);
  Serial.print("Left       : ");
  Serial.println(irLDist);
  #endif
}
