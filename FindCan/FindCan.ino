#include <Arduino.h>
#include <RobotBase.h>
#include <SharpIR.h>
#include <TimedAction.h>
#include <Wire.h>
#include <LiquidTWI2.h>
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

typedef struct {
  double x;
  double y;
} Point;

typedef struct {
  Point start;
  Point mid;
  double width;
  
  int lastDist;
  bool active;
} ObjInfo;

typedef enum {
  IRL,
  IRFL,
  IRFR,
  IRR,
  
  IR_END
} IR_Index;

ObjInfo obj[IR_END];

TimedAction irAction = TimedAction(100,readIrSensors);  // Scan IR Sensors every 800ms
#if DEBUG_IR
TimedAction debugIrAction = TimedAction(1000,debugIr);
#endif

LiquidTWI2 lcd(0);

// These #defines make it easy to set the backlight color
#define OFF 0x0
#define RED 0x1
#define GREEN 0x2
#define YELLOW 0x3
#define BLUE 0x4
#define VIOLET 0x5
#define TEAL 0x6
#define WHITE 0x7

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

// x, y positions of cans
double canPts[6][2] = {
  {
    0,0  }
  ,
  {
    0,0  }
  ,
  {
    0,0  }
  ,
  {
    0,0  }
  ,
  {
    0,0  }
  ,
  {
    0,0  }  
};
double FLPts[2][2] = {
  {
    0,0  }
  ,
  {
    0,0  }  
};

int canCnt = 0;

typedef enum {
  mWander,
  mTurnCan,
  mDriveCan,
  mGrabCan,
  mTurnGoal,
  mDriveGoal,
  mDropCan,
  mStop
} 
Mode;

Mode mode = mWander;

bool left = false;

void setup() {
  //Establish PID gains
  RobotBase.setPID(15, 5, 0);

  //Set allowed accel for wheel velocity targets (cm/s/s)
  RobotBase.setAccel(50);

  //Set max velocity and turn rate
  RobotBase.setMax(20, 2.0); //cm/s, Rad/s

  //set motor output ranges - works both positive and negative
  //Max, dead zone, min
  // -deadZone > X < deadZone : X = 0
  // X < min : X = min
  // X > max : x = max
  RobotBase.setOutputRange(300, 5, 50);

  //set ticks per desired distance unit
  RobotBase.setTicksPerUnit(71.65267); //units of cm

  //set the wheelbase of in desired distance unit
  RobotBase.setWidth(17.4); //units of cm

  RobotBase.setNavThresh(2, 0.035);
  RobotBase.setOdomPeriod(1);
  RobotBase.setNavPeriod(1);


  Serial.begin(115200);
  lcdInit();
  delay(2000);

  RobotBase.driveTo(200, 0);
  RobotBase.time();
}

void loop() {
  irAction.check();
#if DEBUG_IR
  debugIrAction.check();
#endif

  RobotBase.update();

  switch (mode) {
  case mWander:
    if (RobotBase.navDone()) {
      mode = mStop;
    } 
    else if (canPts[0][0] != 0) {
      RobotBase.turnTo(canPts[0][0], canPts[0][1]);
      mode = mTurnCan;
    }
    break;

  case mTurnCan:
    if (RobotBase.navDone()) {
      RobotBase.driveTo(canPts[0][0], canPts[0][1]);
      mode = mDriveCan;
    }
    break;

  case mDriveCan:
    if (RobotBase.navDone()) {
      mode = mGrabCan;
    }
    break;


  case mGrabCan:

    RobotBase.turnTo(0, MAX_Y);
    mode = mTurnGoal;

    break;

  case mTurnGoal:
    if (RobotBase.navDone()) {
      RobotBase.driveTo(0, MAX_Y);
      mode = mDriveGoal;
    }
    break;

  case mDriveGoal:
    if (RobotBase.navDone()) {
      mode = mDropCan;
    }
    break;

  case mDropCan:
    RobotBase.driveTo(0, MAX_Y / 2.0);
    mode = mWander;
    break;

  case mStop:
    RobotBase.stop();
    break;
  }  
}

void readIrSensors() {
  long _start = millis();
  float xL,  yL
    ,xFL, yFL
    ,xF,  yF
    ,xFR, yFR
    ,xR,   yR
    ,lastFLDist;
  int objDistThresh = 10;
  int objMinWidth = 3;
  int objMaxWidth = 14;


  lastFLDist = irFLDist;
  irFLDist = irFL.distance();
  if (irFLDist > 10 && irFLDist < 80) {  // ignore values outside the sensors specs
    // calculate detected objects x,y position

    xFL = irFLDist * (0.573576436) + RobotBase.getX();
    yFL = irFLDist * (0.819152044) + RobotBase.getY();

    if (lastFLDist - irFLDist > objDistThresh) {
      FLPts[0][0] = xFL;
      FLPts[0][1] = yFL;
      left = true;
    } 
    else if (left && irFLDist - lastFLDist > objDistThresh) {
      left = false;
      double dX = FLPts[0][0] - FLPts[1][0], dY = FLPts[0][1] - FLPts[1][1];
      double width = hypot(dX, dY);

      if (width > objMinWidth && width < objMaxWidth){
        canPts[canCnt][0] = (FLPts[0][0] + FLPts[1][0]) / 2;
        canPts[canCnt][1] = (FLPts[0][1] + FLPts[1][1]) / 2;
        FLPts[0][0] = 0;
        FLPts[0][1] = 0;
        FLPts[1][0] = 0;
        FLPts[1][1] = 0;
        ++canCnt;
      }
    } 
    else if (left) {
      FLPts[1][0] = xFL;
      FLPts[1][1] = yFL;
    }

    /*if (xFL > MIN_X || xFL < MAX_X) {
      if (FLPts[0][0] == 0) {
        FLPts[0][0] = xFL;
        FLPts[0][1] = yFL;
      }
      else {
        if (irFLDist < lastFLDist + objDistThresh && irFLDist > lastFLDist - objDistThresh) {
          FLPts[1][0] = xFL;
          FLPts[1][1] = yFL;

        }
        else if (irFLDist > lastFLDist + objDistThresh) {
          double dX = FLPts[0][0] - FLPts[1][0], dY = FLPts[0][1] - FLPts[1][1];
          double width = hypot(dX, dY);

          if (width > objMinWidth && width < objMaxWidth){
            canPts[canCnt][0] = (FLPts[0][0] + FLPts[1][0]) / 2;
            canPts[canCnt][1] = (FLPts[0][1] + FLPts[1][1]) / 2;
            FLPts[0][0] = 0;
            FLPts[0][1] = 0;
            FLPts[1][0] = 0;
            FLPts[1][1] = 0;
            ++canCnt;
          }
          else {

          }         
        }
        else if (irFLDist < lastFLDist - objDistThresh) {
          FLPts[0][0] = xFL;
          FLPts[0][1] = yFL;
        }
      }
    }*/


  }

  /*lcd.clear();
   lcd.setCursor(0,0);
   lcd.print(RobotBase.getX());
   lcd.print(" ");
   lcd.print(RobotBase.getY());
   
   lcd.setCursor(0,1);
   lcd.print(canPts[0][0]);
   lcd.print(" ");
   lcd.print(canPts[0][1]);*/

  irFRDist = irFR.distance();

  irFDist  = irF.distance();

  irLDist  = irL.distance();

  irRDist  = irR.distance();

}

void lcdInit() {
  lcd.setMCPType(LTI_TYPE_MCP23017);
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

  //   lcd.print(canPts[canCnt][0]);
  //    lcd.print(" ");
  //    lcd.print(canPts[canCnt][1]);
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

