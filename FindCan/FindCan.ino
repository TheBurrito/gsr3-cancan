#include <Arduino.h>
#include <RobotBase.h>
#include <SharpIR.h>
#include <TimedAction.h>
#include <Wire.h>
#include <LiquidTWI2.h>
#include <Adafruit_RGBLCDShield.h>
#include <Servo.h>
#include <Pins.h>

#define DEBUG_USE_LCD true
#define DEBUG_USE_SERIAL false
#define DEBUG_IR false

int irSamples = 10;
int irThresh = 93;
// (Sensor Pin, No. Samples, % Range Threshold, Model)
SharpIR irL(IR_L, irSamples, irThresh, 20150);
SharpIR irFL(IR_FL, irSamples, irThresh, 1080);
SharpIR irF(IR_F, irSamples, irThresh, 1080);
SharpIR irFR(IR_FR, irSamples, irThresh, 1080);
SharpIR irR(IR_R, irSamples, irThresh, 20150); 

volatile int irLDist;
volatile int irFLDist;
volatile int irFDist;
volatile int irFRDist;
volatile int irRDist;

Servo servoG; // define the Gripper Servo
#define SERVO_G_CLOSE 36
#define SERVO_G_OPEN  88
int i;

typedef enum {
  IRL,
  IRFL,
  IRFR,
  IRR,

  IR_END
} 
IR_Index;

struct Point {
  double x;
  double y;
};

struct ObjInfo {
  Point start;
  Point mid;
  double width;

  int lastDist;
  bool active;
};

ObjInfo obj[IR_END];


TimedAction irAction = TimedAction(50,readIrSensors);  // Scan IR Sensors every 800ms
TimedAction debugIrAction = TimedAction(1000,debugIr);

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
#define MAX_X 200  
#define MAX_Y 40
#define MIN_X -40
#define MIN_Y 0
#endif

// x, y positions of cans
double canPts[6][2] = {
  {
    0,0      }
  ,
  {
    0,0      }
  ,
  {
    0,0      }
  ,
  {
    0,0      }
  ,
  {
    0,0      }
  ,
  {
    0,0      }  
};
double FLPts[2][2] = {
  {
    0,0      }
  ,
  {
    0,0      }  
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

bool goBot = false;  // used to wait for keypad press to start

void setup() {

  //Establish PID gains
  RobotBase.setPID(10, 4, 0);

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
  RobotBase.setOdomPeriod(10);
  RobotBase.setNavPeriod(10);


  Serial.begin(115200);
  lcdInit();
  servoG.attach(SERVO_G);  // init gripper servo
  servoG.write(SERVO_G_OPEN);  

  RobotBase.driveTo(200, 0);
  RobotBase.time();
}

void loop() {
  while (!goBot) {
    irAction.check();
    debugIrAction.check();
    uint8_t buttons = lcd.readButtons();
    if (buttons && BUTTON_UP) {
      lcd.clear();
      lcd.setBacklight(GREEN);
      goBot = !goBot;  // stop everything if any button is pressed
      delay(500);
    }
  }

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
      RobotBase.setMax(20, 2.0); //cm/s, Rad/s
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
    if (RobotBase.navDone()) {
      for ( i = SERVO_G_OPEN; i > SERVO_G_CLOSE; --i) {
        servoG.write(i);
        delay(20);
      }
      RobotBase.turnTo(MAX_X, 0);
      mode = mTurnGoal;
    }
    break;

  case mTurnGoal:
    if (RobotBase.navDone()) {
      RobotBase.setMax(60, 2.0); //cm/s, Rad/s
      RobotBase.driveTo(MAX_X, 0);
      mode = mDriveGoal;
    }
    break;

  case mDriveGoal:
    if (RobotBase.navDone()) {

      mode = mDropCan;
    }
    break;

  case mDropCan:
    if (RobotBase.navDone()) {
      for ( i = SERVO_G_CLOSE; i < SERVO_G_OPEN; ++i) {
        servoG.write(i);
        delay(20);
      }
      RobotBase.driveTo(0, MAX_Y / 2.0);
      mode = mWander;
    }
    break;

  case mStop:
    RobotBase.stop();
    break;
  }  
}

void detectCan(float lastDist, float curDist) {
  float objX, objY;
  int objDistThresh = 15;
  int objMinWidth = 1;
  int objMaxWidth = 14;

  if (irFLDist > 10 && irFLDist < 80) {  // ignore values outside the sensors specs
    // calculate detected object x,y position
    objX = irFLDist * (0.573576436) + RobotBase.getX();
    objY = irFLDist * (0.819152044) + RobotBase.getY();

    if (lastDist - curDist > objDistThresh) {  // we found something
      FLPts[0][0] = objX;
      FLPts[0][1] = objY;
      FLPts[1][0] = objX;
      FLPts[1][1] = objY;
      left = true;
    } 
    else if (left && fabs(curDist - lastDist) > objDistThresh) {
      left = false;
      double dX = FLPts[0][0] - FLPts[1][0], dY = FLPts[0][1] - FLPts[1][1];
      double width = hypot(dX, dY);

      if (width > objMinWidth && width < objMaxWidth){  // I think it's a can
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
      FLPts[1][0] = objX;
      FLPts[1][1] = objY;
    }


  }  
}
void readIrSensors() {
  float lastLDist, lastFLDist, lastFDist, lastFRDist, lastRDist;

  lastLDist = irLDist;
  irLDist  = irL.distance();

  lastFLDist = irFLDist;
  irFLDist = irFL.distance();
  detectCan(lastFLDist,irFLDist);

  lastFDist = irFDist;
  irFDist  = irF.distance();

  lastFRDist = irFRDist;
  irFRDist = irFR.distance();

  lastRDist = irRDist;
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
  lcd.print("   ");
  lcd.print(irFLDist);
  lcd.print("  ");
  lcd.print(irFDist);
  lcd.print("  ");
  lcd.print(irFRDist);

  lcd.setCursor(0,1);
  lcd.print(irLDist);
  lcd.print(" <--  --> ");
  lcd.print(irRDist);
#endif
#if DEBUG_USE_SERIAL
  Serial.print("Left       : ");
  Serial.println(irLDist);
  Serial.print("Front Left : ");  
  Serial.println(irFLDist);
  Serial.print("Front      : ");  
  Serial.println(irFDist);
  Serial.print("Front Right: ");  
  Serial.println(irFRDist);
  Serial.print("Right      : ");  
  Serial.println(irRDist);
#endif
}















