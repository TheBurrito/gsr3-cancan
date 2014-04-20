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

int irSamples = 1;
int irThresh = 90;
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
  IRF,
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
  Point last;
  double width;
  long ts;  // timestamp
  bool active;
};

ObjInfo obj[IR_END];



TimedAction irAction = TimedAction(20,readIrSensors);  // Scan IR Sensors every 800ms
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

// Distances from robot center to outer dimensions
#define ROBOT_GRIP_OFFSET 8.7  // gripper closed position
#define ROBOT_FRONT_OFFSET 7.3 // to front of case
#define ROBOT_REAR_OFFSET 10.7 // to rear of case
#define ROBOT_WHEEL_OFFSET 9.2 // to outside of wheel

// Sensor offsets from robot center
#define IR_L_OFFSET_X 0
#define IR_L_OFFSET_Y -8.0
#define IR_FL_OFFSET_X 4.1
#define IR_FL_OFFSET_Y -6.2
#define IR_F_OFFSET_X 7.3
#define IR_F_OFFSET_Y -1.2
#define IR_FR_OFFSET_X 5.7
#define IR_FR_OFFSET_Y 4.8
#define IR_L_OFFSET_X 0
#define IR_L_OFFSET_Y 8.1

#define WALL_BUFFER 10 // padding around walls to exclude from object detection
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
#define MAX_X 244 - WALL_BUFFER - ROBOT_FRONT_OFFSET - ROBOT_GRIP_OFFSET - 13
#define MAX_Y 61 - WALL_BUFFER - ROBOT_FRONT_OFFSET - ROBOT_GRIP_OFFSET
#define MIN_X 0 - ROBOT_FRONT_OFFSET + WALL_BUFFER + ROBOT_FRONT_OFFSET + ROBOT_GRIP_OFFSET
#define MIN_Y -61 + WALL_BUFFER + ROBOT_FRONT_OFFSET + ROBOT_GRIP_OFFSET
#endif

int grid[17568][2];

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

Mode mode = mWander, lastMode = mode;
// x, y positions of cans
double canPts[6][2] = {
  {
    0,0                    }
  ,
  {
    0,0                    }
  ,
  {
    0,0                    }
  ,
  {
    0,0                    }
  ,
  {
    0,0                    }
  ,
  {
    0,0                    }  
};

int canCnt = 0;

bool left = false;

bool goBot = false;  // used to wait for keypad press to start

bool newState = true;

void setup() {

  obj[IRFL].active = false;

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
  //  Serial1.begin(9600);
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
  
  newState = (mode != lastMode);

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


#define OBJ_MIN_WIDTH 3
#define OBJ_MAX_WIDTH 14

void detectCan(int sensor, float lastDist, float curDist) {
  float objX, objY;
  static int objDistThresh = 5;
  static int objTimeThresh = 300;

  if (curDist > 10 && curDist < 80) {  // ignore values outside the sensors specs
    // calculate detected object x,y position
    objX = curDist * (0.573576436) + RobotBase.getX();
    objY = curDist * (0.819152044) + RobotBase.getY();
    if (objX < MAX_X && objX > MIN_X && objY < MAX_Y && objY > MIN_Y) {
      Serial.print("Dist: ");
      Serial.println(curDist);

      if (lastDist - curDist > objDistThresh) {  // we found something
        obj[sensor].start.x = objX;
        obj[sensor].start.y = objY;
        obj[sensor].active = true;
        obj[sensor].ts = millis();
        Serial.println("");
        Serial.println("!! START !!");
        Serial.print("Pos: ");
        Serial.print(int(objX));
        Serial.print(",");
        Serial.println(int(objY));
        Serial.println(millis());
        Serial.println("");
      } 
      else if (obj[sensor].active && fabs(curDist - lastDist) > objDistThresh) {
        obj[sensor].last.x = objX;
        obj[sensor].last.y = objY;          
        double dX = obj[sensor].start.x - obj[sensor].last.x;
        double dY = obj[sensor].start.y - obj[sensor].last.y;
        obj[sensor].width = hypot(dX, dY);
        Serial.print("width: ");
        Serial.println(obj[sensor].width);
        if (obj[sensor].width > OBJ_MIN_WIDTH && obj[sensor].width < OBJ_MAX_WIDTH) {  // I think it's a can
          Serial.println("$$ FOUND CAN $$");
          canPts[canCnt][0] = (obj[sensor].start.x + obj[sensor].last.x) / 2;
          canPts[canCnt][1] = (obj[sensor].start.y + obj[sensor].last.y) / 2;
          ++canCnt;           
        }
        resetCanDetection(sensor);
        Serial.println("## RESET ##");
      }
      else if (obj[sensor].active ) {
        obj[sensor].last.x = objX;
        obj[sensor].last.y = objY;          
      }
    }
  }  
}

void resetCanDetection(int sensor) {
  obj[sensor].start.x = 0;
  obj[sensor].start.y = 0;
  obj[sensor].last.x = 0;
  obj[sensor].last.y = 0;
  obj[sensor].width = 0;  
  obj[sensor].active = false;
}

void readIrSensors() {

  //  long irStart = millis();
  float lastLDist, lastFLDist, lastFDist, lastFRDist, lastRDist;

  //  lastLDist = irLDist;
  //  irLDist  = irL.distance();
  //  detectCan(IRL,lastLDist,irLDist);

  lastFLDist = irFLDist;
  irFLDist = irFL.distance();
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(irFLDist);
  
  detectCan(IRFL,lastFLDist,irFLDist);
  /*
  lastFDist = irFDist;
   irFDist  = irF.distance();
   detectCan(IRF,lastFDist,irFDist);
   
   lastFRDist = irFRDist;
   irFRDist = irFR.distance();
   detectCan(IRFR,lastFRDist,irFRDist);
   
   lastRDist = irRDist;
   irRDist  = irR.distance();
   detectCan(IRR,lastRDist,irRDist);
   */

  //  Serial.println(millis() - irStart);
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






















