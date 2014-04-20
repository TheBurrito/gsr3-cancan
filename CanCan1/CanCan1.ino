#include <Arduino.h>
#include <RobotBase.h>
#include <TimedAction.h>
#include <Wire.h>
#include <LiquidTWI2.h>
#include <Adafruit_RGBLCDShield.h>
#include <Servo.h>
#include <Pins.h>

Servo servoG; // define the Gripper Servo
#define SERVO_G_CLOSE 36
#define SERVO_G_OPEN  88
int i;

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

typedef enum {
  mWaitStart,
  mWander,
  mDriveCan,
  mGrabCan,
  mDriveGoal,
  mDropCan,
  mBackup,
  mStop
} 
Mode;

Mode mode = mWander, lastMode = mode;

bool newState = true, restart = false;

int curGrip, gripStep = 1, gripDelay = 20;
long lastGrip;

unsigned long targetTime;

bool atGoal = false;

void setup() {
  for (int i = 0; i < IR_END; ++i) {
    obj[i].active = false;
  }
  
  RobotBase.setPID(10, 4, 0);
  RobotBase.setAccel(50);
  RobotBase.setMax(20, PI / 2);
  RobotBase.setOutputRange(350, 5, 50);
  RobotBase.setTicksPerUnit(71.65267);
  RobotBase.setWidth(17.4);
  RobotBase.setNavThresh(2, 0.035);
  RobotBase.setOdomPeriod(10);
  RobotBase.setNavPeriod(10);
  RobotBase.setIRPeriod(10);
  
  Serial.begin(115200);
  
  servoG.attach(SERVO_G);
  servoG.write(SERVO_G_OPEN);
  
  RobotBase.time();
}

void loop() {
  RobotBase.update();
  
  newState = (lastMode != mode) || restart;
  restart = false;
  
  lastMode = mode;
  
  switch (mode) {
    case mWaitStart:
      //TODO: Add in button check
      mode = mWander;
      break;
      
    case mWander:
      if (newState) {
        int destX;
        
        if (atGoal) destX = 0;
        else destX = 180;
        
        RobotBase.driveTo(destX, 0);
      } else {
        if (detectCan()) {
          mode = mDriveCan;
        } else if (RobotBase.navDone()) {
          atGoal = !atGoal;
          restart = true;
        }
      }
      break;
      
    case mDriveCan:
      if (newState) {
        RobotBase.turnToAndDrive(getCanX(), getCanY());
      } else if (RobotBase.navDone()) {
        mode = mGrabCan;
      }
      break;
      
    case mGrabCan:
      if (newState) {
        lastGrip = millis();
        curGrip = SERVO_G_OPEN;
      }
      
      if (millis() - lastGrip >= gripDelay) {
        curGrip -= gripStep;
        if (curGrip <= SERVO_G_CLOSE) {
          curGrip = SERVO_G_CLOSE;
          mode = mDriveGoal;
        }
        
        lastGrip = millis();
        
        servoG.write(curGrip);
      }
      break;
      
    case mDriveGoal:
      if (newState) {
        RobotBase.turnToAndDrive(MAX_X, 0);
      } else if (RobotBase.navDone()) {
        mode = mDropCan;
      }
      break;
      
    case mDropCan:
      if (newState) {
        lastGrip = millis();
        curGrip = SERVO_G_CLOSE;
      }
      
      if (millis() - lastGrip >= gripDelay) {
        curGrip += gripStep;
        
        if (curGrip >= SERVO_G_OPEN) {
          curGrip = SERVO_G_OPEN;
          mode = mBackup;
          atGoal = true;
        }
        
        lastGrip = millis();
        
        servoG.write(curGrip);
      }
      break;
      
    case mBackup:
      if (newState) {
        RobotBase.setVelocityAndTurn(-20.0, 0.0);
        targetTime = millis() + 2000;
      }
      
      if (millis() >= targetTime) {
        RobotBase.stop();
        mode = mWander;
      }
        
      break;
      
    case mStop:
      RobotBase.stop();
      break;
}
