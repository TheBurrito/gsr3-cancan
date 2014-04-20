#include <Arduino.h>
#include <RobotBase.h>
#include <TimedAction.h>
#include <Wire.h>
#include <LiquidTWI2.h>
#include <Adafruit_RGBLCDShield.h>
#include <Servo.h>
#include <Pins.h>

#define DEBUG_USE_LCD true
#define DEBUG_USE_SERIAL false
#define DEBUG_IR false

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
  bool active;
  int lastDist;
};

ObjInfo obj[IR_END];

TimedAction irAction = TimedAction(20,readIrSensors);  // Scan IR Sensors every 800ms
TimedAction debugIrAction = TimedAction(1000,debugIr);
TimedAction chooseCanAction = TimedAction(1000,chooseCan);

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
#define IR_R_OFFSET_X 0
#define IR_R_OFFSET_Y 8.0

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
#define GOAL_X MAX_X
#define GOAL_Y 0
#endif

typedef enum {
  mWaitStart,
  mWait1Sec,
  mWander,
  mDriveCan,
  mGrabCan,
  mDriveGoal,
  mDropCan,
  mBackup,
  mStop
} 
Mode;

Mode mode = mWaitStart, lastMode = mode, nextMode;

bool newState = true, restart = false;

int curGrip, gripStep = 1, gripDelay = 20;
long lastGrip;

unsigned long targetTime;

bool atGoal = false;

struct canInfo {
  unsigned long ts;  // timestamp
  Point pos;
  double distToGoal;
  int next;
};
const int canCapacity = 20;
canInfo cans[canCapacity];
int targetCan = -1;  // index for cans[]
int nextCan = 0;
bool firstCan = true;  // flag used for fist can we search for

uint8_t buttons;

void setup() {

  for (int i = 0; i < IR_END; i++) {
    obj[i].active = false;
  }

  for (int i = 0; i < canCapacity; i++) {
    cans[i].next = i + 1;
    cans[i].ts = 0;
    if (i == canCapacity - 1) {
      cans[i].next = -1;
    }
  }

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
  RobotBase.setIRPeriod(10);
  RobotBase.setIRFilter(0.7);


  Serial.begin(115200);
  //  Serial1.begin(9600);
  lcdInit();
  servoG.attach(SERVO_G);  // init gripper servo
  servoG.write(SERVO_G_OPEN);
  RobotBase.time();
}

void loop() {
  RobotBase.update();

  if (mode != mWaitStart) {
    chooseCanAction.check();
    irAction.check();
  }

  newState = (lastMode != mode) || restart;
  restart = false;

  lastMode = mode;

  switch (mode) {
  case mWaitStart:
    debugIrAction.check();
    buttons = lcd.readButtons();
    if (buttons & BUTTON_UP) {
      lcd.clear();
      lcd.setBacklight(GREEN);
      mode = mWait1Sec;
      nextMode = mWander;
    }
    break;

  case mWait1Sec:
    if (newState) {
      targetTime = millis() + 1000;
    }

    if (millis() >= targetTime) {
      mode = nextMode;
    };
    break;

  case mWander:
    if (newState) {
      int destX;

      if (atGoal) destX = 0;
      else destX = 180;

      RobotBase.driveTo(destX, 0);
    } 
    else {
      if (targetCan != -1) {
        mode = mDriveCan;
      } 
      else if (RobotBase.navDone()) {
        atGoal = !atGoal;
        restart = true;
      }
    }
    break;

  case mDriveCan:
    if (newState) {
      RobotBase.turnToAndDrive(cans[targetCan].pos.x, cans[targetCan].pos.y);
    } 
    else if (RobotBase.navDone()) {
      mode = mGrabCan;
    }
    break;

  case mGrabCan:
    if (newState) {
      lastGrip = millis();
      curGrip = SERVO_G_OPEN;
      RobotBase.stop();
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
    } 
    else if (RobotBase.navDone()) {
      mode = mDropCan;
    }
    break;

  case mDropCan:
    if (newState) {
      lastGrip = millis();
      curGrip = SERVO_G_CLOSE;
      RobotBase.stop();
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
}

#define OBJ_MIN_WIDTH 3
#define OBJ_MAX_WIDTH 14

void detectCan(int sensor, int curDist) {
  float objX, objY;
  static int objDistThresh = 5;
  int diff = curDist - obj[sensor].lastDist;
  obj[sensor].lastDist = curDist;

  int min = 10;
  int max = 80;

  if (sensor == 0 || sensor == 4) {
    min = 20;
    max = 150;
  }

  if (curDist > min && curDist < max) {  // ignore values outside the sensors specs
    // calculate detected object x,y position
    objX = curDist * (0.573576436) + RobotBase.getX();
    objY = curDist * (0.819152044) + RobotBase.getY();
    if (objX < MAX_X && objX > MIN_X && objY < MAX_Y && objY > MIN_Y) {
      Serial.print(sensor);
      Serial.print(":");
      Serial.print(curDist);
      Serial.print(", ");
      Serial.println(diff);


      if (diff < -objDistThresh) {  // we found something
        obj[sensor].start.x = objX;
        obj[sensor].start.y = objY;
        obj[sensor].active = true;
        Serial.println("");
        Serial.println("!! START !!");
        Serial.print("Pos: ");
        Serial.print(int(objX));
        Serial.print(",");
        Serial.println(int(objY));
        Serial.println(millis());
        Serial.println("");
      } 
      else if (obj[sensor].active && diff > objDistThresh) {
        obj[sensor].last.x = objX;
        obj[sensor].last.y = objY;          
        double dX = obj[sensor].start.x - obj[sensor].last.x;
        double dY = obj[sensor].start.y - obj[sensor].last.y;
        obj[sensor].width = hypot(dX, dY);
        Serial.print("width: ");
        Serial.println(obj[sensor].width);
        if (obj[sensor].width > OBJ_MIN_WIDTH && obj[sensor].width < OBJ_MAX_WIDTH) {  // I think it's a can
          Serial.println("$$ FOUND CAN $$");
          float posX = (obj[sensor].start.x + obj[sensor].last.x) / 2;
          float posY = (obj[sensor].start.y + obj[sensor].last.y) / 2;
          addCan(posX,posY);
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

void chooseCan() {
  float dist = 1000;
  if (targetCan == -1) {
    for (int i = 0; i < canCapacity; i++) {
      if (cans[i].ts != 0) {
        if (firstCan) {
          float dX = cans[i].pos.x - RobotBase.getX();
          float dY = cans[i].pos.y - RobotBase.getY();
          float _dist = hypot(dX,dY);
          if (_dist < dist) {
            dist = _dist;
            targetCan = i;
          }
        }
        else {
          if (cans[i].distToGoal < dist) {
            dist = cans[i].distToGoal;
            targetCan = i;
          }
        }
      }
    }
    firstCan = false;
  }
}
void readIrSensors() {
  for (int i = 0; i < IR_END; ++i) {
    detectCan(i, RobotBase.irDistance((IR_Index)i));
  }
}

void addCan(float x, float y) {
  double dX = GOAL_X - x;
  double dY = GOAL_Y - y;
  double distToGoal = hypot(dX,dY);

  cans[nextCan].ts = millis();  
  cans[nextCan].pos.x = x;
  cans[nextCan].pos.y = y;
  cans[nextCan].distToGoal = distToGoal;
  nextCan = cans[nextCan].next;
  if (nextCan == -1) {  // our can capacity is full clear out an old one
    // ToDo: write a function that searches for oldest timestamp
  }
}

void removeCan(int canIndex) {
  cans[canIndex].next = nextCan;
  nextCan = canIndex;
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
  lcd.print(RobotBase.irDistance(IRFL));
  lcd.print("  ");
  lcd.print(RobotBase.irDistance(IRF));
  lcd.print("  ");
  lcd.print(RobotBase.irDistance(IRFR));

  lcd.setCursor(0,1);
  lcd.print(RobotBase.irDistance(IRL));
  lcd.print(" <--  --> ");
  lcd.print(RobotBase.irDistance(IRR));
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


























