#include <Arduino.h>
#include <RobotBase.h>
#include <TimedAction.h>
#include <Wire.h>
#include <LiquidTWI2.h>
#include <Adafruit_RGBLCDShield.h>
#include <LSM303.h>
#include <Servo.h>
#include <Pins.h>

#include <deque>

#define DEBUG_USE_LCD true
#define DEBUG_USE_SERIAL false
#define DEBUG_IR false
#define DEBUG_DETECT_CAN false
#define DEBUG_HEADING false
#define DEBUG_SONAR true

int objMinWidth = 4;
int objMaxWidth = 8;

LSM303 compass;
float headingOffset, adjHeading;

Servo servoG; // define the Gripper Servo
#define SERVO_G_CLOSE 36
#define SERVO_G_OPEN  88
int i;

struct ObjInfo {
  Point start;
  Point last;
  double width;
  bool active;
  int lastDist;
};

ObjInfo obj[IR_END];

struct ReturnPos {
  bool active;
  Point pos;
};

ReturnPos returnPos;

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

#define WALL_BUFFER 10 // padding around walls to exclude from object detection
/*
  There are three known arena sizes
 1 - official arena size 7'x12'
 2 - contigent arena size ?x?
 3 - home arena size 4'x8'
 */
#define ARENA 3

#if ARENA == 1 // ft * in/ft * cm/in
#define ARENA_W 7
#define ARENA_L 10
#define MAX_X ARENA_L * 12 * 2.54 - WALL_BUFFER - ROBOT_FRONT_OFFSET - ROBOT_GRIP_OFFSET
#define MAX_Y ARENA_W / 2 * 12 * 2.54 - WALL_BUFFER - ROBOT_FRONT_OFFSET - ROBOT_GRIP_OFFSET
#define MIN_X 0 - ROBOT_FRONT_OFFSET + WALL_BUFFER + ROBOT_FRONT_OFFSET + ROBOT_GRIP_OFFSET
#define MIN_Y -ARENA_W / 2 * 12 * 2.54 + WALL_BUFFER + ROBOT_FRONT_OFFSET + ROBOT_GRIP_OFFSET
#define GOAL_X ARENA_L * 12 * 2.54 + 20
#define GOAL_Y 0
#endif

#if ARENA == 2 // ft * in/ft * cm/in
#define MAX_X 
#define MAX_Y 
#define MIN_X 
#define MIN_Y 
#endif

#if ARENA == 3 // ft * in/ft * cm/in
#define ARENA_W 4
#define ARENA_L 8
#define MAX_X ARENA_L * 12 * 2.54 - WALL_BUFFER - ROBOT_FRONT_OFFSET - ROBOT_GRIP_OFFSET - 13
#define MAX_Y ARENA_W / 2 * 12 * 2.54 - WALL_BUFFER - ROBOT_FRONT_OFFSET - ROBOT_GRIP_OFFSET
#define MIN_X WALL_BUFFER + ROBOT_FRONT_OFFSET + ROBOT_GRIP_OFFSET 
#define MIN_Y -ARENA_W / 2 * 12 * 2.54 + WALL_BUFFER + ROBOT_FRONT_OFFSET + ROBOT_GRIP_OFFSET
#define GOAL_X MAX_X + 5
#define GOAL_Y 0
#endif

typedef enum {
  mWaitStart,
  mWait1Sec,
  mWander,
  mDriveCan,
  mGrabCan,
  mDriveGoal,
  mDriveInGoal,
  mDropCan,
  mBackup,
  mLocalize,
  mStop,
  mReset,
  mEvadeRight,
  mEvadeLeft,
  mReturnToPos
} 
Mode;

typedef enum {
  gOpen,
  gClose
} 
GripState;

GripState gripState = gOpen;

Mode mode = mWaitStart, lastMode = mode, nextMode;

bool newState = true, restart = false;

int curGrip = SERVO_G_OPEN, gripStep = 1, gripDelay = 20;

unsigned long targetTime;

bool atGoal = false;
bool celebrateGoal = false;
int cCycle = 1;

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

Pose sensors[IR_END];  

struct wayPtsStruct {
  Point pos;
};
const int wayPtsCnt = 7;
wayPtsStruct wayPts[wayPtsCnt];
int wayPt = 0;

// Attempt to prevent any infinite loops
int errorThresh = 5;  
int errorCnt = 0;

double dXcan, dYcan, dHcan;  // distance to can calculations

int scanSpeed = 20;
int goalSpeed = 45;
int canSpeed = 35;

// Sonar start and end times in micros
volatile long pulseStart, pulseEnd;
volatile int sonarDist;

TimedAction irAction = TimedAction(40,readIrSensors); 
TimedAction gripAction = TimedAction(gripDelay,gripper); 
TimedAction debugIrAction = TimedAction(1000,debugIr);
TimedAction chooseCanAction = TimedAction(2000,chooseCan);
TimedAction celebrateAction = TimedAction(150,celebrate);
TimedAction compassAction = TimedAction(10,getHeading);
TimedAction debugHeadingAction = TimedAction(500,debugHeading);
TimedAction debugSonarAction = TimedAction(1000,debugSonar);
TimedAction pingAction = TimedAction(200,ping);
TimedAction localizeAction = TimedAction(40, localize);
TimedAction backupAction = TimedAction(40, checkBackup);

typedef enum {
  lReset,
  lRun,
  lFinish
} 
LocalizeState;

LocalizeState localState = lReset;

unsigned long lastLocal, curLocal;
int localizeStep = 0;
float ir[5];

bool doLocalize = false;

bool localizeFailX = false;
bool localizeFailY = false;
bool localizeFailTheta = false;

int localDist = 78;

#define FIELDWIDTH (ARENA_W * 12 * 2.54)
#define FIELDLENGTH (ARENA_L * 12 * 2.54)

int backupCount = 0;

void checkObstacles() {
    /*if (digitalReadFast(IRB_FL) == 0) {
      if (mode != mBackup && mode != mDropCan && RobotBase.getX() < MAX_X - 15) {
        if (mode != mEvadeRight)  {
          nextMode = mode;
          if (nextMode == mDriveCan) nextMode = mWander;
          mode = mEvadeRight;
        }
      }
    }

    if (digitalReadFast(IRB_FR) == 0) {
      if (mode != mBackup && mode != mDropCan && RobotBase.getX() < MAX_X - 15) {
        if (mode != mEvadeLeft) nextMode = mode;
        if (nextMode == mDriveCan) nextMode = mWander;
        mode = mEvadeLeft;
      }    
    }*/
  
  switch (mode) {
    case mDriveCan:
    
    case mWander:
    
}

void setLCD(char *s) {
  //lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(s);
}

void fillPath() {
  path.reset();
  NavPoint np;
  for (int i = 0; i < wayPtsCnt; ++i) {
    np.p.x = wayPts[i].pos.x;
    np.p.y = wayPts[i].pos.y;
    np.m = mWander;
    path.pushBack(p);
  }
}

void checkBackup() {
  int left, right, width;

  left = RobotBase.irDistance(IRL);
  right = RobotBase.irDistance(IRR);

  width = left + right + 16;

  Serial.print("Backup: ");
  Serial.print(width);
  Serial.print(" / ");
  Serial.print(FIELDWIDTH);

  if (left > 150 || right > 150) {
    Serial.println(" **");
    return;
  }
  Serial.println();

  if (width > FIELDWIDTH - 1) {
    if (doLocalize) {
      mode = mLocalize;
    } 
    else {
      mode = mWander;
    }

    RobotBase.stop(true);
  }
}

void localize() {
  curLocal = millis();
  int i;
  float c, calcTheta, width, calcY, calcX = 0;
  Point left, right;

  switch (localState) {
  case lReset:
    for (i = 0; i < 5; ++i) {
      ir[i] = 0;
    }

    localState = lRun;

    localizeStep = 0;
    break;

  case lRun:
    if (curLocal - 40 > lastLocal) {
      lastLocal = curLocal;
      ++localizeStep;

      for (i = 0; i < 5; ++i) {
        ir[i] += RobotBase.irDistance((IR_Index)i);
      }
    }
    break;

  case lFinish:
    for (i = 0; i < 5; ++i) {
      ir[i] /= localizeStep;
    }

    if (ir[IRL] > 150 || ir[IRR] > 150) {
      localizeFailY = true;
      localizeFailTheta = true;
    } 
    else {
      width = ir[IRL] + ir[IRR] + 16;
      if (width > FIELDWIDTH) {          
        c = FIELDWIDTH / width;
        calcTheta = acos(c);

        calcY = (c * (ir[IRR] - ir[IRL])) / 2.0;

        RobotBase.localizeY(calcY);
        RobotBase.localizeTheta(calcTheta);

        localizeFailY = false;
        localizeFailTheta = false;
      }

      if (ir[IRFL] < localDist) {
        left = RobotBase.obsPos(IRFL, sensors[IRFL]);
        left = rotate(left, RobotBase.getTheta());
        calcX = FIELDLENGTH - left.x;
      }

      if (ir[IRFR] < localDist) {
        right = RobotBase.obsPos(IRFR, sensors[IRFR]);
        right = rotate(right, RobotBase.getTheta());

        if (calcX == 0) {
          calcX = (calcX + FIELDLENGTH - right.x) / 2.0;
        } 
        else {
          calcX = FIELDLENGTH - right.x;
        }
      }

      if (calcX != 0) {
        RobotBase.localizeX(calcX);
        localizeFailX = false;
      } 
      else {
        localizeFailX = true;
      }
    }
    break;
  }        
}

void initCans() {
  nextCan = 0;

  for (int i = 0; i < canCapacity; i++) {
    cans[i].next = i + 1;
    cans[i].ts = 0;
    if (i == canCapacity - 1) {
      cans[i].next = -1;
    }
  }
}

void setup() {

  for (int i = 0; i < IR_END; i++) {
    obj[i].active = false;
  }

  initCans();

  wayPts[0].pos.x = MAX_X;
  wayPts[0].pos.y = 0;
  //  wayPts[1].pos.x = MIN_X;
  //  wayPts[1].pos.y = 0;  
  wayPts[1].pos.x = MAX_X;
  wayPts[1].pos.y = MAX_Y / 2;
  wayPts[2].pos.x = MIN_X;
  wayPts[2].pos.y = MAX_Y / 2;
  wayPts[3].pos.x = MIN_X;
  wayPts[3].pos.y = MIN_Y /2;
  wayPts[4].pos.x = MAX_X;
  wayPts[4].pos.y = MIN_Y / 2;
  wayPts[5].pos.x = MAX_X;
  wayPts[5].pos.y = 0;
  wayPts[6].pos.x = MIN_X;
  wayPts[6].pos.y = 0;

  // Sensor offsets from robot center
  sensors[IRL].offset.x = 0;
  sensors[IRL].offset.y = 8.0;
  sensors[IRL].angle = 90 * PI / 180;
  sensors[IRFL].offset.x = 4.1;
  sensors[IRFL].offset.y = 6.2;
  sensors[IRFL].angle = 45 * PI / 180;  
  sensors[IRF].offset.x = 7.3;
  sensors[IRF].offset.y = 1.2;
  sensors[IRF].angle = 0 * PI / 180;
  sensors[IRFR].offset.x = 5.7;
  sensors[IRFR].offset.y = -4.8;
  sensors[IRFR].angle = -45 * PI / 180;
  sensors[IRR].offset.x = 0;
  sensors[IRR].offset.y = -8.0;
  sensors[IRR].angle = -90 * PI / 180;

  //Establish PID gains
  RobotBase.setPID(10, 5, 0);

  //Set allowed accel for wheel velocity targets (cm/s/s)
  RobotBase.setAccel(50);

  //Set max velocity and turn rate
  //RobotBase.setMax(scanSpeed, 2.0); //cm/s, Rad/s
  RobotBase.setVelocityRange(20.0, 0, 1.0);
  RobotBase.setTurnRange(2.0, 0.01, 0.2);

  //set motor output ranges - works both positive and negative
  //Max, dead zone, min
  // -deadZone > X < deadZone : X = 0
  // X < min : X = min
  // X > max : x = max
  RobotBase.setOutputRange(350, 5, 70);

  //set ticks per desired distance unit
  RobotBase.setTicksPerUnit(71.65267); //units of cm

  //set the wheelbase of in desired distance unit
  RobotBase.setWidth(17.4); //units of cm

  RobotBase.setNavThresh(2, 0.04);
  RobotBase.setOdomPeriod(10);
  RobotBase.setNavPeriod(10);
  RobotBase.setIRPeriod(40);
  RobotBase.setIRFilter(0.7);
  RobotBase.setIRSamples(1);

  Serial.begin(115200);
  lcdInit();
  compassInit();
  sonarInit();
  bumperInit();
  servoG.attach(SERVO_G);  // init gripper servo
  RobotBase.time();
}

void loop() {
  RobotBase.update();

  compassAction.check();
  gripAction.check();

  if (mode != mWaitStart) {
    chooseCanAction.check();
    irAction.check();
    pingAction.check();   
    debugSonarAction.check();
    debugHeadingAction.check();

    if (celebrateGoal) celebrateAction.check();
    if (errorCnt > errorThresh) {
      mode = mReset;
    }

    if (digitalReadFast(IRB_FL) == 0) {
      if (mode != mBackup && mode != mDropCan && RobotBase.getX() < MAX_X - 15) {
        if (mode != mEvadeRight) nextMode = lastMode;
        if (nextMode == mDriveCan) nextMode = mWander;
        mode = mEvadeRight;
      }
    }

    if (digitalReadFast(IRB_FR) == 0) {
      if (mode != mBackup && mode != mDropCan && RobotBase.getX() < MAX_X - 15) {
        if (mode != mEvadeLeft) nextMode = lastMode;
        if (nextMode == mDriveCan) nextMode = mWander;
        mode = mEvadeLeft;
      }    
    }
  }  // if !mWaitStart

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
      RobotBase.setMax(scanSpeed, 2.0); //cm/s, Rad/s
      lcd.setBacklight(YELLOW);
      RobotBase.turnToAndDrive(wayPts[wayPt].pos.x, wayPts[wayPt].pos.y, false);
    } 
    else {
      if (digitalRead(IRB_F) == 0 && curGrip == SERVO_G_CLOSE) {
        mode = mDriveGoal;
      }      
      else if (targetCan != -1) {
        if (!returnPos.active) {
          returnPos.active = true;
          returnPos.pos.x = RobotBase.getX();
          returnPos.pos.y = RobotBase.getY();
        }
        mode = mDriveCan;
      } 
      else if (RobotBase.navDone()) {
        if (wayPt == wayPtsCnt - 1) {
          wayPt = 0;
        }
        else  wayPt++;
        restart = true;
      }
    }
    break;

  case mDriveCan:
    if (newState) {
      lcd.setBacklight(TEAL);
      gripState = gOpen;
      RobotBase.setMax(canSpeed, 2.0); //cm/s, Rad/s
      RobotBase.turnToAndDrive(cans[targetCan].pos.x, cans[targetCan].pos.y, true);
    }
    else if (RobotBase.navDone()) {
      mode = mGrabCan;
    }
    break;

  case mGrabCan:
    if (newState) {
      lcd.setBacklight(VIOLET);
      gripState = gClose;
      RobotBase.stop(false);     
    }
    if (curGrip == SERVO_G_CLOSE) {
      if (digitalRead(IRB_F) == 0) {
        mode = mDriveGoal;
      }
      else {
        if (errorCnt == errorThresh) {
          errorCnt = 0;
          gripState = gOpen;
          removeCan(targetCan);
          if (returnPos.active) mode = mReturnToPos;
          else mode = mWander;
        }
        else errorCnt++;
      }
    }
    break;

  case mDriveGoal:
    if (newState) {
      lcd.setBacklight(GREEN);
      removeCan(targetCan);
      RobotBase.setMax(goalSpeed, 2.0); //cm/s, Rad/s
      RobotBase.turnToAndDrive(GOAL_X - 40, GOAL_Y, false);
    } 
    else if (RobotBase.navDone()) {
      mode = mDriveInGoal;
    }
    break;

  case mDriveInGoal:
    if (newState) {
      lcd.setBacklight(GREEN);
      RobotBase.setMax(20, 2.0); //cm/s, Rad/s
      RobotBase.turnToAndDrive(GOAL_X, GOAL_Y, false);
    } 
    else if (RobotBase.navDone()) {
      mode = mDropCan;
    }
    break;

  case mDropCan:
    if (newState) {
      celebrateGoal = true;
      gripState = gOpen;
      RobotBase.stop(false);
    }
    if (curGrip == SERVO_G_OPEN) {
      mode = mBackup;
    }
    break;

  case mBackup:
    if (newState) {
      RobotBase.setVelocityAndTurn(-20.0, 0.0);
    } else {
      backupAction.check();     
    }
    break;

  case mLocalize:
    Serial.println(localizeStep);
    if (newState) {
      localState = lReset;
      localize();
      localizeAction.reset();
    } 
    else {
      localizeAction.check();
      if (localizeStep == 25) {
        localState = lFinish;
        localize();
		if (returnPos.active) mode = mReturnToPos;
		else mode = mWander;
      }
    }
    break;

  case mStop:
    RobotBase.stop(false);
    break;

  case mReset:
    if (newState) {
      RobotBase.stop(false);
      curGrip = SERVO_G_OPEN;
      servoG.write(curGrip);
      targetCan = -1;
      errorCnt = 0;
    }
    else if (RobotBase.navDone()) {
      mode = mWander;
    }
    break;

  case mEvadeRight:
    if (newState) {
      float relX = 25 * cos(-.17);
      float relY = 25 * sin(-.17);
      float relXrotated = relX * cos(RobotBase.getTheta()) - relY * sin(RobotBase.getTheta());
      float relYrotated = relX * sin(RobotBase.getTheta()) + relY * cos(RobotBase.getTheta());
      double destX = RobotBase.getX() + relXrotated;
      double destY = RobotBase.getY() + relYrotated;
      RobotBase.driveTo(destX,destY, true);
      gripState = gClose;
    }
    else if (RobotBase.navDone()) {
      mode = nextMode;
    }
    break;


  case mEvadeLeft:
    if (newState) {
      float relX = 25 * cos(.17);
      float relY = 25 * sin(.17);
      float relXrotated = relX * cos(RobotBase.getTheta()) - relY * sin(RobotBase.getTheta());
      float relYrotated = relX * sin(RobotBase.getTheta()) + relY * cos(RobotBase.getTheta());
      float destX = RobotBase.getX() + relXrotated;
      float destY = RobotBase.getY() + relYrotated;
      RobotBase.driveTo(destX,destY, true);
      gripState = gClose;
    }
    else if (RobotBase.navDone()) {
      mode = nextMode;
    }
    break;

  case mReturnToPos:
    if (newState) {
      RobotBase.setMax(scanSpeed, 2.0); //cm/s, Rad/s
      RobotBase.turnToAndDrive(returnPos.pos.x, returnPos.pos.y, false);     
    }
    else {
      if (digitalRead(IRB_F) == 0 && curGrip == SERVO_G_CLOSE) {
        mode = mDriveGoal;
      }      
      else if (targetCan != -1) {
        mode = mDriveCan;
      } 
      else if (RobotBase.navDone()) {
        returnPos.active = false;
        mode = mWander;
      }
    }

    break;

  }  // close switch mode
}  // close loop

void detectCan(int sensor, int curDist) {
  if (mode != mWander && mode != mReturnToPos) return;
  if (sensor == IRF) return;  // don't try to detect cans by width with the front sensor
  if (RobotBase.getVelocity() < 2.0) return; // don't detect cans if we'er not rolling forward
  bool edgeFound = false;
  static int objDistThresh = 10;
  int diff = curDist - obj[sensor].lastDist;
  obj[sensor].lastDist = curDist;

  int min = 10;
  int max = 80;

  if (sensor == 0 || sensor == 4) {
    min = 20;
    max = 140;  // too many false postives if we scan all the way out to 150
  }

  if (curDist > min && curDist < max) {  // ignore values outside the sensors specs
    // calculate detected object x,y position
    float relX = curDist * (cos(sensors[sensor].angle) ) + sensors[sensor].offset.x;
    float relY = curDist * (sin(sensors[sensor].angle) ) + sensors[sensor].offset.y;
    float relXrotated = relX * cos(RobotBase.getTheta()) - relY * sin(RobotBase.getTheta());
    float relYrotated = relX * sin(RobotBase.getTheta()) + relY * cos(RobotBase.getTheta());
    float objX = RobotBase.getX() + relXrotated;
    float objY = RobotBase.getY() + relYrotated;


    if (objX < MAX_X && objX > MIN_X && objY < MAX_Y && objY > MIN_Y) {

      if (!obj[sensor].active) {
#if DEBUG_DETECT_CAN       
        Serial.print("IR: ");
        Serial.print(sensor);
        Serial.print("  ** START **  ");
        Serial.print(int(RobotBase.getX()));
        Serial.print(",");
        Serial.print(int(RobotBase.getY()));
        Serial.print("  Dist: ");
        Serial.print(curDist);
        Serial.print("  oPos: ");
        Serial.print(int(objX));
        Serial.print(",");
        Serial.println(int(objY));
#endif      
        obj[sensor].start.x = objX;
        obj[sensor].start.y = objY;
        obj[sensor].active = true;
      }

      else if (diff < -objDistThresh) {  // found something closer
#if DEBUG_DETECT_CAN       
        Serial.print("IR: ");
        Serial.print(sensor);
        Serial.print("  ** RESTART **  ");
        Serial.print(int(RobotBase.getX()));
        Serial.print(",");
        Serial.print(int(RobotBase.getY()));
        Serial.print("  Dist: ");
        Serial.print(curDist);
        Serial.print("  oPos: ");
        Serial.print(int(objX));
        Serial.print(",");
        Serial.println(int(objY));
#endif
        obj[sensor].start.x = objX;
        obj[sensor].start.y = objY;
        obj[sensor].active = true;

      } 
      else if (obj[sensor].active && diff > objDistThresh) {
        edgeFound = true;
      }
      else if (obj[sensor].active ) {
        obj[sensor].last.x = objX;
        obj[sensor].last.y = objY;
      }

    } // endif object reading is out of bounds
    else {  
      if (obj[sensor].active) {
        edgeFound = true;
      }
    }

  }  // endif sensor reading within specs
  else {
    if (obj[sensor].active) {
      edgeFound = true;
    }
  }

  if (edgeFound) {
    //   obj[sensor].last.x = objX;
    //   obj[sensor].last.y = objY;          
    double dX = obj[sensor].start.x - obj[sensor].last.x;
    double dY = obj[sensor].start.y - obj[sensor].last.y;
    obj[sensor].width = hypot(dX, dY);

    if (obj[sensor].width > objMinWidth && obj[sensor].width < objMaxWidth) {  // I think it's a can
      float posX = (obj[sensor].start.x + obj[sensor].last.x) / 2;
      float posY = (obj[sensor].start.y + obj[sensor].last.y) / 2;
#if DEBUG_DETECT_CAN   
      Serial.println("");
      Serial.print("IR: ");
      Serial.print(sensor);
      Serial.print("  $$ FOUND CAN $$  ");
      Serial.print("Pos: ");
      Serial.print(posX);
      Serial.print(",");
      Serial.print(posY);
      Serial.print("  width: ");
      Serial.println(obj[sensor].width);          
      Serial.println("");
#endif
      addCan(posX,posY);

    }
    resetCanDetection(sensor);
#if DEBUG_DETECT_CAN  
    Serial.print("IR: ");
    Serial.print(sensor);
    Serial.println("  ## RESET ## (Edge Found)");   
#endif    
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
    double x = RobotBase.getX();
    double y = RobotBase.getY();
    for (int i = 0; i < canCapacity; i++) {
      if (cans[i].ts != 0) {
        if (firstCan) {
          float dX = cans[i].pos.x - x;
          float dY = cans[i].pos.y - y;
          float _dist = hypot(dX,dY);
          if (_dist < dist) {
            dist = _dist;
            targetCan = i;
          }
        }
        else {  // not first can
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
  bool duplicate = false;  // prevent duplicate cans in the list
  for (int i = 0; i < canCapacity; i++) {
    if (cans[i].ts != 0 && fabs(cans[i].pos.x - x) < 10 && fabs(cans[i].pos.y - y) < 10) {
      return;
    }
  }

  if (!duplicate) {  // if the can is not a duplicate add it to the list
    double dX = GOAL_X - x;
    double dY = GOAL_Y - y;
    double distToGoal = hypot(dX,dY);

  cans[nextCan].ts = millis();  
  cans[nextCan].pos.x = x;
  cans[nextCan].pos.y = y;
  cans[nextCan].distToGoal = distToGoal;
  nextCan = cans[nextCan].next;
  if (nextCan < 0 || nextCan >= canCapacity) {  // our can capacity is full clear out an old one
    lcd.setBacklight(RED);
    int a = 0;
    int b = 3 / a;
  }
  else {
#if DEBUG_DETECT_CAN       
    Serial.println("** Discard Duplicate Can **");
#endif
  }
}

void removeCan(int canIndex) {
  cans[canIndex].ts = 0;
  cans[canIndex].next = nextCan;
  nextCan = canIndex;
  targetCan = -1;
}

void getHeading() {  
  compass.read();
  adjHeading = compass.heading() - headingOffset;
  if (adjHeading < 0) adjHeading += 360;
  else if (adjHeading > 360) adjHeading -= 360;

  if (adjHeading < 180) adjHeading = -adjHeading;
  else if (adjHeading > 180) adjHeading = -(adjHeading - 360);
}

void ping() {
  digitalWrite(SNR_RX, HIGH);
  delayMicroseconds(20);
  digitalWrite(SNR_RX, LOW);

  /* Using interupts instead of pulseIn which is blocking
   sonarDist = pulseIn(SNR_PW, HIGH) / 147.0; 
   */
}

void readSonarPulse() {
  if (digitalRead(SNR_PW) == HIGH) {
    pulseStart = micros();
  }
  else
    pulseEnd = micros();
  sonarDist = ((pulseEnd - pulseStart) / 147.0) * 2.54;
}

void gripper() {
  if (gripState == gClose) {
    curGrip -= gripStep;

    if (curGrip <= SERVO_G_CLOSE) {
      curGrip = SERVO_G_CLOSE;
    }
  }
  if (gripState == gOpen) {
    curGrip += gripStep;

    if (curGrip >= SERVO_G_OPEN) {
      curGrip = SERVO_G_OPEN;
    }
  }

  servoG.write(curGrip);
}

void celebrate() {
  switch (cCycle) {
  case 1:
    lcd.setBacklight(OFF);
    break; 
  case 2:
    lcd.setBacklight(WHITE);
    break; 
  case 3:
    lcd.setBacklight(OFF);
    break; 
  case 4:
    lcd.setBacklight(RED);
    break; 
  case 5:
    lcd.setBacklight(OFF);
    break; 
  case 6:
    lcd.setBacklight(BLUE);
    break; 
  case 7:
    lcd.setBacklight(OFF);
    break; 
  case 8:
    lcd.setBacklight(GREEN);
    break; 
  case 9:
    lcd.setBacklight(OFF);
    break; 
  case 10:
    lcd.setBacklight(VIOLET);
    break; 
  case 11:
    lcd.setBacklight(OFF);
    break; 
  case 12:
    lcd.setBacklight(TEAL);
    break; 
  case 14:
    lcd.setBacklight(OFF);
    break; 
  case 15:
    lcd.setBacklight(YELLOW);
    cCycle = 1;
    celebrateGoal = false;
    break; 
  }
  cCycle++;
}

void lcdInit() {
  lcd.setMCPType(LTI_TYPE_MCP23017);
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  lcd.setBacklight(WHITE);
}

void compassInit () {  
  compass.init();
  compass.enableDefault();

  /*
      Calibration values obtained from running calibration example.
   */
  compass.m_min = (LSM303::vector<int16_t>){
    -398, -374, -489  };
  compass.m_max = (LSM303::vector<int16_t>){
    +240, +250, +0  };

  float _heading = 0;
  for (int i = 0; i < 100; i++) {
    delay(10);
    compass.read();
    _heading = _heading + compass.heading();
  }
  headingOffset = _heading / 100;
}

void sonarInit() {
  pinMode(SNR_RX, OUTPUT);
  pinMode(SNR_PW, INPUT);  
  attachInterrupt(SNR_PW, readSonarPulse, CHANGE);
}

void getPulseTime() {
  pulseEnd = micros();
}

void bumperInit(){
  pinMode(IRB_FR, INPUT);
  pinMode(IRB_F, INPUT);
  pinMode(IRB_FL, INPUT);
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

void debugHeading() {
#if DEBUG_HEADING
#if DEBUG_USE_LCD
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Theta  : ");
  lcd.print(RobotBase.getTheta() * 180 / PI);
  lcd.setCursor(0,1);
  lcd.print("Heading: ");
  lcd.print(adjHeading);
#endif
#if DEBUG_USE_SERIAL
  Serial.print("Theta  : ");
  Serial.println(RobotBase.getTheta());
  Serial.print("Heading: ");
  Serial.println(adjHeading);
#endif
#endif
}

void debugSonar() {
#if DEBUG_SONAR
#if DEBUG_USE_LCD
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Sonar: ");
  lcd.print(sonarDist);
#endif
#if DEBUG_USE_SERIAL
  Serial.print("Sonar:");
  Serial.println(sonarDist);  
#endif
#endif
}





































