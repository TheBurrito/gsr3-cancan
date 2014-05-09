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
#define DEBUG_DETECT_CAN false
#define DEBUG_SONAR false

int objMinWidth = 4;
int objMaxWidth = 8;

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
#define BLANK 0x0
#define RED 0x1
#define GREEN 0x2
#define YELLOW 0x3
#define BLUE 0x4
#define VIOLET 0x5
#define TEAL 0x6
#define WHITE 0x7

// Distances from robot center to outer dimensions
#define ROBOT_GRIP_OFFSET 8.7  // gripper closed position#define ROBOT_FRONT_OFFSET 7.3 // to front of case#define ROBOT_REAR_OFFSET 10.7 // to rear of case#define ROBOT_WHEEL_OFFSET 9.2 // to outside of wheel#define ROBOT_SONAR_OFFSET 6.0 // to end of sonar#define WALL_BUFFER 10 // padding around walls to exclude from object detection/*There are three known arena sizes
1 - official arena size 7'x12'
2 - contigent arena size ?x?
3 - home arena size 4'x8'
*/
#define ARENA 3

#if ARENA == 1 // ft * in/ft * cm/in#define ARENA_W 7#define ARENA_L 10
#define MAX_X ARENA_L * 12 * 2.54 - WALL_BUFFER - ROBOT_FRONT_OFFSET - ROBOT_GRIP_OFFSET
#define MAX_Y ARENA_W / 2 * 12 * 2.54 - WALL_BUFFER - ROBOT_FRONT_OFFSET - ROBOT_GRIP_OFFSET
#define MIN_X 0 - ROBOT_FRONT_OFFSET + WALL_BUFFER + ROBOT_FRONT_OFFSET + ROBOT_GRIP_OFFSET
#define MIN_Y -ARENA_W / 2 * 12 * 2.54 + WALL_BUFFER + ROBOT_FRONT_OFFSET + ROBOT_GRIP_OFFSET
#define GOAL_X ARENA_L * 12 * 2.54 + 20
#define GOAL_Y 0
#endif

#if ARENA == 2 // ft * in/ft * cm/in#define MAX_X #define MAX_Y 
#define MIN_X 
#define MIN_Y 
#endif

#if ARENA == 3 // ft * in/ft * cm/in#define ARENA_W 4#define ARENA_L 8
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
    mStop,
    mReset,
    mEvadeRight,
    mEvadeLeft,
    mReturnToPos
}Mode;

typedef enum {
    gOpen, gClose
} GripState;

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

TimedAction irAction = TimedAction(40, readIrSensors);
TimedAction gripAction = TimedAction(gripDelay, gripper);
TimedAction debugIrAction = TimedAction(1000, debugIr);
TimedAction chooseCanAction = TimedAction(700, chooseCan);
TimedAction celebrateAction = TimedAction(150, celebrate);
TimedAction debugSonarAction = TimedAction(1000, debugSonar);
TimedAction pingAction = TimedAction(200, ping);

#include "ObjectAvoidance.h"
#include "inits.h"

void setup() {
    Serial.begin(115200);
    canDetectInit();
    canStorageInit();
    wayPtInit();
    sensorOffsetInit();
    robotBaseInit();
    lcdInit();
    sonarInit();
    bumperInit();
    gripperInit();
    RobotBase.time();
}

void loop() {
    RobotBase.update();

    gripAction.check();

    if (mode != mWaitStart) {
        chooseCanAction.check();
        irAction.check();
        pingAction.check();
        lookForObstacleAction.check();
        debugSonarAction.check();

        if (celebrateGoal)
            celebrateAction.check();
        if (errorCnt > errorThresh) {
            mode = mReset;
        }

        if (digitalRead(IRB_FL) == 0) {
            if (mode != mBackup && mode != mDropCan && mode != mGrabCan
                    && mode != mDriveCan && RobotBase.getX() < MAX_X - 15) {
                if (mode != mEvadeRight)
                    nextMode = lastMode;
                if (nextMode == mDriveCan)
                    nextMode = mWander;
                obstacleLeft = true;
                mode = mEvadeRight;
            }
        }

        if (digitalRead(IRB_FR) == 0) {
            if (mode != mBackup && mode != mDropCan && mode != mGrabCan
                    && mode != mDriveCan && RobotBase.getX() < MAX_X - 15) {
                if (mode != mEvadeLeft)
                    nextMode = lastMode;
                if (nextMode == mDriveCan)
                    nextMode = mWander;
                obstacleRight = true;
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
        }
        ;
        break;

    case mWander:
        RobotBase.setMax(scanSpeed, 2.0); //cm/s, Rad/s
        if (newState) {
            lcd.setBacklight(YELLOW);
            RobotBase.turnToAndDrive(wayPts[wayPt].pos.x, wayPts[wayPt].pos.y,
                    false);
        } else {
            if (digitalRead(IRB_F) == 0 && curGrip == SERVO_G_CLOSE) {
                mode = mDriveGoal;
            } else if (targetCan != -1) {
                if (!returnPos.active) {
                    returnPos.active = true;
                    returnPos.pos.x = RobotBase.getX();
                    returnPos.pos.y = RobotBase.getY();
                }
                mode = mDriveCan;
            } else if (RobotBase.navDone()) {
                nextWayPt();
            }
        }
        break;

    case mDriveCan:
        RobotBase.setMax(canSpeed, 2.0); //cm/s, Rad/s
        if (newState) {
            lcd.setBacklight(TEAL);
            gripState = gOpen;
            RobotBase.turnToAndDrive(cans[targetCan].pos.x,
                    cans[targetCan].pos.y, false);
        } else if (RobotBase.navDone()) {
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
            } else {
                if (errorCnt == errorThresh) {
                    errorCnt = 0;
                    gripState = gOpen;
                    removeCan(targetCan);
                    if (returnPos.active)
                        mode = mReturnToPos;
                    else
                        mode = mWander;
                } else
                    errorCnt++;
            }
        }
        break;

    case mDriveGoal:
        RobotBase.setMax(goalSpeed, 2.0); //cm/s, Rad/s
        if (newState) {
            lcd.setBacklight(GREEN);
            removeCan(targetCan);
            RobotBase.turnToAndDrive(GOAL_X - 20, GOAL_Y, false);
        } else if (RobotBase.navDone()) {
            mode = mDriveInGoal;
        }
        break;

    case mDriveInGoal:
        RobotBase.setMax(20, 2.0); //cm/s, Rad/s
        if (newState) {
            lcd.setBacklight(GREEN);
            RobotBase.turnToAndDrive(GOAL_X, GOAL_Y, false);
        } else if (RobotBase.navDone()) {
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
            if (RobotBase.getX() < GOAL_X - 10) {
                RobotBase.stop(false);
                mode = mWander;
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
        } else if (RobotBase.navDone()) {
            mode = mWander;
        }
        break;

    case mEvadeRight:
        if (newState) {
            findRoute();
            gripState = gClose;
        } else if (RobotBase.navDone()) {
            mode = nextMode;
        }
        break;

    case mEvadeLeft:
        if (newState) {
            findRoute();
            gripState = gClose;
        } else if (RobotBase.navDone()) {
            mode = nextMode;
        }
        break;

    case mReturnToPos:
        RobotBase.setMax(scanSpeed, 2.0); //cm/s, Rad/s
        if (newState) {
            RobotBase.turnToAndDrive(returnPos.pos.x, returnPos.pos.y, false);
        } else {
            if (digitalRead(IRB_F) == 0 && curGrip == SERVO_G_CLOSE) {
                mode = mDriveGoal;
            } else if (targetCan != -1) {
                mode = mDriveCan;
            } else if (RobotBase.navDone()) {
                returnPos.active = false;
                mode = mWander;
            }
        }

        break;

    }  // close switch mode
}  // close loop

void detectCan(int sensor, int curDist) {
    if (mode != mWander && mode != mReturnToPos)
        return;
    if (sensor == IRF)
        return;  // don't try to detect cans by width with the front sensor
    if (RobotBase.getVelocity() < 2.0)
        return; // don't detect cans if we'er not rolling forward
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

    if (curDist > min && curDist < max) { // ignore values outside the sensors specs
    // calculate detected object x,y position
        float relX = curDist * (cos(sensors[sensor].angle))
                + sensors[sensor].offset.x;
        float relY = curDist * (sin(sensors[sensor].angle))
                + sensors[sensor].offset.y;
        float relXrotated = relX * cos(RobotBase.getTheta())
                - relY * sin(RobotBase.getTheta());
        float relYrotated = relX * sin(RobotBase.getTheta())
                + relY * cos(RobotBase.getTheta());
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

            } else if (obj[sensor].active && diff > objDistThresh) {
                edgeFound = true;
            } else if (obj[sensor].active) {
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

        if (obj[sensor].width > objMinWidth
                && obj[sensor].width < objMaxWidth) {  // I think it's a can
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
            addCan(posX, posY);

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
        for (int i = 0; i < canCapacity; i++) {
            if (cans[i].ts != 0) {
                if (firstCan) {
                    float dX = cans[i].pos.x - RobotBase.getX();
                    float dY = cans[i].pos.y - RobotBase.getY();
                    float _dist = hypot(dX, dY);
                    if (_dist < dist) {
                        dist = _dist;
                        targetCan = i;
                    }
                } else {  // not first can
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
        detectCan(i, RobotBase.irDistance((IR_Index) i));
    }
}

void addCan(float x, float y) {
    bool duplicate = false;  // prevent duplicate cans in the list
    for (int i = 0; i < canCapacity; i++) {
        if (!duplicate) {
            if (cans[i].ts != 0) {
                if (fabs(cans[i].pos.x - x) < 10
                        && fabs(cans[i].pos.y - y) < 10) {
                    duplicate = true;
                }
            }
        }
    }

    if (!duplicate) {  // if the can is not a duplicate add it to the list
        double dX = GOAL_X - x;
        double dY = GOAL_Y - y;
        double distToGoal = hypot(dX, dY);

        cans[nextCan].ts = millis();
        cans[nextCan].pos.x = x;
        cans[nextCan].pos.y = y;
        cans[nextCan].distToGoal = distToGoal;
        nextCan = cans[nextCan].next;
        if (nextCan == -1) {  // our can capacity is full clear out an old one
            // ToDo: write a function that searches for oldest timestamp
        }
    } else {
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
    } else
        pulseEnd = micros();
    sonarDist = ((pulseEnd - pulseStart) / 147.0) * 2.54;
    if (sonarDist < 0)
        sonarDist = 300;
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
        lcd.setBacklight(BLANK);
        break;
    case 2:
        lcd.setBacklight(WHITE);
        break;
    case 3:
        lcd.setBacklight(BLANK);
        break;
    case 4:
        lcd.setBacklight(RED);
        break;
    case 5:
        lcd.setBacklight(BLANK);
        break;
    case 6:
        lcd.setBacklight(BLUE);
        break;
    case 7:
        lcd.setBacklight(BLANK);
        break;
    case 8:
        lcd.setBacklight(GREEN);
        break;
    case 9:
        lcd.setBacklight(BLANK);
        break;
    case 10:
        lcd.setBacklight(VIOLET);
        break;
    case 11:
        lcd.setBacklight(BLANK);
        break;
    case 12:
        lcd.setBacklight(TEAL);
        break;
    case 14:
        lcd.setBacklight(BLANK);
        break;
    case 15:
        lcd.setBacklight(YELLOW);
        cCycle = 1;
        celebrateGoal = false;
        break;
    }
    cCycle++;
}

void getPulseTime() {
    pulseEnd = micros();
}

void debugIr() {
#if DEBUG_USE_LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("   ");
    lcd.print(RobotBase.irDistance(IRFL));
    lcd.print("  ");
    lcd.print(RobotBase.irDistance(IRF));
    lcd.print("  ");
    lcd.print(RobotBase.irDistance(IRFR));

    lcd.setCursor(0, 1);
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

void debugSonar() {
#if DEBUG_SONAR
#if DEBUG_USE_LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sonar: ");
    lcd.print(sonarDist);
#endif
#if DEBUG_USE_SERIAL
    Serial.print("Sonar:");
    Serial.println(sonarDist);
#endif
#endif
}

