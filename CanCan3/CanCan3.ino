#include <RobotBase.h>
#include <TimedAction.h>
#include <Wire.h>
#include <LiquidTWI2.h>
#include <Adafruit_RGBLCDShield.h>
#include <Servo.h>
#include <Pins.h>
#include <CircularBuffer.h>
#include <CanCanState.h>

bool DEBUG_useLCD = false;
bool DEBUG_useSerial = false;
bool DEBUG_IR = false;
bool DEBUG_detectCan = false;
bool DEBUG_sonar = false;

Servo gripper;
#define SERVO_G_CLOSE 36
#define SERVO_G_OPEN 88

struct ArenaSize {
  float width;
  float length;
  Point min;
  Point max;
  Point goal;
};

LiquidTWI2 lcd(0);

#define OFF 0x0
#define RED 0x1
#define GREEN 0x2
#define YELLOW 0x3
#define BLUE 0x4
#define VIOLET 0x5
#define TEAL 0x6
#define WHITE 0x7

#define ROBOT_GRIP_OFFSET 8.7  // gripper closed position
#define ROBOT_FRONT_OFFSET 7.3 // to front of case
#define ROBOT_REAR_OFFSET 10.7 // to rear of case
#define ROBOT_WHEEL_OFFSET 9.2 // to outside of wheel

float wallBuffer = 10;

int arenaNum = 2;
const int arenaCount = 3;

ArenaSize arena[arenaCount];

void setArenas() {
  arena[0] = 
    {
      7, 10,
      {
        0 + wallBuffer + ROBOT_FRONT_OFFSET + ROBOT_GRIP_OFFSET, -7 / 2 * 12 * 2.54 + wallBuffer + ROBOT_FRONT_OFFSET + ROBOT_GRIP_OFFSET      }
      ,
      {
        10 * 12 * 2.54 - wallBuffer - ROBOT_FRONT_OFFSET - ROBOT_GRIP_OFFSET, 7 / 2 * 12 * 2.54 - wallBuffer - ROBOT_FRONT_OFFSET - ROBOT_GRIP_OFFSET      }
      ,
      {
        10 * 12 * 2.54 + 20, 0      }
    };

    arena[1] = {
      0, 0, {
        0, 0      }
      , {
        0, 0      }
      , {
        0, 0      }
    };

    arena[2] = {
      4, 8,
      {
        0 + wallBuffer + ROBOT_FRONT_OFFSET + ROBOT_GRIP_OFFSET, -4 / 2 * 12 * 2.54 + wallBuffer + ROBOT_FRONT_OFFSET + ROBOT_GRIP_OFFSET      }
      ,
      {
        8 * 12 * 2.54 - wallBuffer - ROBOT_FRONT_OFFSET - ROBOT_GRIP_OFFSET, 4 / 2 * 12 * 2.54 - wallBuffer - ROBOT_FRONT_OFFSET - ROBOT_GRIP_OFFSET      }
      ,
      {
        180 * 12 * 2.54 + 20, 0      }
    };
}

void setup() {
}

void loop() {
}

