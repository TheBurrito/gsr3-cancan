#include <Arduino.h>
#include <DualMC33926MotorShield.h>
#include <RobotBase.h>
#include <Encoder.h>
#include <Pins.h>

#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>

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

unsigned long basePeriod = 10; //10mS period, 100Hz
unsigned long lastBase;
unsigned long curMillis;

Encoder m1Enc(m1EncA, m1EncB); 
Encoder m2Enc(m2EncB, m2EncA); // reversed so forward counts up

DualMC33926MotorShield md;

void setup() {
  md.init();

  //Establish PID gains
  RobotBase.setPID(20, 5, 0);

  //Set allowed accel for wheel velocity targets (cm/s/s)
  RobotBase.setAccel(10.0);

  //Set max velocity and turn rate
  RobotBase.setMax(50.0, 0.5); //30cm/s, 0.2 Rad/s

  //set motor output ranges - works both positive and negative
  //Max, dead zone, min
  // -deadZone > X < deadZone : X = 0
  // X < min : X = min
  // X > max : x = max
  RobotBase.setOutput(400, 10, 50);

  //set ticks per desired distance unit
  RobotBase.setTicksPerUnit(71.65267); //units of cm

  //set the wheelbase of in desired distance unit
  RobotBase.setWidth(17.5); //units of cm

  Serial.begin(115200);
  lcdSetup();
  lastBase = millis();
}

void loop() {
  curMillis = millis();
  

  long encL = m1Enc.read();
  long encR = m2Enc.read();

  //determine goal velocity (usually just distance)
  double targetVel = 22.94; //15cm/s

  //determine goal turn rate (usually just heading error)
  double targetTurn = 0.0; //0.1 Rad/s

  if (curMillis - lastBase >= basePeriod) {
    
    lastBase = curMillis;

    RobotBase.update(encL, encR, targetVel, targetTurn, 0.001 * basePeriod);
   md.setM1Speed(RobotBase.getLeftOut());
   md.setM2Speed(RobotBase.getRightOut());

    //***********************************
    //replace following commented function calls with actual output calls
    //motor1.output(RobotBase.getLeft();
    //motor2.output(RobotBase.getRight();
    //***********************************

    //debug output to terminal / LCD

    Serial.print(RobotBase.getX(), DEC);
    Serial.print(",");
    Serial.println(RobotBase.getY(), DEC);
    

  }
}

void lcdSetup() {
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  lcd.setBacklight(WHITE);
}
