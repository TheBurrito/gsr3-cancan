#include <Arduino.h>
#include <SharpIR.h>
#include <TimedAction.h>
#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>
#include <Pins.h>

#define DEBUG_USE_LCD true
#define DEBUG_USE_SERIAL false
#define DEBUG_IR true

// (Sensor Pin, No. Samples, % Range, Model)
SharpIR irL(IR_L, 10, 93, 20150);
SharpIR irFL(IR_FL, 10, 93, 1080);
SharpIR irF(IR_F, 10, 93, 1080);
SharpIR irFR(IR_FR, 10, 93, 1080);
SharpIR irR(IR_R, 10, 93, 20150); 

int irRDist;
int irFRDist;
int irFDist;
int irFLDist;
int irLDist;

TimedAction irAction = TimedAction(100,readIrSensors);  // Scan IR Sensors
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


#define TURN_THETA 0.1

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
  #define MAX_X 50  
  #define MAX_Y 230
  #define MIN_X -50
  #define MIN_Y 0
#endif

void setup() {

  Serial.begin(115200);

  lcdInit();
  delay(2000);

}

void loop() {
  irAction.check();
  #if DEBUG_IR
    debugIrAction.check();
  #endif
  
}

void readIrSensors() {
  float _objX,_objY,sensTheta;
  irRDist  = irR.distance();
  irLDist  = irL.distance();
  
  irFLDist = irFL.distance();
  if (irFLDist > 10 && irFLDist < 80) {  // ignore values outside the sensors specs
    // calculate detected objects x,y position
    //sensTheta = RobotBase.getTheta() - HALF_PI;
    _objX = irFLDist * (0.573576436);
    _objY = irFLDist * (0.819152044);
    
//    lcd.clear();
//    lcd.setCursor(0,0);
//    lcd.print(irFLDist);
    lcd.setCursor(0,1);
    lcd.print(int(_objX));
    lcd.print(" ");
    lcd.print(int(_objY));
     
  irFRDist = irFR.distance();
  if (irFRDist > 10 && irFRDist < 80) {  // ignore values outside the sensors specs
    _objX = irFRDist * (0.819152044);
    _objY = irFRDist * (0.573576436);
    
    lcd.print("  ");
    lcd.print(int(_objX));
    lcd.print(" ");
    lcd.print(int(_objY));
      
  }
    
}

  irFDist  = irF.distance();

  
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

