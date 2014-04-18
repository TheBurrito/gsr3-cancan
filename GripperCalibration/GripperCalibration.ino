
#include <Wire.h>
#include <LiquidTWI2.h>
#include <Adafruit_RGBLCDShield.h>
#include <Servo.h>
#include <Pins.h>

#define MAX = 1.938
#define MIN = 0.310

LiquidTWI2 lcd(0);

Servo servoG;

int posServoG;

void setup()
{
  servoG.attach(SERVO_G);  //the pin for the servo control 
//  servoG.write(125);
  lcdInit();

}

void loop()
{
  servoG.write(36);
  delay(1000);
  int rawServoG = analogRead(SERVO_G_FB);
  float voltServoG = rawServoG * (3.3 / 1023.0);
  posServoG = map(voltServoG*1000, 0.310*1000, 1.938*1000, 0, 180);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Volts:");
  lcd.print(voltServoG);
  lcd.setCursor(0,1);
  lcd.print("Angle:");
  lcd.print(posServoG);
  delay(1000);
  servoG.write(88);
  delay(1000);
  rawServoG = analogRead(SERVO_G_FB);
  voltServoG = rawServoG * (3.3 / 1023.0);
  posServoG = map(voltServoG*100, 0.310*100, 1.938*100, 0, 180);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Volts:");
  lcd.print(voltServoG);
  lcd.setCursor(0,1);
  lcd.print("Angle:");
  lcd.print(posServoG);
  delay(1000);

}


void lcdInit() {
  lcd.setMCPType(LTI_TYPE_MCP23017);
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  lcd.setBacklight(WHITE);
}





