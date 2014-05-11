
#include <Pins.h>
#include <TimedAction.h>
#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>

#define DEBUG_SONAR true

// These #defines make it easy to set the backlight color
#define OFF 0x0
#define RED 0x1
#define GREEN 0x2
#define YELLOW 0x3
#define BLUE 0x4
#define VIOLET 0x5
#define TEAL 0x6
#define WHITE 0x7

#if DEBUG_SONAR
TimedAction debugSonarAction = TimedAction(500,debugSonar);
#endif

TimedAction pingAction = TimedAction(200,ping);

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// Sonar start and end times in micros
volatile long pulseStart, pulseEnd;
volatile int sonarDist;

void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  sonarInit();  
  lcdInit();
}

void loop() {
  pingAction.check();
#if DEBUG_SONAR
  debugSonarAction.check();
#endif  
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
  if (digitalReadFast(SNR_PW) == HIGH) {
    pulseStart = micros();
  }
  else
    pulseEnd = micros();
    sonarDist = ((pulseEnd - pulseStart) / 147.0) * 2.54;
}

void lcdInit() {
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  lcd.setBacklight(WHITE);
}

void debugSonar() {
  Serial.print("Sonar:");
  Serial.println(sonarDist);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Sonar: ");
  lcd.print(sonarDist);  
}

void sonarInit() {
  pinMode(SNR_RX, OUTPUT);
  pinMode(SNR_PW, INPUT);  
  attachInterrupt(SNR_PW, readSonarPulse, CHANGE);
}

