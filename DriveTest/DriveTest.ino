#include <Arduino.h>
#include <DualMC33926MotorShield.h>
#include <RobotBase.h>
#include <Encoder.h>

#define m2EncA 3          // pin 3 (PWM / INT1)
// Motor Controller D2     // pin 4
#define m2EncB 5           // pin 6 (PWM)
#define m1EncA 6           // pin 5 (PWM)
// Motor Controller M1DIR  // pin 7 
// Motor Controller M2DIR  // pin 8 (PCINT0)
// Motor Controller M1PWM  // pin 9 (PWM / PCINT1 / OC1A)
// Motor Controller M2PWM  // pin 10 (PWM / SS / PCINT2 / OC1B)
#define m1EncB 11          // pin 11 (PWM / MOSI / PCINT3 / OC2A)
// Motor Controller SF     // pin 12 (MISO / PCINT4)
// LED pin 13 (SCK / PCINT5)
// Motor Controller M1FB    // pin A0 (14)
// Motor Controller M2FB    // pin A1 (15)
// pin A2 (16)
// pin A3 (17)
// pin A4 (18) (I2C SDA)
// pin A5 (19) (I2C SCL)
// pin A6 (20)
// pin A7 (21)
// pin A8 (22)
// pin A9 (23)
#define IR_R   A12 // pin A12 (Analog Only)
#define IRB_FR 24  // Bumper pin 24  
#define IRB_F  25  // pin 25 (PWM)
#define IR_FR A15  // pin 26 (A15)
#define IR_F  A16  // pin 27 (A16)
// pin 28 (A17)
// pin 29 (A18)
// pin 30 (A19)
#define IR_FL  A20 // pin 31 (A20)
int SERVO_G =  32; // pin 32 (PWM)
#define IRB_FL 33  // pin 33
#define IR_L   A13 // pin A13 (Analog Only)

unsigned long basePeriod = 10; //10mS period, 100Hz
unsigned long lastBase;
unsigned long curMillis;

Encoder m1Enc(m1EncA, m1EncB); 
Encoder m2Enc(m2EncB, m2EncA); // reversed so forward counts up

DualMC33926MotorShield md;

void setup() {
  md.init();

  //Establish PID gains
  RobotBase.setPID(1000, 10, 0);

  //Set allowed accel for wheel velocity targets (cm/s/s)
  RobotBase.setAccel(2.0);

  //Set max velocity and turn rate
  RobotBase.setMax(30.0, 0.2); //30cm/s, 0.2 Rad/s

  //set motor output ranges - works both positive and negative
  //Max, dead zone, min
  // -deadZone > X < deadZone : X = 0
  // X < min : X = min
  // X > max : x = max
  RobotBase.setOutput(200, 10, 50);

  //set ticks per desired distance unit
  RobotBase.setTicksPerUnit(71.65267); //units of cm

  //set the wheelbase of in desired distance unit
  RobotBase.setWidth(13.5); //units of cm

  Serial.begin(115200);

  lastBase = millis();
}

void loop() {
  curMillis = millis();
  
  /*m1Enc.read();
  m2CurrentPos = m2Enc.read();*/

  long encL = m1Enc.read();
  long encR = m2Enc.read();

  //determine goal velocity (usually just distance)
  double targetVel = 15.0; //15cm/s

  //determine goal turn rate (usually just heading error)
  double targetTurn = 0.1; //0.1 Rad/s

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
    Serial.print((int)(RobotBase.getTheta() * 180 / PI), DEC);
    Serial.print(",");
    Serial.print((int)RobotBase.getX(), DEC);
    Serial.print(",");
    Serial.println((int)RobotBase.getY(), DEC);
    
    double dist = hypot(RobotBase.getX(), RobotBase.getY());
  }
}

