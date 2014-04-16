#include <Arduino.h>
#include <DualMC33926MotorShield.h>
#include <RobotBase.h>
#include <Encoder.h>
#include <Pins.h>

unsigned long basePeriod = 10; //10mS period, 100Hz
unsigned long lastBase;
unsigned long curMillis;

//Navigation target distance 
#define DIST 2.0

#define BASE_P 20
#define BASE_I 5
#define BASE_D 0

Encoder m1Enc(m1EncA, m1EncB); 
Encoder m2Enc(m2EncB, m2EncA); // reversed so forward counts up

DualMC33926MotorShield md;

//x, y pairs. X is forward and 0 degree heading
double wayPts[] = {
	100.0,   0.0,
	100.0, 100.0,
	  0.0, 100.0,
	  0.0,   0.0
};

//Number of coordinate pairs
int numPts = 4;
int curPt;

void setup() {
	//Establish PID gains
	RobotBase.setPID(BASE_P, BASE_I, BASE_D);
	
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
	RobotBase.setTicksPerUnit(223.48); //units of cm
	
	//set the wheelbase of in desired distance unit
	RobotBase.setWidth(17.4); //units of cm
	
	Serial.begin(115200);
	
	curPt = 0;
	
	lastBase = millis();
}

void loop() {
	curMillis = millis();
	
	while (curPt < numPts) {
		double x = RobotBase.getX(), y = RobotBase.getY();
		double dX = wayPts[curPt * 2] - x, dY = wayPts[curPt * 2 + 1] - y;
		double dist = hypot(dX, dY);
		
		if (dist < DIST) {
			curPt++;
		} else {
			long encL = m1Enc.read();
			long encR = m2Enc.read();
			
			double dTheta = atan2(dY, dX);
	
			if (curMillis - lastBase >= basePeriod) {
				lastBase = curMillis;
		
				RobotBase.update(encL, encR, dist, dTheta, 0.001 * basePeriod);
		
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
			}
		}
	}
}
