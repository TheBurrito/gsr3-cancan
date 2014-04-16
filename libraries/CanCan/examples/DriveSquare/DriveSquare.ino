#include <Arduino.h>
#include <RobotBase.h>

unsigned long basePeriod = 10; //10mS period, 100Hz
unsigned long lastBase;
unsigned long curMillis;

//Navigation target distance 
#define DIST 2.0

#define BASE_P 1000
#define BASE_I 10
#define BASE_D 0

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
	RobotBase.setTicksPerUnit(223.48); //units of cm
	
	//set the wheelbase of in desired distance unit
	RobotBase.setWidth(13.5); //units of cm
	
	Serial.begin(115200);
	
	curPt = 0;
	
	lastBase = millis();
}

void loop() {
	curMillis = millis();
	
	while (curPt < numPts) {
		if (
		long encL = 0; //replace with actual encoder readings
		long encR = 0;
	
		//determine goal velocity (usually just distance)
		double targetVel = 15.0; //15cm/s
	
		//determine goal turn rate (usually just heading error)
		double targetTurn = 0.1; //0.1 Rad/s
	
		if (curMillis - lastBase >= basePeriod) {
			lastBase = curMillis;
		
			RobotBase.update(encL, encR, targetVel, targetTurn, 0.001 * basePeriod);
		
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
