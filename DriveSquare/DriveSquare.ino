#include <Arduino.h>
#include <RobotBase.h>

//x, y pairs. X is forward and 0 degree heading
double wayPts[] = {
  50.0,   0.0,
  50.0,  50.0,
  0.0,   50.0,
  0.0,    0.0
};

int mode = 0;

//Number of coordinate pairs
int numPts = 4;
int curPt = -1;

void setup() {

  //Establish PID gains
  RobotBase.setPID(12, 5, 0);

  //Set allowed accel for wheel velocity targets (cm/s/s)
  RobotBase.setAccel(100);

  //Set max velocity and turn rate
  RobotBase.setMax(30, 3.0); //30cm/s, 0.2 Rad/s

  //set motor output ranges - works both positive and negative
  //Max, dead zone, min
  // -deadZone > X < deadZone : X = 0
  // X < min : X = min
  // X > max : x = max
  RobotBase.setOutputRange(400, 15, 50);

  //set ticks per desired distance unit
  RobotBase.setTicksPerUnit(71.65267); //units of cm

  //set the wheelbase of in desired distance unit
  RobotBase.setWidth(17.5); //units of cm

  RobotBase.setNavThresh(2, 0.02);
  RobotBase.setOdomPeriod(10);
  RobotBase.setNavPeriod(20);
  RobotBase.setIRPeriod(10);

  Serial.begin(115200);

  delay(5000);
  
  //RobotBase.driveTo(50, 0);

  RobotBase.time();
}

void loop() {
  RobotBase.update();
  
  switch (mode) {
    case 0:
      curPt = (curPt + 1) % numPts;
      RobotBase.turnToAndDrive(wayPts[curPt * 2], wayPts[curPt * 2 + 1]);
      mode = 1;
      break;
      
    case 1:
      if (RobotBase.navDone()) {
        mode = 0;
      }
      break;
  }
}




