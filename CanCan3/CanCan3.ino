
#include <Wire.h>
#include <LiquidTWI2.h>
#include <Adafruit_RGBLCDShield.h>
#include <RobotBase.h>
#include <Gripper.h>
#include <ranging.h>
#include <TimedAction.h>
#include <Detect.h>
#include <geometry.h>
#include <CanCan.h>
#include <Servo.h>
#include <Pins.h>

#include <debug.h>

#include <CanCanState.h>

TimedAction detectAction = TimedAction(40, doDetect);

bool resetDetect = true;

void doDetect() {
  resetDetect = RobotBase.getVelocity() < 2.0;
  if (resetDetect) {
  	return;
  	}
  
  int dist[4];
  dist[0] = ranging::getDistance(IRL);
  dist[1] = ranging::getDistance(IRFL);
  dist[2] = ranging::getDistance(IRFR);
  dist[3] = ranging::getDistance(IRR);
  
  Pose pose;
  pose.p.x = RobotBase.getX();
  pose.p.y = RobotBase.getY();
  pose.a = RobotBase.getTheta();
  
  detect::detectCan(dist, pose, resetDetect);
}

void initDetect() {
  detect::initDetection();
  detect::setEdgeThreshold(10.0);
  detect::setBounds(arena.min, arena.max);
  detect::setSensorCount(4);
  detect::setSensorInfo(0, sonarPose[IRL], 20, 150);
  detect::setSensorInfo(1, sonarPose[IRFL], 10, 80);
  //Skip front center sensor for can detection
  detect::setSensorInfo(2, sonarPose[IRFR], 10, 80);
  detect::setSensorInfo(3, sonarPose[IRR], 20, 150);
}

void initBase() {
  RobotBase.setPID(5, 5, 0);
  RobotBase.setAccel(50);
  
  RobotBase.setVelocityRange(20, 0, 1);
  RobotBase.setTurnRange(2, 0, 0.2);
  RobotBase.setOutputRange(400, 5, 70);
  
  RobotBase.setTicksPerUnit(71.65267);
  RobotBase.setWidth(17.4);
  RobotBase.setNavThresh(2, 0.04);
  
  RobotBase.setOdomPeriod(10);
  RobotBase.setNavPeriod(10);
}

void initGripper() {
  Gripper.setPin(SERVO_G);
  Gripper.setOpen(88);
  Gripper.setClosed(36);
  Gripper.setSpeed(1);
  Gripper.setPeriod(20);
}

void initBumpers() {
  pinMode(IRB_FR, INPUT);
  pinMode(IRB_F, INPUT);
  pinMode(IRB_FL, INPUT);
}

void fillPath() {
  path.clear();
  Point p;
  
  //7
  p.x = arena.min.x;
  p.y = 0;
  path.push(p);
  
  //6
  p.x = arena.max.x;
  p.y = 0;
  path.push(p);
  
  //5
  p.x = arena.max.x;
  p.y = arena.min.y / 2;
  path.push(p);
  
  //4
  p.x = arena.min.x;
  p.y = arena.min.y / 2;
  path.push(p);
  
  //3
  p.x = arena.min.x;
  p.y = arena.max.y / 2;
  path.push(p);
  
  //2
  p.x = arena.max.x;
  p.y = arena.max.y / 2;
  path.push(p);
  
  //1
  p.x = arena.max.x;
  p.y = 0;
  path.push(p);
}
  
void setup() {
  initLCD();
  setArena(2, 10.0 + ROBOT_FRONT_OFFSET + ROBOT_GRIP_OFFSET);
  initDetect();
  initGripper();
  initBase();
  initBumpers();
}

void loop() {
  ranging::update();
  detectAction.check();
  stateAction.check();
  Gripper.update();
  RobotBase.update();
  
  debug();
  
  if (path.empty()) {
    fillPath();
  }
}

