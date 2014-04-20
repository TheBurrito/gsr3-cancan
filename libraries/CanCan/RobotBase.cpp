#include "RobotBase.h"

#include <Arduino.h>

#include <DualMC33926MotorShield.h>
#include <Encoder.h>
#include "../Pins/Pins.h"

Encoder m1Enc(m1EncA, m1EncB); 
Encoder m2Enc(m2EncB, m2EncA); // reversed so forward counts up

DualMC33926MotorShield md;

CRobotBase RobotBase;

CRobotBase::CRobotBase() {
  md.init();
  _lastOdom = millis();
  _lastNav = _lastOdom;
  
  _driving = false;
  _turning = false;
  _turnFirst = false;
}

CRobotBase::~CRobotBase(){
}

void CRobotBase::setPID(const double& p, const double& i, const double& d) {
	SimplePID::initPID(_pidL, p, i, d);
	SimplePID::initPID(_pidR, p, i, d);
	reset();
}

void CRobotBase::setAccel(const double& accel) {
	_accel = accel;
}

void CRobotBase::setMax(const double& maxVelocity, const double& maxTurn) {
	_maxVel = maxVelocity;
	_maxTurn = maxTurn;
}

void CRobotBase::setOutputRange(int maxOut, int deadOut, int minOut) {
	_maxOut = maxOut;
	_deadOut = deadOut;
	_minOut = minOut;
}

void CRobotBase::setTicksPerUnit(const double& tpu) {
	_tpu = tpu;
}

void CRobotBase::setWidth(const double& width) {
	_width = width;
}

void CRobotBase::update(const long& encL, const long& encR, const double& targetVel, const double& targetTurn, const double& dt) {
	updateOdometry(encL, encR, dt);
	updateVelocity(targetVel, targetTurn, dt);
}

void CRobotBase::reset() {
	SimplePID::resetPID(_pidL);
	SimplePID::resetPID(_pidR);
}

double CRobotBase::getTheta() {
	return _theta;
}

double CRobotBase::getX() {
	return _posX;
}

double CRobotBase::getY() {
	return _posY;
}

int CRobotBase::getLeftOut() {
	return _left;
}

int CRobotBase::getRightOut() {
	return _right;
}

double forward;

void CRobotBase::updateOdometry(const long& encL, const long& encR, const double& dt) {
	double distL = encL / _tpu;
	double distR = encR / _tpu;
	
	_dDistL = distL - _lastDistL;
	_dDistR = distR - _lastDistR;
	
	_lastDistL = distL;
	_lastDistR = distR;
	
	forward = (_dDistL + _dDistR) / 2.0;
	_velL = _dDistL / dt;
	_velR = _dDistR / dt;
	
	_theta += (double)(_dDistR - _dDistL) / _width;
	if (_theta > PI) {
		_theta -= TWO_PI;
	} else if (_theta < -PI) {
		_theta += TWO_PI;
	}
	
	_posX += cos(_theta) * forward;
	_posY += sin(_theta) * forward;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void CRobotBase::updateVelocity(double targetVel, double targetTurn, const double& dt) {
	//Serial.print("T: ");
	//Serial.print(targetVel);
	//Serial.print(", ");
	//Serial.println(targetTurn);
	
	if (targetTurn > _maxTurn) targetTurn = _maxTurn;
	else if (targetTurn < -_maxTurn) targetTurn = -_maxTurn;
	
	if (targetVel > _maxVel) targetVel = _maxVel;
	else if (targetVel < -_maxVel) targetVel = -_maxVel;
	
	double turn = 0.5 * _width * targetTurn;
	double leftVel = targetVel - turn;
	double rightVel = targetVel + turn;
	
	double accel = _accel * dt;
	
	if (leftVel > _curVelL + accel) leftVel = _curVelL + accel;
	else if (leftVel < _curVelL - accel) leftVel = _curVelL - accel;
	
	if (abs(leftVel) < accel) leftVel = 0;
	if (abs(rightVel) < accel) rightVel = 0;
	
	if (rightVel > _curVelR + accel) rightVel = _curVelR + accel;
	else if (rightVel < _curVelR - accel) rightVel = _curVelR - accel;
	
	_curVelL = leftVel;
	_curVelR = rightVel;
	
	double errorL = leftVel - _velL;
	double errorR = rightVel - _velR;
	
	_left = (int)SimplePID::calcPID(_pidL, errorL, dt);
	_right = (int)SimplePID::calcPID(_pidR, errorR, dt);
	
	int signL = sgn(_left);
	int signR = sgn(_right);
	int absL = abs(_left);
	int absR = abs(_right);
	
	if (absL < _deadOut) _left = 0;
	else if (absL < _minOut) _left = _minOut * signL;
	else if (absL > _maxOut) _left = _maxOut * signL;
	
	if (absR < _deadOut) _right = 0;
	else if (absR < _minOut) _right = _minOut * signR;
	else if (absR > _maxOut) _right = _maxOut * signR;

    md.setM1Speed(_left);
    md.setM2Speed(_right);
}

void CRobotBase::setOdomPeriod(long odom_ms) {
	_odom_ms = odom_ms;
}

void CRobotBase::setNavPeriod(long nav_ms) {
	_nav_ms = nav_ms;
}

void CRobotBase::update() {
	unsigned long curMillis = millis();
	double dt;
	
	if (_lastOdom + _odom_ms <= curMillis) {
		long encL = m1Enc.read();
		long encR = m2Enc.read();
		
		dt = (curMillis - _lastOdom) * 0.001;
		
		updateOdometry(encL, encR, dt);
		
		_lastOdom = curMillis;
	}
	
	curMillis = millis();
	
	if (_lastNav + _nav_ms <= curMillis) {
		double dX;
		double dTheta;
		
		if (_driving || _turning) {
			dX = _navX - _posX;
			double dY = _navY - _posY;
			double dist = hypot(dX, dY);
			
			if (_driving && dist < _navThresh) {
				_driving = false;
			}
			
			double targetTheta = 0;
			if (_driving) {
				targetTheta = atan2(dY, dX);
			} else if (_turning){
				targetTheta = _navTheta;
			}
			
			dTheta = targetTheta - _theta;
			
			while (dTheta > PI) {
				dTheta -= TWO_PI;
			}
			
			while (dTheta < -PI) {
				dTheta += TWO_PI;
			}
			
			dTheta *= 2;
			
			if (_driving) {
				dX = cos(dTheta) * dist;
			} else if (_turning && abs(dTheta) < _thetaThresh) {
				_turning = false;
				dTheta = 0;
			}
		}
		
		if (!_driving) {
			dX = 0;
			
			if (!_turning) {
				dTheta = 0;
				
				if (_turnFirst) {
					_driving = true;
					_turnFirst = false;
				}
			}
		}
		
		//Serial.print("N: ");
		//Serial.print(dX);
		//Serial.print(", ");
		//Serial.println(dTheta);
		
		dt = (curMillis - _lastNav) * 0.001;
		updateVelocity(dX, dTheta, dt);
		
		_lastNav = curMillis;
	}				
}

void CRobotBase::setMaxVel(const double& maxVelocity) {
	_maxVel = maxVelocity;
}

void CRobotBase::setMaxTurn(const double& maxTurn) {
	_maxTurn = maxTurn;
}

void CRobotBase::setNavThresh(const double& posThresh, const double& thetaThresh) {
	_navThresh = posThresh;
	_thetaThresh = thetaThresh;
}

void CRobotBase::setPosThresh(const double& posThresh) {
	_navThresh = posThresh;
}

void CRobotBase::setHeadingThresh(const double& thetaThresh) {
	_thetaThresh = thetaThresh;
}

void CRobotBase::driveTo(const double& x, const double& y) {
	stop();
	
	_navX = x;
	_navY = y;
	
	_driving = true;
}

void CRobotBase::driveTo(const double& x, const double& y, const double& theta) {
	stop();
	
	_navX = x;
	_navY = y;
	_navTheta = theta;
	
	_driving = true;
	_turning = true;
}

void CRobotBase::turnTo(const double& theta) {
	stop();
	
	_navTheta = theta;
	
	_turning = true;
}

void CRobotBase::turnTo(const double& x, const double& y) {
	stop();
	
	double dX = x - _posX;
	double dY = y - _posY;
	_navTheta = atan2(dY, dX);
	
	_turning = true;
}

void CRobotBase::turnToAndDrive(const double& x, const double& y) {
	stop();
	
	double dX = x - _posX;
	double dY = y - _posY;
	_navTheta = atan2(dY, dX);
	_navX = x;
	_navY = y;
	
	_turning = true;
	_turnFirst = true;
}

void CRobotBase::stop() {
	_driving = false;
	_turning = false;
	_turnFirst = false;
	reset();
}

void CRobotBase::time() {
	_lastOdom = millis();
	_lastNav = _lastOdom;
}

bool CRobotBase::navDone() {
	return !(_driving || _turning);
}
