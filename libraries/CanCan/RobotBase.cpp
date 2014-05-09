#include "RobotBase.h"

#include <Arduino.h>
//#include <algorithm>
#include <DualMC33926MotorShield.h>
#include <Encoder.h>
#include "Pins.h"

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
  
    analogReference(DEFAULT);
	
	_distSumL = 0;
	_distSumR = 0;
	
	_theta = 0;
	_posX = 0;
	_posY = 0;
	
	_fixTheta = 0;
	_fixX = 0;
	_fixY = 0;
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
	
void CRobotBase::setVelocityRange(double maxVel, double deadVel, double minVel) {
	_maxVel = maxVel;
	_deadVel = deadVel;
	_minVel = minVel;
}

void CRobotBase::setTurnRange(double maxTurn, double deadTurn, double minTurn) {
	_maxTurn = maxTurn;
	_deadTurn = deadTurn;
	_minTurn = minTurn;
}

void CRobotBase::setTicksPerUnit(const double& tpu) {
	_tpu = tpu;
}

void CRobotBase::setWidth(const double& width) {
	_width = width;
}

void CRobotBase::reset() {
	SimplePID::resetPID(_pidL);
	SimplePID::resetPID(_pidR);
}

double CRobotBase::getTheta() {
	return _theta + _fixTheta;
}

double CRobotBase::getX() {
	return _posX + _fixX;
}

double CRobotBase::getY() {
	return _posY + _fixY;
}
	
void CRobotBase::setTheta(const double& theta) {
	_theta = theta;
}

void CRobotBase::setY(const double& y) {
	_posY = y;
}

void CRobotBase::setX(const double& x) {
	_posX = x;
}
	
double CRobotBase::getFixTheta() {
	return _fixTheta;
}

double CRobotBase::getFixX() {
	return _fixX;
}

double CRobotBase::getFixY() {
	return _fixY;
}
	
void CRobotBase::localizeX(const double& x) {
	_fixX = x - _posX;
}

void CRobotBase::localizeY(const double& y) {
	_fixY = y - _posY;
}

void CRobotBase::localizeTheta(const double& theta) {
	_fixTheta = theta - _theta;
}
	
void CRobotBase::setLocalization(const double& x, const double& y, const double& theta) {
	_fixX = x - _posX;
	_fixY = y - _posY;
	_fixTheta = theta - _theta;
}

void CRobotBase::setFixX(const double& x) {
	_fixX = x;
}

void CRobotBase::setFixY(const double& y) {
	_fixY = y;
}

void CRobotBase::setFixTheta(const double& theta) {
	_fixTheta = theta;
}

double CRobotBase::getVelocity() {
	return _curVel;
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
	
	_distSumL += _dDistL;
	_distSumR += _dDistR;
	
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
	//Serial.print(targetTurn);
	
	int signVel = sgn(targetVel);
	int signTurn = sgn(targetTurn);
	double absVel = abs(targetVel);
	double absTurn = abs(targetTurn);
	
	//Serial.print(" MIN: ");
	//Serial.print(_minVel);
	//Serial.print(", ");
	//Serial.print(_minTurn);
	
	if (absVel < _deadVel) targetVel = 0;
	else if (absVel < _minVel) targetVel = _minVel * signVel;
	else if (absVel > _maxVel) targetVel = _maxVel * signVel;
	
	if (absTurn < _deadTurn) targetTurn = 0;
	else if (absTurn < _minTurn) targetTurn = _minTurn * signTurn;
	else if (absTurn > _maxTurn) targetTurn = _maxTurn * signTurn;
	
	//Serial.print(" A: ");
	//Serial.print(targetVel);
	//Serial.print(", ");
	//Serial.print(targetTurn);

	//Serial.println();
	
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
	_curVel = (_curVelL + _curVelR) / 2;
	
	_velL = _distSumL / dt;
	_velR = _distSumR / dt;
	
	_distSumL = 0;
	_distSumR = 0;
	
	//Serial.print("V: ");
	//Serial.print(_velL);
	//Serial.print(", ");
	//Serial.println(_velR);
	
	double errorL = leftVel - _velL;
	double errorR = rightVel - _velR;
	
	//Serial.print("E: ");
	//Serial.print(errorL);
	//Serial.print(", ");
	//Serial.println(errorR);
	
	_left = (int)(SimplePID::calcPID(_pidL, errorL, dt) /*+ errorL * 7*/);
	_right = (int)(SimplePID::calcPID(_pidR, errorR, dt) /*+ errorR * 7*/);
	
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
		double dX, dY;
		double dTheta;
		double targetVelocity = 0, targetTurn = 0;
		
		if (_driving || _turning) {
			dX = _navX - getX();
			dY = _navY - getY();
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
			
			dTheta = targetTheta - getTheta();
			
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
				if (_turnFirst) {
					stop(false);
					_driving = true;
					dX = 0;
				}
				
				dTheta = 0;
			}
		}
		
		if (_velocity) {
			targetVelocity = _targetVelocity;
			targetTurn = _targetTurn;
		} else if (_driving) {
			targetVelocity = dX;
			targetTurn = dTheta;
		} else if (_turning) {
			targetVelocity = 0;
			targetTurn = dTheta;
		} else {
			stop(false);
		}
		
		dt = (curMillis - _lastNav) * 0.001;
		updateVelocity(targetVelocity, targetTurn, dt);
		
		_lastNav = curMillis;
	}		
}
	
void CRobotBase::setVelocityAndTurn(const double& vel, const double& turn) {
	stop(false);
	
	_targetVelocity = vel;
	_targetTurn = turn;
	
	_velocity = true;
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

void CRobotBase::driveTo(const double& x, const double& y, bool smooth) {
	stop(smooth);
	
	_navX = x;
	_navY = y;
	
	_driving = true;
}

void CRobotBase::driveTo(const double& x, const double& y, const double& theta, bool smooth) {
	stop(smooth);
	
	_navX = x;
	_navY = y;
	_navTheta = theta;
	
	_driving = true;
	_turning = true;
}

void CRobotBase::turnTo(const double& theta, bool smooth) {
	stop(smooth);
	
	_navTheta = theta;
	
	_turning = true;
}

void CRobotBase::turnTo(const double& x, const double& y, bool smooth) {
	stop(smooth);
	
	double dX = x - getX();
	double dY = y - getY();
	_navTheta = atan2(dY, dX);
	
	_turning = true;
}

void CRobotBase::turnToAndDrive(const double& x, const double& y, bool smooth) {
	turnTo(x, y, smooth);
	
	_navX = x;
	_navY = y;
	
	_turnFirst = true;
}

void CRobotBase::stop(bool smooth) {
	_driving = false;
	_turning = false;
	_turnFirst = false;
	_velocity = false;
	
	if (!smooth) {
		_curVelL = 0;
		_curVelR = 0;
		reset();
	}
}

void CRobotBase::time() {
	_lastOdom = millis();
	_lastNav = _lastOdom;
}

bool CRobotBase::navDone() {
	return !(_driving || _turning);
}
