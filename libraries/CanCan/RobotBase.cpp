#include "RobotBase.h"

#include <Arduino.h>

CRobotBase RobotBase;

CRobotBase::CRobotBase() {
}

CRobotBase::~CRobotBase(){
}

void CRobotBase::setPID(const double& p, const double& i, const double& d) {
	SimplePID::initPID(_pidL, p, i, d);
	SimplePID::initPID(_pidR, p, i, d);
}

void CRobotBase::setAccel(const double& accel) {
	_accel = accel;
}

void CRobotBase::setMax(const double& maxVelocity, const double& maxTurn) {
	_maxVel = maxVelocity;
	_maxTurn = maxTurn;
}

void CRobotBase::setOutput(int maxOut, int deadOut, int minOut) {
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
	
	if (targetTurn > _maxTurn) targetTurn = _maxTurn;
	else if (targetTurn < -_maxTurn) targetTurn = -_maxTurn;
	
	if (targetVel > _maxVel) targetVel = _maxVel;
	else if (targetVel < -_maxVel) targetVel = -_maxVel;
	
	double turn = 0.5 * _width * targetTurn;
	double leftVel = targetVel - turn;
	double rightVel = targetVel + turn;
	
	if (leftVel > _curVelL + _accel) leftVel = _curVelL + _accel;
	else if (leftVel < _curVelL - _accel) leftVel = _curVelL - _accel;
	
	if (rightVel > _curVelR + _accel) rightVel = _curVelR + _accel;
	else if (rightVel < _curVelR - _accel) rightVel = _curVelR - _accel;
	
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
}
