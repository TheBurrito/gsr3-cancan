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
    
    _irPins[IRL] = IR_L;
    _irPins[IRFL] = IR_FL;
    _irPins[IRF] = IR_F;
    _irPins[IRFR] = IR_FR;
    _irPins[IRR] = IR_R;
    
    _irModels[IRL] = IR_20150;
    _irModels[IRFL] = IR_1080;
    _irModels[IRF] = IR_1080;
    _irModels[IRFR] = IR_1080;
    _irModels[IRR] = IR_20150;
    
    for (int i = 0; i < IR_END; ++i) {
    	pinMode(_irPins[i], INPUT);
	}
	
	_distSumL = 0;
	_distSumR = 0;
	_posX = 0;
	_posY = 0;
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
	
void CRobotBase::setVelocityRange(int maxVel, int deadVel, int minVel) {
	_maxVel = maxVel;
	_deadVel = deadVel;
	_minVel = minVel;
}

void CRobotBase::setTurnRange(int maxTurn, int deadTurn, int minTurn) {
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
	//Serial.println(targetTurn);
	
	int signVel = sgn(targetVel);
	int signTurn = sgn(targetTurn);
	double absVel = abs(targetVel);
	double absTurn = abs(targetTurn);
	
	if (absVel < _deadVel) targetVel = 0;
	else if (absVel < _minVel) targetVel = _minVel * signVel;
	else if (absVel > _maxVel) targetVel = _maxVel * signVel;
	
	if (absTurn < _deadTurn) targetTurn = 0;
	else if (absTurn < _minTurn) targetTurn = _minTurn * signTurn;
	else if (absTurn > _maxTurn) targetTurn = _maxTurn * signTurn;
	
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

void CRobotBase::setIRPeriod(long ir_ms) {
	_ir_ms = ir_ms;
}

void CRobotBase::setIRSamples(int ir_samples) {
	_irSamples = ir_samples;
}

int CRobotBase::irDistance(IR_Index ir) {
	return _irDist[ir];
}

int CRobotBase::irDiff(IR_Index ir) {
	return _irDist[ir] - _irPrevDist[ir];
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
	if (_lastIR + _ir_ms <= curMillis) {
		readAllIR();
	}
	
	curMillis = millis();
	if (_lastNav + _nav_ms <= curMillis) {
		double dX, dY;
		double dTheta;
		double targetVelocity = 0, targetTurn = 0;
		
		if (_driving || _turning) {
			dX = _navX - _posX;
			dY = _navY - _posY;
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
				if (_turnFirst) {
					_turnFirst = false;
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
		}
		
		dt = (curMillis - _lastNav) * 0.001;
		updateVelocity(targetVelocity, targetTurn, dt);
		
		_lastNav = curMillis;
	}				
}
	
void CRobotBase::setVelocityAndTurn(const double& vel, const double& turn) {
	stop();
	
	_targetVelocity = vel;
	_targetTurn = turn;
	
	_velocity = true;
}


	
int CRobotBase::readIR(IR_Index ir) {
	int raw, val, sum, min, max;
    float voltFromRaw; //based on 3.3V reference
    
    int puntualDistance = -1;
    
    switch (_irModels[ir]) {
    case IR_1080:
    	for (int i = 0; i < _irSamples; i++) {
    		raw = analogRead(_irPins[ir]);
    		voltFromRaw = raw * 3.225806452;
			val = 27.728 * pow(voltFromRaw / 1000, - 1.2045);
			Serial.println(val);
    		if ( i == 0) {
    			sum = val;
    			min = val;
    			max = val;
    		}
    		else {
				if (val < min) {
					min = val;
				}
				if (val > max) {
					max = val;
				}
				sum = sum + val;
    		}
    	}
    	
    	if (_irSamples > 2) {
    		puntualDistance = (sum - min - max) / (_irSamples - 2);
    	}
    	else puntualDistance = sum / _irSamples;

    	if (puntualDistance > 80) puntualDistance = 81;
        if (puntualDistance < 10) puntualDistance  = 9;
        break;
        
        
    case IR_20150:
    	for (int i = 0; i < _irSamples; i++) {
    		raw = analogRead(_irPins[ir]);
    		voltFromRaw = raw * 3.225806452;
    		val = 61.573 * pow(voltFromRaw / 1000, - 1.1068);
    		if (i == 0) {
    			sum = val;
    			min = val;
    			max = val;
    		}
    		else {
				if (val < min) {
					min = val;
				}
				if (val > max) {
					max = val;
				}
				sum = sum + val;
    		}
    	}

    	if (_irSamples > 2) {
    		puntualDistance = (sum - min - max) / (_irSamples - 2);
    	}
    	else puntualDistance = sum / _irSamples;

        if (puntualDistance > 150) puntualDistance = 151;
        if (puntualDistance < 20) puntualDistance  = 19;
        break;
    }
    
    _irDist[ir] = puntualDistance;  // * _irFact) + _irPrevDist[ir] * (1.0 - _irFact);
    _irPrevDist[ir] = _irDist[ir];
    
    return puntualDistance;
}
	
void CRobotBase::setIRFilter(double factor) {
	_irFact = factor;
}

void CRobotBase::readAllIR() {
	for (int i = 0; i < IR_END; ++i) {
		readIR((IR_Index)i);
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
	turnTo(x, y);
	
	_navX = x;
	_navY = y;
	
	_turnFirst = true;
}

void CRobotBase::stop() {
	_driving = false;
	_turning = false;
	_turnFirst = false;
	_velocity = false;
	reset();
}

void CRobotBase::time() {
	_lastOdom = millis();
	_lastNav = _lastOdom;
	_lastIR = _lastOdom;
}

bool CRobotBase::navDone() {
	return !(_driving || _turning);
}
