#ifndef _RobotBase_h_
#define _RobotBase_h_

#include "SimplePID.h"

class CRobotBase {
public:
	CRobotBase();
	virtual ~CRobotBase();
	
	void setPID(const double& p, const double& i, const double& d);
	void setAccel(const double& accel);
	void setMax(const double& maxVelocity, const double& maxTurn);
	void setOutput(int maxOut, int deadOut, int minOut);
	void setTicksPerUnit(const double& tpu);
	void setWidth(const double& width);
	
	void update(const long& encL, const long& encR, const double& targetVel, const double& targetTurn, const double& dt);
	
	double getTheta();
	double getX();
	double getY();
	
	int getLeftOut();
	int getRightOut();
	
private:
	
	void updateOdometry(const long& encL, const long& encR, const double& dt);
	void updateVelocity(double targetVel, double targetTurn, const double& dt);
	
	double _accel, _maxVel, _maxTurn;
	
	int _maxOut, _deadOut, _minOut;
	
	double _lastDistL, _lastDistR, _tpu;
	double _dDistL, _dDistR, _width;
	
	double _curVelL, _curVelR;
	
	double _theta, _posX, _posY;
	
	double _velL, _velR;
	
	int _left, _right;
	
	SimplePID::PID _pidL, _pidR;
};

extern CRobotBase RobotBase;

#endif //_RobotBase_h_
