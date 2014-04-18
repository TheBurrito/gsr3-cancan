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
	void setMaxVel(const double& maxVelocity);
	void setMaxTurn(const double& maxTurn);
	
	void setOutputRange(int maxOut, int deadOut, int minOut);
	
	void setTicksPerUnit(const double& tpu);
	void setWidth(const double& width);
	
	void setOdomPeriod(long odom_ms);
	void setNavPeriod(long nav_ms);
	
	void update();
	
	void update(const long& encL, const long& encR, const double& targetVel, const double& targetTurn, const double& dt);
	
	void reset();
	
	double getTheta();
	double getX();
	double getY();
	
	int getLeftOut();
	int getRightOut();
	
	void setNavThresh(const double& posThresh, const double& thetaThresh);
	void setPosThresh(const double& posThresh);
	void setHeadingThresh(const double& thetaThresh);
	
	void driveTo(const double x, const double& y);
	void driveTo(const double x, const double& y, const double& theta);
	
	void turnTo(const double& theta);
	void turnTo(const double& x, const double& y);
	
	void stop();
	
	bool navDone();
	
	void time();	
	
private:
	
	void updateOdometry(const long& encL, const long& encR, const double& dt);
	void updateVelocity(double targetVel, double targetTurn, const double& dt);
	
	double _accel, _maxVel, _maxTurn;
	
	int _maxOut, _deadOut, _minOut;
	
	double _lastDistL, _lastDistR, _tpu;
	double _dDistL, _dDistR, _width;
	
	double _curVelL, _curVelR;
	
	double _theta, _posX, _posY;
	
	double _thetaAccel, _curTurn;
	
	double _velL, _velR;
	
	int _left, _right;
	
	double _navX, _navY, _navTheta;
	double _navThresh, _thetaThresh;
	bool _driving;
	bool _turning;
	
	long _odom_ms;
	long _nav_ms;
	
	unsigned long _lastOdom;
	unsigned long _lastNav;
	
	SimplePID::PID _pidL, _pidR;
};

extern CRobotBase RobotBase;

#endif //_RobotBase_h_
