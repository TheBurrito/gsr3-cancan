#ifndef _RobotBase_h_
#define _RobotBase_h_

#include "SimplePID.h"

typedef enum {
  IRL,
  IRFL,
  IRF,
  IRFR,
  IRR,

  IR_END,
  IR_ALL
} 
IR_Index;

typedef enum {
	IR_1080,
	IR_20150
} IR_Model;

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
	void setIRPeriod(long ir_ms);
	
	void setIRFilter(double factor);
	void readAllIR();
	
	int irDistance(IR_Index ir);
	int irDiff(IR_Index ir);
	
	void update();
	
	void reset();
	
	double getTheta();
	double getX();
	double getY();
	
	int getLeftOut();
	int getRightOut();
	
	void setNavThresh(const double& posThresh, const double& thetaThresh);
	void setPosThresh(const double& posThresh);
	void setHeadingThresh(const double& thetaThresh);
	
	void driveTo(const double& x, const double& y);
	void driveTo(const double& x, const double& y, const double& theta);
	
	void turnToAndDrive(const double& x, const double& y);
	
	void turnTo(const double& theta);
	void turnTo(const double& x, const double& y);
	
	void setVelocityAndTurn(const double& vel, const double& turn);
	
	void stop();
	
	bool navDone();
	
	void time();	
	
private:
	
	void updateOdometry(const long& encL, const long& encR, const double& dt);
	void updateVelocity(double targetVel, double targetTurn, const double& dt);
	
	int readIR(IR_Index ir);
	
	double _accel, _maxVel, _maxTurn;
	
	int _maxOut, _deadOut, _minOut;
	
	double _lastDistL, _lastDistR, _tpu;
	double _dDistL, _dDistR, _width;
	double _distSumL, _distSumR;
	
	double _curVelL, _curVelR;
	
	double _theta, _posX, _posY;
	
	double _thetaAccel, _curTurn;
	
	double _velL, _velR;
	
	int _left, _right;
	
	double _navX, _navY, _navTheta;
	double _navThresh, _thetaThresh;
	
	double _targetVelocity, _targetTurn;
	
	bool _turnFirst;
	bool _driving;
	bool _turning;
	bool _velocity;
	
	long _odom_ms;
	long _nav_ms;
	long _ir_ms;
	
	unsigned long _lastOdom;
	unsigned long _lastNav;
	unsigned long _lastIR;
	
	int _irPins[IR_END];
	IR_Model _irModels[IR_END];
	
	int _irDist[IR_END];
	int _irPrevDist[IR_END];
	
	double _irFact;
	
	SimplePID::PID _pidL, _pidR;
};

extern CRobotBase RobotBase;

#endif //_RobotBase_h_
