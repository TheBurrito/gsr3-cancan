#ifndef _RobotBase_h_
#define _RobotBase_h_

#include "SimplePID.h"
#include "Arduino.h"

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

struct Point {
  double x;
  double y;
};

struct Pose {
  float angle;
  Point offset;
};

inline Point rotate(const Point& p, const double& theta) {
  double c = cos(theta);
  double s = sin(theta);
  Point n;

  n.x = p.x * c - p.y * s;
  n.y = p.x * s + p.y * c;
  
  return n;
}

class CRobotBase {
public:
	CRobotBase();
	virtual ~CRobotBase();
	
	void setPID(const double& p, const double& i, const double& d);
	void setAccel(const double& accel);
	
	void setMax(const double& maxVelocity, const double& maxTurn);
	void setMaxVel(const double& maxVelocity);
	void setMaxTurn(const double& maxTurn);
	
	void setVelocityRange(double maxVel, double deadVel, double minVel);
	void setTurnRange(double maxTurn, double deadTurn, double minTurn);
	void setTurnAdjust(double turnAdjust);
	
	void setOutputRange(int maxOut, int deadOut, int minOut);
	
	void setTicksPerUnit(const double& tpu);
	void setWidth(const double& width);

	void setOdomPeriod(long odom_ms);
	void setNavPeriod(long nav_ms);
	void setIRPeriod(long ir_ms);
	
	void setIRFilter(double factor);
	void setIRSamples(int ir_samples);
	void readAllIR();
	
	int irDistance(IR_Index ir);
	int irDiff(IR_Index ir);
	Point obsPos(IR_Index ir, const Pose& pose);
	
	void update();
	
	void reset();
	
	double getTheta();
	double getX();
	double getY();
	bool getTurning();
	
	double getFixTheta();
	double getFixX();
	double getFixY();
	
	void setFixX(const double& x);
	void setFixY(const double& y);
	void setFixTheta(const double& theta);
	
	void localizeX(const double& x);
	void localizeY(const double& y);
	void localizeTheta(const double& theta);
	
	void setLocalization(const double& x, const double& y, const double& theta);
	
	double getVelocity();
	
	void setTheta(const double& theta);
	void setY(const double& y);
	void setX(const double& x);
	
	int getLeftOut();
	int getRightOut();
	
	void setNavThresh(const double& posThresh, const double& thetaThresh);
	void setPosThresh(const double& posThresh);
	void setHeadingThresh(const double& thetaThresh);
	
	void driveTo(const double& x, const double& y, bool smooth);
	void driveTo(const double& x, const double& y, const double& theta, bool smooth);
	
	void turnToAndDrive(const double& x, const double& y, bool smooth);
	
	void turnTo(const double& theta, bool smooth);
	void turnTo(const double& x, const double& y, bool smooth);
	
	void setVelocityAndTurn(const double& vel, const double& turn);
	
	void stop(bool smooth);
	
	bool navDone();
	
	void time();	
	
private:
	
	void updateOdometry(const long& encL, const long& encR, const double& dt);
	void updateVelocity(double targetVel, double targetTurn, const double& dt);
	
	int readIR(IR_Index ir);
	
	double _accel;
	
	int _maxOut, _deadOut, _minOut;
	
	double _maxVel, _deadVel, _minVel;
	double _maxTurn, _deadTurn, _minTurn, _turnAdjust;
		
	double _lastDistL, _lastDistR, _tpu;
	double _dDistL, _dDistR, _width;
	double _distSumL, _distSumR;
	
	double _curVelL, _curVelR, _curVel;
	
	double _theta, _posX, _posY;
	double _fixTheta, _fixX, _fixY;
	
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
	int _irDiff[IR_END];
	
	double _irFact;
	
	int _irSamples;
	
	SimplePID::PID _pidL, _pidR;
};

extern CRobotBase RobotBase;

#endif //_RobotBase_h_
