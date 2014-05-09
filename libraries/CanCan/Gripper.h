#ifndef gripper_h_
#define gripper_h_

#include <Servo.h>

class CGripper {
public:
	CGripper();
	~CGripper();
	
	void setPin(int pin);
	void setOpen(int val);
	void setClosed(int val);
	void setPeriod(unsigned long ms);
	void setSpeed(int speed);
	
	void open();
	void close();
	
	void moveTo(int val);
	
	void update();
	void quickMove();
	
	bool done();
	
private:
	Servo _servo;
	int _pin;
	
	int _open;
	int _closed;
	
	int _speed;
	
	int _target;
	int _current;
	
	unsigned long _lastUpdate;
	unsigned long _period;
};

extern CGripper Gripper;

#endif //gripper_h_
