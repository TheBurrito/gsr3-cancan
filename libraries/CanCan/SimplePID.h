#ifndef _SimplePID_h_
#define _SimplePID_h_

namespace SimplePID {

typedef struct {
	double p;
	double i;
	double d;
	
	double sum_max;
	
	double sum;
	double error;
} PID;

inline double timeSecs();

void initPID(PID& pid, double p, double i, double d);
void initPID(PID& pid, double p, double i, double d, double sum_max);

double calcPID(PID& pid, const double error, const double dt);

void resetPID(PID& pid);

} //namespace SimplePID

#endif //_SimplePID_h_
