#include "SimplePID.h"

#include <Arduino.h>

namespace SimplePID {

double timeSecs() {
	return millis() / 1000.0;
}

void initPID(PID& pid, double p, double i, double d) {
	pid.p = p;
	pid.i = i;
	pid.d = d;
	pid.error = 0;
	pid.sum = 0;
	pid.sum_max = 0;
}

void initPID(PID& pid, double p, double i, double d, double sum_max) {
	initPID(pid, p, i, d);
	pid.sum_max = sum_max;
}

void resetPID(PID& pid) {
	pid.error = 0;
	pid.sum = 0;
}

double calcPID(PID& pid, const double error, const double dt) {
	pid.sum += error * dt;
	
	double p = pid.p * error;
	double i = pid.i * pid.sum;
	double d = pid.d * (pid.error - error) / dt;
	
	/*if (pid.sum_max && pid.i) {
		double max = pid.sum_max - p;
		if (i  > max ) {
			pid.sum = max / pid.i;
			i = max;
		} else if (i < -max) {
			pid.sum = -max / pid.i;
			i = max;
		}
	}*/
	
	pid.error = error;
	
	return p + i - d;
}

}
