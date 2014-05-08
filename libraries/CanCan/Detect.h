#ifndef Detect_h_
#define Detect_h_

#include "common.h"
#include <TimedAction.h>

void initDetection();

void setEdgeThreshold(float thresh);

void setSensorCount(int num);
void setSensorInfo(int i, const Pose& pose, int min, int max);

int canCount();

int closestCanTo(const Point& robotPos);
int oldestCan();
int newestCan();

const Point& getCan(int i);
void removeCan(int i);

extern TimedAction detectAction;

#endif //Detect_h_
