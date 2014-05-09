#ifndef Detect_h_
#define Detect_h_

#include "common.h"
#include "TimedAction.h"

namespace detect {

void initDetection();

void setEdgeThreshold(float thresh);

void setBounds(Point min, Point max);

void setSensorCount(int num);
void setSensorInfo(int i, const Pose& pose, int min, int max);

void detectCan(int* dist, const Pose& pose, bool reset);

int canCount();

int closestCanTo(const Point& robotPos);
int oldestCan();
int newestCan();

const Point& getCan(int i);
void removeCan(int i);

}

#endif //Detect_h_
