#include "Detect.h"

#include <string.h>
#include "Arduino.h"
#include "gsr_lcd.h"
#include "debug.h"

namespace detect {

void detectCan();
void addCan(const Point& p);

struct DetectInfo {
	Point start;
	Point last;
	float width;
	bool active;
	int lastDist;
	int minDist;
	int maxDist;
	Pose pose;
};

struct CanInfo {
	unsigned long ts;
	Point p;
};

struct CanStore {
	CanInfo can;
	int next;
};

const int maxCans = 20;
CanStore cans[maxCans];
int numCans = 0;

DetectInfo *detectInfo = 0;
int sensorCount = 0;
int nextCan;

int *lastDist = 0;
float minCanWidth = 4;
float maxCanWidth = 8;

float edgeThresh = 5;
float minCanDist = 10;

Point minBounds, maxBounds;

unsigned long lastDetect = 0;
unsigned long detectMS = 40;

void setBounds(Point min, Point max) {
	minBounds = min;
	maxBounds = max;
}

void detectCan(int* dist, const Pose& pose, bool reset) {
	
	//iterate over each sensor used for can detection
	for (int i = 0; i < sensorCount; ++i) {
		int diff = dist[i] - detectInfo[i].lastDist;
		detectInfo[i].lastDist = dist[i];
		
		//if (reset) continue;
		
		bool edge = false;
		
		if (dist[i] > detectInfo[i].maxDist || dist[i] < detectInfo[i].minDist) {
			edge = detectInfo[i].active;
		} else {
			Point p;
			p.x = dist[i] * (cos(detectInfo[i].pose.a) ) + detectInfo[i].pose.p.x;
			p.y = dist[i] * (sin(detectInfo[i].pose.a) ) + detectInfo[i].pose.p.y;
			p = rotate(p, pose.a);
			p.x += pose.p.x;
			p.y += pose.p.y;
			
			if (p.x > maxBounds.x || p.x < minBounds.x || p.y > maxBounds.y || p.y < minBounds.y) {
				edge = detectInfo[i].active;
			} else if (!detectInfo[i].active || diff < -edgeThresh) {
			
				if (DEBUG_detect) {
					//setLCD("Start ");
					//lcd.print(diff);
				}
				
				detectInfo[i].start.x = p.x;
				detectInfo[i].start.y = p.y;
				detectInfo[i].last.x = p.x;
				detectInfo[i].last.y = p.y;
				detectInfo[i].active = true;
			} else if (detectInfo[i].active && diff > edgeThresh) {
				edge = true;
			} else if (detectInfo[i].active) {
				detectInfo[i].last.x = p.x;
				detectInfo[i].last.y = p.y;
			}
		}
		
		if (edge) {
			if (DEBUG_detect) {
				//setLCD("Stop ");
				//lcd.print(diff);
			}
			
			double ax = detectInfo[i].start.x - detectInfo[i].last.x;
			double ay = detectInfo[i].start.y - detectInfo[i].last.y;
			detectInfo[i].width = hypot(ax, ay);
			detectInfo[i].active = false;
			
			if (detectInfo[i].width < minCanWidth || detectInfo[i].width > maxCanWidth) {
				continue;
			}
			
			Point p;
			p.x = (detectInfo[i].start.x + detectInfo[i].last.x) / 2;
			p.y = (detectInfo[i].start.y + detectInfo[i].last.y) / 2;
			
			addCan(p);
			lcd.clear();
		}
	}
}

void initDetection() {
	nextCan = 0;

	for (int i = 0; i < maxCans; i++) {
		cans[i].next = i + 1;
		cans[i].can.ts = 0;
		if (i == maxCans - 1) {
			cans[i].next = -1;
		}
	}
	
	numCans = 0;
}

void setEdgeThreshold(float thresh) {
	edgeThresh = thresh;
}

void setBounds(float xmin, float ymin, float xmax, float ymax) {
	minBounds.x = xmin;
	minBounds.y = ymin;
	maxBounds.x = xmax;
	maxBounds.y = ymax;
}

void setSensorCount(int num) {
	DetectInfo * old = detectInfo;
	detectInfo = (DetectInfo *)malloc(num * sizeof(DetectInfo));
	
	if (old) {
		memcpy(detectInfo, old, sensorCount * sizeof(DetectInfo));
		free(old);
	}
	
	sensorCount = num;
}
	
void setSensorInfo(int i, const Pose& pose, int min, int max) {
	detectInfo[i].pose = pose;
	detectInfo[i].minDist = min;
	detectInfo[i].maxDist = max;
}

int canCount() {
	return numCans;
}

int closestCanTo(const Point& p) {
	float minDist = 100000;
	int minIndex = 0;
	
	for (int i = 0; i < maxCans; ++i) {
		float dist = hypot(cans[i].can.p.x - p.x, cans[i].can.p.y - p.y);
		if (dist < minDist) {
			minDist = dist;
			minIndex = i;
		}
	}
	
	return minIndex;
}

int oldestCan() {
	unsigned long oldest = millis();
	int oldestIndex = 0;
	
	for (int i = 0; i < maxCans; ++i) {
		if (cans[i].can.ts > 0 && cans[i].can.ts < oldest) {
			oldestIndex = i;
			oldest = cans[i].can.ts;
		}
	}
	
	return oldestIndex;
}

int newestCan() {
	unsigned long newest = 1;
	int newestIndex = 0;
	
	for (int i = 0; i < maxCans; ++i) {
		if (cans[i].can.ts > newest) {
			newest = cans[i].can.ts;
			newestIndex = i;
		}
	}
	
	return newestIndex;
}

void addCan(const Point &p) {
	for (int i = 0; i < maxCans; ++i) {
		if (cans[i].can.ts > 0 && abs(cans[i].can.p.x - p.x) < minCanDist && abs(cans[i].can.p.y - p.y) < minCanDist) {
			//duplicate can or too close to other can
			return;
		}
	}
	
	cans[nextCan].can.ts = millis();
	cans[nextCan].can.p = p;
	nextCan = cans[nextCan].next;
	
	++numCans;
}

const Point& getCan(int i) {
	return cans[i].can.p;
}
	
void removeCan(int i) {
	if (i < 0 || i > maxCans || cans[i].can.ts == 0) {
		return;
	}
	
	cans[i].can.ts = 0;
	cans[i].next = nextCan;
	nextCan = i;
	
	--numCans;
}

}
