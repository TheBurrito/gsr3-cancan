#include "ranging.h"
#include "Pins.h"

namespace ranging {

unsigned long lastUpdate = 0, period = 8;
int curSensor = IRL;

SharpIR irL(IR_L, 1, 0, 20150);
SharpIR irFL(IR_FL, 1, 0, 1080);
SharpIR irF(IR_F, 1, 0, 1080);
SharpIR irFR(IR_FR, 1, 0, 1080);
SharpIR irR(IR_R, 1, 0, 20150);

SharpIR* ir[5] = {&irL, &irFL, &irF, &irFR, &irR};

int dist[5];

void update() {
	unsigned long curMillis = millis();
	
	if (lastUpdate + period <= curMillis) {
		dist[curSensor] = ir[curSensor]->distance();

		++curSensor;
		if (curSensor == IR_END) {
			curSensor = 0;
		}
	}
}

void setPeriod(unsigned long ms) {
	period = ms;
}

int getDistance(IR_Index i) {
	return dist[i];
}

}
