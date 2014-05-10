#include "debug.h"
#include "TimedAction.h"
#include "gsr_lcd.h"
#include "ranging.h"

bool DEBUG_useLCD = true;
bool DEBUG_useSerial = true;

bool DEBUG_IR = true;
bool DEBUG_detect = true;
bool DEBUG_sonar = false;
bool DEBUG_path = false;
bool DEBUG_state = false;
bool DEBUG_gripper = true;

void doDebug();
void debugIR();
void debugPath();

TimedAction debugAction(200, doDebug);

void doDebug() {
	if (DEBUG_IR) {
		debugIR();
	}
}

void debug() {
	debugAction.check();
}

void debugIR() {
	if (DEBUG_useLCD) {
		lcd.clear();
		lcd.setCursor(3,0);
		lcd.print(ranging::getDistance(IRFL));
		lcd.print("  ");
		lcd.print(ranging::getDistance(IRF));
		lcd.print("  ");
		lcd.print(ranging::getDistance(IRFR));

		lcd.setCursor(0, 1);
		lcd.print(ranging::getDistance(IRL));
		lcd.print(" <--  --> ");
		lcd.print(ranging::getDistance(IRR));
	}
}
