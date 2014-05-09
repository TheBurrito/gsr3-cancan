#ifndef cancanstate_h_
#define cancanstate_h_

#include <TimedAction.h>
//#include <CircularBuffer.h>
#include "stack.h"
#include <common.h>
#include <CanCan.h>
#include "gsr_lcd.h"
#include "debug.h"

typedef enum {
	mMenu,
	mDelay,
	mNavigate,
	mDriveCan,
	mGrabCan,
	mDriveGoal,
	mDriveInGoal,
	mDropCan,
	mBackup,
	mLocalize,
	mStop,
	mResetGrab,
	mAvoid
} Mode;

void setState(Mode m);
void restartState();

Stack<Point> path;

void runState();
TimedAction stateAction = TimedAction(100, runState);

int delayMS = 1000;

Point navPoint;

Mode curState = mMenu, lastState = curState, delayState = mNavigate;
bool restart = true;

int curCan = -1;

void setState(Mode m) {
	curState = m;
}

void restartState() {
	restart = true;
}

void menuMode(bool init) {
	if (init) {
		//lcd.clear();
		//lcd.setCursor(0,0);
		//lcd.print("CanCan Soccer");
	} else {
		int buttons = lcd.readButtons();
		if (buttons & BUTTON_SELECT) {
			curState = mDelay;
			delayState = mNavigate;
		}
	}
}

unsigned long targetTime;

void delayMode(bool init) {
	if (init) {
		targetTime = millis() + delayMS;
	} else {
		if (millis() <= targetTime) {
			curState = delayState;
		}
	}
}

void navMode(bool init) {
	if (init) {
		path.peek(navPoint);
		RobotBase.turnToAndDrive(navPoint.x, navPoint.y, false);
		RobotBase.setMaxVel(20);
		
		if (DEBUG_path) {
			lcd.clear();
			lcd.setCursor(0,0);
			lcd.print(path.size());
			lcd.print(": ");
			lcd.print(navPoint.x);
			lcd.print(",");
			lcd.print(navPoint.y);
		}
	} else {
		if (RobotBase.navDone()) {
			path.pop(navPoint);
			restartState();
		} else {
			if (detect::canCount() > 0) {
				curState = mDriveCan;
			}
		}
	}
}

void driveCanMode(bool init) {
	if (init) {
		curCan = detect::closestCanTo(arena.goal);
		Point can = detect::getCan(curCan);
		
		if (DEBUG_detect) {
			lcd.clear();
			lcd.setCursor(0,0);
			lcd.print(curCan);
			lcd.print(":");
			lcd.print(can.x);
			lcd.print(",");
			lcd.print(can.y);
		}
		
		Point pos;
		pos.x = RobotBase.getX();
		pos.y = RobotBase.getY();
		
		float dx = can.x - pos.x;
		float dy = can.y - pos.y;
		float len = hypot(dx, dy);
		float r = (len - 10) / len;
		float x = dx * r + pos.x;
		float y = dy * r + pos.y;
		
		if (DEBUG_detect) {
			lcd.setCursor(0, 1);
			lcd.print(x);
			lcd.print(", ");
			lcd.print(y);
		}
		
		RobotBase.turnToAndDrive(x, y, false);
		RobotBase.setMaxVel(30);
	} else {
		if (RobotBase.navDone()) {
			curState = mGrabCan;
		}
	}
}

void grabMode(bool init) {
	if (init) {
		Gripper.close();
	} else {
		if (Gripper.done()) {
			if (digitalReadFast(IRB_F) == 0) {
				curState = mDriveGoal;
			} else {
				detect::removeCan(curCan);
				curCan = -1;
				
				Gripper.open();
				Gripper.quickMove();
				curState = mNavigate;
			}
		}
	}
}

void driveGoalMode(bool init) {
	if (init) {
		RobotBase.turnToAndDrive(arena.goal.x, arena.goal.y, false);
		RobotBase.setMaxVel(40);
	} else {
		if (RobotBase.navDone()) {
			curState = mDriveInGoal;
		}
			
		if (DEBUG_state) {
			lcd.setCursor(0,1);
			lcd.print(arena.goal.x - 40);
			lcd.print(",");
			lcd.print(arena.goal.y);
		}
	}
}

void driveInMode(bool init) {
	if (init) {
		RobotBase.turnToAndDrive(arena.goal.x + 20, arena.goal.y, false);
		RobotBase.setMaxVel(20);
	} else {
		if (RobotBase.navDone()) {
			curState = mDropCan;
		}
	}
}

void dropMode(bool init) {
	if (init) {
		Gripper.open();
	} else {
		if (Gripper.done()) {
			detect::removeCan(curCan);
			curCan = -1;
			
			curState = mBackup;
		}
	}
}

unsigned long backupTimer;
void backupMode(bool init) {
	if (init) {
		RobotBase.setVelocityAndTurn(-20, 0);
		backupTimer = millis() + 2000;
	} else {
		if (millis() >= backupTimer) {
			RobotBase.stop(true);
			curState = mLocalize;
		}
	}
}

void localizeMode(bool init) {
	if (init) {
		//For now, do nothing. Return to Navigate state
		curState = mNavigate;
	} else {
	}
}

void stopMode(bool init) {
	if (init) {
		RobotBase.stop(true);
	} else {
		//stuck forever!!
	}
}

void resetGrabMode(bool init) {
	if (init) {
	} else {
	}
}

void avoidMode(bool init) {
	if (init) {
	} else {
	}
}

void debugState() {
	switch (curState) {
		case mMenu:
		setLCD("menu");
		break;
	
	case mDelay:
		setLCD("delay");
		break;
	
	case mNavigate:
		setLCD("nav");
		break;
	
	case mDriveCan:
		setLCD("drivecan");
		break;
		
	case mGrabCan:
		setLCD("grabcan");
		break;
		
	case mDriveGoal:
		setLCD("drivegoal");
		break;
		
	case mDriveInGoal:
		setLCD("ingoal");
		break;
		
	case mDropCan:
		setLCD("dropcan");
		break;
		
	case mBackup:
		setLCD("backup");
		break;
		
	case mLocalize:
		setLCD("localize");
		break;
		
	case mStop:
		setLCD("stop");
		break;
		
	case mResetGrab:
		setLCD("resetgrab");
		break;
		
	case mAvoid:
		setLCD("avoid");
		break;
	}
}

void runState() {
	static Mode lastState;
	
	bool initState = (lastState != curState || restart);
	lastState = curState;
	restart = false;
	
	if (DEBUG_state) {
		debugState();
	}
	
	switch (curState) {
	case mMenu:
		menuMode(initState);
		break;
	
	case mDelay:
		delayMode(initState);
		break;
	
	case mNavigate:
		navMode(initState);
		break;
	
	case mDriveCan:
		driveCanMode(initState);
		break;
		
	case mGrabCan:
		grabMode(initState);
		break;
		
	case mDriveGoal:
		driveGoalMode(initState);
		break;
		
	case mDriveInGoal:
		driveInMode(initState);
		break;
		
	case mDropCan:
		dropMode(initState);
		break;
		
	case mBackup:
		backupMode(initState);
		break;
		
	case mLocalize:
		localizeMode(initState);
		break;
		
	case mStop:
		stopMode(initState);
		break;
		
	case mResetGrab:
		resetGrabMode(initState);
		break;
		
	case mAvoid:
		avoidMode(initState);
		break;
	}
}

#endif //cancanstate_h_
