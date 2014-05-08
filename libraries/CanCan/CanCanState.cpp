#include "CanCanState.h"

#include "RobotBase.h"

void runState();
TimedAction stateAction = TimedAction(10, runState);

int delayMS = 1000;

Mode curState = mMenu, lastState;
bool restart = true;

void setState(Mode m) {
	curState = m;
}

void restartState() {
	restart = true;
}

void menuMode(bool init) {
	if (init) {
	} else {
	}
}

void delayMode(bool init) {
	if (init) {
	} else {
	}
}

void navMode(bool init) {
	if (init) {
	} else {
	}
}

void driveCanMode(bool init) {
	if (init) {
	} else {
	}
}

void grabMode(bool init) {
	if (init) {
	} else {
	}
}

void driveGoalMode(bool init) {
	if (init) {
	} else {
	}
}

void driveInMode(bool init) {
	if (init) {
	} else {
	}
}

void dropMode(bool init) {
	if (init) {
	} else {
	}
}

void backupMode(bool init) {
	if (init) {
	} else {
	}
}

void localizeMode(bool init) {
	if (init) {
	} else {
	}
}

void stopMode(bool init) {
	if (init) {
	} else {
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

void runState() {
	static Mode lastState;
	bool initState = (lastState != curState || restart);
	lastState = curState;
	restart = false;
	
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
