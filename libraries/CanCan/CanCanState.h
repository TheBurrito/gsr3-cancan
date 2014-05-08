#ifndef cancanstate_h_
#define cancanstate_h_

#include <TimedAction.h>

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

extern int delayMS;
extern TimedAction stateAction;

void setState(Mode m);
void restartState();

#endif //cancanstate_h_
