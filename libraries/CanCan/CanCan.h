#ifndef CanCan_h_
#define CanCan_h_
#include "stack.h"

namespace CanCan {

typedef enum {
	eStart,
	eFindCan,
	eCheckCan,
	eGrabCan,
	eFindGoal,
	eScore,
	
	eStateEnd
} CanCanState;

} //namespace CanCan

#endif //CanCan_h_
