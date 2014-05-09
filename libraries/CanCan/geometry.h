#ifndef geometry_h_
#define geometry_h_

#include "common.h"

#define ROBOT_GRIP_OFFSET 8.7  // gripper closed position
#define ROBOT_FRONT_OFFSET 7.3 // to front of case
#define ROBOT_REAR_OFFSET 10.7 // to rear of case
#define ROBOT_WHEEL_OFFSET 9.2 // to outside of wheel

typedef enum {
  IRL,
  IRFL,
  IRF,
  IRFR,
  IRR,

  IR_END,
  IR_ALL
} 
IR_Index;

extern Pose sonarPose[5];

#endif
