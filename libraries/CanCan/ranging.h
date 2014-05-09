#ifndef ranging_h_
#define ranging_h_

#include <SharpIR.h>
#include "geometry.h"

namespace ranging {

void update();

void setPeriod(unsigned long ms);
int getDistance(IR_Index i);

}

#endif //ranging_h_
