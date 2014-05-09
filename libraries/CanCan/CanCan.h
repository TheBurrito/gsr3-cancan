#ifndef CanCan_h_
#define CanCan_h_

#include "common.h"

//namespace arena {

struct Bounds {
  float width;
  float length;
  Point min;
  Point max;
  Point goal;
};

const int arenaCount = 3;
extern Bounds arenas[arenaCount];
extern Bounds arena;

void setArena(int i, double buffer);

//} //namespace arena

#endif //CanCan_h_
