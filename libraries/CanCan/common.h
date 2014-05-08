#ifndef common_h_
#define common_h_

#include "Arduino.h"

struct Point {
  double x;
  double y;
};

struct Pose {
  float a;
  Point p;
};

inline Point rotate(const Point& p, const double& theta) {
  double c = cos(theta);
  double s = sin(theta);
  Point n;

  n.x = p.x * c - p.y * s;
  n.y = p.x * s + p.y * c;
  
  return n;
}

#endif //common_h_
