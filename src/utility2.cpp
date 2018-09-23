
#include "utility.h"

double pi()    { return M_PI;           }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double mph2ms(double x)  { return x * 0.44704;    }
double mstomph(double ms){ return ms * 2.23694;	  }


int get_lane(double d) {
  return (int)(d / 4);
}


double distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
