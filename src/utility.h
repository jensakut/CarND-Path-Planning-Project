
#ifndef UTILITY_H
#define UTILITY_H

//#include "utility.cpp" 

struct Coord {
  double x;
  double y;
};

struct Frenet {
  double s;
  double d;
};

struct Road {
	double speed_limit; 
	int lanes; 
	double lane_width; 
	Road ( double SL=0, int L = 1, int LW = 4): 
			speed_limit(SL), lanes(L), lane_width(LW){}
};
	
struct CarState {
  double x;
  double y;
  double s; 
  double s_pred;
  double d;
  double yaw;
  double speed;
  double target_speed;
  int    lane;
  CarState (double X=0, double Y=0, double S=0, double D=0, 
		double YAW=0, double V=0, double TV=0, double L=0) : 
		x(X), y(Y), s(S), yaw(YAW), speed(V), target_speed(TV), lane(L)  {}
};


double pi()    { return M_PI;           }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double mph2ms(double x)  { return x * 0.44704;    }
double mstomph(double ms){ return ms * 2.23694;	  }


int get_lane(double d, Road road) {
	int lane = (int)(d / road.lane_width);
	if (lane < 0) 
	{ 
		lane = 0; 
		std::cout << "lane < 0 d=" << d << "!!!!!!!!!!!!!!!!" << std::endl;
	}
	if (lane > road.lanes-1)
	{	
		lane = road.lanes-1; 
		std::cout << "lane > road.lanes!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
	}
	return lane; 
}

double distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}




#endif // UTILITY_H 
