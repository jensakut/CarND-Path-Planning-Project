
#ifndef PREDICTIONS_H
#define PREDICTIONS_H

#include <vector>
#include "utility.h"


class Predictions 
{
	private: 
		//bool tooClose = false; 
		
		//some tunable semi-static parameters to be set with a constructor 
		double safety_distance = 40; // 200 m is a big number
		double clearance = 15; 
		double FOV=150; 
		Road road; 
		double inf = 100; //max speed?!
		
		std::vector<bool>   lane_free       = {true, true, true};		//is this lane next to me free?  
		std::vector<double> lane_speed      = {inf, inf, inf}; // slowest car in lane
		std::vector<double> lane_occupation = {0, 0, 0}; //number of cars in that lane in front
		std::vector<double> dist_front      = {FOV, FOV, FOV};
		std::vector<double> dist_back       = {FOV, FOV, FOV};
		int cars_around = 0; 

	public: 
		Predictions(double safety_distance_in, double clearance_in, double FOV_in, Road road_in);

		void update(std::vector<std::vector<double>> const &sensor_fusion, CarState ego, int keep_size);
		
		bool    get_lane_free(int lane);
		double  get_lane_speed(int lane);
		double  get_lane_occupation(int lane); 
		double  get_dist_front(int lane);
		double  get_dist_back(int lane);
		int 	get_cars_around();
};



Predictions::Predictions(double safety_distance_in, double clearance_in, double FOV_in, Road road_in)
{
	safety_distance=safety_distance_in;
	clearance = clearance_in; 
	FOV = FOV_in;
	road = road_in;
}

void Predictions::update(std::vector<std::vector<double>> const &sensor_fusion, CarState ego, int keep_size)
{
	lane_free       = {true, true, true};		//is this lane next to me free?  
	lane_speed      = {road.speed_limit, road.speed_limit, road.speed_limit}; // slowest car in lane
	lane_occupation = {0, 0, 0}; //number of cars in that lane in front
	dist_front      = {FOV, FOV, FOV};
	dist_back       = {-FOV, -FOV, -FOV};
	cars_around 	= 0; 
	
	for(int i = 0; i < sensor_fusion.size(); i++) 
	{
		double veh_x 		= sensor_fusion[i][3];
		double veh_y 		= sensor_fusion[i][4];		
		double veh_s 		= sensor_fusion[i][5];
		float  veh_d 		= sensor_fusion[i][6];

		double veh_v 		= sqrt(veh_x*veh_x+veh_y*veh_y);
		double veh_s_pred 	= (double)keep_size * .02 * veh_v + veh_s;	//increment position into the future
		int veh_lane 		= get_lane(veh_d, road); 
		double delta_s 		= veh_s - ego.s;
		double delta_s_pred	= veh_s_pred - ego.s_pred;
		
		//count cars in sight 
		if (abs(delta_s_pred) < FOV) { cars_around +=1; } 
		//sort the car into a lane and add to lanespeeds and occupied lanes respectively
		if (veh_s_pred > ego.s_pred && veh_s_pred < ego.s_pred + FOV) 
		{ 
			//compute lane occupation for cost
			lane_occupation[veh_lane] +=1;
			// check if it's the closest vehicle in the front and note it's speed
			if (delta_s_pred < dist_front[veh_lane]) {
				dist_front[veh_lane] = delta_s_pred;
				lane_speed[veh_lane]=veh_v; 
			}
			
		}
		// if veh is behind the vehicle and within FOV, check if it's the closest behind. 
		else if (veh_s_pred < ego.s_pred)// && veh_s_pred > ego.s_pred - FOV)
			if (delta_s_pred > dist_back[veh_lane]) {
				dist_back[veh_lane] = delta_s_pred; 
			}
		
		//check whether "future car" has someone futuristic neighbour 
		if (abs(delta_s_pred) < clearance) 
		{
			lane_free[veh_lane] = false; 
		}
		//check whether "present car" has a neighbour
		else if (abs(delta_s) < clearance) 
		{
			lane_free[veh_lane] = false; 
		}
		//if car is behind but faster than us, a further distance is needed 
		else if (delta_s < 0 && delta_s > 2* clearance && veh_v > ego.speed + 2)
			lane_free[veh_lane] = false;
	}
}


bool Predictions::get_lane_free(int lane) 
{ 
	if (lane >= 0 && lane < road.lanes)
		return lane_free[lane];
	else
	{
		std::cout << "lane free " << lane << " not found!!!!!!!!!!!!" << std::endl; 
		return false;
	}
}

double Predictions::get_lane_speed(int lane) 
{
	if (lane >= 0 && lane < road.lanes)
		return lane_speed[lane]; 
	else
	{
		std::cout << "lane_speed i " << lane << " not found!!!!!!!!!!!!" << std::endl; 
		return 1001;
	}

};

double Predictions::get_lane_occupation(int lane)
{
	if (lane >= 0 && lane < road.lanes)
		return lane_occupation[lane];
	else
	{
		std::cout << "lane occupation" << lane << " not found!!!!!!!!!!!!" << std::endl; 
		return 1001;
	}
}	
double Predictions::get_dist_front(int lane)
{
	if (lane >= 0 && lane < road.lanes)
		return dist_front[lane];
	else
	{
		std::cout << "lane front" << lane << " not found!!!!!!!!!!!!" << std::endl; 
		return 1001;
	}
}

double Predictions::get_dist_back(int lane)
{
	if (lane >= 0 && lane < road.lanes)
		return dist_back[lane];
	else
	{
		std::cout << "lane back" << lane << " not found!!!!!!!!!!!!" << std::endl; 
		return 1001;
	}
}

int Predictions::get_cars_around(){
	return cars_around;
}

#endif // PREDICTIONS_H

