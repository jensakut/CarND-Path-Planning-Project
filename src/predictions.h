
#ifndef PREDICTIONS_H
#define PREDICTIONS_H

#include <vector>
#include "utility.h"


class Predictions 
{
	private: 
		bool tooClose = false; 
		
		//some tunable semi-static parameters to be set with a constructor 
		double safety_distance = 40; // 200 m is a big number
		double clearance = 15; 
		double FOV=200; 
		
		double inf = 1000;
		
		std::vector<bool>   lane_free       = {true, true, true};		//is this lane next to me free?  
		std::vector<double> lane_speed      = {inf, inf, inf}; // slowest car in lane
		std::vector<double> lane_occupation = {0, 0, 0}; //number of cars in that lane in front
		std::vector<double> dist_front      = {FOV, FOV, FOV};
		std::vector<double> dist_back       = {FOV, FOV, FOV};
		
	public: 
		Predictions(double safety_distance_in, double clearance_in, double FOV_in);

		void update(std::vector<std::vector<double>> const &sensor_fusion, CarState ego, int prev_size);
		
		std::vector<bool>    get_lane_free();
		std::vector<double>  get_lane_speed();
		std::vector<double>  get_lane_occupation(); 
		std::vector<double>  get_dist_front();
		std::vector<double>  get_dist_back();
};



Predictions::Predictions(double safety_distance_in, double clearance_in, double FOV_in)
{
	safety_distance=safety_distance_in;
	clearance = clearance_in; 
	FOV = FOV_in;
}

void Predictions::update(std::vector<std::vector<double>> const &sensor_fusion, CarState ego, int prev_size)
{
	lane_free       = {true, true, true};		//is this lane next to me free?  
	lane_speed      = {inf, inf, inf}; // slowest car in lane
	lane_occupation = {0, 0, 0}; //number of cars in that lane in front
	dist_front      = {FOV, FOV, FOV};
	dist_back       = {FOV, FOV, FOV};
	
	for(int i = 0; i < sensor_fusion.size(); i++) 
	{
		double veh_x 		= sensor_fusion[i][3];
		double veh_y 		= sensor_fusion[i][4];		
		double veh_s 		= sensor_fusion[i][5];
		float  veh_d 		= sensor_fusion[i][6];

		double veh_v 		= sqrt(veh_x*veh_x+veh_y*veh_y);
		veh_s 	   			+= (double)prev_size * .02 * veh_v;	//increment position into the future
		int veh_lane 		= get_lane(veh_d); 
		double delta_s		= veh_s - ego.s;
		
		//sort the car into a lane and add to lanespeeds and occupied lanes respectively
		if (veh_s > ego.s && veh_s < ego.s + FOV) 
		{ 
			lane_speed[veh_lane]=veh_v;
			lane_occupation[veh_lane] +=1;
			// check if it's the closest vehicle in the front
			if (delta_s < abs(dist_front[veh_lane]))
				dist_front[veh_lane] = delta_s;
		}
		// if veh is not front of the vehicle, check if it's the closest behind. 
		else 
			if (delta_s < abs(dist_back[veh_lane]))
				dist_back[veh_lane] = delta_s; 

		//check whether "future car" has someone next to it. 
		if (abs(delta_s) < clearance) 
		{
			lane_free[veh_lane] = false; 
		}
	}
}
std::vector<bool> Predictions::get_lane_free() 
{ 
	return lane_free;
}

std::vector<double> Predictions::get_lane_speed() 
{
	return lane_speed; 
};

std::vector<double> Predictions::get_lane_occupation()
{
	return lane_occupation; 
}	
std::vector<double> Predictions::get_dist_front()
{
	return dist_front;
}

std::vector<double> Predictions::get_dist_back()
{
	return dist_back;
}


#endif // PREDICTIONS_H

