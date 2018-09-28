#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <math.h>
#include <iostream>
#include <vector>
#include <string> 





class Behavior {
public:
	Behavior	(double safety_distance_in, double clearance_in, double FOV_in, Road road_in, double speed_control_p_in, double max_accel_in);
	virtual ~Behavior();

	void update(std::vector<std::vector<double>> const &sensor_fusion, CarState &ego, Predictions &predictions);
	
	
	
private:
	std::string state = "KL";
	void keep_lane(CarState &ego, Predictions &predictions);
	void prepare_lane_change(CarState &ego, Predictions &predictions);
	void lane_change(CarState &ego, Predictions &predictions);
	double compute_cost(CarState &ego, Predictions &predictions, int lane);

	double safety_distance; 
	double clearance; 
	double FOV; 
	Road road; 
	double speed_control_p;
	double max_accel; 
	double frontal_space=75;
	int previous_lane; 
	int final_lane; 
	int next_lane; 
	
};



Behavior::Behavior(double safety_distance_in, double clearance_in, double FOV_in, Road road_in, double speed_control_p_in, double max_accel_in)
{
	safety_distance=safety_distance_in;
	clearance = clearance_in; 
	FOV = FOV_in;
	road = road_in;
	speed_control_p = speed_control_p_in;
	max_accel = max_accel_in;
	
}


Behavior::~Behavior() {}

void Behavior::update(const std::vector<std::vector<double>> &sensor_fusion, CarState &ego, Predictions &predictions) 
{

	//call states depending on state variable 
	if (state == "KL") {keep_lane(ego, predictions); }
	if (state == "PLC") {prepare_lane_change(ego, predictions); }
	if (state == "LC") {lane_change(ego, predictions); }
	
	if (state == "KL") { 
		previous_lane=ego.lane; 
		next_lane = ego.lane; 
		final_lane = ego.lane;
	}
	double prev_ego_speed = ego.speed;
	// check actual road position to prevent running into cars while being changing lanes
	// ego.lane is already computing the next lane at that point. 
	int current_lane = get_lane(ego.d, road);
	// always check speed of vehicle in front to prevent crashes 
	// in case of lane change never run into someone
	
	double frontal_space =  std::min(predictions.get_dist_front(current_lane), predictions.get_dist_front(next_lane)) - safety_distance;
	
	
	std::cout << "frontal_space is " << frontal_space << std::endl; 
	if (frontal_space < 0) 
	{
		double a_brake = 0;
		if (ego.speed+1 > predictions.get_lane_speed(ego.lane))
		{
			a_brake = speed_control_p * -frontal_space;
		}
		std::cout << "brake deceleration " << a_brake << std::endl; 
		if (a_brake < 0) { a_brake = 0;}
		if (a_brake > 0.2) { a_brake = 0.2;} // protect min/max braking acceleration to pass simulator 
		ego.speed -= a_brake;
	}
	//if driving "too slow" speed up 
	else if (ego.speed < road.speed_limit)
	{
		ego.speed +=0.1;
		if (predictions.get_lane_occupation(ego.lane) == 0)
		// if road ahead is free accelerate faster
			ego.speed += 0.05; 
	}
	else if (ego.speed > road.speed_limit) {
		ego.speed -=0.2;		
	}
}

void Behavior::keep_lane(CarState &ego, Predictions &predictions)
{
	std::cout << "keep lane" << std::endl; 

	//check if current lane is good
	int best_lane = ego.lane;
	
	//start with cost for keeping lanes 
	double least_cost = 1;
				
	for (int i = 0; i<road.lanes; i++) {
		double lane_cost = compute_cost(ego, predictions, i);
		std::cout << "The lane " << i << " costs " << lane_cost << std::endl; 
		if (lane_cost < least_cost) // the +2 is a temporary hysteresis
		{ 
			least_cost = lane_cost; 
			best_lane = i;
		}	
	}		
	std::cout << "The best lane is " << best_lane << " with speed " << predictions.get_lane_speed(best_lane) << std::endl; 
	// set next and final lane 
	// TODO include check if next_lane is kind of blocked
	if (best_lane != ego.lane) {
		if (best_lane < ego.lane)
		{
			final_lane = best_lane; 
			next_lane = ego.lane-1; 
			state = "PLC";
		}
		else if (best_lane > ego.lane) 
		{
			final_lane = best_lane; 
			next_lane = ego.lane + 1;
			state = "PLC"; 
			
		}
	}
	//else stay in lane 
	else 
	{
		next_lane = ego.lane; 
		final_lane = ego.lane;
	}
}

void Behavior::prepare_lane_change(CarState &ego, Predictions &predictions)
{	
	std::cout << "prepare lane change" << std::endl; 
	//if cost is still profitable 
	if (compute_cost(ego, predictions, ego.lane) > compute_cost(ego, predictions, final_lane))  
	{
		
		//check if space is good
		if (predictions.get_lane_free(next_lane))
		{
			state = "LC";			
		}
		//if there's a car on the left, and a car in front try slowing down
		//TODO: Predict whether a car is about to overtake to use it here
		/*else if (predictions.get_dist_front(ego.lane) < 2*safety_distance && predictions.get_dist_front(next_lane) < 2*safety_distance) 
			//since there isn't room ahead, slow down 
			// but don't get too slow 
			if (ego.target_speed > predictions.get_lane_speed(ego.lane) + 3){
				std::cout << "slowly slowing down to find some room to overtake" << std::endl; 
				ego.target_speed -= 0.05; 
			}*/
	}
	//abort lane change 
	else 
	{
		state = "KL"; 
	}
};
void Behavior::lane_change(CarState &ego, Predictions &predictions)
{	
	//heal target speed if necessary 
	std::cout << "lane change" << std::endl; 
	if (predictions.get_lane_free(next_lane)) {
		ego.lane = next_lane; 
	}
	else 
	{
		ego.lane = get_lane(ego.d, road);
		state = "PLC";
	}
	
	if (get_lane(ego.d+1, road) == final_lane && get_lane(ego.d-1, road) == final_lane) 
	{
		state = "KL";
	}
	else if (get_lane(ego.d+1, road) == next_lane && get_lane(ego.d-1, road) == next_lane) 
	{ 
		if (final_lane < ego.lane)
		{
			next_lane = ego.lane-1; 
			state = "PLC";
		}
		else if (final_lane > ego.lane)  { 
			next_lane = ego.lane + 1;
			state = "PLC"; 
		}
	}
};

double Behavior::compute_cost(CarState &ego, Predictions &predictions, int lane) 
{
	double cost = 1.5*predictions.get_lane_speed(lane)/road.speed_limit; // relative speed possible
	cost += predictions.get_dist_front(lane)/FOV; // space ahead
	cost += exp(-abs(lane - ego.lane)); // add benefit for keeping the lane! 
	cost = exp(-cost); //sigmoid 
	return cost; 
}

#endif // BEHAVIOR_H
