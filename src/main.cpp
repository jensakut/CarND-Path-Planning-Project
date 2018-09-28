#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

#include "utility.h"
#include "predictions.h"
#include "behavior.h"

using namespace std;


// for convenience
using json = nlohmann::json;





// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


// closest (euklidian) without considering direction
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}
// next waypoint in driving direction
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
// 
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}



int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  
  
  
  //initial model parameters

  
  Road road = Road(mph2ms(49.3), 3, 4); //speed_limit, lanes, lane_width 
  CarState ego = CarState(0., 0., 0., 0., 0., 0., road.speed_limit,0.1);
  bool diagnostics = false;
  double safety_distance_in = 25; 
  double clearance_in = 10; 
  double FOV_in = 100; 
  double speed_control_p = 0.5; 
  double max_accel = 0.25; 
  Predictions predictions(safety_distance_in, clearance_in, FOV_in, road); 
  Behavior behavior(safety_distance_in, clearance_in, FOV_in, road, speed_control_p, max_accel);
 
  h.onMessage([&diagnostics, &ego, &predictions, &behavior, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &road](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
			system("clear");
			//cout << endl << endl << endl;
        	// Main car's localization Data
          	ego.x = j[1]["x"];
          	ego.y = j[1]["y"];
          	ego.s = j[1]["s"];
          	ego.d = j[1]["d"];
          	ego.yaw = j[1]["yaw"];
			ego.lane = get_lane(ego.d, road);
			
			
			
          	// Previous path data given to the Planner
          	vector<double> previous_path_x = j[1]["previous_path_x"];
          	vector<double> previous_path_y = j[1]["previous_path_y"];


          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];
			
          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
			
			
          	json msgJson;
			
			
          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
			
			int prev_size = previous_path_x.size();
			if (diagnostics) cout << "prev_size " << prev_size << endl; 		
			int keep_size = 10; 
			
			///
			///
			///
			// Sensor fusion analytics and finite state machine
			//cout << "sensor fusion" << endl; 			
			
			if (keep_size < prev_size) {
				
				ego.x 		= previous_path_x[keep_size];
				ego.y 		= previous_path_y[keep_size];
				double x1 	= previous_path_x[keep_size - 1];
				double y1 	= previous_path_y[keep_size - 1];
				double x2 	= previous_path_x[keep_size - 2];
				double y2 	= previous_path_y[keep_size - 2];
				ego.yaw 	= atan2(y1-y2,x1-x2);  //TODO why previous steps
				vector<double> Frenet = getFrenet(ego.x, ego.y, ego.yaw, map_waypoints_x, map_waypoints_y);
				ego.s_pred 	= Frenet[0];
				ego.d 	= Frenet[1];
				ego.lane = get_lane(ego.d, road);
			}
			else//if not enough points are available prevent overflow
			{
				keep_size = prev_size;
				ego.s_pred = ego.s; //still predict some ego position 
			}
			
			if (diagnostics) { cout << "keep_size " << keep_size << endl; }

			predictions.update(sensor_fusion, ego, keep_size); 

			behavior.update(sensor_fusion, ego, predictions);


			// some printing 
			cout << "predictions.get_lane_free ["; 
			for (int i = 0; i < road.lanes; i++) {
				cout << predictions.get_lane_free(i) << " ";	}		
			
			cout << "]" << endl << "predictions.get_lane_occupation [";   
			for (int i = 0; i < road.lanes; i++) {
				cout << predictions.get_lane_occupation(i) << " ";		}		
			
			cout << "]" << endl << "predictions.get_lane_speed [";
			for (int i = 0; i < road.lanes; i++) {
				cout << int(predictions.get_lane_speed(i)) << " ";  }
			
			cout << "]" << endl << "predictions.get_dist_front [";
				for (int i = 0; i < road.lanes; i++) {
					cout << int(predictions.get_dist_front(i)) << " ";  }
			
			cout << "]" << endl << "predictions.get_dist_back [";
			for (int i = 0; i < road.lanes; i++) {
				cout << int(predictions.get_dist_back(i)) << " ";  }
			cout << "]" << endl << "ego_v " << ego.speed << endl;
			cout << "target_v " << ego.target_speed << endl; 
			
			
			///
			/// Computation of path to follow 
			///
			// Create a list of widely spaced (x, y) waypoints, evenly spaced at 30 m
			// to be implemented with a spline
			
			vector<double> ptsx;
			vector<double> ptsy;
			
			//reference x,y, yaw states
			double ref_x = ego.x;
			double ref_y = ego.y;
			double ref_yaw = deg2rad(ego.yaw);
			
			//init: if previous size is empty, use the car init as starting reference
			if(prev_size < 2)
			{				
				if (diagnostics) { cout << "init" << endl; }

				//use two points for continuity
				double prev_car_x = ego.x - cos(ego.yaw); 
				double prev_car_y = ego.y - sin(ego.yaw);
				ptsx.push_back(prev_car_x);
				ptsx.push_back(ego.x);
				
				ptsy.push_back(prev_car_y);
				ptsy.push_back(ego.y);
			}
			else
			{
			
				ref_x = previous_path_x[keep_size-1];
				ref_y = previous_path_y[keep_size-1];
				
				double ref_x_prev = previous_path_x[keep_size-2];
				double ref_y_prev = previous_path_y[keep_size-2];
			
				ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
				
				//if car stops, move the spline anchor point behind the car 
				if (ref_x_prev == ref_x && ref_y_prev == ref_y) 
				{
					ref_x_prev-= cos(ego.yaw);
					ref_y_prev-= sin(ego.yaw); 
				}
				ptsx.push_back(ref_x_prev);
				ptsx.push_back(ref_x);
				
				ptsy.push_back(ref_y_prev);
				ptsy.push_back(ref_y);
				//cout << " else" << endl; 
			}
			
			//In Frenet add evenly 30 m spaced points ahead of the starting reference (anker points)
			//cout << "next wp" << endl; 
			
			vector<double> next_wp0 = getXY(ego.s_pred+30,(2+4*ego.lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
			vector<double> next_wp1 = getXY(ego.s_pred+60,(2+4*ego.lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
			vector<double> next_wp2 = getXY(ego.s_pred+90,(2+4*ego.lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
			ptsx.push_back(next_wp0[0]);
			ptsx.push_back(next_wp1[0]);
			ptsx.push_back(next_wp2[0]);
			
			ptsy.push_back(next_wp0[1]);
			ptsy.push_back(next_wp1[1]);
			ptsy.push_back(next_wp2[1]);

			for (int i = 0; i< ptsx.size(); i++) 
			{
				//shift car reference angle to 0 degrees
				double shift_x = ptsx[i]-ref_x;
				double shift_y = ptsy[i]-ref_y;
				
				ptsx[i] = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
				ptsy[i] = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);
			}
			if (diagnostics)
			{
				cout << " " << endl; 
				cout << "spline points " << ptsx.size() << endl; 
				for (int i = 0; i<ptsx.size(); i++){
					cout << i << " " << ptsx[i] << " " << ptsy[i] << endl; 
				}
			}
			//TODO: Insert check that spline points aren't put in twice or list is empty. Otherwise spline crashes. 
			
			// create a spline 
			tk::spline s;
			//cout << "spline done" << endl; 
			//set x,y) points to the spline
			s.set_points(ptsx,ptsy);
			
			//define the actual (x,y) points used for the planner
			vector<double> next_x_vals;
			vector<double> next_y_vals;
			
			// prevent adding non-existing points when lagging or at init

			//start with previous points from last time
			for(int i = 0; i<keep_size; i++)
			{
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}


			//cout << "target" << endl; 
			//calculate how to break up spline points to travel at desired speed 
			double target_x = 30.0;
			double target_y = s(target_x);
			double target_dist = sqrt(target_x*target_x+target_y*target_y);
			double x_add_on = 0; 
			if (diagnostics) { cout << "next_x_vals.size() " << next_x_vals.size() << endl; }
			//fill up the rest of the path planner after filling it with previous points
			for (int i = 1; i <= 50-keep_size; i++) {
				double N = target_dist/(0.02*ego.speed); 
				double x_point = x_add_on+(target_x)/N;
				double y_point = s(x_point);
				
				x_add_on = x_point;
				
				double x_ref = x_point;
				double y_ref = y_point;
				
				//rotate back to normal coordinates 
				x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
				y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));
				x_point +=ref_x;
				y_point +=ref_y;
				
				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);
				
			}
			if (diagnostics) { cout << "next_x_vals.size() " << next_x_vals.size() << endl; }

			//cout << "end!" << endl; 
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";
			
			
          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
