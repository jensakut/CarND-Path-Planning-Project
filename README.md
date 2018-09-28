# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. 
You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. 
The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, 
note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving 
inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete 
loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also 
the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Implementation

The code consists of 3 header files and a main function. The headers are utility.h for small functions, the prediction and the behavior class. The resulting spline itself is computed in the main function.

### Utility functions

The utility functions provide the definitions of the struct CarState, the struct Road, and helper functions like getLane(). Road defines the speedlimit, the number of lanes and their width. 
Thus, if the road parameters change, they can be changed here and the other modules change accordingly. 
The CarState is used to compute the ego vehicles position and speed parameters. The function getLane uses the coordinate d and the struct Road to compute a lane. 


### Prediction module  
The prediction module computes the space in front and in the back of the vehicle. The closest vehicle in front defines the lane speed. The closest vehicle in each lane defines the dist_front and the dist_back vectors. Both the difference between ego's and vehicle's distance in present and future are used to compute the possibility to overtake, which is
available using the get_lane_free(int lane) method. Because both the present and the future positions are used, each vehicle has to be checked individually. 

### Behavior module
The behavior module computes the lateral speed of the ego vehicle and uses a finite state machine to compute the lane. 
The speed of the lane is limited to the road speed limit or a vehicle in front of the ego vehicle. A simple controller is used to break, and a constant factor is used to accelerate. 
The state machine consists of the keep_lane, prepare_lane_change and change_lanes states. They are controlled using the private variable state. 
In keep_lane, the cost function determines whether or not to switch to lane_change. The cost-function uses the lane speed limit, the space in front, and a penalty for changing lanes to determine
whether or not to switch lanes. 
If profitable, the state prepare_lane_change checks whether the lane change is safe. Here, additional functions like speed up or slow down could be implemented. If it's good to change, the lane change 
function gets called. As soon as the actual position is within 1 meter in the next_lane, either keep lane or the next lane change in case of a double lane change gets called. 

### Spline computation in the main routine

The spline uses the current and future points to compute a smooth trajectory. Computing 50 points in advance but discarding everything but ten gives a quick reaction in case of sudden changes of situations. 





## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
