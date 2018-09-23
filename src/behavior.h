#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <math.h>
#include <iostream>
#include <vector>
#include <cassert>



struct Target {
  double lane;
  double lane_final; 
  double velocity; // for JMT trajectories
  double time;  // for manoeuver
  double accel; // in case of dynamic lane changing
};


class Behavior {
public:
  Behavior(Road road_init); 
  virtual ~Behavior();

  void updateBehavior(CarState &ego, Predictions &predictions);
  std::vector<Target> get_targets();

private:
  std::vector<Target> targets_;
  Road road; 
  
};







Behavior::Behavior(Road road_init) {
	road = road_init; 
}

Behavior::~Behavior() {}

std::vector<Target> Behavior::get_targets() {
  return targets_;
}





#endif // BEHAVIOR_H
