#include "behavior.h"

using namespace std;

Behavior::Behavior(Road road_init) {
	road = road_init; 
}

Behavior::~Behavior() {}

vector<Target> Behavior::get_targets() {
  return targets_;
}