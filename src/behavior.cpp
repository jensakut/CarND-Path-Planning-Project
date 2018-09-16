#include "behavior.h"

using namespace std;

Behavior::Behavior(vector<vector<double>> const &sensor_fusion) {

}

Behavior::~Behavior() {}

vector<Target> Behavior::get_targets() {
  return targets_;
}