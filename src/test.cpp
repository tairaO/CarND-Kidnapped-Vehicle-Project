#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "particle_filter.h"

using namespace std;


int main(){

  //Set up parameters here
  double delta_t = 0.1; // Time elapsed between measurements [sec]
  double sensor_range = 50; // Sensor range [m]

  double sigma_pos [3] = {0.3, 0.3, 0.01}; // GPS measurement uncertainty [x [m], y [m], theta [rad]]
  double sigma_landmark [2] = {0.3, 0.3}; // Landmark measurement uncertainty [x [m], y [m]]

  // Sense noisy position data from the simulator
  double sense_x = 0.0;
  double sense_y = 0.0;
  double sense_theta = 0.0;

  // Create particle filter
  ParticleFilter pf;

  // Read map data
  Map map;
  if (!read_map_data("../data/map_data.txt", map)) {
	  cout << "Error: Could not open map file" << endl;
	  return -1;
  }

  pf.init(sense_x, sense_y, sense_theta, sigma_pos);

  double previous_velocity = 0;
  // double previous_yawrate  = M_PI / 8;
  double previous_yawrate  = 0.0001;
  pf.prediction(delta_t, sigma_pos, previous_velocity, previous_yawrate);


  // Update the weights and resample
  vector<LandmarkObs> noisy_observations;
  LandmarkObs obs;
  obs.x = 17;
  obs.y = -4.5;
  noisy_observations.push_back(obs);
  obs.x = -7.1;
  obs.y = -34;
  noisy_observations.push_back(obs);
  obs.x = 29;
  obs.y = -39;
  noisy_observations.push_back(obs);

  pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map);
  pf.resample();

  return 0;
}
