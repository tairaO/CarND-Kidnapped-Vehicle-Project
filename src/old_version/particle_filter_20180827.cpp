/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	// Set number of particles
	num_particles_ = 100;

	// Set standard deviations for x, y, and theta.
	double std_x, std_y, std_theta;
	std_x     = std[0];
	std_y     = std[1];
	std_theta = std[2];

	// Define distributions of x,y, theta
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);
	default_random_engine gen;

	// Set each particle's position and orientation
	Particle tmp_particle;
	for(unsigned int pi=0; pi < num_particles_ ; pi++){
		tmp_particle.x     = dist_x(gen);
		tmp_particle.y     = dist_y(gen);
		tmp_particle.theta = dist_theta(gen);
		tmp_particle.weight = 1.0;
		weights_.push_back(tmp_particle.weight);
		particles_.push_back(tmp_particle);
	}

/*
	// Print your samples to the terminal.
  for(unsigned int pi=0; pi < num_particles_; pi++){
    cout << "Sample" << pi << " " << particles_[pi].x << " " <<
      particles_[pi].y << " " << particles_[pi].theta << endl;
	}
	cout << "-------------------------------------" << endl;
	*/

	is_initialized_ = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// Set standard deviations for x, y, positions.
	double std_x, std_y, std_theta;
	std_x     = std_pos[0];
	std_y     = std_pos[1];
	std_theta = std_pos[2];

	// Define noise of x, y, theta
	normal_distribution<double> noise_x(0.0, std_x);
	normal_distribution<double> noise_y(0.0, std_y);
	normal_distribution<double> noise_theta(0.0, std_theta);
	default_random_engine gen;

	// Set each particle's new position and orientation
	double old_theta, new_theta; // for readability
	for(unsigned int pi=0; pi < num_particles_ ; pi++){
		old_theta = particles_[pi].theta;
		new_theta = particles_[pi].theta + yaw_rate * delta_t;
		// Set new value
		if(yaw_rate < 0.001){
			particles_[pi].x += velocity * delta_t * cos(old_theta);
			particles_[pi].y += velocity * delta_t * sin(old_theta);
		}else{
			particles_[pi].x += velocity / yaw_rate * (sin(new_theta)-sin(old_theta));
			particles_[pi].y += velocity / yaw_rate * (cos(old_theta)-cos(new_theta));
		}
		particles_[pi].theta = new_theta;

		// Add noise
		particles_[pi].x     += noise_x(gen);
		particles_[pi].y 		 += noise_y(gen);
		particles_[pi].theta += noise_theta(gen);

	}
	/*
	// Print your samples to the terminal.
  for(unsigned int pi=0; pi < num_particles_; pi++){
    cout << "Sample" << pi << " " << particles_[pi].x << " " <<
      particles_[pi].y << " " << particles_[pi].theta << endl;
		}
	cout << "-------------------------------------" << endl;
	*/
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.


}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	// TODO : Use sensor range
	double std_x, std_y;
	std_x     = std_landmark[0];
	std_y     = std_landmark[1];

	double x_p, y_p, theta_p; // particles position and coordinate
	double x_obs, y_obs; // observations
	LandmarkObs tmp_landmark_obs;

	// variables that stores NN
	double dist_min, tmp_dist;


	for(unsigned int pi=0; pi < num_particles_ ; pi++){
		x_p     = particles_[pi].x;
		y_p     = particles_[pi].y;
		theta_p = particles_[pi].theta;

		// Declare single_landmark:
		Map map_obs;
		Map::single_landmark_s tmp_landmark_obs;
		Map::single_landmark_s tmp_landmark_NN;

		for(unsigned int zi=0; zi < observations.size(); zi++){
			// transform coordinate
			x_obs = observations[zi].x;
			y_obs = observations[zi].y;
			tmp_landmark_obs.x_f = x_p + cos(theta_p)*x_obs - sin(theta_p)*y_obs;
			tmp_landmark_obs.y_f = y_p + sin(theta_p)*x_obs + cos(theta_p)*y_obs;

			map_obs.landmark_list.push_back(tmp_landmark_obs);

			// find associations
			dist_min = 100000;
			particles_[pi].weight = 1.0;
			for(unsigned int mi=0; mi < map_landmarks.landmark_list.size(); mi++){
				tmp_dist = dist(tmp_landmark_obs.x_f, tmp_landmark_obs.y_f, map_landmarks.landmark_list[mi].x_f, map_landmarks.landmark_list[mi].y_f);
				if(dist_min > tmp_dist){
					dist_min = tmp_dist;
					map_obs.landmark_list[zi].id_i = map_landmarks.landmark_list[mi].id_i;
					tmp_landmark_NN = map_landmarks.landmark_list[mi];
					}


				}
				particles_[pi].weight *= multivariate_gauss_prob(map_obs.landmark_list[zi].x_f, map_obs.landmark_list[zi].y_f,
																									tmp_landmark_NN.x_f, tmp_landmark_NN.y_f, std_x, std_y);
				/*
				cout << "Sample_weight" << pi << " " << particles_[pi].weight << endl;
				cout << "Sample_ID" << pi << " " << map_obs.landmark_list[zi].id_i << endl;
				*/
			}
		}

		// update weights_
		for(unsigned int pi=0; pi < num_particles_ ; pi++){
			weights_[pi] = particles_[pi].weight;
		}

		// Print your samples to the terminal.
	  for(unsigned int pi=0; pi < num_particles_; pi++){
	    cout << "Sample" << pi << " " << particles_[pi].weight << endl;
			}
		cout << "-------------------------------------" << endl;

	}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	// generator of weighted random interger
	discrete_distribution<> id(weights_.begin(), weights_.end());
	default_random_engine gen;

	// store new particles
	std::vector<Particle> new_particles;
	int index;

	// resample
	for(unsigned int pi=0; pi < num_particles_; pi++){
		index = id(gen);
		new_particles.push_back(particles_[index]);
		// new_particles[pi].weight = 1.0;
		}

	particles_ = new_particles;
	// TODO: Where should I change weights update
		/*
		// Print your samples to the terminal.
	  for(unsigned int pi=0; pi < num_particles_; pi++){
	    cout << "Sample" << pi << " " << particles_[pi].x << " " <<
	      particles_[pi].y << " " << particles_[pi].theta << endl;
		}
		cout << "-------------------------------------" << endl;
		*/

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
