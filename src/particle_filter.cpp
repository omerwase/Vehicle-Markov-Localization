/*
 * particle_filter.cpp
 *
 * Udacity Self Driving Car Nano Degree
 * Project 3: Kidnapped Vehicle
 * Submitted by: Omer Waseem
 * Date: Feb 24th, 2018
 *
 *  Created on: Dec 12, 2016
 *  Created by: : Tiffany Huang
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
	
	// set number of particles
	// high number of particles (i.e. 10000) significantly degrade performance
	/* RMSE results:
	 * @num_particles = 10		: x = 0.159, y = 0.137; yaw = 0.005, system time = 49.94s
	 * @num_particles = 50		: x = 0.121, y = 0.112; yaw = 0.004, system time = 49.26
	 * @num_particles = 100		: x = 0.114, y = 0.104; yaw = 0.004, system time = 50.60s
	 * @num_particles = 250		: x = 0.111, y = 0.103; yaw = 0.004, system time = 52.48s
	 * @num_particles = 1000	: x = 0.108, y = 0.101; yaw = 0.003, system time = 52.06
	 * @num_particles = 3000	: x = 0.108, y = 0.102; yaw = 0.003, system time = 103.62
	 */
	num_particles = 100;
	
	// retrieve std values from std[]
	double std_x = std[0];
	double std_y = std[1];
	double std_theta = std[2];
	
	// create Gaussian distributions for x, y and theta, with std as noise
	default_random_engine gen;
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);
	
	// resize weights and particle vectors
	particles.resize(num_particles);
	weights.resize(num_particles);
	
	// initialize particles from Gaussian distributions, initialize weights to 1
	for (size_t i = 0; i < num_particles; ++i) {
		particles[i].id = i;
		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
		particles[i].weight = 1.0;
		weights[i] = 1.0;
	}
	
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// retrieve std values from std_pos[]
	double std_x = std_pos[0];
	double std_y = std_pos[1];
	double std_theta = std_pos[2];
	
	// create Gaussian distributions for noise values, with mean 0
	default_random_engine gen;
	normal_distribution<double> dist_x(0, std_x);
	normal_distribution<double> dist_y(0, std_y);
	normal_distribution<double> dist_theta(0, std_theta);
	
	// update each particle
	for (size_t i = 0; i < num_particles; ++i) {
		double theta = particles[i].theta;
		
		// yaw_rate is greater than a minimum value (below which 0 yaw_rate is assumed)
		if (fabs(yaw_rate) > 0.000001f) {
			double theta_yaw_rate_dt = theta + yaw_rate * delta_t;
			particles[i].x += (velocity/yaw_rate) * (sin(theta_yaw_rate_dt) - sin(theta));
			particles[i].y += (velocity/yaw_rate) * (cos(theta) - cos(theta_yaw_rate_dt));
			particles[i].theta += yaw_rate * delta_t;
		} else { // yaw_rate is equal or close to 0
			particles[i].x += velocity * cos(theta) * delta_t;
			particles[i].y += velocity * sin(theta) * delta_t;
		}
		
		// add noise from Gaussian distributions of 0 mean
		particles[i].x += dist_x(gen);
		particles[i].y += dist_y(gen);
		particles[i].theta += dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	
	// ensure there are predictions to measure against
	if (predicted.size() == 0) {
		cout << "Error in dataAssociation(): predicted landmark array is empty" << endl;
		return;
	}
	
	// ensure there are observations to measure against
	if (observations.size() == 0) {
		cout << "Error in dataAssociation(): observations array is empty" << endl;
		return;
	}
	
	// for each observation find the closest predicted landmark
	for (size_t i = 0; i < observations.size(); ++i) {
		
		// initialize minimum distance to the max value of a double
		double min_distance = numeric_limits<double>::max();
		
		// initialize landmark id to first predicted landmark index
		int landmark_index = 0;
		
		// find predicted landmark closest to observation
		for (size_t j = 0; j < predicted.size(); ++j) {
			double distance_x = predicted[j].x - observations[i].x;
			double distance_y = predicted[j].y - observations[i].y;
			double distance = sqrt(distance_x * distance_x + distance_y * distance_y);
			if (min_distance > distance) {
				min_distance = distance;
				landmark_index = j; // assign predicted landmark index
			}
		}
		
		// assign closest predicted landmark index to observation id
		observations[i].id = landmark_index;
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a multi-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	
	/* for each particle perform the following:
	 		1) transform observations from vehicle to map coordinates based on the particle's position
	 		2) find map_landmarks within sensor_range from the particle
	 		3) associate transformed observations with landmarks within range of the particle
	 		4) update the particle's weight using using a multi-variate Gaussian distribution
		  5) assign particle's weight to weights array for resampling
	*/
	for (size_t i = 0; i < num_particles; ++i) {
		double x_part = particles[i].x;
		double y_part = particles[i].y;
		double theta_part = particles[i].theta;
		
		// 1) transform observations from vehicle to map coordinates based on the particle's position
		vector<LandmarkObs> t_observations; // contains transformed observations
		for (size_t j = 0; j < observations.size(); ++j) {
			double x_obs = observations[j].x;
			double y_obs = observations[j].y;
			LandmarkObs obs;
			obs.id = observations[j].id;
			obs.x = x_part + cos(theta_part) * x_obs - sin(theta_part) * y_obs;
			obs.y = y_part + sin(theta_part) * x_obs + cos(theta_part) * y_obs;
			t_observations.push_back(obs);
		}
		
		// 2) find map_landmarks within sensor_range from the particle
		vector<LandmarkObs> predicted_landmarks;
		for (size_t j = 0; j < map_landmarks.landmark_list.size(); ++j) {
			double distance_x = x_part - map_landmarks.landmark_list[j].x_f;
			double distance_y = y_part - map_landmarks.landmark_list[j].y_f;
			double distance = sqrt(distance_x * distance_x + distance_y * distance_y);
			if (sensor_range >= distance) {
				LandmarkObs obs;
				obs.id = map_landmarks.landmark_list[j].id_i;
				obs.x = map_landmarks.landmark_list[j].x_f;
				obs.y = map_landmarks.landmark_list[j].y_f;
				predicted_landmarks.push_back(obs);
			}
		}
		
		// 3) associate transformed observations with landmarks within range of the particle
		dataAssociation(predicted_landmarks, t_observations);
		
		// 4) update the particle's weight using using a multi-variate Gaussian distribution
		double sig_x = std_landmark[0];
		double sig_y = std_landmark[1];
		double sig_x2 = sig_x * sig_x;
		double sig_y2 = sig_y * sig_y;
		double gauss_norm = 1.0 / (2.0 * M_PI * sig_x * sig_y);
		// initialize particle weight to 1.0 for multiplication
		particles[i].weight = 1.0;
		// determine particle weight by multiplying the multi-variate Gaussian
		// probability of each transformed observation
		for (size_t j = 0; j < t_observations.size(); ++j) {
			double x_obs = t_observations[j].x;
			double y_obs = t_observations[j].y;
			// index of closest predicted landmark
			int closet_landmark_index = t_observations[j].id;
			double x_mu = predicted_landmarks[closet_landmark_index].x;
			double y_mu = predicted_landmarks[closet_landmark_index].y;
			double delta_x = x_obs - x_mu;
			double delta_y = y_obs - y_mu;
			double exponent = -0.5 * (delta_x*delta_x/sig_x2 + delta_y*delta_y/sig_y2);
			double obs_prob = gauss_norm * exp(exponent);
			// ensure observation probability is at least lowest possible double value
			if (obs_prob > numeric_limits<double>::min()) {
				particles[i].weight *= obs_prob;
			} else {
				particles[i].weight *= numeric_limits<double>::min();
			}
		}
		
		// 5) assign particle's weight to weights array for resampling
		weights[i] = particles[i].weight;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	// resample particles with probability proportional to their weight
	random_device rd;
	mt19937 gen(rd());
	discrete_distribution<double> d(weights.begin(), weights.end());
	vector<Particle> resamples;
	for (size_t i = 0; i < num_particles; ++i) {
		resamples.push_back(particles[d(gen)]);
	}
	particles = resamples;
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
