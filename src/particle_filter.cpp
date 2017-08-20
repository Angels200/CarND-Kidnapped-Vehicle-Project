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

static std::default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	//Particle p;
	num_particles = 80;
	std::normal_distribution<double> noise_x(0,std[0]);
	std::normal_distribution<double> noise_y(0,std[1]);
	std::normal_distribution<double> noise_theta(0,std[2]);

	//Particles Initialization
	for(int i=0;i<num_particles;i++){
		Particle p;
		p.id = i;
		p.x = x;
		p.y = y;
		p.theta = theta;
		p.weight = 1;

		//Noise Addition
		p.x += noise_x(gen);
		p.y += noise_y(gen);
		p.theta += noise_theta(gen);

		particles.push_back(p);
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// define normal distributions for sensor noise
	  std::normal_distribution<double> n_x(0, std_pos[0]);
	  std::normal_distribution<double> n_y(0, std_pos[1]);
	  std::normal_distribution<double> n_theta(0, std_pos[2]);

	  for (int i = 0; i < num_particles; i++) {

	    // New state estimation
	    if (fabs(yaw_rate) < 0.00001) { //do not consider yawrate
	      particles[i].x += velocity * delta_t * cos(particles[i].theta);
	      particles[i].y += velocity * delta_t * sin(particles[i].theta);
	    }
	    else {
	      particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
	      particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
	      particles[i].theta += yaw_rate * delta_t;
	    }

	    // add noise to particle state
	    particles[i].x += n_x(gen);
	    particles[i].y += n_y(gen);
	    particles[i].theta += n_theta(gen);
	  }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	int map_id = -1;
	for (int i = 0; i < observations.size(); i++) {
        // init minimum distance to maximum possible
	    double min_dist = numeric_limits<double>::max();

	    // init id of landmark from map placeholder to be associated with the observation
	    for (int j = 0; j < predicted.size(); j++) {
          // get distance between current/predicted landmarks
	      double cur_dist = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
	      // find the predicted landmark nearest the current observed landmark
	      if (cur_dist < min_dist) {
	        min_dist = cur_dist;
	        map_id = predicted[j].id;
	      }
	    }
	    // set the observation's id to the nearest predicted landmark's id
	    observations[i].id = map_id;
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
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

	for (int i = 0; i < num_particles; i++) {

	    // get the particle x, y coordinates
	    double px = particles[i].x;
	    double py = particles[i].y;
	    double ptheta = particles[i].theta;


	    vector<LandmarkObs> predictions;


	    for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {

	      // get id and x,y coordinates
	      float lmx = map_landmarks.landmark_list[j].x_f;
	      float lmy = map_landmarks.landmark_list[j].y_f;
	      int lmid = map_landmarks.landmark_list[j].id_i;

	      // Take into account only landmarks that are located within sensor range of the particle
	      if (fabs(lmx - px) <= sensor_range && fabs(lmy - py) <= sensor_range) {

	        // add prediction to vector
	        predictions.push_back(LandmarkObs{ lmid, lmx, lmy });
	      }
	    }

	    // create and populate a copy of the list of observations transformed from vehicle coordinates to map coordinates
	    vector<LandmarkObs> transformed_os;
	    for (int j = 0; j < observations.size(); j++) {
	      double t_x = cos(ptheta)*observations[j].x - sin(ptheta)*observations[j].y + px;
	      double t_y = sin(ptheta)*observations[j].x + cos(ptheta)*observations[j].y + py;
	      transformed_os.push_back(LandmarkObs{ observations[j].id, t_x, t_y });
	    }

	    // perform dataAssociation for the predictions and transformed observations on current particle
	    dataAssociation(predictions, transformed_os);

	    // reinit weight
	    particles[i].weight = 1.0;

	    for (int j = 0; j < transformed_os.size(); j++) {

	      // placeholders for observation and associated prediction coordinates
	      double o_x, o_y, pr_x, pr_y;
	      o_x = transformed_os[j].x;
	      o_y = transformed_os[j].y;

	      int associated_prediction = transformed_os[j].id;

	      // get the x,y coordinates of the prediction associated with the current observation
	      for (unsigned int k = 0; k < predictions.size(); k++) {
	        if (predictions[k].id == associated_prediction) {
	          pr_x = predictions[k].x;
	          pr_y = predictions[k].y;
	        }
	      }

	      // calculate weight for this observation with multivariate Gaussian
	      double s_x = std_landmark[0];
	      double s_y = std_landmark[1];
	      double obs_w = ( 1/(2*M_PI*s_x*s_y)) * exp( -( pow(pr_x-o_x,2)/(2*pow(s_x, 2)) + (pow(pr_y-o_y,2)/(2*pow(s_y, 2))) ) );

	      // product of this obersvation weight with total observations weight
	      particles[i].weight *= obs_w;
	    }
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	  vector<Particle> new_particles;

	  // get all of the current weights
	  vector<double> weights;
	  for (int i = 0; i < num_particles; i++) {
	    weights.push_back(particles[i].weight);
	  }

	  // generate random starting index for resampling wheel
	  std::uniform_int_distribution<int> uniintdist(0, num_particles-1);
	  auto index = uniintdist(gen);

	  // get max weight
	  double max_weight = *max_element(weights.begin(), weights.end());

	  // uniform random distribution [0.0, max_weight)
	  std::uniform_real_distribution<double> unirealdist(0.0, max_weight);

	  double beta = 0.0;

	  // spin the resample wheel!
	  for (int i = 0; i < num_particles; i++) {
	    beta += unirealdist(gen) * 2.0;
	    while (beta > weights[index]) {
	      beta -= weights[index];
	      index = (index + 1) % num_particles;
	    }
	    new_particles.push_back(particles[index]);
	  }

	particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
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
