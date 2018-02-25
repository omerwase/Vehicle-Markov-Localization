# Kidnapped Vehicle Project

[//]: # (Image References)
[image1]: ./images/algorithm_flowchart.png

### Overview

This project implements markov localization using map landmarks and LIDAR sensor data. A kidnapped vehicle is simulated which must localize its position based on landmarks (map data) and LIDAR observations. This is achieved using a particle filter updated through Bayesian inference. The implementation can be found in [particle_filter.cpp](https://github.com/omerwase/SDC_P8_Kidnapped_Vehicle/blob/master/src/particle_filter.cpp). The program works in conjunction with [Udacity's Self Driving Car Simulator](https://github.com/udacity/self-driving-car-sim).

---
### Methodology and Implementation

This flowchart illustrates the various parts of the particle filter implementation, with details below.

![flowchart][image1] 

#### Initialization
During intialization the number of particles is set. Each particle is initialized based on a Gaussian distribution, centred around GPS coordinates of the car's starting position. GPS coordinates are only used to initialize the particle filter. Beyond this they are not used in localization since GPS has a lower-than-desired accuracy.

Each particle represents a prediction of where the car might be located. This prediction is refined through Bayesian inference, given map (landmarks) and sensor (LIDAR) data, in the folowing steps.

#### Prediction Step


#### Update Step


#### Resample


---
### Results


---
### Dependencies

* Udacity's SDC Simulator: simulates the kidnapped vehicles, provides landmark map and LIDAR sensor data
* cmake >= 3.5
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
* gcc/g++ >= 5.4

For detailed instructions on installing the dependencies see [Udacity's original project repo](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project).


---
### Build Instructions

* Clone this repo
* Navigate to the project's root directory
* Run: ./clean.sh
* Run: ./build.sh
* Run: ./run.sh
* Launch Udacity's SDC Simulator and start Project 3: Kidnapped Vehicle
