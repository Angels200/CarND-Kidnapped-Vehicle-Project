# CarND-Kidnapped-Vehicle-Project

## Introduction
The purpose of this project is about the localization of a kidnapped robot. Therefor we need to implement a 2 dimensional particle filter in C++. The particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step that filter will also get observation and control data. The localization process contains 4 steps : Initialization, Prediction, Update of weights and resampling. This process is repeated at each iteration triggered by the simulator.

[//]: # (Image References)
[init]: ./Images/Initialization.JPG
[diagram]: ./Images/Localization.JPG
[initeq]: ./Images/init_eq.gif
[state]: ./Images/statevector.gif
[update]: ./Images/updatequation.gif
![Diagram][diagram]

## Initialization

The initialization phase aims to generate N particles with different cooridinates and heading which representing the probability distribution of the robot (vehicle) initial position. The initialization algorithm follows the bellow steps :

•.The initiale position (Xo,Yo,Psio) of the kidnaped robot(vehicle) is defined as GPS-like input. This GPS position is impacted by by noise

•.	The noise is represented by a Guassian distribution with a mean and standard deviation :
Mean = (Xgps,Ygps, PSIgps)

Std = (2,2,0.05)

•.	The State vector at t=0 (initial time) is :

![Initeq][initeq]

•.	The Gaussian sampling provides us with a multiple initialized particles 

![Init][init]


## Prediction

The prediction phase algorithm follows the bellow steps :

•. Predict the next time step (state) using the motion model 

•. For each particle, updates the particle's location 

![State][state]

based on velocity and yaw rate measurements

•. Account for the the uncertainty in the control input, adds fgaussian noise to velocity and yaw rate

Recall that the motion model is : 

![Update][update]

