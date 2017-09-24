# CarND-Kidnapped-Vehicle-Project

## Introduction


[//]: # (Image References)
[init]: ./Images/Initialization.JPG
[diagram]: ./Images/Localization.JPG
[initeq]: ./Images/init_eq.gif


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


