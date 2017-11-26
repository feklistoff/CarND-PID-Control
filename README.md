# CarND-Controls-PID
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)
---

<img src="overview.png" width="600px">

## Overview

In this project I implemented a PID controller in C++ to maneuver the vehicle around the track in simulator. The simulator provides the cross track error (CTE), steering angle and the velocity (mph) in order to compute the appropriate steering angle.

## Results

### About PID controller

A proportional–integral–derivative controller (PID controller) is a control loop feedback mechanism widely used in industrial control systems and a variety of other applications requiring continuously modulated control.

[Here](https://www.youtube.com/watch?v=4Y7zG48uHRo) is a great and short video explanation.
 
**P** is proportional to the current value of the error *CTE = desired state - measured state*. For example, if the error is large and positive, the control output will be proportionately large and positive, taking into account the gain factor "K". Using proportional control alone will always result in an error between the setpoint (desired state) and the actual process value, because it requires an error to generate the proportional response. If there is no error, there is no corrective response. In simple words if the car is far to the right it steers hard to the left, if it's slightly to the left it steers slightly to the right.

**I** accounts for past values of the CTE and integrates them over time to produce the *I* term.
Due to limitation of P-controller where there always exists an offset between the process variable and setpoint, I-controller is needed, which provides necessary action to eliminate the steady state error. For example, if there is a residual error after the application of proportional control, the integral term seeks to eliminate the residual error by adding a control effect due to the historic cumulative value of the error.

**D** is a best estimate of the future trend of the error based on its current rate of change. It is effectively seeking to reduce the effect of the error by exerting a control influence generated by the rate of error change. The more rapid the change, the greater the controlling or dampening effect. The parameter is used to reduce overshooting and dump oscillations caused by the P-controller.

### Process of finding "right" parameters

Before the working of PID controller takes place, it must be tuned to suit with dynamics of the process to be controlled. The default values for P, I and D terms and these values couldn’t give the desired performance and sometimes lead to instability and low performance.

There are a lot of methods to tune parameters. For example:
* simple trial-and-error metod
* [Zeigler-Nichols](http://www.mstarlabs.com/control/znrule.html) method
* Iterative Feedback Tuning
* Neural Network-based on-line tuning methods (for ex. [here](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5038707/))

I used Twiddle that was presented in lessons.

Initial parameters were also  taken from lessons `Kp = 0.2, Ki = 0.004, Kd = 3.0`. Using these gains the car was able to smoothly drive along the track. [Here](https://github.com/feklistoff/CarND-PID-Control/pid_30max.mov) is a demo video. However, as soon as I tried to increase speed (more than 30 mph), the car started to oscillate and go off road. From this point I used Twiddle to find stable parameters and gradually incsease speed.

My final parameters are `Kp = 0.16, Ki = 0.00252, Kd = 2.1` at max speed 55 mph. Video [here](https://github.com/feklistoff/CarND-PID-Control/pid_55max.mov)

## Dependencies

* cmake >= 3.5
* make >= 4.1 
* gcc/g++ >= 5.4
* uWebSocketIO

[This](https://github.com/udacity/CarND-PID-Control-Project) repository contains all instructions for the Project.

This project involves the Udacity's Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and intall uWebSocketIO for either Linux or Mac systems.

Once the install for uWebSocketIO is complete, the main program can be built and ran.

## Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

