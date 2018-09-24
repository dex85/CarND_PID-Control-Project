[P-step]: ./p-step.jpg "Logo P step response"
[PID-step]: ./pid-step.png "Logo PID step response"


# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

## Introduction
In this project the lake race track from the Behavioral Cloning Project from term 1 is used to steer a trained self-driving car. Whereas the steering is accomplished by a PID controller in C++ which has to be implemented as part of this project. PID stands for Proportional Integral Differential, which describes the way the controller corrects the error of the reference value/trajectory or setpoint and the current state. The reference value is set and the state by sensors, or in this case by a simulation. It is possible to only use partial or a combination of the controller in order to successfully (more or less) steer the vehicle. In the following paragraphs the P controller, PD controller, PID controller and the optimal parameters for each should be described and evaluated with the simulator.

### P controller
The simplest (after two-point) controller which was introduced in lesson 16.3. The idea is that the controller gives an output that is proportional to the error (in our case current position and reference trajectory, CTE) and then feedbacks to the actual actuators (through converters...) or in the car's case the steering angle. In simple terms it is amplified or multiplied. Which leads to a model of
###### alpha (t) = K_P * CTE(t)
where
* alpha is the output of the controller
* K_P is the proportional gain or tuning parameter
* CTE is the error that should be ideally zero in steady state, in our case it's the difference of the current value and reference value

The picture below describes a common step response of a P-Controller
!['step response'][P-step]
The problem is the overshooting of the P-controllers error which can lead to instabilities and oscillations. The graphs shows as well the effect of the CONSTANT, which is in general how fast the target value or reference should be reached. The side effect of a fast P-controller is a high overshoot and oscillation.
If the simulation would be started with only the P value, the car would act like a pretty much drunk driver and finally drive into the water. (either way, low or high P value) `./pid 0.1 0 0`

### PD controller
In order to avoid the overshooting and oscillating but don't slow the controller down, a differentiating term was added and lead to the PD controller. The differentiating term has the effect of counteracting the P part by using the the previous error and subtracting it from the current error. Dividing it the the elapsed time between the two values leads to the first derivative of the CTE. That compensating works, because (usually) the previous error is bigger than the current one, which leads to a "negative" derivative and that is in opposite to the P term and so it "damps" it and hopefully it prevents overshooting. The PD model is given by
###### alpha (t) = K_P * CTE(t) + K_D * (CTE(t) - CTE(t-1))/(d_t)
where
* alpha is the output of the controller
* K_P is the proportional gain or tuning parameter
* K_D is the differential gain or tuning parameter
* CTE is the error
* CTE(t-1) is the previous error

If the simulation is started with a PD controller of some common values the car would drive good around the track and not drive off road. `./pid 0.1 0.0 1.0`


### PID controller
It can happen that after P or PD control is applied, that there is still an error that can't be eliminated due to mechanical error or system errors. So, after controlling and reaching steady state the reference trajectory couldn't be reached. In order to compensate this residual error, the Integrating term is added to the control chain. The integrating adds up all past CTEs and so gets larger over time and compensates any remaining residuals. After eliminating the errors, it stops adding errors and so compensates the residual in steady state. the PID model is given by
###### alpha (t) = K_P * CTE(t) + K_D * (CTE(t) - CTE(t-1))/(d_t) + K_I * sum(CTE)
where
* alpha is the output of the controller
* K_P is the proportional gain or tuning parameter
* K_D id the differential gain or tuning parameter
* K_I is the integral gain or tuning parameter
* CTE is the error
* CTE(t-1) is the previous error
* sum(CTE) is the sum over all previous errors.

The graph below shows a typical PID step response
!['step response'][pid-step]
It's good to see, that it has a small rise time, no overshooting and no residual error as well as no oscillation (which is very ideal and usually not the case). The downside is of course it has more computational cost than all the others. And PID is not always the best solution.

### Parameter tuning
Do determine the PID parameters the Twiddle algorithms was partly implemented in the "main.cpp" lines 122-216. After some errors, it seemed to work just fine; at iteration 67 the parameters didn't change much anymore and the tuning was manually stopped and resulted in
* Kp = 0.174
* Kd = 0.9097
* Ki = 0.0006
The process started with a fixed throttle of 0.3 as initially set by the creator. The parameters were all initially 0 and the increments all initially 0.1.(lines 50-54) The total error was calculated by summing up all CTEs for 150 simulation iterations, after the simulation ran with the updated parameters for about 150 points (so it had some time to adjust). If the total error was less than the best error (initially just a large number, line 22) the specific parameter was kept and the simulation started over as well as all errors were reset to zero and the parameter index was increased. If the error was higher, the parameter was decreased  by two times its current delta and the simulation was reseted as well as the errors. The next time, if this parameter performed better and the error was less than the best error, parameter was kept and delta increased as well as the index. If it didn't perform better, parameter was reset to its previous state, the delta was decreased and the parameter index increased.
The throttle of 0.3 resulted in a maximum speed of about 33 mph, which seemed enough at that time. Never the less, a PID controller seemed a little too much, were a P controller or a PD controller would do perfectly as well. So, in order to use Twiddle to find a good PD controller, the pd values for the integrating factor was set to 0 (as suggested in the lecture). The process was similar to the steering part and resulted after approximately 40 Twiddle iterations in
* Kp = 0.01
* Kd = 0.15
The effect is a slowly accelerating car with a top speed of less than 30 mph, even if the target is 50 mph.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it:
* `./pid` for standard values
* `./pid twiddle` for twiddle
* `./pid Kp Ki Kd` for manual settings of Parameters, where Ki, Ki, Kd the PID parameters are, double values
* `./pid Kp Ki Kd throttle` for manual settings of Parameters, where Ki, Ki, Kd the PID parameters are and throttle the throttle value is, all double values

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
