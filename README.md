# Unscented Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program.

In this project an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d).

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On Windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Tuning and Testing Results

After initial Unscented Kalman Filter implementation it did not perform too well. I had to tune standard deviation of longitudinal and yaw acceleration noise as well as initial values for covariance matrix. Here are the final values I ended up using:

```cpp
std_a_ = 2.55;
std_yawdd_ = 1.5;
P_ << 1,  0,  0,  0,  0,
      0,  1,  0,  0,  0,
      0,  0,  10, 0,  0,
      0,  0,  0,  1,  0,
      0,  0,  0,  0,  1;
```

With these values Kalman Filter finished on test dataset within the required RMSE limits (see below).

After tuning the parameters I've also tried to turn on only Radar and only Lidar and compare them to using both sensors. All the results of testing on the first data set are in the table below:

| Field RMSE    | Laser only | Radar only | Laser & Radar | Rubric Requirements |
| ------------- |-------------:| -----:| -----:| -----:|
| P<sub>x</sub>  | 0.1041 | 0.1701 | **0.0719** | 0.09 |
| P<sub>y</sub>  | 0.1022 | 0.2705 | **0.0857** | 0.10 |
| V<sub>x</sub> | 0.5165 | 0.4222 | **0.3811** | 0.40 |
| V<sub>y</sub> | 0.2932 | 0.3549 | **0.2470** | 0.30 |

As expected, using both sensor leads to more accurate car localization. Now let's compare to the results of EKF (see full project [here](https://github.com/ysavchenko/carnd-term2-extended-kalman-filter)):

| Field RMSE    | EKF | UKF |
| ------------- |----:|----:|
| P<sub>x</sub>  | 0.0983 | **0.0719** |
| P<sub>y</sub>  | 0.0852 | **0.0857** |
| V<sub>x</sub>  | 0.4071 | **0.3811** |
| V<sub>y</sub>  | 0.4682 | **0.2470** |

As you can see, Unscented Kalman Filter produce better results on the same data set. This is a result of CTRV (constant turn rate and velocity) model we used in UKF instead of CV (constant velocity) model we used in EKF. In the test data the car does not move linearly, but drives on a curve and for this case CTRV model is better.

## Bonus Challenge

I've also tried to solve [Catch the Run Away Car with UKF](CarND-Catch-Run-Away-Car Github repository) bonus challenge. It has slightly different commands sent to and from simulator via web sockets, so to support both modes (sensor fusion and catching the car) I've modified `main.cpp` a little. Now it parses for command line parameters and calls one of the functions: `message_hunter()` or `message_sensor()` depending on how it is launched. 

So, to launch UKF in regular sensor fusion mode use command without parameters:

```
./UnscentedKF
```

...and to catch the run away car launch it with `catch` parameter:

```
./UnscentedKF catch
```

Without doing any modification to the tracking code the hunter car always aimed where run away car is currently. This way it was not able to catch it (see the video below):

[![](http://img.youtube.com/vi/8JJ26SqNixw/0.jpg)](http://www.youtube.com/watch?v=8JJ26SqNixw)

So I did some modifications to aim not where the car **is**, but where the car **will be** when the hunter catches up with it. To do this I used the formula:

X<sub>target</sub> = X<sub>target</sub> + cos(ψ) * v * Δt

Y<sub>target</sub> = Y<sub>target</sub> + sin(ψ) * v * Δt

...where Δt is amount of time it takes for hunter to reach the car. And we can calculate this time by dividing distance between the hunder and the car by the speed. After this the speed cancels out and we get the following code:

```cpp
target_x += distance_difference * cos(ukf.x_[3]);
target_y += distance_difference * sin(ukf.x_[3]);
```

After these modifications hunter was able to catch the car in about 4 seconds on every test (see video below):

[![](http://img.youtube.com/vi/fLvtjII-kI8/0.jpg)](http://www.youtube.com/watch?v=fLvtjII-kI8)

The adjustment above is not perfect because we use current car positions to calculate Δt (this is why hunter car does not move on the straight line), but I think it is good enough for the purpose of this challenge.