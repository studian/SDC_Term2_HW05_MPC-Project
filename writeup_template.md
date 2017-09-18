# Motion Predictive Control Project

<p align="center">
    <img src="./mpc_result.gif" width="480" alt="main_image" /><br>
    <b>result image</b><br>
</p>

## Introduction

* This is Udacity's Self-Driving Car Nanodegree Term2 MPC Project.
* In this project you'll implement Model Predictive Control (MPC) to drive the car around the track.  
* The basic idea of MPC is to predict the states of the car by considering the equations of motions of the car and minimizing a cost.  

---

## Model

* The model of the MPC is a basic **kinematic model**. Not dynamic model. For more details, refer to [this paper](http://www.me.berkeley.edu/~frborrel/pdfpub/IV_KinematicMPC_jason.pdf).  
* The state of the vehicle consists of `x-position`, `y-position`, `psi` (orientation(heading direction)), `v` (velocity), `CTE` (cross-track-eror), `epsi` (orientation error)   
* Actuator consists of `steering angle` and `acceleration`.(positive = throttle, negative = brake)
* In simulator, MPC predicted trajectory is represented in green, and reference to the vehicle's coordinate is represented in yellow.

### Environment  
  
* ubuntu 16.04 (64bit)
  * memory : 62.8 GB
  * CPU : Intel Core i7-6850K CPU @ 3.60GHz x 12 ea
  * GPU : Titan X (Pascal) x 4 ea
* Ipopt-3.12.1  
* Tested Simulator
  * Term_2_Simulator_V1.3_Updated_MPC_ubuntu
  * Term_2_Simulator_v1.4_Localization_ubuntu
  * Term_2_Simulator_v1.45_PID_ubuntu

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Details

### Model

* The vehicle model is a kinematic model and these are the equations for the model :

```
x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v_[t] = v[t-1] + a[t-1] * dt
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
```

### Timestep and Elapsed Duration( N, dt)

* N is the number of timesteps in the horizon. dt is how much time elapses between actuations. 
* They are hyperparameters and T(N*dt) should be as large as possible, while dt should be as small as possible.  
* To find the appropriate value for N and dt, I tested several values(e.g. N = 7, 10, 15, 20 and corresponding dt value). 
* In my model, N = 10, dt = 0.1 were best values.

### fitting waypoints & latency

* The waypoints of the road are given in the global coordinate. 
* So I need to convert it to the vehicle coordinate. 
* After that, the waypoints are fitted with a 3rd order polynomial.

```
for (int i = 0; i < ptsx.size(); ++i) {

	// move to origin and rotate around origin to align with axis
	double shift_x = ptsx[i] - px;
	double shift_y = ptsy[i] - py;

	ptsx[i] = shift_x * cos(0 - psi) - shift_y * sin(0 - psi);
	ptsy[i] = shift_x * sin(0 - psi) + shift_y * cos(0 - psi);
} 
```

* In the simulator, there is a latency between actuations commands.(100 ms)  
* I considered this latency and the equation is as below.  
 
```
double latency_x = 0 * v * (100 / 1000);
double latency_y = 0;
double latency_psi = 0;
double cte = polyeval(coeffs, 0) - 0;
double epsi = latency_psi - atan(coeffs[1] + 2 * latency_x * coeffs[2] + 3 * pow(latency_x, 2) * coeffs[3]);
```
