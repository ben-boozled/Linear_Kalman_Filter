# Description
The Kalman Filter is an algorithm used to estimate and predict the state of a system. By minimising the mean squared estimation error between sensor measurements (which are usually affected by noise) and a theoretical model of the system behaviour, the Kalman Filter aims to provide the best estimate of the true system state. A key assumption of the Kalman Filter is that process and measurement noises can be modelled using Gaussian distributions. 

This repository provides a C++ OOP (object-oriented programming) implementation of the linear Kalman Filter, which is the simplest form of the Kalman Filter and assumes linear system dynamics. The simulation models the 2D movements of a constant velocity object affected by noise. The Kalman Filter is updated using position measurements from a noisy GPS sensor.

Below is a plot obtained from the simulation, showing the actual, measured, and filtered x-position of the object against time. In general, the filtered position should track the actual position closely and be less affected by noise compared to the GPS measurements.  The code can be modified to plot results for the y-positions, and the x- and y-velocities.
![X Position Plot Example](https://github.com/ben-boozled/Linear_Kalman_Filter/blob/main/Images/X%20Position%20Plot%20Example.png)

# Linear Kalman Filter
The general steps of the linear Kalman Filter are as follows:
1. **Initialisation**: Initialise the filter and the initial conditions of the system state.
2. **Prediction**: Estimate the current state of the system using the system model and information from the previous system state. This step increases uncertainty (which is measured by the state covariance matrix) because it relies on a mathematical model rather than empirical data.
3. **Update (Measurements)**: Incorporate sensor measurements to refine the predicted state. This step reduces the uncertainty in our state estimates.
4. **Iterate**: Repeat the prediction and update steps iteratively as new measurements become available. Since sensors may differ in sample rates, multiple prediction steps may occur before a measurement update, or several sensor measurements may be fused during a single update.

The notations used to represent Kalman Filter equations may vary between literature. This section clarifies the notations used specifically for this repository.

## Prediction Step
**x̄** = **Fx** + **Bu**  
**P̄** = **FPF<sup>T</sup>** + **Q**  
**Q** = **LwL<sup>T</sup>**
* **x**: state mean; initialised with the format \[𝑥, $\dot{𝑥}$, 𝑦, $\dot{𝑦}$\]<sup>T</sup>.
* **F**: state transition matrix.
* **P**: state covariance.
* **B**: control input matrix (assume 0).
* **u**: control inputs to system (assume 0).
* **Q**: process covariance (assume white noise).
* **L** and **w**: noise gain matrix and white noise model respectively, modelled as acceleration.

Note:
* Variables with the bar (i.e. **x̄** and **P̄**) represent the <ins>prior belief</ins>. This is the estimate of the state <ins>before</ins> reading any measurements.
* Variables without the bar (i.e. **x** and **P**) represent the <ins>posterior</ins>. This is the refined state estimate <ins>after</ins> incorporating measurements.

## Update Step
**y** = **z** - **Hx̄**  
**K** = **P̄H<sup>T</sup>**(**HP̄H<sup>T</sup>**+**R**)<sup>-1</sup>  
**x** = **x̄** + **Ky**  
**P** = (**I**-**KH**)**P̄**  
* **y**: residual (i.e. difference between sensor measurements and predicted value).
* **z**: measurement mean.
* **H**: measurement function.
* **K**: Kalman gain.
* **R**: measurement covariance.
* **I**: identity matrix.

# Running Locally
The simulation uses [Eigen 5.0.0](https://gitlab.com/libeigen/eigen/-/releases/5.0.0) to perform linear algebra operations and [Matplot++ v1.2.0](https://github.com/alandefreitas/matplotplusplus) to plot the results. There is no need to separately download these dependencies. The `CMakeLists.txt` file in the main project directory has been configured to automatically download these dependencies and install them in a new `libs/` project subdirectory during project compilation. Note that in this `CMakeLists.txt` file, `FetchContent_Populate()` is used to download the Matplot++ library as the latter has compatibility issues with `FetchContent_MakeAvailable()`. As `FetchContent_Populate()` is deprecated from **CMake 3.30** onwards, `FetchContent_MakeAvailable()` should ideally be used instead.
