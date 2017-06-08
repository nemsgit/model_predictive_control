# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Reflections

* The model
  * State variables and actuators
    * Each state at a given time has six variables, including:
      1. x - x coordinate
      2. y - y coordinate
      3. psi - orientation
      4. v - velocity
      5. cte - cross track error
      6. epsi - orientation error
    * At the same time, the car has two actuators:
      1. delta - steering angle (It is worth noting that the steering angle is defined differently in the simulator and\
       the MPC class. [-1,1] in the simulator corresponds to [deg2rad(25), -deg2rad(25)]. So a conversion factor of\
        -deg2rad(25) is needed between the two systems.)
      2. a - throttle
  * Kinematic model as update equations.
    * The following equations are used for updating the state variables
    ```
    x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
    y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
    psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
    v_[t+1] = v[t] + a[t] * dt
    cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
    epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
    ```
  * Cost function
    * The cost function contains altogether 7 terms, which are associated with the cross track error(cte), the \
    orientation error (epsi), the difference between reference and actual velocity (v - v_ref), the magnitudes of \
    steering angle (delta) and throttle(a), and their respective changing rates ((delta[t+1] - delta[t]) and \
    (a[t+1] - a[t])). Concretely, the following code accumulates the cost value originated from each term. Note that \
    each term has been assigned with a weight factor as an addition knob to balance their relative weights in the \
    total cost.
    ```
    // cte, epsi, and v - v_ref
    for (int t = 0; t < N; t++) {
        fg[0] += w_cte * CppAD::pow(vars[cte_start + t], 2);
        fg[0] += w_epsi * CppAD::pow(vars[epsi_start + t], 2);
        fg[0] += w_v * CppAD::pow(vars[v_start + t] - v_ref, 2);
    }
    // delta and a
    for (int t = 0; t < N - 1; t++) {
        fg[0] += w_delta * CppAD::pow(vars[delta_start + t], 2);
        fg[0] += w_a * CppAD::pow(vars[a_start + t], 2);
    }
    // changing rates of delta and a
    for (int t = 0; t < N - 2; t++) {
        fg[0] += w_ddelta * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
        fg[0] += w_da * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
    ```
  * Lower and upper bounds
    * Variable bounds - I set three variable bounds in the MPC. First, the orientation angle is limited to
    [-5/8 * PI(), 5/8 * PI()], as it is impractical for psi to go beyond this range within N prediction steps. Second,
    the steering angle is constrained to [-25, 25] (degree) according to the physical limit of steering control. Third,
    the throttle is limited to [-1, 1] to avoid excessive acceleration/decelerations.

    * Constraint bounds - Each of the 6*N constraints has its lower and upper bounds as well. The ones corresponding to
    the initial state (t = 0) are constrained to their initial values. The rest constraints are set according to the
    update equation shown above. More explanations can be found in the codes.

* Timestep length and elapsed duration (N and dt)
  * N is the number of future time steps we will be predicting, and dt is the length of each time step. N * dt gives
   the total time we can 'foresee' at each moment. An overly large N value would spend extra computational costs on
   unnecessary information. Moreover, it gives less accuracy as process noise accumulates over time. On the other hand,
   a too small N value would make the car too 'near-sighted' and unable to make sufficient and accurate predictions.
   Similarly, a too small dt value would cost unnecessary computational expanses, while a too large dt would reduce
   prediction accuracy as the steps are too discrete. I have tried a series of N values from 4 to 12 and finally settled
   down with N = 8. The simulation results are pretty consistent with the above analysis.

* Coordinate transformation and polynomial fitting
  * Before poly-fitting the reference curve (ptsx, ptsy), I transform their coordinates to the car's local coordinate
  system. Concretely, it is done by the following code (where px, py, and psi are the global position and orientation
  of the car):
  ```
  // Transform reference points to local coordinate system
  for(int i = 0; i < ptsx.size(); i++) {
      double x = ptsx[i] - px;
      double y = ptsy[i] - py;
      ptsx[i] = x * cos(-psi) - y * sin(-psi);
      ptsy[i] = x * sin(-psi) + y * cos(-psi);
  }
  ```
  * After this step I also reset px, py, and psi to their local coordinates:
  ```
  // Reset px, py, and psi in local coordinate system
  px = 0;
  py = 0;
  psi = 0;
  ```
  * The reference points are then fitted by 3rd order polynomial curve to obtain the fitting coefficients (coeffs):
  ```
  auto coeffs = polyfit(ptsx_evec, ptsy_evec, 3);
  ```
* Latency
  * To deal with latency, we do not send the current state directly to the MPC solver. Instead, we "manually" predict
  the future state after the latency time, and feed that future state to the model. The following code takes care of
  the 'manual prediction'. It is nothing else but reusing the update equation in the car's local coordinate system.
  The LATENCY is set to be 0.1 second in this case. We then construct a state vector with these "future" values and
  send it to the MPC solver.
  ```
  double px_delayed = px + v * LATENCY;
  double py_delayed = py;
  double psi_delayed = psi + v * (delta) / LF * LATENCY ;
  double v_delayed = v + a * LATENCY;
  double cte_delayed = cte + v * sin(epsi) * LATENCY;
  double epsi_delayed = epsi + v * (delta) / LF * LATENCY;
  ```

After completing all the steps the model is fully functional and supports reference velocities up to v_ref = 70.
Higher reference velocity would potentially drive the car out of curb. Further optimization of the hyper-parameters
(N, dt, and weights of different cost terms) will likely enable higher driving speed.



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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
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
