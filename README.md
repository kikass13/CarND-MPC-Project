# CarND-Controls-MPC

## Discussion
### The Model
My model gets the input state from the simulation via json, containing vehicle position, orientation, cross-track error and psi error. After that, the next state is predicted/extrapolated using the given approximated latency of 100ms. By inverting osi, the vehicle now turn in the right direction (simulation lateral delta is inverted):
```
fg[1 + psi_start + t] = psi_t1 - (psi_t0 - v_t0 * delta / Lf * dt);
```

the model consists of the following equation parts:

- positionX = oldPositionX + differential positionalXDelta(cos) resulting from velocityX in deltaT
    -  fg[1 + x_start + t] = x_t1 - (x_t0 + v_t0 * CppAD::cos(psi_t0) * dt); 
- positionY = oldPositionY + differential positionalYDelta(sin) resulting from velocityY in deltaT
      - fg[1 + y_start + t] = y_t1 - (y_t0 + v_t0 * CppAD::sin(psi_t0) * dt);
- orientationAngle = oldorientationAngle - velocityCompensated(delta, Lf) differential orientationDelta resulting in deltaT
      - fg[1 + psi_start + t] = psi_t1 - (psi_t0 - v_t0 * delta / Lf * dt);
- velocity = oldVelocity + velocityOffset resulting from acceleration in deltaT
      - fg[1 + v_start + t] = v_t1 - (v_t0 + a * dt);
 - cte = lateral(f(x)=y)Diff + velocityCompensated lateral offset resulting from actual angularError(epsilon) 
     - fg[1 + cte_start + t] = cte_t1 - ((f_t0 - y_t0) + (v_t0 * CppAD::sin(epsi_t0) * dt));
    - fg[1 + epsi_start + t] = epsi_t1 - ((psi_t0 - psides_t0) - v_t0 * delta / Lf * dt);
    
			
### Timestep Length and Elapsed Duration (N & dt)
 - timestep length = 10 
 - elapsed duration = 0.1
 
Having a shorter deltaT means that my mpc itself reacts to fast and to hard on lateral changes, which causes the vehicle to drive off the road. 

### Polynomial Fitting and MPC Preprocessing
The global waypoints are processsed by transforming them from the simulator coordinate system into the vehicle coordinate system (ego). After that a 3rd order polynomial is fitted through these waypoints.

### Model Predictive Control with Latency
The state is extrapolated into the future (100ms), which is then fed into the mpc model. this ensures a correct state transition (and optimally the rejection of old states)

---

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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.
