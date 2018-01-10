## Project: Model Predictive Control (MPC)

---


**Steps to complete the project:**  


1. Implement the MPC Model in `MPC.cpp`.
2. Fill in the MPC solver for the optimisation library.
3. Set the model system and controller constraints.
4. Fill in the number of model variables, system time step and duration.
5. Transform the global path coordinates into vehicle centric coordinates in `main.cpp`.
6. Calculate the system trajectory errors.
7. Display the planned path and the MPC predicted path onto the road.
8. Find model cost parameters that make the vehicle travel around the Lake circuit as fast as possible without leaving the track.


[image1]: ./img/transform_formula.png

## [Rubric](https://review.udacity.com/#!/rubrics/896/view) Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/896/view) individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Project Implementation

#### 1. Fill in the `MPC.cpp` file with your system model and describe the model, state, actuators and update equations.

Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.

###### State Vector

The first part of an MPC is to determine the systems states that are desired to successfully predict the movement of a vehicle. Since the vehicle will move along a plane, we will need to track it's coordinate position (x & y), as well as the direction / heading angle (ψ) of the vehicle to know which way the vehicle is facing. Since the vehicle is not stationary, We will require tracking its velocity (v) as well. Vehicle acceleration will not be modelled, Due to the following reasons
*  Complexity due to environment dependence. I.e. acceleration is dependent on gravity and will vary dependent on the road gradients.
* Road surface friction.
* Vehicle weight variations.

However, due to a 10Hz update rate, the above acceleration factors can be neglected and we can use the derivative of the velocity function to model the acceleration of the vehicle.

In the implementation of the MPC and the `Ipopt` optimiser library used, the state space also contains the system error. Two model errors are tracked, the Cross Track Error (*cte*) or the difference between the path trajectory and the vehicle's current position and also the heading angle error (*eψ*) which is the difference between the vehicles heading angle and the trajectory paths heading angle.

The vehicle observed state space is (*x*, *y*, *ψ*, *v*, *cte*, *eψ*).
The state is calculated and stored into the state vector on lines 122 to 136 in file `main.cpp`.

###### Model Update Equations

From the state space, we can use vehicle kinematics and dynamic mathematical model equations to predicting the performance of the system in the future.

x and y position update can be calculated using basic trigonometry formulas and is as follows.

```
X(t+1) = X(t) ​​+ V(t) ∗ cos(ψ(​t​)​) ∗ dt
Y(t+1) = Y(t) ​​+ V(t) ∗ sin(ψ(​t​)​) ∗ dt
```

In the above formula, `X(t)` and `Y(t)` are the current vehicle x, y coordinates and `X(t+1)`, `Y(t+1)` are the future coordinates in one time step. `V(t)` is the vehicle velocity at the current timestep, `ψ(​t​)` is the vehicle heading in the current timestep and `dt` is the time step change in time between calculations. The code is located on lines `121` and `122` of the `MPC.cpp` file.

The vehicle heading, *ψ*, is updated with the following mathematical formula.  

```
ψ(t+1) = ψ(t) + ​L​f / ​V(t) ​​∗ δ ∗ dt
```

`ψ(t)` is the current vehicles heading angle and `ψ(t+1)` is the future heading angle in one-time step. `​L​f` is the distance between the front of the vehicle and its centre of gravity. The larger the vehicle, the slower the turn rate. `δ` is the vehicles current steer angle. The code is located on line `123` of the `MPC.cpp` file.

To update the future velocity `V(t+1)`, the following formula is used.

```
V(t+1) = V(t) + a(t) * dt
```
`a(t)` is the vehicle acceleration at the current timestep. The code is located on line `124` of the `MPC.cpp` file.

The `cte` update calculation is;

```
cte(t+1) = f(x) - y(t) + V(t) ∗ sin(ψ(​t​)​) ∗ dt
```

The `cte(t+1)` is the next time step cross track error. `f(x)` is the path trajectory function. In this system, `f(x)` is the polynomial of the path trajectory points. The code is located on line `125` of the `MPC.cpp` file.

The final vehicle state update is the heading angle error, *eψ*. It is calculated with the following formula;

```
eψ(t+1) = eψ(t) - Y(t) + V(t) ∗ sin(eψ(​t​)​) ∗ dt
```
`eψ(t)` is the current vehicles heading angle error and `eψ(t+1)` is the future heading angle error in one time step. The code is located on line `126` of the `MPC.cpp` file.

The update equations are calculated in the function `operator()`, located in the file `MPC.cpp`, between lines 89 and 127.

###### Actuators

As this project is implemented in a simulation environment, the simulation controls used are;

* Steering - Positive angles steer in the right direction.
* Throttle - Positive value accelerates the vehicle and negative applies the brakes.

Both system controls have a limited range of movement due to the physical restriction of the model in physical world, with throttle limits of **1** and **-1** and a steering limit of 25&deg; and -25&deg;

The model controls are limited in the code between lines `167` and `173` of the `MPC.cpp` file.

###### Model

Model Predictive Controllers use the cost of following a predicted path to find the best path to the desired location by finding the predicted path that produces the lowest cost.

The MPC cost is calculated in the function `operator()`, located in the file `MPC.cpp`, between lines 52 and 129.

In the vehicle model used, the cost is calculated in three ways.
1. The error between the desired state and the vehicles current state in position, heading angle and velocity. Lines 61 to 65.
2. The use of the controller throttle and steering inputs. Lines 69 to 71.
3. The rate of change between successive controller throttle and steering inputs. Lines 74 to 77.

The costs penalties for each error can be changed by updating the variables located on lines 25 to 31, in file `MPC.cpp`.
The following table shows the penalties used and their corresponding system effect.

variable | penalty | description
--- | --- | ---
cte_penalty | 4500 | error in path and car position
epsi_penalty | 8000 | heading error between path and car
speed_penalty | 3.25 | penalty for difference in speed
steer_use_penalty | 200 | penalty in steering the vehicle
a_use_penalty | 1 | penalty in accelerating or braking the vehicle
steer_change_penalty | 50 | change between steer angles
a_change_penalty | 1 | change in acceleration and braking penalty

#### 2. Discusses the reasoning behind the chosen N (time step length) and dt (elapsed duration between time steps) values and detail previous values tried.

The choice of *N* and *dt* are critical to the performance of the MPC. If *N* is too small, the MPC will not be able to predict far enough into the future and will not predict an accurate current state and controls to prevent system overshoot or instabilities. However, a large *N* predicts unnecessary data for the future, thus increasing the computational cost.

A good *dt* value should be able to capture a system control response accurately. If *dt* is large, it will miss important control actions and the vehicle will be less responsive. If it is too small, it will respond in a very noisy fashion, fluctuating the controls faster than the car can respond.

The optimal *N* and *dt* combination should follow the rule; **future prediction > system settling time**. For vehicle control, we need to predict far enough into the future to be able to stop the car before it crashes.

For the project *N* is set on line 9 in the `MPC.cpp` file. It was found that if the vehicle travels slower than 60 Mph, a smaller *N* of 6 works but is less responsive to significant system changes. A larger *N* above 18 was found to predict the later *N* values as wayward points due to the quadratic function fitting in certain parts of the track. *N* was chosen to be 10 as it predicted far enough into the future and offered system stability with a very little system performance loss.

*dt* is set on line 10 in the `MPC.cpp` file. It was found that due to the systems 100ms latency, a *dt* value smaller or equal to this produced control instabilities, while larger that 160ms was found to be slow responding to the situation. *dt* was chosen to be 120ms as this provided the best performance for the system.

#### 3. Describe the path trajectory Polynomial Fitting and the system MPC Preprocessing.

In the simulator, a path trajectory is supplied to the system in world coordinates, but as the vehicle controls and sensors are relative to the vehicle itself, the trajectory path must be transformed to vehicle centric coordinates. This involves rotation and translation between the two coordinate systems and is accomplished by the following equations.

```
Xc = Xt ∗ cos(​Θ) - Yt * sin(Θ)
Yc = Xt ∗ sin(​Θ) + Yt * cos(Θ)
```

`Xc` and `Yc` are the x and y coordinates in vehicle centric coordinates. `Xt` and `Yt` are the translation position of the car in the world reference frame and `Θ` is the angle of between the world coordinates *x-axis* and the car coordinates *x-axis* about the world coordinates *z-axis*, provided it is perpendicular to the car frames *x-axis*. This equation is implemented in between lines 102 and 107 in the `main.cpp` file.

Once the path trajectory points have been converted into vehicle centric coordinates, a line of best fit is calculated in the form of a quadratic equation. This equation is feed into the MPC for time stepped path prediction. The polynomial fitting is located on line 115 in the `main.cpp` file.

#### 4. Describe how your Model Predictive Controller handles a 100-millisecond latency.

In order to account for the system latency, the control values that are acted upon, are taken one-time step in advance or at time step *t1* from the current state. This means that as the MPC predicts the system from the current state, we act as if the state was in the past when the calculation was first performed. This prevents the model from lagging the system.

### Simulation

#### 1. The vehicle must successfully drive a lap around the track. No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).

A video of a completed lap can be found on youtube [here](https://youtu.be/t5-I2tkAX5E).
