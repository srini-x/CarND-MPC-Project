# CarND Model Predictive Control Project

## Table of Contents
1. The Model
2. Timestep Length and Elapsed Duration (N & dt)
3. Polynomial Fitting and MPC Preprocessing
4. Model Predictive Control with Latency


## 1. The Model

building an accurate model is important
model describes the behavior of the car
kinematic model
model variables
errors
inputs

cost-function

## 2. Timestep Length and Elapsed Duration (N & dt)

The prediction horizon(T) is the amount of time into the future that MPC is going to predict
the trajectory of the car. T is usually very short because there is no use of
predicting too far ahead in real world. There are many external factors in the
real world that can change the intended or predicted behavior of the car.

Once T is fixed we need to figure out what the Timestep Duration(dt) is and
that gives us the number of Timesteps(N).

dt is the amount of time we wait between each actuation. This has to be small
enough that our car travels mostly according to the model. If dt is too long,
MPC will not work. MPC predicts a continuous reference trajectory with the help
of small discrete paths between each actuation. It is like attempting to draw a
smooth curve by drawing many short straight lines. Longer the dt is longer is
the time between MPC can sense the surroundings and apply a correction.
Therefor, longer dt causes inaccurate approximation of a trajectory, which is
sometimes referred to as "discretization error".


## 3. Polynomial Fitting and MPC Preprocessing

we get raw points from the simulator
car doesnt travel in 3d so x and y are enough
convert them to car local co-ordinates by applying a transformation

polyfit with 3

create initial state and call the solver

build the vars, constraints, call the fg_eval object on coeffs
call the ipopt solver


send the data back to main so it can understand.

## 4. Model Predictive Control with Latency

```cpp
auto coeffs = polyfit(car_ptsx, car_ptsy, 3);
double cte = coeffs[0];
double epsi = -atan(coeffs[1]);

double dt = 0.1;
double Lf = 2.67;

double x_delayed = v * dt;
double y_delayed = 0.0;
double psi_delayed = - v * delta / Lf * dt;
double v_delayed = v + a * dt;
double cte_delayed = cte + v * sin(epsi) * dt;
double epsi_delayed = epsi - v * delta / Lf * dt;

Eigen::VectorXd state(6);
state << x_delayed, y_delayed, psi_delayed, v_delayed, cte_delayed, epsi_delayed;
```
