# CarND Model Predictive Control Project

I was able to successfully drive the car autonomously around the track at
40mph. A video is available on YouTube here:
https://www.youtube.com/watch?v=Skod3GHCJpc


## Table of Contents
1. The Model
2. Timestep Length and Elapsed Duration (N & dt)
3. Polynomial Fitting and MPC Preprocessing
4. Model Predictive Control with Latency


## 1. The Model

Building an accurate model is important. The model describes the behavior of the car
given certain input. If we can not properly describe the behavior we can not
predict it.

We create a model by defining a state for the car. The state
consists of current position, velocity, and anything else that is
required to accurately define the car in its current state. Once a state is
defined, we create equations that relate the parameters of a future state and
the current state. A well defines state along with these equations, make a
model.

The variables in our state are:
- x    : x coordinate of the car
- y    : y coordinate of the car
- psi  : orientation of the car
- v    : forward velocity of the car
- cte  : cross track error
- epsi : orientation error

We included the error because, we want our inputs to be proportional to the errors when we control the car. This
allows us to gradually apply the steering and acceleration resulting in
smoother driving.

Following are the equations that relate the current state and the future state.
Here we assume that both states are separated by a very small time dt.

```cpp
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```


## 2. Timestep Length and Elapsed Duration (N & dt)

The prediction horizon(T) is the amount of time into the future that MPC is going to predict
the trajectory of the car. T is usually very short because there is no use of
predicting too far ahead in the real world. There are many external factors in the
real world that can change the intended or predicted behavior of the car.

Once T is fixed, we need to figure out what the Timestep Duration(dt) is and
that gives us the number of Timesteps(N).

dt is the amount of time we wait between each actuation. This has to be small
enough that our car travels mostly according to the model. If dt is too long,
MPC will not work. MPC predicts a continuous reference trajectory with the help
of small discrete paths between each actuation. It is like attempting to draw a
smooth curve by drawing many short straight lines. Longer the dt is longer is
the time between MPC can sense the surroundings and apply a correction.
Therefor, longer dt causes inaccurate approximation of a trajectory, which is
sometimes referred to as "discretization error".

The values I chose are `N = 10` and `dt = 0.1`. First, I chose the value of T
as 1 second. Then dt is chosen based on the given latency of 100 milliseconds.
Because, trying to calculate or apply the next actuation before the current
actuation is carried out was not intuitive to me. After T and dt are fixed, N
has to be 10. I started by using N=20 and dt=0.05, also tried T=2. But current
values gave better results.


## 3. Polynomial Fitting and MPC Preprocessing

MPC has the following steps:
1. Fit a polynomial to the waypoints
2. Define the initial state
3. Pass the polynomial coefficients and initial state to the Solver
4. Setup solver variables, upper/lower bounds for variables, constraints,
   upper/lower bounds for constraints.
5. Pass the vars, constraints and the cost function to the ipopt solver.
6. Solver gives the most optimum actuations at each timestep.

*Preprocessing*
Before we fit a polynomial on step 1 above, we need to map the world coordinates
to car coordinates.

There is an another preprocessing step to work with latency, which I will
discuss in the next section.


## 4. Model Predictive Control with Latency

Latency in the system is handled by calculating a future state that is exactly
100 milliseconds into the future from the initial state. This future state is then passed to the
solver. This is the suggested technique in the Latency section of the MPC
lesson.

