# Model Predictive Control
## Udacity Self Driving Car - Nano Degree

### Relevant Files
> main.cpp -- controls the interaction between the simulator and the MPC object  
> mpc.cpp -- describes the MPC object  

### 1. Model
> This project utilized a technique called Model Predictive Control (MPC).
> The model takes as input, a state vector.
The state of the vehicle was described with a single vector. The vector included the current position (x, y), 
velocity (v), orientation (psi), cross-track error (cte), and psi error (epsi). The MPC model takes in this 
state vector: 
< x, y, v, psi, cte, epsi > and returns a vector containing steering angle and throttle.
> 
> The model considers the current state of the vehicle, and the desired state for upcoming waypoints (provided
by the simulator). The MPC model then computes multiple possible trajectories given possible changes to the 
steering angle (delta) and the throttle. The model computes which of these possible trajectories is closest to the
desired path by computing the total error. The error is a weighted sum of the cross track error, psi error, and
a few parameters designed to smooth out the driving behavior of the car. [MPC.cpp 74 - 93]. Based on the trajectory with the least 
error, the model returns the corresponding steering angle and throttle for the next time step. This process
is known as Model Predictive Control (MPC)
> 
> The MPC uses the folowing equations to predict the state of the car at a future time, depending on the change
in time, (dt):
> ![equations](images/eqns.png)
> 
> These controls, steering angle and throttle, are passed to the simulator and applied to the car. These two
values are sufficient to drive the car around the track. 
>
### 2. Time
> The MPC model predicts where the car will be at discrete time-steps. For each time-step, the predicted position
and error are calculated. There are then, two parameters to tune. The number of time-steps (N), and the elapsed 
time between each time-step (dt). More time-steps will project the path of the vehicle further into the future. 
Smaller time steps will provide more points for comparing the projected path to the desired path, which allows for
a more accurate path to be traveled by the car. For this model I chose N = 20 and dt = 0.05.
>
> In part, the parameters are dependent on the capabilities of the CPU. The CPU would get clogged up with too
many paths to predict, and errors to calculate. On the other hand, too few points (N * dt) will result in an
unrealisic curve, and minimizing the error will be impossible. Through trial and error, I found that more than 20
points into the future projected the path out to an unusable distance. And a dt smaller than 0.05 did not yield a
better trajectory.
>
> I originally used the values [10, 0.1] based on the QandA video provided by Udacity. I then sequentially 
increased N and decreased dt such that they still projected roughly 1 second into the future. eg, [11, 0.9], 
[15, 0.07], [20, 0.05]. 

### 3. MPC Processing
> The waypoints are received from the simulator in map coordinates. They are then transformed into vehicle
coordinates to make processing easier. After the affine transformation, x and y are at the origin and the
orientation angle was zero (relative to the car). Once the points have been transformed, the MPC process is as
follows.
>
> #### MPC Process:
> 1. Input the current state of the vehicle (transformed into vehicle coordinates)
> 2. Solve for optimized polynomial -- minimize the cost function
> 3. Send control vector [delta, throttle] to the simulator

### 4. Latency
> Part of the project was to be able to handle a 100 ms latency. This is representative of a real world situation
in which there is a delay from the time a steering angle or throttle is calculated, to when they can be applied.
>
> In order to account for latency, I predicted where the vehicle would be 100 ms in the future, and calculated
the appropriate controls for that state, not the current state. This way, the controls are applied at the time
when the car is close to where it was predicted to be via the MPC. MPC is able to directly account for latency, 
where the PID controller was not.


# Equations:
> If the equations images does not show, the equations are:  
x = x + v*cos(ψ)* dt  
y = y + v sin(psi) dt  
v=v+a∗dt  
a in [-1,1]  
ψ=ψ+(v/L_f)*δ∗dt  







