# Model Predictive Control -- MPC
> [![MPC](images/MPC.gif)](https://youtu.be/Fps7wlI__2E "Model Predictive Control")  
> [Full Video](https://youtu.be/Fps7wlI__2E) on YouTube

# Objective
> Implement Model Predictive Control to drive the car around the track.
This time however you're not given the cross track error, you'll have to
calculate that yourself! Additionally, there's a 100 millisecond latency
between actuations commands on top of the connection latency.

# What I Learned
> - Implimented Model Predictive Controller
> - Cost function based motion
> - Fitting Polynomials
> - Following Trajectories
> - Kinematic and Dynamic vehicle models
> ### Languages
> C++

### Included Files
> [main.cpp](src/main.cpp) -- Interfaces with the simulator, connects to the MPC, sends steering and throttle information to the simulator  
> [MPC.cpp](src/MPC.cpp) -- Handles MPC calculations  
> [WriteUp.md](WriteUp.md) -- Full write up  
