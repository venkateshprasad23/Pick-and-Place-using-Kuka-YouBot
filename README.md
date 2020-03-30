# Pick-and-Place-using-Kuka-YouBot
Course Project for MAE 204 : Robotics, taken during Winter 2020.

## Description
The objective of the project is to develop a trajectory generator for the Kuka YouBot to be able to pick and place a cube at a desired location.
I also implemented an appropriately tuned PI controller to drive the error between the actual and desired trajectory to zero. 
The feedback is taken from odometry.

The simulation was done on the V-REP Simulation Platform, while all the programming was done on MATLAB.


### Code Organisation
```
main.m                   -- Pretty self-explanatory.
calcJac.m                -- Calculates the Jacobian of the end-effector and returns its pseudo-inverse.
next_state.m             -- Implements odometry calculation.
reference_trajectory.m   -- Implements a Screw Trajectory Generator to produce the reference trajectory.
feedbackcontrol.m        -- Calculates the robot's commanded twist.
```

### Demo
![](/Results/demo.gif)



