# Differential Drive Robot Motion Planning and Path Tracking.

### Hierarchical Planner
* Mission Planner
* Behavioral Planner
* Local Planner

1. As Mission Planner Hybrid A* is used as it is specifically designed for planning paths for vehicles with non-holonomic constraints, such as differential drive robot. It incorporates additional considerations like the vehicle dynamics and steering constraints to generate feasible paths.
2. The differential drive vehicle kinematics is defined in the behavioral planner.
3. Pure Pursuit is used as a local planner.

### Requirements
* MATLAB
* Navigation Toolbox
* Robotics System Toolbox

### Usage
Run the differential_drive.m file to see the system working.
* You can change the start and goal positions of the robot at lines 61 and 62 of the matlab file. 
* Furthermore you can play with the the local planner parameters such as "Look Ahead Distance", "Desired Linear Velocity" and "Max Angular Velocity" to better understand the pure pursuit algorithm. Especially tweaking Look Ahead Distance will be very helpful in understanding how the high and low values of look ahead distance affect the path tracking capabilities and oscillatory behaviour of the system.
* You can also change the Minimum Turn Radius of the hybrid A* to see how the path joining the start and goal gets affected.


### Caution
PLease make sure that the path to the occupancy map is provided or make sure that the occupancy map lies in the same folder as the MATLAB program file.

### Result
[![Video](https://img.youtube.com/vi/a37EcbaYxFs/0.jpg)](https://www.youtube.com/watch?v=a37EcbaYxFs)
 
