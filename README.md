# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Goals
In this project, the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. Given the car's localization and sensor fusion data, as well as a sparse map list of waypoints around the highway, the car is able to go as close as possible to the 50 MPH speed limit while passing slower traffic as necessary.

The car is able to make at least one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it takes a little over 5 minutes to complete 1 loop. Also, the car never experiences a total acceleration over 10 m/s^2 or a jerk that is greater than 10 m/s^3.


## Input Data:
#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

Here is the data provided from the Simulator to the C++ Program:


#### Main car's localization Data (No Noise):

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH


#### Previous path data given to the Planner:

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator


#### Previous path's end s and d values:

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value


#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise):

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

## Controller Architecture:
As I developed a code architecture for the path planner system, I found that it would be easiest to have the code running sequentially in the main.cpp file. There is a lambda function that is called every time a new telemetry message is received, and given all of the available data, I figured that there needs to simply be three distinct 'stages' that the data needs to flow through. The behavior module takes in localization data, as well as sensor fusion data of nearby vehicles to determine the optimal decision to make. The motion control module adjusts speeds to within the acceleration and jerk constraints. Finally, the trajectory generation module creates smooth spline trajectories from coarse waypoints defined by the previous modules.

### Behavior/Decision Module:
The behavior module is structured as a state machine, and is responsible for making decisions on whether to remain in lane, prepare to change lanes, and actually changing lanes. The states are enumerated as follows:  

0 : Cruise, drive at 49MPH and remain in lane.  

1 : Prepare Lane Change, match the speed of the vehicle directly in front, and look for openings in adjacent lanes.  

2 : Lane Change, lane change has been commanded and is in progress  

In the zeroth state,
### Motion Control Module:

### Trajectory Generation Module:
