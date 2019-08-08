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

### Behavior/Decision Module: (Lines 111-234)
The behavior module is structured as a state machine, and is responsible for making decisions on whether to remain in lane, prepare to change lanes, and actually changing lanes. The states are enumerated as follows:  

0 : Cruise, drive at 49MPH and remain in lane.  

1 : Prepare Lane Change, match the speed of the vehicle directly in front, and look for openings in adjacent lanes.  

2 : Lane Change, lane change has been commanded and is in progress  

In the zeroth state, there are no cars ahead of the ego vehicle that is less than 30 meters away. This number was a predefined "lookahead" distance for allowing adequate time for decelerating. It maintains a constant 49MPH so that there is a buffer preventing the car from overspeeding. In this state, the sensor fusion data is repeatedly scanned for any vehicles ahead that violate the lookahead distance.  

Once a vehicle is detected ahead to within 30 meters of the ego car, the state machine goes to state 1 and prepares to change lanes. The other car's velocity is computed from the sensor fusion information, and the ego car begins to match that speed. At the same time, sensor fusion data is repeatedly scanned to check if there are any cars to the left or to the right. Only a window starting 10 meters behind, to the distance of the followed vehicle is considered. A cost function taking into account the number of vehicles in that window, as well as position within that window, is computed. An impermissible region immediately to the side of the ego car has an arbitrarily large fixed cost to guarantee that the car will never go there.  

For each vehicle, the cost is equal to 1 - 1/s (in Frenet coordinates). This cost is summed up across all vehicles on that side, in the defined window, and normalized by computing 1-exp(total).  

The cost for remaining in lane and simply following the slower vehicle ahead is simply 1 - speed/49. As the car slows down, the more propensity it has to change lanes. In addition, whenever the car was in the far left or far right lanes, lane changes that would have taken the car off the road were hardcoded to have a fixed cost of 1.  

The current lane is computed using the localization data's d value, and is kept separate from the target lane determined by the behavior module. If the two are not equal, the state machine moves to state 2, since it means that a lane change is still in progress. This way, the scan of the surroundings still happens from the starting lane.  

### Motion Control Module: (Lines 240-280)
Perhaps the simplest of the three modules, the motion control module is responsible for maintaining a set speed, determined by the behavior module. When the car is following a slower vehicle ahead, the behavior module commands a speed equal to the followed vehicle's speed. The motion control modules uses a simple P controller to incrementally change the current speed, while also capping the acceleration and jerk of the maneuver.  

### Trajectory Generation Module: (Lines 282-383)
The trajectory generation module is responsible for taking the commanded lane and speed and generating a smooth trajectory using the C++ spline library. For the trajectory to be as smooth as possible, I take advantage of the points from the previous trajectory to ensure continuity between each successive generated trajectory. If the car is starting out from a standstill, this case is handled by using the current point as a point in the previous trajectory.  

Given the previous trajectory points, the future (s,d) points at lookahead distances of 30, 60, and 90 meters are computed from the commanded lane. These points are then used with the previous points to generate a function connecting them. Using the commanded speed, the s points are spaced out accordingly, and the respective y value is calculated with the function. Prior to doing this, however, the points are rotated 90 degrees so that there is a clear 1-to-1 relation between x and y points. Instead of recomputing the entire trajectory of 50 points each time, only the number of points covered in this computation cycle is calculated. That is, during a loop of this algorithm, the car may have moved ahead a few points, and so only that number of points are added to the queue.  

The computed points are pushed onto the trajectory vector, and the process repeats.
