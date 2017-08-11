# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The Simulator will provides the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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

## Description

The Project solution consists out of 4 parts:

.	Process chain
.	Vehicle Model for Behavour Planning
.	Path Prediction and Generation
.	Jerk Minimizing Trajectory Generation

#Process chain
the process chain consists out of these steps:

Initially:
*	load map point database
*	configure vehicle model  and path planer by providing the values for maximum speed, accelaration, yerk, preferred distance buffers, number of points in a planned path and the time step in seconds.

each 10th update cycle plan the car behaviour:
*	update the own car dynamic values.
*	update the state of the car model by estimating the possible state having the lowest cost
*	realize the state anc calulate end values.

each update cycle generate the path:
*	receive the sensor fusion data and the previous point list of not yet reached path points
*	generate a way point point list for the way ahead starting on the current lane and ending on the may be changed future lane
*	generate smoothed point list that remains in the speed, acceleration and yerk limits and realizes the car model end state
*	send the point list to the simulator

#Vehicle Model for Behavour Planning
The Vehicle Model for Behavour Planning is a state machine similar to the Quiz Behavour Planning of the Udacity course.

I use the states Keep Lane(KL), Prepare Left Lane Change(PLCL), Prepare Right Lane Change(PLCR, Change Left Lange(LCL), Change Right Lane(PCR).

The possible state changes are straight forward:


KL -> PLCL if car is not on the left lane
KL -> PLCR if car is not on the right lane
PLCL->LCL 
PLCL->LCR
ALL->KL

PLCL and PLCR do not have a specific behaviour in change speed or direction. The states are just used to have a precheck for a change of the lane, I prepared them for
later enhancements f.ex. adapt speed to the traffic speed on the lane.

#update car dynamic values
the model receives values s, d, v, a as stat values. the lane is calculated using d.

#update the state of the car model. 
the possible states are retrieved.
This requires the calculation of the speed of cars in front and behind and the distance of closest cars in front and behind.
Additionally the minimum or maximum acceleration on the lane are calculated.

The choice of the speed on a lane follows  some rules to ensure the safety of the car:

*	the maximum speed is the speed limit
*	it will be reduced to the speed of the car in front, if the preferred buffer is close to be violated.
*	it will be further reduced if the preferred buffer is violated to reestablished this distance to the car in front
*	for a left or right lane the speed is 0 if preffered buffers in fron or behind would be violated. This will block a lane change in the further process.

acceleration is calculated to ensure the required speed adaptations in one time step (configured by one second)

Then for all possible states, I estimate a cost value. The cost value is increased by:
*	own maximum possible speed less than the than the target speed
*	lack of space to the closest car in front
*	lack of space to the closest car behind (not for KL)

I chosed the weight of the cost functions to ensure that:
*	the car keeps KL if the maximum possible speed left and right is the same or less as on the current lane.
*	the car  changes the Lane, if the speed left or right is is larger and the space in front and behind is larger than a preferred buffer.

The result of the update state method is the estimation of the best state selection the lowest cost.

#realize the state
the chosen state is set and the resulting acceleration, speed, s, d and lane values are update using a simple prediction function for one time step.

#generate a way point list
using the received previous point list the lastz point in the list and the previous are retrieved. The delta of x,y and yaw are calculated. 

I choosed to subsribe the point list and not to create a complete new point list in an update cycle.

This is one of two main design design descisions in the project. After developing and testing the full list creating, I recognized that the car path gets jumps from the old to the new path, which can not be smoothe dproperly to 100%
I always received here and there small breaks of the continuty which violated the max acceleration and jerk constraints.
I had to give up this approach which costed me days of work.

The second descision has been the choice of the coordinate system to create and subssribe the path.
I recognized that the frenet coordinate system adds jumps to the coordinates in larger distances, which can be smoothed but lklead to lane violations, that not to 100% can be controlled with out a
polynominal model solution of the curves. I did not see an benefit in this soficticated approach and decided therefore to switch at that point to the car position coordinate system for extrapolations.

The current car position(0,0) is the reference point for all other points in the path generation. I used my MPC Project transformation functions to convert between car and map positions.

To represent the future path, i choose 3 points having 30m seperation, as this is the regular distance between each waypoint on the map. In case of a lane change, the way point selected have a 60 m distance to ensure a smooth path planning.

These 5 points are used to build and subscribe the path for the simulator.

A spline function is used for interpolation.

In case of a larger speed change jerk minimum polynom is calculated to find a smooth acceleration. This avoid braking the dynamic limits of the car.

The pictures show the speed, and acceleration diagram for a typical case.

 list for the way ahead starting on the current lane and ending on the may be changed future lane
*	generate smoothed point list that remains in the speed, acceleration and yerk limits and realizes the car model end state
*	send the point list to the simulator
.	
