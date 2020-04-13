# ROS integration example
## Prerequisites
This example has updated to compatible with ROS melodic and Gazebo9. If you still using ROS Indogo and Gazebo2, you have to reset the commit back to https://github.com/meco-group/omg-tools/commit/beb4d3c78d2fb269671fde783b234d727b9c1fa1, because there are differences in xacro programming style between two versions. Please also check the previous README for usage if you reset the commit.


Make sure you have the ROS packages *ros-control* and *ros-controllers*:
```
$ sudo apt install ros-melodic-ros-control ros-melodic-ros-controllers
```
For motion planner algorithm, recommended linear solver library is 'ma57', but this library is not available through 'apt' or 'pip', you have to compile it yourself, Please check https://github.com/casadi/casadi/wiki/Obtaining-HSL if you are interest. To check whether 'ma57' is available in your computer, you can run:
```
python /examples/p2p_agv.py
```
to see if it complains about the 'ma57' library missing.


The substitution of 'ma57' is 'mumps', this changes already adressed here. 'mumps' is available through 'apt'
```
sudo apt install libmumps-dev
```
If you have 'ma57' library in your computer, go to line 77 and 82 in 'motionplanner.py', change the 'mumps' to 'ma57' to make palnner running faster.

## Building the example
In the current folder (the one containing this readme):

```
$ catkin_make
$ source devel/setup.bash
```

## Running the example

```
$ roscore 
```
```
$ roslaunch p3dx_gazebo gazebo.launch 
```
```
$ roslaunch p3dx_motionplanner motionplanner.launch
```
