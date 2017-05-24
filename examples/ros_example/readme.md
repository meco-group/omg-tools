# ROS integration example
## Prerequisites
This example assumes that you have a working ROS and Gazebo installed. While creating this example we used ROS Indigo and Gazebo 2. Make sure you have the ROS packages *ros-control* and *ros-controllers*:

```
$ sudo apt-get install ros-indigo-ros-control ros-indigo-ros-controllers
```

## Building the example
In the current folder (the one containing this readme):

```
$ catkin_make
$ source devel/setup.bash
```

## Running the example

```
$ roscore &
$ roslaunch p3dx_gazebo gazebo.launch &
$ roslaunch p3dx_motionplanner motionplanner.launch
```
