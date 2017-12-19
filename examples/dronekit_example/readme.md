# Dronekit integration example
This example shows how omg-tools is coupled with [dronekit](http://python.dronekit.io) for steering a quadcopter. The example uses the Software-In-The-Loop (SITL) library to simulate the quadrotor.

## Prerequisites
In order to run the example succesfully, the following packages need to be installed:
```
$ sudo pip install dronekit
$ sudo pip install dronekit-sitl
$ sudo pip install MAVProxy
```
## Running the example

```
$ dronekit-sitl copter &
$ mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551 --streamrate=-1 &
$ python controller.py
```

## Note
quad3d is not workin well...
holonomic does work.s
