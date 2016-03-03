# Spline Based Motion Planning Toolbox
We should think about a more attractive name...
Something with move? plan? 
Should spline be in there?

This software package provides tools that make it easy to model, simulate and embed motion planning problems. The main goal is to bring several research topics and several techniques concerning (spline-based) motion planning together into a user-friendly package.
Any questions can be addressed to the developers: <a href=”mailto:ruben.vanparys&#64;kuleuven.be”>Ruben Van Parys</a>


## Installation
The software is developed in Python and uses [CasADi](https://github.com/casadi/casadi/wiki) as a framework for symbolic computations. Furthermore CasADi provides an interface to IPOPT, a software package for large-scale nonlinear optimization. For installation instructions regarding these software packages, the user is referred to the [CasADi homepage](https://github.com/casadi/casadi/wiki), and more in particular the installation instructions page.
With the toolbox it is possible to save your simulation results in Tikz format. This functionality uses the following script: [matplotlib2tikz](https://github.com/nschloe/matplotlib2tikz).
To install the toolbox itself, run the following command in the root directory of this repository: `sudo python setup.py install`

## How to start?
The best way to get started with the toolbox is by studying a simple example. Below you find the code for the most elementary example. If you want to know more, check the examples directory. Here you find a simple tutorial example which provides a documented overview of the basic functionality of the toolbox, together with some more advanced examples.

## Elementary example

```python
from motionplanning import *

#Make and set-up vehicle
vehicle = Holonomic()
vehicle.set_initial_pose([-1.5, -1.5])
vehicle.set_terminal_pose([2., 2.])
vehicle.set_options({'safety_distance': 0.1})

#Make and set-up environment
environment = Environment(room={'shape': Square(5.)})

#Add stationary obstacles to environment
rectangle = Rectangle(width=3., height=0.2)
environment.add_obstacle(Obstacle({'pos': [-2.1, -0.5]}, shape=rectangle))
environment.add_obstacle(Obstacle({'pos': [ 1.7, -0.5]}, shape=rectangle))

#Generate trajectory for moving obstacle
traj = {'velocity': ([[3., -0.15, 0.0], [4., 0., 0.15]])}
#Add moving obstacle to environment
environment.add_obstacle(Obstacle({'pos': [1.5, 0.5]}, shape=Circle(0.4),trajectory=traj))

#Give problem settings and create problem
problem = Point2point(vehicle, environment)
problem.init()

#Simulate, plot some signals and save a movie
simulator = Simulator(problem)
simulator.plot.create('input', label=['x-velocity(m/s)', 'y-velocity(m/s)'])
simulator.plot.create('2d')
simulator.run()
simulator.plot.save_movie('2d')
```