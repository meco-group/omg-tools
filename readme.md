[![Build Status](https://travis-ci.org/meco-group/omg-tools.svg?branch=master)](https://travis-ci.org/meco-group/omg-tools)

# OMG-tools
Optimal Motion Generation-tools is a software toolbox that facilitates the modeling, simulation and embedding of motion planning problems. Its main goal is to bring several research topics concerning (spline-based) motion planning together into a user-friendly package in order to enlarge its visibility towards the scientific and industrial world.

Motion planning approaches implemented in OMG-tools are described in the following publications:
* T. Mercy, W. Van Loock and G. Pipeleers. "Real-time motion planning in the presence of moving obstacles", accepted for the European Control Conference (ECC), 2016. ([pdf](https://lirias.kuleuven.be/bitstream/123456789/538718/1/TimMercy_2016_ECC.pdf))
* R. Van Parys and G. Pipeleers. "Online distributed motion-planning for multi-vehicle systems", accepted for the European Control Conference (ECC), 2016. ([pdf](https://lirias.kuleuven.be/bitstream/123456789/526758/3/RubenVanParys_2016_ECC.pdf))

If these approaches help you with your research, please cite us!

Any questions can be addressed to the developers: ruben[dot]vanparys[at]kuleuven[dot]be and tim[dot]mercy[at]kuleuven[dot]be.

## Installation
OMG-tools is written in Python 2.7 and depends on the packages numpy, scipy and matplotlib:

`sudo apt-get install python-numpy python-scipy python-matplotlib`

It uses [CasADi](http://casadi.org) as a framework for symbolic computations and interface to IPOPT, a software package for large-scale nonlinear optimization. For installation instructions regarding these software packages, the user is referred to the [CasADi installation page](http://install.casadi.org). The current implementation of this toolbox relies on CasADi 3.0. In the examples, we use the [HSL linear solvers](https://github.com/casadi/casadi/wiki/Obtaining-HSL), as they result in a much faster execution.

With the toolbox it is possible to save your simulation results in Tikz format. This functionality uses the [matplotlib2tikz](https://github.com/nschloe/matplotlib2tikz) script.

To install the toolbox itself, run the following command in the root directory of this repository:

`sudo python setup.py install`

## Examples
### Elementary example
The code example below illustrates the basic functionality of the toolbox for steering a holonomic vehicle from an initial to terminal pose in an obstructed environment.

```python
from omgtools import *

# make and set-up vehicle
vehicle = Holonomic()
vehicle.set_initial_conditions([-1.5, -1.5])
vehicle.set_terminal_conditions([2., 2.])
vehicle.set_options({'safety_distance': 0.1})

# make and set-up environment
environment = Environment(room={'shape': Square(5.)})

# add stationary obstacles to environment
rectangle = Rectangle(width=3., height=0.2)
environment.add_obstacle(Obstacle({'position': [-2.1, -0.5]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [ 1.7, -0.5]}, shape=rectangle))

# generate trajectory for moving obstacle
traj = {'velocity': {'time': [3., 4.],
                     'values': [[-0.15, 0.0], [0., 0.15]]}}
# add moving obstacle to environment
environment.add_obstacle(Obstacle({'position': [1.5, 0.5]}, shape=Circle(0.4),simulation={'trajectories': traj}))

# give problem settings and create problem
problem = Point2point(vehicle, environment)
problem.init()

# simulate, plot some signals and save a movie
simulator = Simulator(problem)
simulator.plot.show('input', label=['x-velocity(m/s)', 'y-velocity(m/s)'])
simulator.plot.show('scene')
simulator.run()
simulator.plot.save_movie('scene')
```

### More examples
Check out the examples directory for more advanced code examples. There you can find a simple tutorial example which provides a documented overview of the basic functionality of the toolbox. Also examples illustrating distributed motion planning approaches for multi-vehicle systems are available.
