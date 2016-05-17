# This file is part of OMG-tools.
#
# OMG-tools -- Optimal Motion Generation-tools
# Copyright (C) 2016 Ruben Van Parys & Tim Mercy, KU Leuven.
# All rights reserved.
#
# OMG-tools is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA

from omgtools import *

"""
Before we start, first some notes on the so-called OptiLayer:

optilayer.py provides 2 classes: OptiFather & OptiChild.
OptiChild is a general class for which classes Vehicle, Problem,
Environment and Obstacle inherit from. It provides basic functionality for
defining variables, parameters, constraints and part of the objective on an object.
For example a Vehicle object will define its input constraints, while
a Problem object will define constraints specific to the problem type.
When initializing a problem object (using init()), it creates an OptiFather
object that assembles everything in order to retrieve one problem with one big set
of variables, parameters, constraints and with one objective.

It is also possible to define a 'symbol' on an object. This is useful to
access variables or parameters which are defined in another object. The
symbol should have the same name as the corresponding variable or parameter.
Example: a problem object defines T (the total motion time) as 'variable' (in
order to eg. minimize it). A vehicle object also needs this T (for computing
derivatives of its splines). It can access it by defining a 'symbol'
with name T.

By using the OptiLayer, (advanced) users can define eg. vehicle types in a
more high level way, by inheritting from Vehicle and use the functionality
from OptiChild. This is done eg. in the Quadrotor and Holonomic class.
Also a problem is defined in more easy way. Take for example a look at the
Poin2point class.
"""

# Ok, let's start with the creation of a vehicle: a holonomic one!
# First we define some options
options = {}
# Select the norm on which velocity and acceleration are constrained.
options['syslimit'] = 'norm_inf'
# When avoiding collisions with the environment we can define a safety distance.
# The vehicle will try to keep a distance of 0.1m from an obstacle (in a soft way).
# Collision avoidance is imposed in a hard way.
options['safety_distance'] = 0.1
# For simulation, we can add some input disturbance, which is Gaussian noise
# with some mean (default 0) and stdev and which if filtered with some
# cut-off frequency fc (Hz):
# options['input_disturbance'] = {'fc': 0.01, 'stdev': 0.05*np.ones(2)}
# Also we can simulate our system with an extra 1st order delay (model-plant
# mismatch):
# options['1storder_delay'] = True
# options['time_constant'] = 0.1

# Create the vehicle instance
vehicle = Holonomic(options=options)

# We provide our vehicle with a desired initial and terminal position:
vehicle.set_initial_conditions([-1.5, -1.5])
vehicle.set_terminal_conditions([2., 2.])

# Now, we create an environment
# An environment is determined by a room with certain shape
environment = Environment(room={'shape': Square(5.)})
# Also we can add some obstacles
# Let's first define a rectangular shape
rectangle = Rectangle(width=3., height=0.2)
# We create 2 obstacles with this shape and with certain initial position
# (we could also provide an initial velocity or acceleration)
environment.add_obstacle(Obstacle({'position': [-2.1, -0.5]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [1.7, -0.5]}, shape=rectangle))
# Let's also add a circular obstacle which will suddenly start to move.
# Therefore we define a trajectory in velocity:
trajectory = {'velocity': {'time': [3., 4.],
                           'values': [[-0.15, 0.0], [0., 0.15]]}}
# Here we defined the time-axis and the corresponding values for velocity.
# Note that these values should be interpreted relatively: eg. at time 3, we
# _add_ an extra velocity of [-0.15, 0.0].
# You could also change position and acceleration in a similar way.

# trajectories are put in a simulation dictionary
simulation = {'trajectories': trajectory}
# here we can give a different simulation model, by providing an A and B matrix
# simulation['model'] = {'A': A, 'B': B}
# The behaviour of an obstacle is expressed by x_dot = A*x + B*u
# The state x is composed of [position, velocity, acceleration]
# The input can be provided via a trajectory (is by default 0)
# simulation['trajectories']['input'] = {'time': time, 'values': values}
# Here the values should be interpreted as absolute.

environment.add_obstacle(Obstacle({'position': [1.5, 0.5]}, shape=Circle(0.4),
                                  simulation=simulation))

# Create a point-to-point problem
options = {}
# We can provide it with some options concerning c code generation + compilation:
options['codegen'] = {'build': None}
# options['codegen'] = {'build': 'jit', 'flags': '-O2'} # just-in-time compilation
# options['codegen'] = {'build': 'shared', 'flags': '-O2'} # compile to shared object
# options['codegen'] = {'build': 'nlp.so'} # provide a shared object
# Compilation of the code takes some time, while execution is slightly faster
# There are other options, set on a default value. Check them out with
# problem.options

# We can also select the type of problem to be solved. There are two options:
# freeT: in this case the goal function of the optimization problem is the
#        motion time, so this formulates a purely time-optimal problem. During
#        movement the time horizon decreases since the remaining motion time
#        decreases.
# fixedT: in this case a certain motion time is proposed and the vehicle is
#         'pulled' towards its goal position. During movement the time horizon
#         stays the same since the proposed motion time stays the same.
problem = Point2point(vehicle, environment, options, freeT=False)
problem.init()

# Create the simulator
# This allows to simulate a motion planning algorithm and allows to plot
# relevant signals while simulating.
simulator = Simulator(problem)
# We plot our input trajectory of our vehicle
vehicle.plot('input', labels=['v_x (m/s)', 'v_y (m/s)'], knots=True)
# And also the 'scene': this plots the vehicle moving in its environment
problem.plot('scene')

# Run the simulator!
simulator.run()

# Show scene plot at some time (no time argument means 'at the end')
# problem.plot('scene', time=2.)
# Show movie (you can make a movie of all possible plot data)
# problem.plot_movie('scene', repeat=False)
# Save a plot as Tikz: you need matplotlib2tikz for this!
# vehicle.save_plot('state', name='state')
# Save a movie as multiple Tikz: you need matplotlib2tikz for this!
# vehicle.save_movie('input', number_of_frames=4)
# problem.save_movie('scene', number_of_frames=4, name='holonomic', axis=False)
