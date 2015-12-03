# Add the toolbox to your python path and import it! In the future we should
# provide an install script to automate this...
import sys
sys.path.append("/home/ruben/Documents/Work/Programs/motionplanningtoolbox/")
from motionplanning import *

# Before we start let's start with some notes on the so called OptiLayer!
# The OptiLayer is a general class for which classes Vehicle, Problem and
# Environment inherit. It provides the functionality to define specific
# variables, parameters, constraints and part of the objective on an object.
# For example a Vehicle object will define its input constraints, while
# a Point2point Problem object will define initial/terminal state constraints.
# The OptiLayer puts everything together in order to retrieve one problem.
# It is also possible to define a 'symbol' on an object. Another object should
# provide a variable or parameter with the same name.
# Example: a problem object defines T (the total motion time) as _variable_ (in
# order to eg. minimize it). A vehicle object also needs this T (for computing
# derivatives). It can access it by defining a _symbol_ with name T.

# By using the OptiLayer, (advanced) users can define eg. vehicle types in a
# more high level way, by inheritting from Vehicle and use the functionality
# from OptiLayer. This is done eg. in the Quadrotor and Holonomic class.
# Also a problem is defined in more easy way. Take for example a look at the
# Poin2point class, which derives from the more general Problem class.

# Ok, let's start with creating a vehicle: a holonomic one!
vehicle = Holonomic()
# On a vehicle one can define some signals. These value of these are stored
# during simulation and can be plotted.
# In the Holonomic class default signals 'input', 'state' and 'position' are
# defined. You can also define your own signal. This should be defined as a
# function of the symbol 'y'. This symbol represent the splines (and their
# their derivatives). It is in fact a matrix of symbols: 2 rows for 2 splines,
# and the columns represent all the derivatives (up to vehicle.order).
y = vehicle.get_signal('y')
vehicle.define_signal('my_signal', y[0, 0]+y[1, 0])  # the sum of both splines
# We provide our vehicle with a desired initial and terminal pose:
vehicle.set_initial_pose([-1.5, -1.5])
vehicle.set_terminal_pose([2., 2.])
# We are also changing some (default) options. Options are stored by the
# attribute vehicle.options.
# At start: continuity up to degree = 1
vehicle.options['boundary_smoothness']['initial'] = 1
# Collision avoidance where we try to keep a distance 0.1m (in a soft way),
# but distance 0m in a hard way.
vehicle.set_options({'safety_distance': 0.1})
# For simulation, we can add some input disturbance, which is Gaussian noise
# with some mean (default 0) and stdev and which if filtered with some
# cut-off frequency fc:
#   vehicle.set_input_disturbance(fc = 0.01, stdev = 0.05*np.ones(2))
# Also we can simulate our system with an extra 1st order delay (model-plant
# mismatch):
#   vehicle.set_options({'1storder_delay': True, 'time_constant': 0.1})

# Now, we create an environment
# An environment is determined by a room with certain shape and we put the
# vehicle(s) inside the environment
environment = Environment(room={'shape': Square(2.5)}, vehicles=vehicle)
# Also we can add some obstacles
# Let's first define a recangular shape
rectangle = Rectangle(width=3., height=0.2)
# We create 2 obstacles with this shape and with certain initial position
# (we could also provide an initial velocity or acceleration)
environment.add_obstacle(Obstacle({'position': [-2.1, -0.5]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [1.7, -0.5]}, shape=rectangle))
# Let's also add a circular obstacle which will suddenly start to move.
# Therefore we define an 'increment' trajectory in velocity:
trajectory = {'velocity': np.vstack([[3., -0.15, 0.0], [4., 0., 0.15]])}
# This means: at t = 3s it will add velocity [-0.15, 0] to its current velocity
#             at t = 4s it will add velocity [0,  0.15] to its current velocity
# You could also change position and acceleration in discrete steps
environment.add_obstacle(Obstacle({'position': [1.5, 0.5]}, shape=Circle(0.4),
                                  trajectory=trajectory))

# Create a point-to-point problem
# We provide it with some options concerning the C code generation:
#   codegen:   True -> we use code generation
#   compileme: True -> we recompile the generated code
#   buildname: The name of c files (under directory .build)
# There arer other options, set on a default value. Check them out with
# problem.options
codegen = {'compileme': False, 'codegen': True, 'buildname': 'holonomic'}
problem = Point2point(environment, options={'codegen': codegen})

# Create simulator
# This allows to simulate a motion planning algorithm and allows to plot
# relevant signals.
simulator = Simulator(problem)
simulator.plot.set_options({'knots': True})  # let's plot our spline knots
# If you _create_ a plot with certain signal name, it will plot its progress
# during simulating.
simulator.plot.create('input', label=['Thrust force (N/kg)',
                                      'Pitch rate (rad/s)'])
# You can also provide the name '2d': this plots a 2d space with the vehicles
# movement.
simulator.plot.create('2d')

# Run the simulator!
simulator.run()

# Show 2d plot at some time (no time argument means 'at the end')
simulator.plot.show('2d', time=2.)
# or show (defined) signal
simulator.plot.show('input')
simulator.plot.show('my_signal')
# Show movie of some signal
simulator.plot.show_movie('2d', repeat=False)
# Save a plot as Tikz
simulator.plot.save('my_signal', name='MySignals')
# Save a movie as multiple Tikz
simulator.plot.save_movie('input', number_of_frames=4)
simulator.plot.save_movie('2d', number_of_frames=4, name='quadrotor_2d')

matplotlib.pyplot.show(block=True)
