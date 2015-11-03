from motionplanning.point2point import *
from motionplanning.quadrotor import *

codegen = {'compileme': True, 'codegen': True, 'buildname': 'quadrotor'}
options = {'codegen': codegen, 'col_safety': False}
time    = {'horizon_time': 5., 'update_time': 0.1, 'sample_time': 0.01, 'knot_intervals': 10}

# Create environment
environment = Environment(room = {'position': [0.,0.], 'shape': Square(5.)})
environment.addObstacle(Obstacle({'position': [-0.6,4.7]}, shape = Rectangle(width = 0.2, height = 5.)))
environment.addObstacle(Obstacle({'position': [-0.6,-5.4]}, shape = Rectangle(width = 0.2, height = 10.)))

# Create vehicle
vehicle = Quadrotor()
vehicle.setInitialPosition([-4.,-4.])
vehicle.setTerminalPosition([4.,4.])

# Create problem
problem = Point2point(vehicle, environment, time, options)
problem.plot.setOptions({'knots': True})
problem.plot.create(['input','2D'])

# Run it!
problem.run()

# Show/save some results
problem.plot.showMovie(['2D','input'], number_of_frames = 100, repeat = True)
# problem.plot.saveMovie('input', 5, 'holonomic')
# problem.plot.show(['2D','input'])
# problem.plot.save('input','holonomic')

plt.show(block=True)
