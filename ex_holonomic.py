from motionplanning.point2point import *
from motionplanning.holonomic import *


codegen = {'compileme': True, 'codegen': True, 'buildname': 'holonomic'}
options = {'codegen': codegen, 'col_safety':True} # col_safety on True -> use collision safety band
time    = {'horizon_time': 10., 'update_time': 0.1, 'sample_time': 0.01, 'knot_intervals': 10}

# Create environment
environment = Environment(room = {'position': [0.,0.], 'shape': Square(2.5)})
rectangle   = Rectangle(width = 3., height = 0.2)
environment.addObstacle(Obstacle({'position':[-2.1, -0.5]}, shape = rectangle))
environment.addObstacle(Obstacle({'position':[1.7, -0.5]}, shape = rectangle))
environment.addObstacle(Obstacle({'position':[1.5, 0.5]}, shape = Circle(0.4), trajectory = {'velocity': np.vstack([[3.,-0.15, 0.0], [4.,0.,0.15]])}))

# Create vehicle
vehicle = Holonomic()
vehicle.setInitialPosition([-1.5,-1.5])
vehicle.setTerminalPosition([2.,2.])

# Create problem
problem = Point2point(vehicle, environment, time, options)
problem.plot.setOptions({'knots': True})
problem.plot.create(['input','2D'])

# Run it!
problem.run()

# Show/save some results
# problem.plot.showMovie(['2D','input'], number_of_frames = 100, repeat = True)
# problem.plot.saveMovie('input', 5, 'holonomic')
# problem.plot.show(['2D','input'])
# problem.plot.save('input','holonomic')

plt.show(block=True)
