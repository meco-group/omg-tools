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

# With fixed environment
from omgtools import *

# create vehicle
vehicle = Dubins(shapes=Circle(radius=0.3), bounds={'vmax': 0.7, 'wmax': np.pi/3., 'wmin': -np.pi/3.}, # in rad/s
                 options={'substitution': False})
vehicle.define_knots(knot_intervals=10)

# create environment
print 'Using environment for known example'
#for now fix environment to the one for which A*-path is known
start = [2,2,0]
goal = [8,8,0]
vehicle.set_initial_conditions(start)
vehicle.set_terminal_conditions(goal)

envWidth = 10
envHeight = 10
environment = Environment(room={'shape': Rectangle(width=envWidth, height=envHeight), 'position': [5,5], 'draw':True})

rectangle = Rectangle(width=1, height=1)
circle = Circle(radius=0.4)

environment.add_obstacle(Obstacle({'position': [6,2]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [4,2]}, shape=circle))
environment.add_obstacle(Obstacle({'position': [5,6]}, shape=circle))

# make global planner
globalplanner = AStarPlanner(environment, [10,10], start, goal)

# make coordinator
options={'freeT': True, 'horizon_time': 15}
multiproblem=SchedulerProblem(vehicle, environment, globalplanner, options=options, frame_type='min_nobs')
multiproblem.set_options({'solver_options': {'ipopt': {# 'ipopt.linear_solver': 'ma57',
                                                       'ipopt.hessian_approximation': 'limited-memory'}}})
multiproblem.init()

simulator = Simulator(multiproblem)
multiproblem.plot('scene')
vehicle.plot('input', knots=True, prediction=True, labels=['v_x (m/s)', 'v_y (m/s)'])

# run it!
simulator.run()
# multiproblem.save_movie('scene', format='gif', name='multiproblemgif', number_of_frames=100, movie_time=5, axis=False)
# multiproblem.save_movie('scene', format='tikz', name='multiproblemtikz', number_of_frames=100, movie_time=5, axis=False)
