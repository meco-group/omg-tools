import os, sys
sys.path.insert(0, os.getcwd()+'/..')

from omgtools import *

start = [2,2]
goal = [8,8]
width = 10
height = 10
square_size = 1

environment = Environment(room={'shape': Rectangle(width=10, height=10), 'position':[5,5]})
environment.add_obstacle(Obstacle({'position': [5, 5]}, shape=Rectangle(width=2,height=2)))

planner = AStarPlanner(environment, cell_size=square_size, start=start, goal=goal)
waypoints = planner.get_path()
print waypoints