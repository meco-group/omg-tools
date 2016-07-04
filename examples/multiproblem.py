# getObstaclesOnPath: moet ook de waypoints teruggeven, om een begingok te maken

# Ik krijg niet alle frames tegelijk. 
# Statische en dynamische objecten krijg ik apart. Regelmatig de dynamische objecten opnieuw vragen. 
# Bewegen tot ik een nieuw frame krijg --> check aan het einde van elke loop op een nieuw frame


# Todo: 
# maak environment parametrisch, dan hoef je maar een probleem te maken. Voorlopig maak ik voor elk frame een nieuw probleem
# Use locks
# Should getInitialguess only make the guess without moving obstacles or also check for moving obstacles? --> only one task per function!
# How good is initial guess? since starting point of frame 2!= waypoint 0 of frame 2 evt.
# expand verticesToRectangle such that it also sets the velocity

# assign splines to problem = init guess

from omgtools import *

class splinePlanner():

	def __init__(self, sim, key):
	# In which sim is the simulator class of Steven, so I can call methods on this simulator.
	# Idem for the reverse, I give splinePathplanner to Steven.
		self.sim = sim
		self.key = key

		# create vehicle
		vehicle = Holonomic(Circle(radius=0.4))
		# vehicle.set_options({'safety_distance': 0.1})
		vehicle.define_knots(knot_intervals=10)
		self.vehicle = vehicle
	
	def doPathPlanning(self, frame):  # main function, called by simulator

		self.frame = frame  # save frame
		self.frame.movingObstacles = None  # initialize moving obstacles
		while self.frame is not None:  # there is still a frame left in the multiproblem
			if self.newFrame == True:  # set by simulator class
				self.vehState = self.sim.getRobotState()  # [x,y,theta,vx,vy,omega]
				# if there is a new frame, make an initial guess and get movingObstacles
				splines, motionTime, problem = self.getInitialGuess(frame, state)  # gives initial guess for current frame
				self.problem = problem
				border = self.frame['border']
				xmin, ymin, xmax, ymax = border[0], border[1], border[2], border[3]
				movingObstacles = self.sim.getMovingObstacles(xmin, ymin, xmax, ymax, motionTime)
				if movingObtacles is not None:
					self.updateFrame(movingObstacles)  # save current set of moving obstacles
					splines, motionTime, problem = self.getInitialGuess(frame, state)
				vehicle.reinit_splines(problem)  # we solve a new problem
				# self.problem = self.buildProblem(splines)
			# standard iteration
			else:
				# check for new/update moving obstacles in each step
				movingObstacles = self.sim.getMovingObstacles(xmin, ymin, xmax, ymax, motionTime)  # returns x,y,w,h,theta, dx,dy,dw,dh,dtheta
				if len(movingObstacles) != len(self.frame.movingObstacles):  # amount changed, build new problem
					# add new set of moving obstacles to frame
					self.updateFrame(movingObstacles)
					# renew initial guess
					splines, motionTime, problem = self.getInitialGuess(frame, state)  # gives initial guess for current frame
					vehicle.reinit_splines(problem)  # we solve a new problem
					self.problem = problem
					#build problem again
					# self.problem = self.buildProblem(splines)
				elif movingObstacles != self.frame.movingObstacles:  # same mount, different position/velocity/...
					# todo: seems to work for a dict, correct?
					self.updateFrame(movingObstacles)
					#update problem obstacle info. Use overrule_state behind the scenes
					self.problem.update_environment(frame)
			 	# find new trajectory
				# make deployer
				deployer = Deployer(self.problem, update_time = 0.1)
				# update vehicle state
				self.vehState = self.sim.getRobotState()  # [x,y,theta,vx,vy,omega]
				obsState = self.frame['obstacles']
				goalState = self.frame['endpointFrame']
				# run deployer	
				feasible, traj_state, traj_input, motionTime = deployer.run(self.vehState, obsState, goalState)
				
				trajectory = buildTrajectory(traj_state, traj_input)
				
				# current position = first point of trajectory
				# trajectory = [array(x), array(y), array(theta), array(vx), array(vy), array(omega)]
				self.sim.update(trajectory, motionTime, self.key)  # assign current pos to attribute

	def buildTrajectory(traj_state, traj_input):
		trajectory = []
		trajectory.append(np.array(traj_state[0]))
		trajectory.append(np.array(traj_state[1]))
		trajectory.append(np.array(traj_input[0]))
		trajectory.append(np.array(traj_input[1]))
		return trajectory

	def setNewFrame(self, newFrame):  # called by simulator class
		self.newFrame = True
		self.frame = frame

	def updateFrame(self, frame, movingObstacles):  # update frame with new obstacle set
		self.frame.movingObstacles = movingObstacles

	def getInitialGuess(self, frame):
		splines = {}  # holds all initial guesses of spline variables

		# Use waypoints for prediction
	    x, y = [], []
	    for waypoint in frame['waypoints']:
	        x.append(waypoint[0])
	        y.append(waypoint[1])
	    # Calculate total length in x- and y-direction
	    l_x, l_y = 0., 0.
	    for i in range(len(frame['waypoints'])-1):
	        waypoints = frame['waypoints']
	        l_x += waypoints[i+1][0] - waypoints[i][0]
	        l_y += waypoints[i+1][1] - waypoints[i][1]
	    # Calculate distance in x and y between each 2 waypoints and use it as a relative measure to build time vector
	    time_x = [0.]
	    time_y = [0.]
	    for i in range(len(frame['waypoints'])-1):
	        waypoints = frame['waypoints']
	        time_x.append(time_x[-1] + float(waypoints[i+1][0] - waypoints[i][0])/l_x)
	        time_y.append(time_y[-1] + float(waypoints[i+1][1] - waypoints[i][1])/l_y)  # gives time 0...1

	    # Make interpolation functions
	    fx = interp1d(time_x, x, kind='linear')  # kind='cubic' requires a minimum of 4 waypoints
	    fy = interp1d(time_y, y, kind='linear')

	    # Evaluate resulting splines to get evaluations at knots = coeffs-guess
	    coeffs_x = fx(vehicle.knots[vehicle.degree-1:-(vehicle.degree-1)])
	    coeffs_y = fy(vehicle.knots[vehicle.degree-1:-(vehicle.degree-1)])
	    init_guess = np.c_[coeffs_x, coeffs_y]
	    # Pass on initial guess
	    vehicle.set_init_spline_value(init_guess)

	    # Set start and goal
	    vehicle.set_initial_conditions([waypoints[0]])
	    vehicle.set_terminal_conditions([waypoints[-1]])

	    # Solve one time
	    border = verticesToRectangle(frame['border'])
	    environment = Environment(room={'shape': Rectangle(width=border['width'], height=border['height']),
	                                    'position': border['position'], 'orientation': border['angle'] })

	    obstacles = []
	    for obstacle in frame['obstacles']:
	        if obstacle['type'] == 'circle':  # [x,y,dx,dy,r]
	            circle = Circle(radius=obstacle['radius'])
	            traj = {'velocity': {'time': [0.], 'values': [obstacle['velocity']]}}
	            obstacles.append(Obstacle({'position': obstacle['position']}, shape=circle,
	                                                simulation={'trajectories': traj}))
	        elif obstacle['type'] == 'rectangle':  # [x,y,theta,dx,dy,w,h]
	            rectangle = verticesToRectangle(obstacle['position'])
	            rect = Rectangle(width=rectangle['width'], height= rectangle['height'], orientation= obstacle['angle'])
	            traj = {'velocity': {'time': [0.], 'values': [obstacle['velocity']]}}
	            obstacles.append(Obstacle({'position': rectangle['position']}, shape=rect,
	                                                simulation={'trajectories': traj}))
	        else:
	            raise RuntimeError('Obstacle received which was not a circle or a rectangle')  
	    environment.add_obstacle(obstacles)

	    problem = Point2point(vehicle, environment, freeT=True)
	    problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
	    problem.init()
	    simulator = Simulator(problem)
	    # problem.plot('scene')
	    # vehicle.plot('input', knots=True, labels=['v_x (m/s)', 'v_y (m/s)'])

	    simulator.run_once()

	    # Retreive motion time
	    motionTime = simulator.problem.father.get_variables(simulator.problem, 'T',)[0][0]
	    # Save trajectories for all frames, as initial guesses
	    splines[number] = problem.father.get_variables()[vehicle.label, 'splines0']

	    return splines, motionTime, problem
    
    #
	#Todo: will there be a lot of changes over the different iteration? If so use deployer, otherwise use getTrajectory
	#
	
	# use deployer here

	# def getTrajectory(self, frame, vehState, problem, splines):

	# 	# assign current state to problem
	# 	self.vehicle._overrule_state(vehState['state'])
 #        self.vehicle._overrule_input(vehState['input'])
	# 	# assign splines to problem

	# 	simulator = Simulator(problem)

	# 	For moving obstacle use overrule_state()

	# 	curr_pos = simulator.run_iterative(frame)  # run until vehicle reached region

	# 	# trajectory = [array(x), array(y), array(theta), array(vx), array(vy), array(omega)]

	# 	return trajectory, motionTime, splines

	# # todo: not necessary?
	# def buildProblem(self, frame):
	#     self.vehicle.set_init_spline_value(splines[number])

	#     self.vehicle.set_initial_conditions(curr_pos)
	#     self.vehicle.set_terminal_conditions(frame['waypoints'][-1])

	#     # create environment
	#     border = verticesToRectangle(frame['border'])
	#     environment = Environment(room={'shape': Rectangle(width=border['width'], height=border['height']),
	#                                     'position': border['position'], 'orientation': border['angle']})

	#     obstacles = []
	#     for obstacle in frame['obstacles']:
	#         if obstacle['type'] == 'circle':  # [x,y,dx,dy,r]
	#             circle = Circle(radius=obstacle['radius'])
	#             traj = {'velocity': {'time': [0.], 'values': [obstacle['velocity']]}}
	#             obstacles.append(Obstacle({'position': obstacle['position']}, shape=circle,
	#                                                 simulation={'trajectories': traj}))
	#         elif obstacle['type'] == 'rectangle':  # [x,y,theta,dx,dy,w,h]
	#             rectangle = verticesToRectangle(obstacle['position'])
	#             rect = Rectangle(width=rectangle['width'], height= rectangle['height'], orientation= rectangle['angle'])
	#             traj = {'velocity': {'time': [0.], 'values': [rectangle['velocity']]}}
	#             obstacles.append(Obstacle({'position': rectangle['position']}, shape=rect,
	#                                                 simulation={'trajectories': traj}))
	#         else:
	#             raise RuntimeError('Obstacle received which was not a circle or a rectangle')  
	#     environment.add_obstacle(obstacles)

	#     problem = Point2point(vehicle, environment, freeT=True)
	#     problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
	#     problem.init()
	#     simulator = Simulator(problem)

	#     border_tot = frames[0]['border'][:2] + frames[len(frames)-1]['border'][2:]
	#     total_border = verticesToRectangle(border_tot)
	#     total_environment = Environment(room={'shape': Rectangle(width=total_border['width'], height=total_border['height']),
	#                                     'position': total_border['position'], 'orientation': total_border['angle']})    

	#     problem.plot('scene')
	#     vehicle.plot('input', knots=True, labels=['v_x (m/s)', 'v_y (m/s)'])

	#     try:
	#         region = frames[number+1]['border']
	#     except:  # last frame reached, region = goal state
	#         region = (frames[number]['waypoints'][-1] + frames[number]['waypoints'][-1])

	#     simulator.set_problem(problem)
	#     vehicle.set_terminal_conditions(frame['waypoints'][-1])
	#     vehicle.reinit_splines(problem)

