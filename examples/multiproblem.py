# getObstaclesOnPath: moet ook de waypoints teruggeven, om een begingok te maken

# Ik krijg niet alle frames tegelijk. 
# Statische en dynamische objecten krijg ik apart. Regelmatig de dynamische objecten opnieuw vragen. 
# Bewegen tot ik een nieuw frame krijg --> check aan het einde van elke loop op een nieuw frame


# Later: 
# maak environment parametrisch

# Use locks

# For now make new problem for new frame

# Steven gives me a new frame, like a service

from omgtools import *

# nFrames = len(frames)
# start = frames[0]['waypoints'][0]
# goal = frames[nFrames-1]['endpointFrame'][0]  # returns a tuple

class splinePlanner():

	def __init__(self, sim):  # in which sim is the simulator class of Steven, so I can call methods on Steven. Idem for the reverse, I give splinePP to Steven.
		self.sim = sim

		# create vehicle
		vehicle = Holonomic(Circle(radius=0.4))
		# vehicle.set_options({'safety_distance': 0.1})
		vehicle.define_knots(knot_intervals=10)
		self.vehicle = vehicle
	
	def doPathPlanning(self, frame):

		self.frame = frame
		while True:
			if self.newFrame == True:  # set by simulator class
				splines, motionTime = self.getInitialGuess(frame)  # gives initial guess for current frame
				xmin,ymin,xmax,ymax = self.frame.limits
				movingObstacles = self.sim.getMovingObstacles(xmin,ymin,xmax,ymax,motionTime)  # returns x,y,w,h,theta, dx,dy,dw,dh,dtheta
				nMoving = len(movingObstacles)
				if movingObstacles is not None:
					#assign extra obstacles to frame. use overrule
				self.buildProblem(splines)
			# iterate				
			else:
				trajectory, motionTime, splines = self.getTrajectory(self.frame, self.pos)	
				# current position = first point of trajectory
				self.sim.update(trajectory, motionTime)  # assign current pos to attribute

				# check for new moving obstacles
				movingObstacles = self.sim.getMovingObstacles(xmin,ymin,xmax,ymax,motionTime)  # returns x,y,w,h,theta, dx,dy,dw,dh,dtheta
				if len(movingObstacles) != nMoving:
					#assign extra obstacles to frame. use overrule
					#build problem again
					self.buildProblem(splines)
					# renew initial guess
					# splines, motionTime = self.getInitialGuess(frame)  # gives initial guess for current frame

		For moving obstacle use overrule_state()
		For moving obstacle use overrule_input()

	def setNewFrame(self, newFrame):  # called by simulator class

		self.newFrame = True
		self.frame = frame

	def getInitialGuess(self, frame):

		splines = {}  # holds all initial guesses of spline variables

		# Initial guess step
		for number, frame in frames.iteritems():
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

		    motionTime = simulator.problem.father.get_variables(simulator.problem, 'T',)[0][0]

		    # Call object tracker for relevant moving obstacles
		    # Give start and end time
		    # obstacles = moving_obstacles(frame['border'], T)
		    movingObstacles = None

			# If changes, update problem
		    if movingObstacles is not None:
		        for obstacle in obstacles:
		            if obstacle['type'] == 'circle':  # [x,y,dx,dy,r]
		                circle = Circle(radius=obstacle['radius'])
		                traj = {'velocity': {'time': [0.], 'values': [obstacle['velocity']]}}
		                environment.add_obstacle(Obstacle({'position': obstacle['position']}, shape=circle,
		                                                    simulation={'trajectories': traj}))
		            elif obstacle['type'] == 'rectangle':  # [x,y,theta,dx,dy,w,h]
		                rectangle = Rectangle(width=obstacle['width'], height= obstacle['height'], orientation= obstacle['angle'])
		                traj = {'velocity': {'time': [0.], 'values': [obstacle['velocity']]}}
		                environment.add_obstacle(Obstacle({'position': obstacle['position']}, shape=rectangle,
		                                                    simulation={'trajectories': traj}))
		            else:
		                raise RuntimeError('Obstacle received which was not a circle or a rectangle')  
		            # Add moving obstacles to original set of obstacles
		            frame['obstacles'].append(obstacle) 
		        # Make new, complete, problem
		        problem = Point2point(vehicle, environment, freeT=True)
		        problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
		        problem.init()
		        simulator = Simulator(problem)
		        # problem.plot('scene')
		        # vehicle.plot('input', knots=True, labels=['v_x (m/s)', 'v_y (m/s)'])
		        simulator.run_once()
		    # else: no extra obstacles, go on

		    # Save trajectories for all frames, as initial guesses
		    splines[number] = problem.father.get_variables()[vehicle.label, 'splines0']

		    return splines, motionTime

		def buildProblem(self, frame):

			# Iteration step
			curr_pos = frames[0]['waypoints'][0]
			for number, frame in frames.iteritems():
			    
			    # ask for new moving obstacles here, but not in each iteration. 
			    # check if I got a new frame: based on the distance to the goal within the frame, after each iteration
			    # make a class of this script

			    # Get initial guess from first step
			    # todo: how good is this? since starting point of frame 2!= waypoint 0 of frame 2 evt.
			    self.vehicle.set_init_spline_value(splines[number])

			    self.vehicle.set_initial_conditions(curr_pos)
			    self.vehicle.set_terminal_conditions(frame['waypoints'][-1])

			    # create environment
			    border = verticesToRectangle(frame['border'])
			    environment = Environment(room={'shape': Rectangle(width=border['width'], height=border['height']),
			                                    'position': border['position'], 'orientation': border['angle']})

			    obstacles = []
			    for obstacle in frame['obstacles']:
			        if obstacle['type'] == 'circle':  # [x,y,dx,dy,r]
			            circle = Circle(radius=obstacle['radius'])
			            traj = {'velocity': {'time': [0.], 'values': [obstacle['velocity']]}}
			            obstacles.append(Obstacle({'position': obstacle['position']}, shape=circle,
			                                                simulation={'trajectories': traj}))
			        elif obstacle['type'] == 'rectangle':  # [x,y,theta,dx,dy,w,h]
			            rectangle = verticesToRectangle(obstacle['position'])
			            rect = Rectangle(width=rectangle['width'], height= rectangle['height'], orientation= rectangle['angle'])
			            traj = {'velocity': {'time': [0.], 'values': [rectangle['velocity']]}}
			            obstacles.append(Obstacle({'position': rectangle['position']}, shape=rect,
			                                                simulation={'trajectories': traj}))
			        else:
			            raise RuntimeError('Obstacle received which was not a circle or a rectangle')  
			    environment.add_obstacle(obstacles)

			    problem = Point2point(vehicle, environment, freeT=True)
			    problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
			    problem.init()
			    simulator = Simulator(problem)

			    border_tot = frames[0]['border'][:2] + frames[len(frames)-1]['border'][2:]
			    total_border = verticesToRectangle(border_tot)
			    total_environment = Environment(room={'shape': Rectangle(width=total_border['width'], height=total_border['height']),
			                                    'position': total_border['position'], 'orientation': total_border['angle']})    

			    problem.plot('scene')
			    vehicle.plot('input', knots=True, labels=['v_x (m/s)', 'v_y (m/s)'])

			    try:
			        region = frames[number+1]['border']
			    except:  # last frame reached, region = goal state
			        region = (frames[number]['waypoints'][-1] + frames[number]['waypoints'][-1])

			    simulator.set_problem(problem)
			    vehicle.set_terminal_conditions(frame['waypoints'][-1])
			    vehicle.reinit_splines(problem)

			    # curr_pos = simulator.run_iterative(region)  # run until vehicle reached region


		def getTrajectory(self, frame, pos):

			curr_pos = simulator.run_iterative(region)  # run until vehicle reached region
