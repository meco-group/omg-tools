from environment import Environment, Obstacle
from ..basics.shape import Rectangle, Circle

import Tkinter as tk
import pickle

class EnvironmentGUI(tk.Frame):
    # Graphical assistant to make an environment
    # width = environment border width
    # height = environment border height
    # position = top left corner point of environment
    def __init__(self, parent, width=8,height=8, position=[0,0], options=None, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)

        self.root = parent
        self.root.title("Environment")
        self.root.resizable(0,0)

        self.meter2pixel = 50  # conversion of meter to pixels

        cell_size = options['cell_size'] if 'cell_size' in options else 0.5

        # make environment frame and size it
        self.frameWidth = width*self.meter2pixel
        self.frameHeight = height*self.meter2pixel
        self.position = [pos * self.meter2pixel for pos in position]  # top left corner point of the frame
        self.canvas = tk.Canvas(self.root, width=self.frameWidth, height=self.frameHeight, borderwidth=0, highlightthickness=0)
        self.canvas.configure(cursor="tcross")
        self.canvas.grid(row=0,columnspan=3)
        self.cellWidth = int(cell_size*self.meter2pixel)  # in pixels, so int is required
        self.cellHeight = int(cell_size*self.meter2pixel)
        self.rows = self.frameWidth/self.cellWidth
        self.columns = self.frameHeight/self.cellHeight

        # draw grid
        self.rect = {}
        for column in range(self.columns):
            for row in range(self.rows):
                x1 = column*self.cellWidth
                y1 = row * self.cellHeight
                x2 = x1 + self.cellWidth
                y2 = y1 + self.cellHeight
                self.rect[row,column] = self.canvas.create_rectangle(x1,y1,x2,y2, fill="white", tags="rect")

        self.obstacles = []  # list to hold all obstacles
        self.clickedPositions = []  # list to hold all clicked positions

        self.canvas.bind("<Button-1>", self.makeObstacle)  # left mouse button makes obstacle
        self.canvas.bind("<Button-3>", self.getPosition)  # right mouse button gives a position

        # Add buttons
        self.readyButton = tk.Button(self.root, text="Ready", fg="green", command=self.buildEnvironment)
        self.readyButton.grid(row=2, column=0)

        self.quitButton = tk.Button(self.root, text="Quit", fg="red", command=self.canvas.quit)
        self.quitButton.grid(row=3, column=0)

        self.removeButton = tk.Button(self.root, text="Remove", fg="black", command=self.remove_last_obstacle)
        self.removeButton.grid(row=4, column=0)

        self.loadButton = tk.Button(self.root, text="Load", fg="black", command=self.load_environment)
        self.loadButton.grid(row=5, column=0)

        self.save_env = tk.BooleanVar(value=False)
        labelSave = tk.Checkbutton(self.root, text="Save [yes/no]", variable=self.save_env)
        labelSave.grid(row=7,column=0)

        # add cellSize button
        self.cellSize = tk.Label(self.root, text='Cell size: ' + str(float(self.cellWidth)/self.meter2pixel) + 'x' + str(float(self.cellHeight)/self.meter2pixel) + ' [m]')
        self.cellSize.grid(row=1, column=1)

        # Clicked position label
        # self.clickedPos = tk.StringVar()
        # labelClickedPos = tk.Label(self, text="Clicked position [x,y]")
        # labelClickedPosEntry = tk.Entry(self, bd =0, textvariable=self.clickedPos)
        # labelClickedPos.pack()
        # labelClickedPosEntry.pack()

        # Add rectangle or circle
        self.shape = tk.StringVar()
        self.circleButton = tk.Radiobutton(self.root, text="Circle", variable=self.shape, value='circle')
        self.circleButton.grid(row=2, column=2)
        self.rectangleButton = tk.Radiobutton(self.root, text="Rectangle", variable=self.shape, value='rectangle')
        self.rectangleButton.grid(row=3,column=2)

        # Add text
        self.width = tk.DoubleVar()
        self.height = tk.DoubleVar()
        self.radius = tk.DoubleVar()

        labelWidth = tk.Label(self.root, text="Width [m]")
        labelWidthEntry = tk.Entry(self.root, bd =0, textvariable=self.width)
        labelWidth.grid(row=2,column=1)
        labelWidthEntry.grid(row=3,column=1)

        labelHeight = tk.Label(self.root, text="Height [m]")
        labelHeightEntry = tk.Entry(self.root, bd =0, textvariable=self.height)
        labelHeight.grid(row=4,column=1)
        labelHeightEntry.grid(row=5,column=1)

        labelRadius = tk.Label(self.root, text="Radius [m]")
        labelRadiusEntry = tk.Entry(self.root, bd =0, textvariable=self.radius)
        labelRadius.grid(row=6,column=1)
        labelRadiusEntry.grid(row=7,column=1)

        self.vel_x = tk.DoubleVar(value=0.0)
        self.vel_y = tk.DoubleVar(value=0.0)

        labelVelx = tk.Label(self.root, text="x-velocity [m/s]")
        labelVelxEntry = tk.Entry(self.root, bd =0, textvariable=self.vel_x)
        labelVelx.grid(row=4,column=2)
        labelVelxEntry.grid(row=5,column=2)

        labelVely = tk.Label(self.root, text="y-velocity [m/s]")
        labelVelyEntry = tk.Entry(self.root, bd =0, textvariable=self.vel_y)
        labelVely.grid(row=6,column=2)
        labelVelyEntry.grid(row=7,column=2)

        self.bounce = tk.BooleanVar(value=False)
        labelBounce = tk.Checkbutton(self.root, text="Bounce [yes/no]", variable=self.bounce)
        labelBounce.grid(row=8,column=2)

    def makeObstacle(self, event):

        obstacle = {}
        clicked_pos = [event.x, event.y]
        snapped_pos = self.snap_to_grid(clicked_pos)
        clickedPixel = [click+pos for click,pos in zip(snapped_pos,self.position)]  # shift click with position of frame
        pos = self.pixel2World(clickedPixel)
        obstacle['pos'] = pos
        if self.shape.get() == '':
            print 'Select a shape before placing an obstacle!'
            # self.obstacles.append()
        elif self.shape.get() == 'rectangle':
            if (self.width.get() <= 0 or self.height.get() <= 0):
                print 'Select a strictly positive width and height for the rectangle first'
            elif (self.width.get()*self.meter2pixel >= self.frameWidth):
                print 'Selected width is too big'
            elif (self.height.get()*self.meter2pixel >= self.frameHeight):
                print 'Selected height is too big'
            else:
                obstacle_var = self.canvas.create_rectangle(snapped_pos[0]-self.width.get()*self.meter2pixel/2., snapped_pos[1]-self.height.get()*self.meter2pixel/2.,
                                             snapped_pos[0]+self.width.get()*self.meter2pixel/2., snapped_pos[1]+self.height.get()*self.meter2pixel/2.,
                                             fill="black")
                obstacle['shape'] = 'rectangle'
                obstacle['width'] = self.width.get()
                obstacle['height'] = self.height.get()
                obstacle['velocity'] = [self.vel_x.get(), self.vel_y.get()]
                obstacle['bounce'] = self.bounce.get()
                obstacle['variable'] = obstacle_var
                self.obstacles.append(obstacle)
                print 'Created obstacle: ', obstacle
        elif self.shape.get() == 'circle':
            if (self.radius.get() <= 0):
                print 'Select a strictly positive radius before making a circle'
            elif (self.radius.get()*self.meter2pixel >= (self.frameWidth or self.frameHeight)):
                print 'Selected radius is too big'
            else:
                obstacle_var = self.canvas.create_circle(snapped_pos[0], snapped_pos[1], self.radius.get()*self.meter2pixel, fill="black")
                obstacle['shape'] = 'circle'
                obstacle['radius'] = self.radius.get()
                obstacle['velocity'] = [self.vel_x.get(), self.vel_y.get()]
                obstacle['bounce'] = self.bounce.get()
                obstacle['variable'] = obstacle_var
                self.obstacles.append(obstacle)
                print 'Created obstacle: ', obstacle
        # return self.obstacles

    def remove_last_obstacle(self):
        # when clicking the remove button, this removes
        # the obstacle which was just added
        # erase from gui
        self.canvas.delete(self.obstacles[-1]['variable'])
        # remove from list
        del self.obstacles[-1]

    def snap_to_grid(self, point):
        # Snap the user clicked point to a grid point, since obstacles can only
        # be placed on grid points
        snapped = [0,0]
        snapped[0] = int(self.cellWidth * round(float(point[0])/self.cellWidth))
        snapped[1] = int(self.cellHeight * round(float(point[1])/self.cellHeight))
        return snapped

    def getPosition(self, event):
        clicked = [event.x,event.y]
        clicked = self.pixel2World(clicked)
        # self.clickedPos.set('['+str(clicked[0])+','+str(clicked[1])+']')
        print 'You clicked on: ', clicked
        self.clickedPositions.append(clicked)
        if (len(self.clickedPositions) > 2):
            self.clickedPositions[1] = self.clickedPositions[2]  # second last = last
            self.clickedPositions=self.clickedPositions[:2]  # remove last
            print 'You clicked more than two times, the last click replaced the previous one'

    def buildEnvironment(self):
        # only build environment and shut down GUI if start and goal positions are clicked
        if len(self.clickedPositions) == 2:
            # make border
            self.environment = Environment(room={'shape': Rectangle(width = self.frameWidth/self.meter2pixel,
                                                                    height = self.frameHeight/self.meter2pixel),
                                                 'position':[(self.position[0]+self.frameWidth/2.)/self.meter2pixel,
                                                             (self.position[1]+self.frameHeight/2.)/self.meter2pixel]})                
            for obstacle in self.obstacles:
                if obstacle['shape'] == 'rectangle':
                    rectangle = Rectangle(width=obstacle['width'],height=obstacle['height'])
                    pos = obstacle['pos']
                    trajectory = {'velocity': {'time': [0.], 'values': [obstacle['velocity']]}}
                    simulation = {'trajectories': trajectory}
                    self.environment.add_obstacle(Obstacle({'position': pos}, shape=rectangle, simulation=simulation, options={'bounce': obstacle['bounce']}))
                elif obstacle['shape'] == 'circle':
                    circle = Circle(radius=obstacle['radius'])
                    pos = obstacle['pos']
                    trajectory = {'velocity': {'time': [0.], 'values': [obstacle['velocity']]}}
                    simulation = {'trajectories': trajectory}
                    self.environment.add_obstacle(Obstacle({'position': pos}, shape=circle, simulation=simulation, options={'bounce': obstacle['bounce']}))
                else:
                    raise ValueError('For now only rectangles and circles are supported, you selected a ' + obstacle['shape'])
            
            # if save environment is checked, save environment
            if self.save_env.get():
                self.save_environment()

            # close window
            self.destroy()
            self.root.destroy()
        else:
            print 'Please right-click on the start and goal position first'

    def save_environment(self):
        environment = {}
        environment['position'] = [0,0]
        environment['position'][0] = self.position[0]/self.meter2pixel
        environment['position'][1] = self.position[1]/self.meter2pixel
        environment['width'] = self.frameWidth/self.meter2pixel
        environment['height'] = self.frameHeight/self.meter2pixel
        env_to_save = [environment, self.obstacles, self.clickedPositions]
        with open('environment.pickle', 'wb') as handle:
            pickle.dump(env_to_save, handle, protocol=pickle.HIGHEST_PROTOCOL)

    def load_environment(self):
        # first remove all obstacles which are already present
        for obstacle in self.obstacles:
            self.canvas.delete(obstacle['variable'])
        with open('environment.pickle', 'rb') as handle:
            saved_env = pickle.load(handle)
        env = saved_env[0]
        self.position = env['position']*self.meter2pixel
        self.frameWidth = env['width']*self.meter2pixel
        self.frameHeight = env['height']*self.meter2pixel

        self.obstacles = saved_env[1]
        for obs in self.obstacles:
            # obstacle position is saved in world coordinates, convert to pixels to plot
            obs_pos = self.world2Pixel(obs['pos'])
            obs_pos = self.snap_to_grid(obs_pos)
            if obs['shape'] == 'rectangle':
                self.canvas.create_rectangle(obs_pos[0]-obs['width']*self.meter2pixel/2., obs_pos[1]-obs['height']*self.meter2pixel/2.,
                                             obs_pos[0]+obs['width']*self.meter2pixel/2., obs_pos[1]+obs['height']*self.meter2pixel/2.,
                                             fill="black")
            elif obs['shape'] == 'circle':
                self.canvas.create_circle(obs_pos[0], obs_pos[1], obs['radius']*self.meter2pixel, fill="black")
        
        self.clickedPositions = saved_env[2]

    def getEnvironment(self):
        return self.environment

    def getClickedPositions(self):
        return self.clickedPositions

    def _create_circle(self, x, y, r, **kwargs):
        return self.create_oval(x-r, y-r, x+r, y+r, **kwargs)
    tk.Canvas.create_circle = _create_circle

    def pixel2World(self, pixel):
        # pixel min value ([0,0]) = top left
        # pixel max value = bottom right
        # convert this axis frame to the world axis,
        # i.e. min value = bottom left, max value = top right

        vmin = self.position[1]
        vmax = self.position[1] + self.frameHeight
        # umin = self.position[0]
        # umax = self.frameWidth
        # xmin = self.position[0]/self.meter2pixel
        # xmax = self.frameWidth/self.meter2pixel
        # ymin = self.position[1]/self.meter2pixel
        # ymax = self.frameHeight/self.meter2pixel
        u,v = pixel
        u,v = float(u), float(v)
        x = u/self.meter2pixel
        y = (vmax-v)/self.meter2pixel + vmin/self.meter2pixel
        return [x,y]

    def world2Pixel(self, world):
        # convert world coordinate frame to pixel frame
        # = invert y-axis and shift

        vmin = self.position[1]
        vmax = self.position[1] + self.frameHeight

        x,y = world
        x,y = float(x), float(y)
        u = x*self.meter2pixel
        v = -y*self.meter2pixel + vmin + vmax
        return [u,v]