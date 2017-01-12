import Tkinter as tk
from environment import Environment, Obstacle
from ..basics.shape import Rectangle, Circle

class EnvironmentGUI(tk.Tk):
    # Graphical assistant to make an environment
    # width = environment border width
    # height = environment border height
    # position = top left corner point of environment
    def __init__(self, width=8,height=8, position=[0,0], options=None, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)

        self.title("Environment")
        self.resizable(0,0)

        self.meter2pixel = 100  # conversion of meter to pixels

        grid_size = options['grid_size'] if 'grid_size' in options else 0.5

        # make environment frame and size it
        self.frameWidth = width*self.meter2pixel
        self.frameHeight = height*self.meter2pixel
        self.position = [pos * self.meter2pixel for pos in position]  # top left corner point of the frame
        self.canvas = tk.Canvas(self, width=self.frameWidth, height=self.frameHeight, borderwidth=0, highlightthickness=0)
        self.canvas.configure(cursor="tcross")
        # self.canvas.pack(side="top", fill="both", expand="true")
        self.canvas.grid(row=0,columnspan=3)
        self.cellWidth = int(grid_size*self.meter2pixel)
        self.cellHeight = int(grid_size*self.meter2pixel)
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
        self.readyButton = tk.Button(self, text="Ready", fg="green", command=self.buildEnvironment)
        self.readyButton.grid(row=2, column=0)
        # self.readyButton.pack(side=tk.RIGHT)

        self.quitButton = tk.Button(self, text="Quit", fg="red", command=self.canvas.quit)
        self.quitButton.grid(row=3, column=0)
        # self.button.pack(side=tk.RIGHT)

        # add cellSize button
        self.cellSize = tk.Label(self, text='Cell size: ' + str(float(self.cellWidth)/self.meter2pixel) + 'x' + str(float(self.cellHeight)/self.meter2pixel) + ' [m]')
        self.cellSize.grid(row=1, column=1)
        # self.cellSize.pack(expand=0)

        # Clicked position label
        # self.clickedPos = tk.StringVar()
        # labelClickedPos = tk.Label(self, text="Clicked position [x,y]")
        # labelClickedPosEntry = tk.Entry(self, bd =0, textvariable=self.clickedPos)
        # labelClickedPos.pack()
        # labelClickedPosEntry.pack()

        # Add rectangle or circle
        self.shape = tk.StringVar()
        self.circleButton = tk.Radiobutton(self, text="Circle", variable=self.shape, value='circle')#.pack(anchor=tk.W)
        self.circleButton.grid(row=2, column=2)
        self.rectangleButton = tk.Radiobutton(self, text="Rectangle", variable=self.shape, value='rectangle')#.pack(anchor=tk.W)
        self.rectangleButton.grid(row=3,column=2)

        # Add text
        self.width = tk.DoubleVar()
        self.height = tk.DoubleVar()
        self.radius = tk.DoubleVar()

        labelWidth = tk.Label(self, text="Width [m]")
        labelWidthEntry = tk.Entry(self, bd =0, textvariable=self.width)
        labelWidth.grid(row=2,column=1)
        labelWidthEntry.grid(row=3,column=1)

        labelHeight = tk.Label(self, text="Height [m]")
        labelHeightEntry = tk.Entry(self, bd =0, textvariable=self.height)
        labelHeight.grid(row=4,column=1)
        labelHeightEntry.grid(row=5,column=1)

        labelRadius = tk.Label(self, text="Radius [m]")
        labelRadiusEntry = tk.Entry(self, bd =0, textvariable=self.radius)
        labelRadius.grid(row=6,column=1)
        labelRadiusEntry.grid(row=7,column=1)

    def makeObstacle(self, event):

        obstacle = {}
        event = self.snap2Grid(event)
        clickedPixel = [click+pos for click,pos in zip([event.x,event.y],self.position)]  # shift click with position of frame
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
                self.canvas.create_rectangle(event.x-self.width.get()*self.meter2pixel/2., event.y-self.height.get()*self.meter2pixel/2.,
                                             event.x+self.width.get()*self.meter2pixel/2., event.y+self.height.get()*self.meter2pixel/2.,
                                             fill="black")
                obstacle['shape'] = 'rectangle'
                obstacle['width'] = self.width.get()
                obstacle['height'] = self.height.get()
                self.obstacles.append(obstacle)
                print 'Created obstacle: ', obstacle
        elif self.shape.get() == 'circle':
            if (self.radius.get() <= 0):
                print 'Select a strictly positive radius before making a circle'
            elif (self.radius.get()*self.meter2pixel >= (self.frameWidth or self.frameHeight)):
                print 'Selected radius is too big'
            else:
                self.canvas.create_circle(event.x, event.y, self.radius.get()*self.meter2pixel, fill="black")
                obstacle['shape'] = 'circle'
                obstacle['radius'] = self.radius.get()
                self.obstacles.append(obstacle)
                print 'Created obstacle: ', obstacle
        # return self.obstacles

    def snap2Grid(self, event):
        # Snap the user clicked point to a grid point, since obstacles can only
        # be placed on grid points
        event.x = int(self.cellWidth * round(float(event.x)/self.cellWidth))
        event.y = int(self.cellWidth * round(float(event.y)/self.cellWidth))
        return event

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
        # make border
        self.environment = Environment(room={'shape': Rectangle(width = self.frameWidth/self.meter2pixel,
                                                                height = self.frameHeight/self.meter2pixel),
                                             'position':[(self.position[0]+self.frameWidth/2.)/self.meter2pixel,
                                                         (self.position[1]+self.frameHeight/2.)/self.meter2pixel]})
        for obstacle in self.obstacles:
            print obstacle
            if obstacle['shape'] == 'rectangle':
                rectangle = Rectangle(width=obstacle['width'],height=obstacle['height'])
                pos = obstacle['pos']
                self.environment.add_obstacle(Obstacle({'position': pos}, shape=rectangle))
            elif obstacle['shape'] == 'circle':
                circle = Circle(radius=obstacle['radius'])
                pos = obstacle['pos']
                self.environment.add_obstacle(Obstacle({'position': pos}, shape=circle))
            else:
                raise ValueError('For now only rectangles and circles are supported, you selected a ' + obstacle['shape'])
        # close window
        self.destroy()

    def getEnvironment(self):
        return self.environment

    def getClickedPositions(self):
        return self.clickedPositions

    def _create_circle(self, x, y, r, **kwargs):
        return self.create_oval(x-r, y-r, x+r, y+r, **kwargs)
    tk.Canvas.create_circle = _create_circle

    def pixel2World(self, pixel):
        # umin = self.position[0]
        # umax = self.frameWidth
        vmin = self.position[1]
        vmax = self.position[1] + self.frameHeight
        # xmin = self.position[0]/self.meter2pixel
        # xmax = self.frameWidth/self.meter2pixel
        # ymin = self.position[1]/self.meter2pixel
        # ymax = self.frameHeight/self.meter2pixel
        u,v = pixel
        u,v = float(u), float(v)
        x = u/self.meter2pixel
        y = (vmax-v)/self.meter2pixel + vmin/self.meter2pixel
        return [x,y]

## grid to place buttons more neatly
## snap to grid
## position buttons --> placed at certain position when you click
## 
