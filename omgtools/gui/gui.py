from ..environment.environment import Environment, Obstacle
from ..basics.shape import Rectangle, Circle
from svg_reader import SVGReader

import Tkinter as tk
import tkFileDialog as tkfiledialog
import tkMessageBox as tkmessagebox
import pickle
import numpy as np

class EnvironmentGUI(tk.Frame):
    # Graphical assistant to make an environment
    # width = environment border width [m]
    # height = environment border height [m]
    # position = top left corner point of environment [m]
    def __init__(self, parent, width=8,height=8, position=[0,0], options={}, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)

        # set root of frame
        self.root = parent
        self.root.title("Environment")
        self.root.resizable(0,0)

        # assign svg-reader, allows reading in svg-figures
        self.reader = SVGReader()

        # set constants
        # conversion of meter to pixels, required for plotting
        self.meter_to_pixel = options['meter_to_pixel'] if 'meter_to_pixel' in options else 50
        self.n_cells = options['n_cells'] if 'n_cells' in options else [20, 20]  # [horizontal, vertical]

        # make environment frame and size it
        self.frame_width_px = int(width*self.meter_to_pixel)
        self.frame_height_px = int(height*self.meter_to_pixel)
        self.frame_width_m = width
        self.frame_height_m = height
        # top left corner point of the frame [px]
        self.position = [pos * self.meter_to_pixel for pos in position]
        self.canvas = tk.Canvas(self.root, width=self.frame_width_px, height=self.frame_height_px,
                                borderwidth=0, highlightthickness=0)
        self.canvas.configure(cursor="tcross")
        self.canvas.grid(row=0,columnspan=3)
        # round up sizes, such that frame width is guaranteed to be reached
        self.cell_width_px = int(np.ceil(self.frame_width_px*1./self.n_cells[0]))
        self.cell_height_px = int(np.ceil(self.frame_height_px*1./self.n_cells[1]))
        self.rows = self.n_cells[0]
        self.columns = self.n_cells[1]

        self.draw_grid()  # draw grid in canvas

        self.obstacles = []  # list to hold all obstacles
        self.clicked_positions = []  # list to hold all clicked positions

        self.canvas.bind("<Button-1>", self.make_obstacle)  # left mouse button makes obstacle
        self.canvas.bind("<Button-3>", self.get_position)  # right mouse button gives start and goal position

        # Add buttons
        ready_button = tk.Button(self.root, text="Ready", fg="green", command=self.build_environment)
        ready_button.grid(row=2, column=0)

        quit_button = tk.Button(self.root, text="Quit", fg="red", command=self.canvas.quit)
        quit_button.grid(row=3, column=0)

        remove_button = tk.Button(self.root, text="Remove", fg="black", command=self.remove_last_obstacle)
        remove_button.grid(row=4, column=0)

        load_button = tk.Button(self.root, text="Load", fg="black", command=self.load_environment)
        load_button.grid(row=5, column=0)

        load_svg_button = tk.Button(self.root, text="Load SVG", fg="black", command=self.load_svg)
        load_svg_button.grid(row=6, column=0)

        self.save_env = tk.BooleanVar(value=False)
        label_save = tk.Checkbutton(self.root, text="Save [yes/no]", variable=self.save_env)
        label_save.grid(row=7,column=0)

        # indicate cell size
        self.cell_size_label = tk.Label(self.root, text='Cell size: ' + str(np.round(self.frame_width_m*1./self.n_cells[0],3)) + 'x' + str(np.round(self.frame_height_m*1./self.n_cells[0],3)) + ' [m]')
        self.cell_size_label.grid(row=1, column=1)

        # indicate clicked position
        # self.clickedPos = tk.StringVar()
        # labelClickedPos = tk.Label(self, text="Clicked position [x,y]")
        # labelClickedPosEntry = tk.Entry(self, bd =0, textvariable=self.clickedPos)
        # labelClickedPos.pack()
        # labelClickedPosEntry.pack()

        # select obstacle shape (rectangle or circle)
        self.shape = tk.StringVar()
        self.circle_button = tk.Radiobutton(self.root, text="Circle", variable=self.shape, value='circle')
        self.circle_button.grid(row=2, column=2)
        self.rectangle_button = tk.Radiobutton(self.root, text="Rectangle", variable=self.shape, value='rectangle')
        self.rectangle_button.grid(row=3,column=2)

        # select obstacle size
        self.width = tk.DoubleVar()
        label_width = tk.Label(self.root, text="Width [m]")
        label_width_entry = tk.Entry(self.root, bd =0, textvariable=self.width)
        label_width.grid(row=2,column=1)
        label_width_entry.grid(row=3,column=1)

        self.height = tk.DoubleVar()
        label_height = tk.Label(self.root, text="Height [m]")
        label_height_entry = tk.Entry(self.root, bd =0, textvariable=self.height)
        label_height.grid(row=4,column=1)
        label_height_entry.grid(row=5,column=1)

        self.radius = tk.DoubleVar()
        label_radius = tk.Label(self.root, text="Radius [m]")
        label_radius_entry = tk.Entry(self.root, bd =0, textvariable=self.radius)
        label_radius.grid(row=6,column=1)
        label_radius_entry.grid(row=7,column=1)

        # select obstacle velocity
        self.vel_x = tk.DoubleVar(value=0.0)
        label_velx = tk.Label(self.root, text="x-velocity [m/s]")
        label_velx_entry = tk.Entry(self.root, bd =0, textvariable=self.vel_x)
        label_velx.grid(row=4,column=2)
        label_velx_entry.grid(row=5,column=2)

        self.vel_y = tk.DoubleVar(value=0.0)
        label_vely = tk.Label(self.root, text="y-velocity [m/s]")
        label_vely_entry = tk.Entry(self.root, bd =0, textvariable=self.vel_y)
        label_vely.grid(row=6,column=2)
        label_vely_entry.grid(row=7,column=2)

        # select if moving obstacle should bounce off other obstacles or off the walls
        self.bounce = tk.BooleanVar(value=False)
        label_bounce = tk.Checkbutton(self.root, text="Bounce [yes/no]", variable=self.bounce)
        label_bounce.grid(row=8,column=2)

        # add scrollbars
        self.hbar=tk.Scrollbar(self.root,orient=tk.HORIZONTAL)
        self.hbar.grid(row=9, column = 0)
        self.hbar.config(command=self.canvas.xview)
        self.vbar=tk.Scrollbar(self.root,orient=tk.VERTICAL)
        self.vbar.grid(row=0, column = 3)
        self.vbar.config(command=self.canvas.yview)
        self.canvas.config(width=1000, height=600)  # limit canvas size such that it fits on screen
        self.canvas.config(xscrollcommand=self.hbar.set, yscrollcommand=self.vbar.set)

    def reinitialize(self, width, height, meter_to_pixel, position=[0,0], n_cells=[50,50], **kwargs):
        # change the original size of the environment, e.g. after loading an SVG-image

        self.canvas.delete('all')  # remove all current obstacles
        self.canvas.config(width=1000, height=600)  # limit canvas size such that it fits on screen

        # save meter_to_pixel conversion factor
        # this has to be an input given by the user, when loading an svg
        self.meter_to_pixel = meter_to_pixel
        
        # make environment frame and size it
        self.frame_width_px = int(width*self.meter_to_pixel)
        self.frame_height_px = int(height*self.meter_to_pixel)
        self.frame_width_m = width
        self.frame_height_m = height
        self.position = position # top left corner point of the frame [px]
        self.n_cells = n_cells

        # make sure cell_size is a multiple of frame_width and frame_height, both in px and m
        # round up, such that frame width is guaranteed to be reached
        self.cell_width_px = int(np.ceil(self.frame_width_px*1./self.n_cells[0]))
        self.cell_height_px = int(np.ceil(self.frame_height_px*1./self.n_cells[1]))
        self.columns = self.n_cells[0]
        self.rows = self.n_cells[1]

        self.draw_grid()  # draw grid in canvas
    
        # update cell_size text
        self.cell_size_label['text'] = 'Cell size: ' + str(np.round(self.frame_width_m*1./self.n_cells[0],3)) + 'x' + str(np.round(self.frame_height_m*1./self.n_cells[1],3)) + ' [m]'

    def draw_grid(self):
        # draw grid in canvas
        self.rect = {}
        for column in range(self.columns):
            for row in range(self.rows):
                x1 = column*self.cell_width_px
                y1 = row * self.cell_height_px
                x2 = x1 + self.cell_width_px
                y2 = y1 + self.cell_height_px
                self.rect[row,column] = self.canvas.create_rectangle(x1,y1,x2,y2, fill="white", tags="rect")

    def make_obstacle(self, event):
        # draw obstacle in the GUI and save it to self.obstacles
        obstacle = {}
        clicked_x = self.canvas.canvasx(event.x)  # get the position in the canvas, not in the window
        clicked_y = self.canvas.canvasy(event.y)  # this deals with the scrollbars
        clicked_pos = [clicked_x, clicked_y]
        snapped_pos = self.snap_to_grid(clicked_pos)  # snap clicked position to intersection of cells
        # shift clicked position over the center position of frame
        clicked_pixel = [click+pos for click,pos in zip(snapped_pos,self.position)]
        pos = self.pixel_to_world(clicked_pixel)  # convert [px] to [m]
        obstacle['pos'] = pos
        if self.shape.get() == '':
            print 'Select a shape before placing an obstacle!'
        elif self.shape.get() == 'rectangle':
            if (self.width.get() <= 0 or self.height.get() <= 0):
                print 'Select a strictly positive width and height for the rectangle first'
            elif (self.width.get()*self.meter_to_pixel >= self.frame_width_px):
                print 'Selected width is too big'
            elif (self.height.get()*self.meter_to_pixel >= self.frame_height_px):
                print 'Selected height is too big'
            else:  # valid, draw and add to self.obstacles
                obstacle_var = self.canvas.create_rectangle(snapped_pos[0]-self.width.get()*self.meter_to_pixel*0.5, snapped_pos[1]-self.height.get()*self.meter_to_pixel*0.5,
                                             snapped_pos[0]+self.width.get()*self.meter_to_pixel*0.5, snapped_pos[1]+self.height.get()*self.meter_to_pixel*0.5,
                                             fill="black")
                obstacle['shape'] = 'rectangle'
                obstacle['width'] = self.width.get()
                obstacle['height'] = self.height.get()
                obstacle['velocity'] = [self.vel_x.get(), self.vel_y.get()]
                obstacle['bounce'] = self.bounce.get()
                # save variable to be able to delete obstacle from canvas
                obstacle['variable'] = obstacle_var
                self.obstacles.append(obstacle)
                print 'Created obstacle: ', obstacle
        elif self.shape.get() == 'circle':
            if (self.radius.get() <= 0):
                print 'Select a strictly positive radius before making a circle'
            elif (self.radius.get()*self.meter_to_pixel >= (self.frame_width_px or self.frame_height_px)):
                print 'Selected radius is too big'
            else:  # valid, draw and add to self.obstacles
                obstacle_var = self.canvas.create_circle(snapped_pos[0], snapped_pos[1], self.radius.get()*self.meter_to_pixel, fill="black")
                obstacle['shape'] = 'circle'
                obstacle['radius'] = self.radius.get()
                obstacle['velocity'] = [self.vel_x.get(), self.vel_y.get()]
                obstacle['bounce'] = self.bounce.get()
                # save variable to be able to delete obstacle from canvas
                obstacle['variable'] = obstacle_var
                self.obstacles.append(obstacle)
                print 'Created obstacle: ', obstacle

    def remove_last_obstacle(self):
        # called when clicking the remove button,
        # this removes the obstacle which was created the last,
        # and erases it from the GUI
        if self.obstacles:
            # get all ids of obstacles drawn on canvas
            ids = self.canvas.find_all()
            # delete the last id, corresponding to the last obstacle
            self.canvas.delete(ids[-1])
            # remove obstacle from list
            del self.obstacles[-1]

    def snap_to_grid(self, point):
        # Snap the user clicked point to a grid point, since obstacles can only
        # be placed on grid points (i.e. an intersection of cells, not the center of a cell)
        snapped = [0,0]
        snapped[0] = int(self.cell_width_px * round(float(point[0])/self.cell_width_px))
        snapped[1] = int(self.cell_height_px * round(float(point[1])/self.cell_height_px))
        return snapped

    def get_position(self, event):
        clicked_x = self.canvas.canvasx(event.x)  # get the position in the canvas, not in the window
        clicked_y = self.canvas.canvasy(event.y)  # this deals with the scrollbars
        clicked = [clicked_x, clicked_y]
        clicked = self.pixel_to_world(clicked)
        print 'You clicked on: ', clicked
        self.clicked_positions.append(clicked)  # save position which the user clicked on
        if (len(self.clicked_positions) > 2):
            self.clicked_positions[0] = self.clicked_positions[1]  # second = first
            self.clicked_positions[1] = self.clicked_positions[2]  # second last = last
            self.clicked_positions=self.clicked_positions[:2]  # remove last
            print 'You clicked more than two times, your initial first click was removed'

    def build_environment(self):
        # only build environment and shut down GUI if start and goal positions are clicked
        if len(self.clicked_positions) == 2:
            # make border
            self.environment = Environment(room={'shape': Rectangle(width = self.frame_width_m,
                                                                    height = self.frame_height_m),
                                                 'position':[self.position[0]*1./self.meter_to_pixel+self.frame_width_m*0.5,
                                                             self.position[1]*1./self.meter_to_pixel+self.frame_height_m*0.5]})                
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
            
            # if save environment is checked, save environment in a file
            if self.save_env.get():
                self.save_environment()

            # close window
            self.destroy()
            self.root.destroy()
        else:  # start and goal not selected yet
            print 'Please right-click on the start and goal position first'

    def add_obstacle(self, obstacle):
        # draws an obstacle on the current canvas
        pos = obstacle['pos']
        # don't snap for a loaded obstacle, this gives a less accurate conversion from file to OMG-tools
        obstacle['pos'] = self.pixel_to_world(pos)  # convert [px] to [m]
        if obstacle['shape'] == 'rectangle':
            obstacle_var = self.canvas.create_rectangle(pos[0]-obstacle['width']*0.5, pos[1]-obstacle['height']*0.5,
                                             pos[0]+obstacle['width']*0.5, pos[1]+obstacle['height']*0.5,
                                             fill="black")
            obstacle['variable'] = obstacle_var
            obstacle['width'] = obstacle['width']*1./self.meter_to_pixel  # [px] to [m]
            obstacle['height'] = obstacle['height']*1./self.meter_to_pixel  # [px] to [m]
            print 'Added obstacle: ', obstacle
        elif obstacle['shape'] == 'circle':
            obstacle_var = self.canvas.create_circle(pos[0], pos[1], obstacle['radius'], fill="black")
            obstacle['variable'] = obstacle_var
            obstacle['radius'] = obstacle['radius']*1./self.meter_to_pixel  # [px] to [m]
            print 'Added obstacle: ', obstacle

    def save_environment(self):
        environment = {}
        environment['meter_to_pixel'] = self.meter_to_pixel
        environment['position'] = self.position
        environment['width'] = self.frame_width_m
        environment['height'] = self.frame_height_m
        env_to_save = [environment, self.obstacles, self.clicked_positions, self.n_cells]
        with open('environment.pickle', 'wb') as handle:  # save environment in 'environment.pickle'
            pickle.dump(env_to_save, handle, protocol=pickle.HIGHEST_PROTOCOL)

    def load_environment(self):
        # first remove all obstacles which were already present in canvas
        for obstacle in self.obstacles:
            self.canvas.delete(obstacle['variable'])
        with open('environment.pickle', 'rb') as handle:
            saved_env = pickle.load(handle)
        env = saved_env[0]  # the environment itself
        self.meter_to_pixel = env['meter_to_pixel']
        self.position = env['position']
        self.frame_width_px = env['width']*self.meter_to_pixel
        self.frame_height_px = env['height']*self.meter_to_pixel
        self.frame_width_m = env['width']
        self.frame_height_m = env['height']

        self.clicked_positions = saved_env[2]  # clicked start and goal positions
        self.n_cells = saved_env[3]  # number of cells
        
        # resize the canvas, according to the loaded settings
        self.reinitialize(width=self.frame_width_m, height=self.frame_height_m,
                          meter_to_pixel = self.meter_to_pixel, position=self.position, n_cells=self.n_cells)

        self.obstacles = saved_env[1]  # obstacles in the environment
        for obs in self.obstacles:
            # obstacle position is saved in world coordinates [m], convert to pixels to plot
            obs_pos = self.world_to_pixel(obs['pos'])
            if obs['shape'] == 'rectangle':
                self.canvas.create_rectangle(obs_pos[0]-obs['width']*self.meter_to_pixel*0.5, obs_pos[1]-obs['height']*self.meter_to_pixel*0.5,
                                             obs_pos[0]+obs['width']*self.meter_to_pixel*0.5, obs_pos[1]+obs['height']*self.meter_to_pixel*0.5,
                                             fill="black")
            elif obs['shape'] == 'circle':
                self.canvas.create_circle(obs_pos[0], obs_pos[1], obs['radius']*self.meter_to_pixel, fill="black")

    def load_svg(self):
        # show a popup window to let the user select an environment defined as an svg-figure
        filename = tkfiledialog.askopenfilename(filetypes=[('SVG figures', '.svg'), ('all files','.*')])
        if filename:
            try:
                data = open(filename, 'rb')
            except Exception, details:
                tkmessagebox.showerror(('Error'),details)
                return

        self.reader.init(data)  # load the figure
        self.reader.build_environment()  # save obstacles in figure

        if not hasattr(self.reader, 'meter_to_pixel'):
            # give pop-up to ask user for meter_to_pixel conversion factor
            self.meter_to_pixel = 0
            while self.meter_to_pixel <= 0:
                self.window_m2p=popupWindow(self.root, 'How many pixels are there in 1 meter?')
                self.root.wait_window(self.window_m2p.top)
                try:
                    self.meter_to_pixel = int(self.window_m2p.value)
                except:
                    print 'Please fill in a natural number'
                if self.meter_to_pixel < 0:
                    print 'Please fill in a positive number'

        if not hasattr(self.reader, 'n_cells'):
            #  give pop-up to ask user for n_cells in horizontal and vertical direction
            self.n_cells = [0, 0]
            while 0 in self.n_cells:  # n_cells cannot contain any 0
                self.window_m2p=popupWindow(self.root, 'How many cells n, m do you want in the n x m grid?')
                self.root.wait_window(self.window_m2p.top)
                try:
                    value = self.window_m2p.value.split(',')
                    if len(value) != 2:
                        print 'Please enter exactly two natural numbers, in the form n, m.'
                        self.n_cells = [0, 0]  # reset values
                        continue # skip next part
                    self.n_cells[0] = int(value[0])
                    self.n_cells[1] = int(value[1])
                except:
                    print ('Please input your desired cell numbers in the form n, m.' +  
                          'In which n and m are natural numbers')
                    self.n_cells = [0, 0]  # reset values  
                    continue  # skip next part
                if 0 in self.n_cells:
                    print 'Your desired number of cells cannot be 0 in one of the directions'
                if any(c < 0 for c in self.n_cells):
                    self.n_cells = [0, 0]
                    print 'Please use positive numbers to indicate the desired number of cells'

        self.position = self.reader.position  # top left corner of the image [px]
        self.obstacles = self.reader.obstacles

        self.frame_width_px = int(self.reader.width_px)  # assign width of loaded SVG [px]
        self.frame_height_px = int(self.reader.height_px)  # assign height of loaded SVG [px]
        self.frame_width_m = self.frame_width_px*1./self.meter_to_pixel
        self.frame_height_m = self.frame_height_px*1./self.meter_to_pixel

        # reset/redraw GUI
        # All data is taken from an svg-file, so units are in pixels
        # Since OMG-tools requires meters, the user has to provide a meter_to_pixel factor
        # It may be possible that there is already information in mm in the svg-file, in this case
        # there is already a meter_to_pixel factor
        # Convert all pixel data to meter, using the conversion factor
        # position is the top left corner of the image, in [px]
        self.reinitialize(width=self.frame_width_m, height=self.frame_height_m, meter_to_pixel = self.meter_to_pixel, position=self.position, n_cells=self.n_cells)

        for obstacle in self.obstacles:
            self.add_obstacle(obstacle)

    def get_environment(self):
        return self.environment

    def get_clicked_positions(self):
        return self.clicked_positions

    def _create_circle(self, x, y, r, **kwargs):
        return self.create_oval(x-r, y-r, x+r, y+r, **kwargs)
    tk.Canvas.create_circle = _create_circle

    def pixel_to_world(self, pixel):
        # pixel min value [0,0] = top left
        # pixel max value = bottom right
        # convert this axis frame to the world axis,
        # i.e. min value = bottom left, max value = top right

        vmin = self.position[1]
        vmax = self.position[1] + self.frame_height_px

        u,v = pixel
        u,v = float(u), float(v)

        x = u/self.meter_to_pixel
        y = (vmax-v)/self.meter_to_pixel + vmin/self.meter_to_pixel
        
        return [x,y]

    def world_to_pixel(self, world):
        # convert world coordinate frame to pixel frame
        # = invert y-axis and shift

        vmin = self.position[1]
        vmax = self.position[1] + self.frame_height_px

        x,y = world
        x,y = float(x), float(y)
        
        u = x*self.meter_to_pixel
        v = -y*self.meter_to_pixel + vmin + vmax
        
        return [u,v]

class popupWindow(object):

    def __init__(self, master, text):
        self.top=tk.Toplevel(master)
        self.label=tk.Label(self.top,text=text)
        self.label.pack()
        self.entry=tk.Entry(self.top)
        self.entry.pack()
        self.button=tk.Button(self.top,text='Ok',command=self.cleanup)
        self.button.pack()

    def cleanup(self):
        self.value=self.entry.get()
        self.top.destroy()