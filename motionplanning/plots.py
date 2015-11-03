from mycolors import *
import matplotlib
matplotlib.use('TKAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as an
import numpy as np
from spline_extra import *
from matplotlib2tikz import save as tikz_save
from os import system

class Plots:
    def __init__(self, vehicles, environment, options = {}):
        self.vehicles       = vehicles if isinstance(vehicles, list) else [vehicles]
        self.environment    = environment
        self.vehicle_types  = {}
        for veh in self.vehicles:
            typename = veh.__class__.__name__
            if typename in self.vehicle_types:
                self.vehicle_types[typename].append(veh)
            else:
                self.vehicle_types[typename] = [veh]
        self.defineColors()
        self.figures        = {}
        self.axes           = {}
        self.plot_types     = []
        self.other_plots    = []
        # Plots
        self.plt_other  = {}
        self.plt_traj   = {}
        self.plt_path   = {}
        plt.show()
        self.setOptions(options)

    def setOptions(self, options):
        if 'knots' in options and options['knots']:
            knots = {}
            knots['name']   = 'knots'
            knots['symbol'] = 'x'
            knots['time']   = lambda veh,k: veh.trajectories['knots']['time'][k]
            knots['data']   = lambda veh,plot_type,ind,k:  veh.trajectories['knots'][plot_type][k][ind,:]
            self.other_plots.append(knots)
            self.plt_other['knots'] = {}
        if 'prediction' in options and options['prediction']:
            prediction = {}
            prediction['name']   = 'prediction'
            prediction['symbol'] = 'o'
            prediction['time']   = lambda veh,k: veh.path['time'][k]
            prediction['data']   = lambda veh,plot_type,ind,k:  veh.predictions[plot_type][k][ind]
            self.other_plots.append(prediction)
            self.plt_other['prediction'] = {}

    def defineColors(self):
        self.colors     = color_tmpl
        self.colors_w   = [mixwithwhite(self.colors[l]) for l in range(len(self.colors))]

    def create(self, plot_types):
        plot_types = plot_types if isinstance(plot_types, list) else [plot_types]
        plt.ion()
        for plot_type in plot_types:
            if plot_type == '2D':
                self.figures[plot_type], self.axes[plot_type] = self.initPlot2D()
            else:
                self.figures[plot_type], self.axes[plot_type] = self.initPlotCurves(plot_type)
        self.plot_types.extend(plot_types)

    def initPlot2D(self):
        figure = plt.figure()
        axis   = figure.add_subplot(111)
        canvas_lim = self.environment.getCanvasLimits()
        plt.xlim(canvas_lim['xlim'][0],canvas_lim['xlim'][1]); plt.ylim(canvas_lim['ylim'][0],canvas_lim['ylim'][1])
        self.plt_2D     = {}
        self.plt_2D['obstacles']    = [axis.plot([], [], 'k-')[0] for l in range(self.environment.No)]
        self.plt_2D['trajectory']   = [axis.plot([], [], '-', color = self.colors_w[veh.ind])[0]  for veh in self.vehicles]
        self.plt_2D['path']         = [axis.plot([], [], '-', color = self.colors[veh.ind])[0]    for veh in self.vehicles]
        self.plt_2D['vehicle']      = [axis.plot([], [], '-', color = self.colors[veh.ind])[0]    for veh in self.vehicles]
        return figure, axis

    def initPlotCurves(self, plot_type):
        figures = {}
        axes    = {}
        self.plt_traj[plot_type] = {}
        self.plt_path[plot_type] = {}
        for other in self.other_plots:
            self.plt_other[other['name']][plot_type] = {}
        for vehicle_type in self.vehicle_types:
            vehicles= self.vehicle_types[vehicle_type]
            info    = vehicles[0].getPlotInfo(plot_type)
            figures[vehicle_type], axes[vehicle_type] = plt.subplots(len(info['indices']), 1, squeeze = False)
            for k in info['indices']:
                axes[vehicle_type][k,0].set_xlabel('t (s)')
                axes[vehicle_type][k,0].set_ylabel(info['names'][k])
            self.plt_traj[plot_type][vehicle_type]  = [[axes[vehicle_type][k,0].plot([],[],'-',color = self.colors_w[veh.ind])[0] for k in info['indices']] for veh in vehicles]
            self.plt_path[plot_type][vehicle_type]  = [[axes[vehicle_type][k,0].plot([],[],'-',color = self.colors[veh.ind])[0]   for k in info['indices']] for veh in vehicles]
            for other in self.other_plots:
                self.plt_other[other['name']][plot_type][vehicle_type]  = [[axes[vehicle_type][k,0].plot([],[],other['symbol'],color = self.colors[veh.ind])[0]   for k in info['indices']] for veh in vehicles]
        return figures, axes

    def update(self, k = -1, **kwargs):
        plot_types = kwargs['plot_types'] if 'plot_types' in kwargs else self.plot_types
        for plot_type in plot_types:
            if plot_type == '2D':
                self.updatePlot2D(k)
                self.figures[plot_type].canvas.draw()
            else:
                self.updatePlotCurves(plot_type, k)
                for vehicle_type in self.vehicle_types:
                    ax = self.axes[plot_type][vehicle_type]
                    for i in range(ax.shape[0]):
                        for j in range(ax.shape[1]):
                            ax[i,j].relim()
                            ax[i,j].autoscale_view(True,True,True)
                    self.figures[plot_type][vehicle_type].canvas.draw()

    def updatePlot2D(self, k = -1):
        environment = self.environment.draw(k)
        for l in range(self.environment.No):
            self.plt_2D['obstacles'][l].set_data(environment['obstacles'][l][0,:],environment['obstacles'][l][1,:])
        cnt = 0
        for veh in self.vehicles:
            if k == -1:
                pos_path    = veh.getPosition(veh.path['y'][:,:])
            else:
                pos_path    = veh.getPosition(veh.path['y'][:,:k+1])
            pos_traj    = veh.getPosition(veh.trajectories['y'][k])
            veh_cnt     = veh.draw(k)
            self.plt_2D['trajectory'][cnt].set_data(pos_traj[0,:], pos_traj[1,:])
            self.plt_2D['path'][cnt].set_data(pos_path[0,:], pos_path[1,:])
            self.plt_2D['vehicle'][cnt].set_data(veh_cnt[0], veh_cnt[1])
            cnt += 1

    def updatePlotCurves(self, plot_type, k = -1):
        for vehicle_type in self.vehicle_types:
            vehicles= self.vehicle_types[vehicle_type]
            info    = vehicles[0].getPlotInfo(plot_type)
            cnt = 0
            for veh in vehicles:
                for ind in range(len(info['indices'])):
                    self.plt_traj[plot_type][vehicle_type][cnt][ind].set_data(veh.trajectories['time'][k], veh.trajectories[plot_type][k][info['indices'][ind],:])
                    if k == -1:
                        self.plt_path[plot_type][vehicle_type][cnt][ind].set_data(veh.path['time'], veh.path[plot_type][info['indices'][ind],:])
                    else:
                        self.plt_path[plot_type][vehicle_type][cnt][ind].set_data(veh.path['time'][:k+1], veh.path[plot_type][info['indices'][ind],:k+1])
                    for other in self.other_plots:
                        self.plt_other[other['name']][plot_type][vehicle_type][cnt][ind].set_data(other['time'](veh,k), other['data'](veh,plot_type,info['indices'][ind],k))
                cnt += 1

    def resetPlots(self, plot_type):
        if plot_type == '2D':
            [plt.set_data([],[]) for plt in self.unravelPlt(self.plt_2D)]
        else:
            [plt.set_data([],[]) for plt in self.unravelPlt(self.plt_traj[plot_type])]
            [plt.set_data([],[]) for plt in self.unravelPlt(self.plt_path[plot_type])]
            for other in self.other_plots:
                [plt.set_data([],[]) for plt in self.unravelPlt(self.plt_other[other['name']][plot_type])]

    def unravelPlt(self, plt):
        plt_list = []
        if isinstance(plt, dict):
            for key,value in plt.items():
                plt_list.extend(self.unravelPlt(value))
        elif isinstance(plt, list):
            for pl in plt:
                plt_list.extend(self.unravelPlt(pl))
        else:
            plt_list.append(plt)
        return plt_list

    def show(self, plot_types):
        plot_types = plot_types if isinstance(plot_types,list) else [plot_types]
        for plot_type in plot_types:
            if not(plot_type in self.figures):
                self.create(plot_type)
        self.update(k = -1, plot_types = plot_types)

    def save(self, plot_types, name, path = 'images/'):
        plot_types = plot_types if isinstance(plot_types,list) else [plot_types]
        for plot_type in plot_types:
            if not(plot_type in self.figures):
                self.create(plot_type)
        directory = path+name
        if not os.path.isdir(directory):
            os.makedirs(directory)
        for plot_type in plot_types:
            self.update(k = -1, plot_types = plot_types)
            tikz_save(directory+'/'+name+'_'+plot_type+'.tikz',figurewidth='8cm')

    def showMovie(self, plot_types, repeat = False, **kwargs):
        plot_types = plot_types if isinstance(plot_types,list) else [plot_types]
        t   = self.vehicles[0].path['time']
        if 'number_of_frames' in kwargs and kwargs['number_of_frames'] <= (len(t)-1):
            number_of_frames = kwargs['number_of_frames']
        else:
            number_of_frames = len(t)-1
        subsample = (len(t)-1)/(number_of_frames-1)
        for plot_type in plot_types:
            if not(plot_type in self.figures):
                self.create(plot_type)
        while True:
            for k in range(0,len(t)-1,subsample):
                self.update(k, plot_types = plot_types)
            if not repeat:
                break

    def saveMovie(self, plot_types, name, path = 'movies/', **kwargs):
        plot_types  = plot_types if isinstance(plot_types,list) else [plot_types]
        directories = {}
        for plot_type in plot_types:
            directories[plot_type] = path+name+'_'+plot_type+'_mov'
        for directory in directories:
            if not os.path.isdir(directory):
                os.makedirs(directory)
        t = self.vehicles[0].path['time']
        if 'number_of_frames' in kwargs and kwargs['number_of_frames'] <= (len(t)-1):
            number_of_frames = kwargs['number_of_frames']
        else:
            number_of_frames = len(t)-1
        subsample = (len(t)-1)/(number_of_frames-1)
        for plot_type in plot_types:
            if not(plot_type in self.figures):
                self.create(plot_type)
        cnt = 0.
        plt.axis('off')
        for k in range(0,len(t)-1,subsample):
            for plot_type in plot_types:
                self.update(k, plot_types = plot_types)
                tikz_save(directories[plot_type]+'/'+name+'_'+plot_type+'_'+str(cnt)+'.tikz',figurewidth='8cm')
            cnt += 1
