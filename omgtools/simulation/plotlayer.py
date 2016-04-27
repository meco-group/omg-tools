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

import os
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import warnings
matplotlib.use('TKAgg')


def mix_with_white(color, perc_white=80.):
    r, g, b = color[0], color[1], color[2]
    r_m = ((100. - perc_white)*r + perc_white)/100.
    g_m = ((100. - perc_white)*g + perc_white)/100.
    b_m = ((100. - perc_white)*b + perc_white)/100.
    return [r_m, g_m, b_m]


def _init_axis_2d(axis, info):
    axis.set_xlabel(info['labels'][0])
    axis.set_ylabel(info['labels'][1])
    if 'xlim' in info and info['xlim'] is not None:
        axis.set_xlim(info['xlim'][0], info['xlim'][1])
    if 'ylim' in info and info['ylim'] is not None:
        axis.set_ylim(info['ylim'][0], info['ylim'][1])
    for line in info['lines']:
        axis.plot([], [], **line)


def _init_axis_3d(axis, info, view=None):
    axis.set_xlabel(info['labels'][0])
    axis.set_ylabel(info['labels'][1])
    axis.set_zlabel(info['labels'][2])
    if 'xlim' in info and info['xlim'] is not None:
        axis.set_xlim(info['xlim'][0], info['xlim'][1])
    if 'ylim' in info and info['ylim'] is not None:
        axis.set_ylim(info['ylim'][0], info['ylim'][1])
    if 'zlim' in info and info['zlim'] is not None:
        axis.set_zlim(info['zlim'][0], info['zlim'][1])
    if view is not None:
        elevation, azimuth = view
        axis.view_init(elev=elevation, azim=azimuth)
    for line in info['lines']:
        axis.plot([], [], [], **line)


def _update_axis_2d(axis, info, data):
    for p, dat in enumerate(data):
        axis.lines[p].set_data(dat[0].ravel(), dat[1].ravel())
    axis.relim()
    scalex = ('xlim' not in info or info['xlim'] is None)
    scaley = ('ylim' not in info or info['ylim'] is None)
    axis.autoscale_view(True, scalex, scaley)


def _update_axis_3d(axis, info, data):
    for p, dat in enumerate(data):
        axis.lines[p].set_data(dat[0].ravel(), dat[1].ravel())
        axis.lines[p].set_3d_properties(dat[2].ravel())
    axis.relim()
    scalex = ('xlim' not in info or info['xlim'] is None)
    scaley = ('ylim' not in info or info['ylim'] is None)
    scalez = ('zlim' not in info or info['zlim'] is None)
    axis.autoscale_view(True, scalex, scaley, scalez)


def _cleanup_rubbish(path, root=None):
    # cleanup rubbish due to bugs in matplotlib2tikz
    with open(path, 'r+') as f:
        body = f.read()
        body = body.replace('fill opacity=0', 'opacity=0')
        # add root at beginning of tikz file
        if root is not None:
            body = '%root=' + root + '\n' + body
        f.seek(0)
        f.truncate()
        f.write(body)


class PlotLayer(object):
    simulator = 0

    def __init__(self):
        # default colors
        blue = [17., 110., 138.]
        red = [138.,  31.,  17.]
        green = [17., 138.,  19.]
        lightblue = [106., 194., 238.]
        self.set_color_template([blue, red, green, lightblue])
        self.plots = []
        plt.ion()

    def set_color_template(self, colors):
        self.colors = [[c/255. for c in color] for color in colors]
        self.colors_w = [mix_with_white(c) for c in self.colors]

    # ========================================================================
    # Plot creation and initialization
    # ========================================================================

    def plot(self, argument=None, **kwargs):
        plot = {'argument': argument, 'kwargs': kwargs}
        self.plots.append(plot)
        if 'time' in kwargs:
            index = self.__class__.simulator.time2index(kwargs['time'])
        elif 'index' in kwargs:
            index = kwargs['index']
        else:
            index = -1
        no_update = kwargs['no_update'] if 'no_update' in kwargs else False
        if not no_update:
            self.update_plots(plots=plot, t=index)
        return plot

    def _init_plot(self, plot):
        argument, kwargs = plot['argument'], plot['kwargs']
        info = self.init_plot(argument, **kwargs)
        if info is None:
            return
        ax_r, ax_c = len(info), len(info[0])
        figure = plt.figure()
        for k in range(ax_r):
            for l in range(ax_c):
                if 'projection' in info[k][l] and info[k][l]['projection'] == '3d':
                    axis = figure.add_subplot(
                        ax_r, ax_c, k*ax_c+l+1, projection='3d')
                    if 'view' in kwargs:
                        view = kwargs['view']
                    else:
                        view = None
                    _init_axis_3d(axis, info[k][l], view)
                else:
                    axis = figure.add_subplot(ax_r, ax_c, k*ax_c+l+1)
                    _init_axis_2d(axis, info[k][l])
                if 'aspect_equal' in info[k][l] and info[k][l]['aspect_equal']:
                    axis.set_aspect('equal')
        plot.update({'figure': figure, 'info': info})

    # ========================================================================
    # Plot update
    # ========================================================================

    def update_plots(self, plots=None, t=-1):
        plots = plots or self.plots
        plots = plots if isinstance(plots, list) else [plots]
        for plot in plots:
            if 'figure' not in plot:
                self._init_plot(plot)
            self._update_plot(plot, t)

    def _update_plot(self, plot, t=-1):
        argument, kwargs = plot['argument'], plot['kwargs']
        data = self.update_plot(argument, t, **kwargs)
        if data is None:
            return
        fig, info = plot['figure'], plot['info']
        ax_r, ax_c = len(data), len(data[0])
        for k in range(ax_r):
            for l in range(ax_c):
                axis = fig.axes[k*ax_c+l]
                if 'projection' in info[k][l] and info[k][l]['projection'] == '3d':
                    _update_axis_3d(axis, info[k][l], data[k][l])
                else:
                    _update_axis_2d(axis, info[k][l], data[k][l])
        if 'axis' in kwargs and not kwargs['axis']:
            plt.axis('off')
        fig.canvas.draw()

    # ========================================================================
    # Post processing operations
    # ========================================================================

    def save_plot(self, argument=None, name='plot', path='images/', **kwargs):
        from matplotlib2tikz import save as tikz_save
        directory = path
        if not os.path.isdir(directory):
            os.makedirs(directory)
        plot = self.plot(argument, **kwargs)
        info = plot['info']
        proj_3d = False
        for k, _ in enumerate(info):
            for l, _ in enumerate(info[0]):
                if 'projection' in info[k][l] and info[k][l]['projection'] == '3d':
                    proj_3d = True
        if proj_3d:
            warnings.warn('3D plotting is not supported by matplotlib2tikz. ' +
                          'Saving to pdf instead.')
            path = directory+'/'+name+'.pdf'
            plt.savefig(path, bbox_inches=0)
        else:
            figurewidth = kwargs[
                'figurewidth'] if 'figurewidth' in kwargs else '8cm'
            figureheight = kwargs[
                'figureheight'] if 'figureheight' in kwargs else None
            path = directory+'/'+name+'.tikz'
            if figureheight is None:
                tikz_save(path, figurewidth=figurewidth)
            else:
                tikz_save(
                    path, figurewidth=figurewidth, figureheight=figureheight)
            _cleanup_rubbish(path)

    def plot_movie(self, argument=None, repeat=False, **kwargs):
        t = self.__class__.simulator.time
        if ('number_of_frames' in kwargs and kwargs['number_of_frames'] <= (len(t)-1)):
            number_of_frames = kwargs['number_of_frames']
        else:
            number_of_frames = len(t)-1
        subsample = (len(t)-1)/(number_of_frames-1)
        kwargs['no_update'] = True
        plot = self.plot(argument, **kwargs)
        while True:
            for k in range(0, len(t)-1, subsample):
                self.update_plots(plot, k)
            if not repeat:
                break

    def save_movie(self, argument=None, name='movie', path='movies/', **kwargs):
        from matplotlib2tikz import save as tikz_save
        directory = path + name
        if not os.path.isdir(directory):
            os.makedirs(directory)
        t = self.__class__.simulator.time
        if ('number_of_frames' in kwargs and kwargs['number_of_frames'] <= (len(t)-1)):
            number_of_frames = kwargs['number_of_frames']
        else:
            number_of_frames = len(t)-1
        root = kwargs['root'] if 'root' in kwargs else None
        subsample = (len(t)-2)/(number_of_frames-1)
        kwargs['no_update'] = True
        plot = self.plot(argument, **kwargs)
        figurewidth = kwargs[
            'figurewidth'] if 'figurewidth' in kwargs else '8cm'
        figureheight = kwargs[
            'figureheight'] if 'figureheight' in kwargs else None
        cnt = 0
        proj_3d = False
        for k in range(0, len(t)-1, subsample):
            self.update_plots(plot, k)
            if not proj_3d:
                info = plot['info']
                for inf in info:
                    for i in inf:
                        if 'projection' in i and i['projection'] == '3d':
                            proj_3d = True
                if proj_3d:
                    warnings.warn('3D plotting is not supported by matplotlib2tikz. ' +
                                  'Saving to pdf instead.')
            if proj_3d:
                path = directory+'/'+name+'_'+str(cnt)+'.pdf'
                plt.savefig(path, bbox_inches=0)
            else:
                path = directory+'/'+name+'_'+str(cnt)+'.tikz'
                if figureheight is None:
                    tikz_save(path, figurewidth=figurewidth)
                else:
                    tikz_save(
                        path, figurewidth=figurewidth, figureheight=figureheight)
                _cleanup_rubbish(path, root)
            cnt += 1
