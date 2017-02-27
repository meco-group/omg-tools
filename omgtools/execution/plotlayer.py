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
import shutil
import matplotlib
matplotlib.use('Cairo')
print matplotlib.__version__
print matplotlib.__version__
print matplotlib.__version__
print matplotlib.__version__
import matplotlib.pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D, proj3d
import numpy as np
import warnings


# def orthogonal_proj(zfront, zback):
#     a = (zfront+zback)/(zfront-zback)
#     b = -2*(zfront*zback)/(zfront-zback)
#     return np.array([[1,0,0,0],
#                         [0,1,0,0],
#                         [0,0,a,b],
#                         [0,0,0,zback]])
# proj3d.persp_transformation = orthogonal_proj


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
        axis.plot([], [], **line)[0]


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
    if 'aspect_equal' in info and info['aspect_equal']:
        # hack due to bug in matplotlib3d
        limits = []
        if 'xlim' in info:
            limits.append(info['xlim'])
        else:
            limits.append(axis.get_xlim3d())
        if 'ylim' in info:
            limits.append(info['ylim'])
        else:
            limits.append(axis.get_ylim3d())
        if 'zlim' in info:
            limits.append(info['zlim'])
        else:
            limits.append(axis.get_zlim3d())
        ranges = [abs(lim[1] - lim[0]) for lim in limits]
        centra = [np.mean(lim) for lim in limits]
        radius = 0.5*max(ranges)
        axis.set_xlim3d([centra[0] - radius, centra[0] + radius])
        axis.set_ylim3d([centra[1] - radius, centra[1] + radius])
        axis.set_zlim3d([centra[2] - radius, centra[2] + radius])


def _cleanup_rubbish(path, info, root=None):
    # cleanup rubbish due to bugs in matplotlib2tikz
    with open(path, 'r+') as f:
        body = f.read()
        # matplotlib2tikz's \path lines are buggy and comprise errors.
        # Let's remove them and replace them with our own.
        index = body.find('\path [draw=black, fill opacity=0]')
        while index >= 0:
            body = body.replace(body[index:].split(';')[0]+';', '')
            index = body.find('\path [draw=black, fill opacity=0]')
        index = 0
        ax_r, ax_c = len(info), len(info[0])
        for k in range(ax_r):
            for l in range(ax_c):
                insert = ''
                index = body.find('\end{axis}', index)
                if 'xlim' in info[k][l] and info[k][l]['xlim'] is not None:
                    x_min = info[k][l]['xlim'][0]
                    x_max = info[k][l]['xlim'][1]
                    insert += ('\n\path [draw=black, opacity=0] ' +
                               '(axis cs:'+str(x_min)+',0)--' +
                               '(axis cs:'+str(x_max)+',0);')
                if 'ylim' in info[k][l] and info[k][l]['ylim'] is not None:
                    y_min = info[k][l]['ylim'][0]
                    y_max = info[k][l]['ylim'][1]
                    insert += ('\n\path [draw=black, opacity=0] ' +
                               '(axis cs:0,'+str(y_min)+')--' +
                               '(axis cs:0,'+str(y_max)+');')
                insert += '\n'
                body = body[:index] + insert + body[index:]
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
        plot = {'argument': argument, 'kwargs': kwargs, 'figure': plt.figure()}
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
        figure = plot['figure']
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
        plot.update({'info': info})

    # ========================================================================
    # Plot update
    # ========================================================================

    def update_plots(self, plots=None, t=-1):
        plots = plots or self.plots
        plots = plots if isinstance(plots, list) else [plots]
        for plot in plots:
            if 'info' not in plot:
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
            plt.savefig(path, bbox_inches='tight', pad_inches=0)
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
            _cleanup_rubbish(path, info)

    def plot_movie(self, argument=None, repeat=False, **kwargs):
        t = self.__class__.simulator.time
        if ('number_of_frames' in kwargs and kwargs['number_of_frames'] <= (len(t)-1)):
            number_of_frames = kwargs['number_of_frames']
        else:
            number_of_frames = len(t)-1
        subsample = (len(t)-1)/(number_of_frames-1)
        indices = range(0, len(t)-1, subsample)
        indices.extend([len(t)-1 for k in range(number_of_frames-len(indices))])
        kwargs['no_update'] = True
        plot = self.plot(argument, **kwargs)
        while True:
            for k in indices:
                self.update_plots(plot, k)
            if not repeat:
                break

    def save_movie(self, argument=None, name='movie', path='movies/', format='tikz', **kwargs):
        t = self.__class__.simulator.time
        if ('number_of_frames' in kwargs and kwargs['number_of_frames'] <= (len(t)-1)):
            number_of_frames = kwargs['number_of_frames']
        else:
            number_of_frames = len(t)-1
        if 'movie_time' in kwargs:
            interval = kwargs['movie_time']/(number_of_frames-1)
        else:
            interval = 10./(number_of_frames-1)
        subsample = (len(t)-1)/(number_of_frames-1)
        indices = range(0, len(t)-1, subsample)
        indices.extend([len(t)-1 for k in range(number_of_frames-len(indices))])
        kwargs['no_update'] = True
        plot = self.plot(argument, **kwargs)
        directory = path+'/'+name
        if not os.path.isdir(directory):
            os.makedirs(directory)
        cnt = 0
        if format == 'gif':
            for k in indices:
                self.update_plots(plot, k)
                output = os.path.join(directory, name+'_'+str(cnt)+'.png')
                cnt += 1
                plt.savefig(output, bbox_inches='tight', pad_inches=0)
            filenames = [os.path.join(directory, name+'_'+str(k)+'.png') for k in range(cnt)]
            output = os.path.join(path, name+'.gif')
            os.system('convert -delay %f %s %s' % (interval, ' '.join(filenames), output))
            shutil.rmtree(directory)
        elif format == 'tikz':
            from matplotlib2tikz import save as tikz_save
            root = kwargs['root'] if 'root' in kwargs else None
            figurewidth = kwargs[
                'figurewidth'] if 'figurewidth' in kwargs else '8cm'
            figureheight = kwargs[
                'figureheight'] if 'figureheight' in kwargs else None
            # by default scale only axis is true, but you can disable it
            # this option makes sure the plot has the provided width and height
            scaleonlyaxis = kwargs[
                'scaleonlyaxis'] if 'scaleonlyaxis' in kwargs else True
            cnt = 0
            proj_3d = False
            for k in range(0, len(t)-1, subsample):
                self.update_plots(plot, k)
                info = plot['info']
                if not proj_3d:
                    for inf in info:
                        for i in inf:
                            if 'projection' in i and i['projection'] == '3d':
                                proj_3d = True
                    if proj_3d:
                        warnings.warn('3D plotting is not supported by matplotlib2tikz. ' +
                                      'Saving to pdf instead.')
                if proj_3d:
                    path = directory+'/'+name+'_'+str(cnt)+'.pdf'
                    plt.savefig(path, bbox_inches='tight', pad_inches=0)
                else:
                    path = directory+'/'+name+'_'+str(cnt)+'.tikz'
                    if figureheight is None:
                        tikz_save(path, figurewidth=figurewidth,
                                  extra={'scale only axis='+str(scaleonlyaxis).lower()})
                        # tikz requires a lowercase true/false
                    else:
                        tikz_save(
                            path, figurewidth=figurewidth, figureheight=figureheight,
                            extra={['scale only axis='+str(scaleonlyaxis).lower()]})
                        # tikz requires a lowercase true/false
                    _cleanup_rubbish(path, info, root)
                cnt += 1
