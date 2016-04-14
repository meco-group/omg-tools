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
import numpy as np
matplotlib.use('TKAgg')

# color definition (rgb)
blue = [17., 110., 138.]
red = [138.,  31.,  17.]
green = [17., 138.,  19.]
lightblue = [106., 194., 238.]


def mix_with_white(color, perc_white=80.):
    r, g, b = color[0], color[1], color[2]
    r_m = ((100. - perc_white)*r + perc_white)/100.
    g_m = ((100. - perc_white)*g + perc_white)/100.
    b_m = ((100. - perc_white)*b + perc_white)/100.
    return [r_m, g_m, b_m]


class Plots:

    def __init__(self, problem, options={}):
        self.fleet = problem.fleet
        self.vehicles = self.fleet.vehicles
        self.environment = problem.environment

        self.varia = {}
        self.plots = []

        # default colors
        self.set_color_template([blue, red, green, lightblue])

        self.set_options(options)
        plt.ion()
        plt.show()

    def set_color_template(self, colors):
        colors = [[c/255. for c in color] for color in colors]
        ind = 0
        while self.fleet.N > len(colors):
            colors.append(colors[ind])
            ind += 1
        self.col = {veh: colors[l] for l, veh in enumerate(self.vehicles)}
        self.col_w = {veh: mix_with_white(c) for veh, c in self.col.items()}

    # ========================================================================
    # Plot options
    # ========================================================================

    def set_options(self, options):
        if 'knots' in options and options['knots']:
            knots = {'symbol': 'x'}
            knots['time'] = lambda veh, t: veh.traj_storage_kn['time'][t]
            knots['data'] = lambda veh, signal, k, t: veh.traj_storage_kn[
                signal][t][k, :]
            self.varia['knots'] = knots
        if 'prediction' in options and options['prediction']:
            prediction = {'symbol': 'o'}
            prediction['time'] = lambda veh, t: veh.signals['time'][:, t]
            prediction['data'] = lambda veh, signal, k, t: veh.pred_storage[
                signal][t][k]
            self.varia['prediction'] = prediction

    # ========================================================================
    # Plot creation and initialization
    # ========================================================================

    def show(self, signal, **kwargs):
        vehicles = kwargs[
            'vehicles'] if 'vehicles' in kwargs else self.vehicles
        if signal == 'scene' and self.environment.n_dim == 2:
            plots = [{'signal': signal, 'vehicles': vehicles,
                      'type': '2d', 'kwargs': kwargs}]
        elif signal == 'scene' and self.environment.n_dim == 3:
            plots = [{'signal': signal, 'vehicles': vehicles,
                      'type': '3d', 'kwargs': kwargs}]
        else:
            vehicle_types = self._sort_vehicles(vehicles)
            plots = []
            for vehicle_type, vehicles in vehicle_types.items():
                plots.append({'signal': signal, 'vehicles': vehicles,
                              'type': 'curve', 'kwargs': kwargs})
        self.plots.extend(plots)
        if 'time' in kwargs:
            index = self._time2index(self.vehicles[0], kwargs['time'])
        elif 'index' in kwargs:
            index = kwargs['index']
        else:
            index = -1
        no_update = kwargs['no_update'] if 'no_update' in kwargs else False
        if hasattr(self.vehicles[0], 'signals') and not no_update:
            self.update(t=index, plots=plots)
        return plots

    def init(self, plot):
        if plot['type'] == '2d':
            plot.update(self._init_2d_plot(plot))
        elif plot['type'] == '3d':
            plot.update(self._init_3d_plot(plot))
        else:
            plot.update(self._init_curve_plot(plot))

    def _init_curve_plot(self, plot):
        signal, vehicles, kwargs = plot[
            'signal'], plot['vehicles'], plot['kwargs']
        plt_traj, plt_sign, plt_varia = [], [], {}
        for key in self.varia.keys():
            plt_varia[key] = []
        size = vehicles[0].signals[signal].shape[0]
        if 'label' in kwargs:
            if not isinstance(kwargs['label'], list):
                label = [kwargs['label']]
            else:
                label = kwargs['label']
        else:
            if size > 1:
                label = [signal+str(k) for k in range(size)]
            else:
                label = [signal]
        figure, axis = plt.subplots(size, 1, squeeze=False)
        for k in range(size):
            axis[k, 0].set_xlabel('t (s)')
            axis[k, 0].set_ylabel(label[k])
        plt_traj = [[axis[k, 0].plot([], [], '-', color=self.col_w[veh])[0]
                     for k in range(size)] for veh in vehicles]
        plt_sign = [[axis[k, 0].plot([], [], '-', color=self.col[veh])[0]
                     for k in range(size)] for veh in vehicles]
        for key, varia in self.varia.items():
            plt_varia[key] = [[axis[k, 0].plot([], [], varia['symbol'],
                                               color=self.col[veh])[0]
                               for k in range(size)]
                              for veh in vehicles]
        return {'figure': figure, 'axis': axis, 'plt_traj': plt_traj,
                'plt_sign': plt_sign, 'plt_varia': plt_varia, 'size': size}

    def _init_2d_plot(self, plot):
        vehicles = plot['vehicles']
        figure = plt.figure()
        axis = figure.add_subplot(111)
        # axis.set_aspect('equal')
        if 'limits' in plot['kwargs']:
            canvas_lim = plot['kwargs']['limits']
        else:
            canvas_lim = self.environment.get_canvas_limits()
        plt.xlim(canvas_lim[0][0], canvas_lim[0][1])
        plt.ylim(canvas_lim[1][0], canvas_lim[1][1])
        plt_2d = {}
        plt_2d['environment'] = [[axis.plot(
            [], [], 'k-')[0] for line in obst.draw()] for obst in self.environment.obstacles]
        plt_2d['environment'].append([axis.plot(
            [], [], 'k-')[0] for line in self.environment.room['shape'].draw()])  # plot room shape
        plt_2d['pos_traj'] = [axis.plot(
            [], [], '-', color=self.col_w[veh])[0] for veh in vehicles]
        plt_2d['pos_sign'] = [axis.plot(
            [], [], '-', color=self.col[veh])[0] for veh in vehicles]
        plt_2d['vehicle'] = [[axis.plot([], [], '-', color=self.col[veh])[0]
                              for line in veh.draw()] for veh in vehicles]
        return {'figure': figure, 'axis': axis, 'plt_2d': plt_2d}

    def _init_3d_plot(self, plot):
        vehicles = plot['vehicles']
        figure = plt.figure()
        axis = figure.add_subplot(111, projection='3d')
        axis.set_aspect('equal', adjustable='box')
        if 'limits' in plot['kwargs']:
            canvas_lim = plot['kwargs']['limits']
        else:
            canvas_lim = self.environment.get_canvas_limits()
        axis.set_xlim(canvas_lim[0][0], canvas_lim[0][1])
        axis.set_ylim(canvas_lim[1][0], canvas_lim[1][1])
        axis.set_zlim(canvas_lim[2][0], canvas_lim[2][1])
        if 'view' in plot['kwargs']:
            elevation, azimuth = plot['kwargs']['view']
            axis.view_init(elev=elevation, azim=azimuth)
        plt_3d = {}
        plt_3d['environment'] = [[axis.plot(
            [], [], [], 'k-')[0] for line in obst.draw()] for obst in self.environment.obstacles]
        plt_3d['pos_traj'] = [axis.plot(
            [], [], [], '-', color=self.col_w[veh])[0] for veh in vehicles]
        plt_3d['pos_sign'] = [axis.plot(
            [], [], [], '-', color=self.col[veh])[0] for veh in vehicles]
        plt_3d['vehicle'] = [[axis.plot([], [], [], '-', color=self.col[veh])[0]
                              for line in veh.draw()] for veh in vehicles]
        return {'figure': figure, 'axis': axis, 'plt_3d': plt_3d}

    def _sort_vehicles(self, vehicles):
        vehicle_types = {}
        for vehicle in vehicles:
            veh_type = vehicle.__class__.__name__
            if veh_type in vehicle_types:
                vehicle_types[veh_type].append(vehicle)
            else:
                vehicle_types[veh_type] = [vehicle]
        return vehicle_types

    # ========================================================================
    # Plot update
    # ========================================================================

    def update(self, t=-1, plots=None):
        plots = self.plots if (plots is None) else plots
        plots = plots if isinstance(plots, list) else [plots]
        for plot in plots:
            if 'figure' not in plot:
                self.init(plot)
            if plot['type'] == 'curve':
                self._update_curve_plot(plot, t)
            if plot['type'] == '2d':
                self._update_2d_plot(plot, t)
            if plot['type'] == '3d':
                self._update_3d_plot(plot, t)

    def _update_curve_plot(self, plot, t=-1):
        plt_traj = plot['plt_traj']
        plt_sign = plot['plt_sign']
        plt_varia = plot['plt_varia']
        size = plot['size']
        signal, vehicles = plot['signal'], plot['vehicles']
        for l, veh in enumerate(vehicles):
            for k in range(size):
                plt_traj[l][k].set_data(
                    veh.traj_storage['time'][t].ravel(),
                    veh.traj_storage[signal][t][k, :].ravel())
                if t == -1:
                    plt_sign[l][k].set_data(
                        veh.signals['time'].ravel(),
                        veh.signals[signal][k, :].ravel())
                else:
                    plt_sign[l][k].set_data(
                        veh.signals['time'][:, :t+1].ravel(),
                        veh.signals[signal][k, :t+1].ravel())
                for key, varia in self.varia.items():
                    plt_varia[key][l][k].set_data(
                        varia['time'](veh, t).ravel(),
                        varia['data'](veh, signal, k, t).ravel())
        fig, ax = plot['figure'], plot['axis']
        for i in range(ax.shape[0]):
            for j in range(ax.shape[1]):
                ax[i, j].relim()
                ax[i, j].autoscale_view(True, True, True)
        fig.canvas.draw()

    def _update_2d_plot(self, plot, t=-1):
        plt_2d, vehicles = plot['plt_2d'], plot['vehicles']
        environment = self.environment.draw(t)
        for e, env in enumerate(environment):
            for l, line in enumerate(env):
                plt_2d['environment'][e][l].set_data(
                    line[0, :].ravel(), line[1, :].ravel())
        for v, veh in enumerate(vehicles):
            if t == -1:
                pos_sign = veh.signals['pose'][:, :]
            else:
                pos_sign = veh.signals['pose'][:, :t+1]
            pos_traj = veh.traj_storage['pose'][t]
            plt_2d['pos_traj'][v].set_data(pos_traj[0, :].ravel(),
                                           pos_traj[1, :].ravel())
            plt_2d['pos_sign'][v].set_data(pos_sign[0, :].ravel(),
                                           pos_sign[1, :].ravel())
            veh_cnt = veh.draw(t)
            for l, line in enumerate(veh_cnt):
                plt_2d['vehicle'][v][l].set_data(
                    line[0, :].ravel(), line[1, :].ravel())
        plot['figure'].canvas.draw()

    def _update_3d_plot(self, plot, t=-1):
        plt_3d, vehicles = plot['plt_3d'], plot['vehicles']
        environment = self.environment.draw(t)
        for e, env in enumerate(environment):
            for l, line in enumerate(env):
                plt_3d['environment'][e][l].set_data(
                    line[0, :].ravel(), line[1, :].ravel())
                plt_3d['environment'][e][
                    l].set_3d_properties(line[2, :].ravel())
        for v, veh in enumerate(vehicles):
            if t == -1:
                pos_sign = veh.signals['pose'][:, :]
            else:
                pos_sign = veh.signals['pose'][:, :t+1]
            pos_traj = veh.traj_storage['pose'][t]
            plt_3d['pos_traj'][v].set_data(pos_traj[0, :].ravel(),
                                           pos_traj[1, :].ravel())
            plt_3d['pos_traj'][v].set_3d_properties(pos_traj[2, :].ravel())
            plt_3d['pos_sign'][v].set_data(pos_sign[0, :].ravel(),
                                           pos_sign[1, :].ravel())
            plt_3d['pos_sign'][v].set_3d_properties(pos_sign[2, :].ravel())
            veh_cnt = veh.draw(t)
            for l, line in enumerate(veh_cnt):
                plt_3d['vehicle'][v][l].set_data(
                    line[0, :].ravel(), line[1, :].ravel())
                plt_3d['vehicle'][v][l].set_3d_properties(line[2, :].ravel())
        plot['figure'].canvas.draw()

    # ========================================================================
    # Post processing operations
    # ========================================================================

    def _time2index(self, vehicle, time):
        time_axis = vehicle.signals['time']
        Ts = time_axis[:, 1] - time_axis[:, 0]
        for k, t in enumerate(time_axis[0, :]):
            t = np.round(t, 6)
            if (t <= time) and (time < (t+Ts)) and ((time-t) <= (t+Ts-time)):
                return k

    def save(self, signal, name='plot', path='images/', **kwargs):
        from matplotlib2tikz import save as tikz_save
        directory = path
        if not os.path.isdir(directory):
            os.makedirs(directory)
        plot = self.show(signal, **kwargs)[0]
        if signal == 'scene':
            plt.axis('off')
        if plot['type'] == '3d':
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
            self._cleanup_rubbish(path)

    def show_movie(self, signal, repeat=False, **kwargs):
        t = self.vehicles[0].signals['time']
        if ('number_of_frames' in kwargs and
                kwargs['number_of_frames'] <= (t.shape[1]-1)):
            number_of_frames = kwargs['number_of_frames']
        else:
            number_of_frames = t.shape[1]-1
        subsample = (t.shape[1]-1)/(number_of_frames-1)
        kwargs['no_update'] = True
        plot = self.show(signal, **kwargs)
        while True:
            for k in range(0, t.shape[1]-1, subsample):
                self.update(k, plots=plot)
            if not repeat:
                break

    def save_movie(self, signal, name='movie', path='movies/', **kwargs):
        from matplotlib2tikz import save as tikz_save
        directory = path + name
        if not os.path.isdir(directory):
            os.makedirs(directory)
        t = self.vehicles[0].signals['time']
        if ('number_of_frames' in kwargs and
                kwargs['number_of_frames'] <= (t.shape[1]-1)):
            number_of_frames = kwargs['number_of_frames']
        else:
            number_of_frames = t.shape[1]-1
        root = kwargs['root'] if 'root' in kwargs else None
        subsample = (t.shape[1]-2)/(number_of_frames-1)
        kwargs['no_update'] = True
        plots = self.show(signal, **kwargs)
        if plots[0]['type'] == '3d':
            print 'ho'
            warnings.warn('3D plotting is not supported by matplotlib2tikz. ' +
                          'Saving to pdf instead.')
        else:
            figurewidth = kwargs[
                'figurewidth'] if 'figurewidth' in kwargs else '8cm'
            figureheight = kwargs[
                'figureheight'] if 'figureheight' in kwargs else None
        cnt = 0
        for k in range(0, t.shape[1]-1, subsample):
            self.update(k, plots=plots)
            if signal == 'scene':
                plt.axis('off')
            if plots[0]['type'] == '3d':
                path = directory+'/'+name+'_'+str(cnt)+'.pdf'
                plt.savefig(path, bbox_inches=0)
            else:
                path = directory+'/'+name+'_'+str(cnt)+'.tikz'
                if figureheight is None:
                    tikz_save(path, figurewidth=figurewidth)
                else:
                    tikz_save(
                        path, figurewidth=figurewidth, figureheight=figureheight)
                self._cleanup_rubbish(path, root)
            cnt += 1

    def _cleanup_rubbish(self, path, root=None):
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
