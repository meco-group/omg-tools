import os
import matplotlib
import matplotlib.pyplot as plt
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
        if signal == 'scene':
            plots = [{'signal': signal, 'vehicles': vehicles,
                      'type': '2d', 'kwargs': kwargs}]
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
        canvas_lim = self.environment.get_canvas_limits()
        plt.xlim(canvas_lim[0][0], canvas_lim[0][1])
        plt.ylim(canvas_lim[1][0], canvas_lim[1][1])
        plt_2d = {}
        plt_2d['environment'] = [
            axis.plot([], [], 'k-')[0] for l in range(self.environment.n_obs)]
        plt_2d['pos_traj'] = [axis.plot(
            [], [], '-', color=self.col_w[veh])[0] for veh in vehicles]
        plt_2d['pos_sign'] = [axis.plot(
            [], [], '-', color=self.col[veh])[0] for veh in vehicles]
        plt_2d['vehicle'] = [[axis.plot([], [], '-', color=self.col[veh])[0] for shape in veh.shapes] for veh in vehicles]
        # plt_2d['vehicle'] = [
        #     axis.plot([], [], '-', color=self.col[veh])[0] for veh in vehicles]
        return {'figure': figure, 'axis': axis, 'plt_2d': plt_2d}

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
        for l, env in enumerate(environment):
            plt_2d['environment'][l].set_data(
                env[0, :].ravel(), env[1, :].ravel())
        for l, veh in enumerate(vehicles):
            if t == -1:
                pos_sign = veh.signals['position'][:, :]
            else:
                pos_sign = veh.signals['position'][:, :t+1]
            pos_traj = veh.traj_storage['position'][t]
            plt_2d['pos_traj'][l].set_data(pos_traj[0, :].ravel(),
                                           pos_traj[1, :].ravel())
            plt_2d['pos_sign'][l].set_data(pos_sign[0, :].ravel(),
                                           pos_sign[1, :].ravel())
            veh_cnt = veh.draw(t)
            for k, shape in enumerate(veh.shapes):
                plt_2d['vehicle'][l][k].set_data(veh_cnt[k][0].ravel(),
                                                 veh_cnt[k][1].ravel())
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
        self.show(signal, **kwargs)
        figurewidth = kwargs[
            'figurewidth'] if 'figurewidth' in kwargs else '8cm'
        tikz_save(directory+'/'+name+'.tikz', figurewidth=figurewidth)

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
        figwidth = kwargs['figurewidth'] if 'figurewidth' in kwargs else '8cm'
        subsample = (t.shape[1]-2)/(number_of_frames-1)
        kwargs['no_update'] = True
        plot = self.show(signal, **kwargs)
        cnt = 0
        if signal in ('scene'):
            plt.axis('off')
        for k in range(0, t.shape[1]-1, subsample):
            self.update(k, plots=plot)
            tikz_save(
                directory+'/'+name+'_'+str(cnt)+'.tikz', figurewidth=figwidth)
            cnt += 1
