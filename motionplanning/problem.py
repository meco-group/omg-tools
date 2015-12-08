from optilayer import OptiLayer
from fleet import get_fleet_vehicles
from plots import Plots
import time


class Simulator:

    def __init__(self, problem, options={}):
        self.set_default_options()
        self.set_options(options)
        self.problem = problem
        self.plot = Plots(problem.fleet, problem.environment)

    def set_default_options(self):
        self.options = {'update_time': 0.1}

    def set_options(self, options):
        self.options.update(options)

    def run(self):
        current_time = 0.
        stop = False
        while not stop:
            stop = self.update(current_time)
            current_time += self.options['update_time']
        self.problem.final()

    def update(self, current_time):
        # solve problem
        self.problem.solve(current_time)
        # update vehicle(s) and environment
        self.problem.update(current_time, self.options['update_time'])
        self.plot.update()
        # check termination criteria
        stop = self.problem.stop_criterium()
        return stop


class Problem(OptiLayer):

    def __init__(self, fleet, environment, options={}, **kwargs):
        self.fleet, self.vehicles = get_fleet_vehicles(fleet)
        self.environment = environment
        self.environment.add_vehicle(self.vehicles)
        if 'opti_name' in kwargs:
            OptiLayer.__init__(self, kwargs['opti_name'])
        else:
            OptiLayer.__init__(self, 'problem')
        self.set_default_options()
        self.set_options(options)
        self.iteration = 0
        self.update_times = []

    # ========================================================================
    # Problem options
    # ========================================================================

    def set_default_options(self):
        self.options = {'verbose': 1, 'update_time': 0.1}
        self.options['casadi'] = {'solver': 'ipopt', 'tol': 1e-3,
                                  'linear_solver': 'ma57',
                                  'warm_start_init_point': 'yes',
                                  'print_level': 0, 'print_time': 0}
        self.options['codegen'] = {
            'codegen': True, 'compileme': True, 'buildname': 'problem'}

    def set_options(self, options):
        self.options.update(options)

    # ========================================================================
    # Create and solve problem
    # ========================================================================

    def solve(self, current_time):
        self.current_time = current_time
        self.init_step()
        # set parameters and initial guess
        self.problem.setInput(OptiLayer.get_variables(), 'x0')
        self.problem.setInput(OptiLayer.get_parameters(current_time), 'p')
        # set lb & ub
        lb, ub = OptiLayer.update_bounds(current_time)
        self.problem.setInput(lb, 'lbg')
        self.problem.setInput(ub, 'ubg')
        # solve!
        t0 = time.time()
        self.problem.evaluate()
        t1 = time.time()
        t_upd = t1-t0
        OptiLayer.set_variables(self.problem.getOutput('x'))
        stats = self.problem.getStats()
        if stats.get("return_status") != "Solve_Succeeded":
            print stats.get("return_status")
        # print
        if self.options['verbose'] >= 1:
            self.iteration += 1
            if ((self.iteration-1) % 20 == 0):
                print "----|------------|------------"
                print "%3s | %10s | %10s " % ("It", "t upd", "time")
                print "----|------------|------------"
            print "%3d | %.4e | %.4e " % (self.iteration, t_upd, current_time)
        self.update_times.append(t_upd)

    # ========================================================================
    # Methods encouraged to override (very basic implementation)
    # ========================================================================

    def init_step(self):
        pass

    def init_variables(self):
        return {}

    def get_parameters(self, time):
        return {}

    def final(self):
        pass

    # ========================================================================
    # Methods required to override (no general implementation possible)
    # ========================================================================

    def update(self, current_time):
        raise NotImplementedError('Please implement this method!')

    def stop_criterium(self):
        raise NotImplementedError('Please implement this method!')
