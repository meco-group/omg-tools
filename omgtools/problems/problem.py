from ..basics.optilayer import OptiFather, OptiChild
from ..vehicles.fleet import get_fleet_vehicles
import time

class Problem(OptiChild):

    def __init__(self, fleet, environment, options={}, label='problem'):
        OptiChild.__init__(self, label)
        self.fleet, self.vehicles = get_fleet_vehicles(fleet)
        self.environment = environment
        self.set_default_options()
        self.set_options(options)
        self.iteration = 0
        self.update_times = []

    # ========================================================================
    # Problem options
    # ========================================================================

    def set_default_options(self):
        self.options = {'verbose': 2, 'update_time': 0.1}
        self.options['solver'] = {'tol': 1e-3, 'linear_solver': 'mumps',
                                  'warm_start_init_point': 'yes',
                                  'print_level': 0, 'print_time': 0}
        self.options['codegen'] = {
            'jit': False, 'jit_options': {'flags': ['-O0']}}

    def set_options(self, options):
        if 'solver' in options:
            self.options['solver'].update(options['solver'])
        if 'codegen' in options:
            self.options['codegen'].update(options['codegen'])
        for key in options:
            if key not in ['solver', 'codegen']:
                self.options[key] = options[key]

    # ========================================================================
    # Create and solve problem
    # ========================================================================

    def init(self):
        self.environment.add_vehicle(self.vehicles)
        children = [vehicle for vehicle in self.vehicles]
        children += [obstacle for obstacle in self.environment.obstacles]
        children += [self, self.environment]
        self.father = OptiFather(children)
        self.problem, compile_time = self.father.construct_problem(
            self.options)
        self.father.init_transformations(self.init_primal_transform,
                                         self.init_dual_transform)

    def solve(self, current_time):
        self.init_step(current_time)
        # set initial guess, parameters, lb & ub
        var = self.father.get_variables()
        par = self.father.set_parameters(current_time)
        lb, ub = self.father.update_bounds(current_time)
        # solve!
        t0 = time.time()
        self.problem({'x0': var, 'p': par, 'lbg': lb, 'ubg': ub})
        t1 = time.time()
        t_upd = t1-t0
        self.father.set_variables(self.problem.getOutput('x'))
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
    # Methods encouraged to override
    # ========================================================================

    def init_step(self, current_time):
        pass

    def final(self):
        pass

    def initialize(self):
        pass

    def init_primal_transform(self, basis):
        return None

    def init_dual_transform(self, basis):
        return None

    def set_parameters(self, time):
        return {}

    # ========================================================================
    # Methods required to override
    # ========================================================================

    def update(self, current_time):
        raise NotImplementedError('Please implement this method!')

    def stop_criterium(self):
        raise NotImplementedError('Please implement this method!')

    # ========================================================================
    # Methods for exporting problem to C++ library
    # ========================================================================

    def export(self, language='c++', options={}):
        if not hasattr(self, 'father'):
            self.init()
        if language == 'c++':
            export.ExportCpp(self, 'point2point', options)
        else:
            raise ValueError(language+' export is not implemented.')
