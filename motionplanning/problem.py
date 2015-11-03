from vehicle import *
from environment import *
from casadi import *
from casadi.tools  import *
from codegeneration import *
from plots import *

class Problem:
    def __init__(self, vehicle, environment, time, options = {}):
        self.setOptions(options)
        self.vehicle        = vehicle
        self.vehicle.initialize(time)
        self.environment    = environment
        self.horizon_time   = time['horizon_time']
        self.update_time    = time['update_time']
        self.sample_time    = time['sample_time']
        self.knot_intervals = time['knot_intervals']
        self.knot_time      = (int(self.horizon_time*1000)/self.knot_intervals)/1000.
        self.createStructs()
        self.createProblem()
        self.initVariables()
        self.plot           = Plots([self.vehicle], self.environment)

    def setOptions(self, options):
        self.col_safety     = options['col_safety'] if 'col_safety' in options else False
        self.codegen_opt    = options['codegen']

    def run(self):
        current_time = 0.
        # Plots
        proceed = True
        while proceed:
            # Solve problem
            self.solve(current_time)
            # Update vehicle and environment
            self.update(current_time)
            current_time += self.update_time
            self.plot.update()
            # Check termination criteria
            proceed = False
            # if la.norm(self.vehicle.prediction['dy'])> 1e-2:
            if la.norm(self.vehicle.prediction['y'] - self.vehicle.yT) > 1e-3:
                proceed = True
        self.final()

    def createProblem(self):
        raise NotImplementedError('Please implement this method!')
    def createStructs(self):
        raise NotImplementedError('Please implement this method!')
    def initVariables(self):
        raise NotImplementedError('Please implement this method!')
    def solve(self):
        raise NotImplementedError('Please implement this method!')
    def final(self):
        raise NotImplementedError('Please implement this method!')

