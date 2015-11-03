from spline_extra import *
from shape import *
from casadi import *
from casadi.tools import *
from plant import *

class Vehicle:

    def __init__(self, index, shape, degree, options):
        self.setOptions(options)
        self.ind            = index
        self.shape          = shape
        self.degree         = degree

        self.trajectory     = {}
        self.trajectory['knots'] = {}
        self.prediction     = {}
        self.path           = {}
        self.trajectories   = {'time':[], 'y':[], 'dy':[], 'input':[], 'state':[], 'knots': {}}
        self.trajectories['knots'] = {'time':[], 'y':[], 'dy':[], 'input':[], 'state':[]}
        self.predictions    = {'y':[], 'dy':[], 'input':[], 'state':[]}
        self.plant          = Plant(self.updateReal, self.updateModel) if self.model_mismatch else Plant(self.updateModel, self.updateModel)

    def initialize(self, time, **kwargs):
        if 'knots' in kwargs:
            self.knots = kwargs['knots']
        else:
            self.knots = np.r_[np.zeros(self.degree), np.linspace(0., 1., time['knot_intervals']+1), np.ones(self.degree)]
        self.Basis  = BSplineBasis(self.knots, self.degree)
        self.plant.setDisturbance(self.getDisturbance(time['sample_time']))

    def setNeighbours(self, neighbours, **kwargs):
        self.neighbours = neighbours
        if 'rel_pos' in kwargs:
            self.rel_pos = kwargs['rel_pos']
        self.Nn         = len(neighbours)
        nghb_list       = [neighbour.ind for neighbour in self.neighbours]
        for i in range(len(self.ind_nghb)):
            for l in range(self.Nn):
                if nghb_list[l] == i:
                    self.ind_nghb[i] = l

    def setOptions(self, options):
        self.input_dist     = options['input_disturbance'] if ('input_disturbance' in options) else False
        self.model_mismatch = options['model_mismatch'] if ('model_mismatch' in options) else False
        self.ideal          = not (self.model_mismatch or self.input_dist)

    def setInitialConditions(self, y0, dy0):
        self.y0    = y0
        self.dy0   = dy0
        self.path['y']      = np.vstack(y0)
        self.path['dy']     = np.vstack(dy0)
        self.path['input']  = self.getInput(y0, dy0)
        self.path['state']  = self.getState(y0, dy0)
        self.path['time']   = np.array([0.])
        self.prediction['y']      = np.vstack(y0)
        self.prediction['dy']     = np.vstack(dy0)
        self.prediction['input']  = self.getInput(y0, dy0)
        self.prediction['state']  = self.getState(y0, dy0)

        self.plant.setState(self.path['state'])

    def setTerminalConditions(self, yT, dyT):
        self.yT     = yT
        self.dyT    = dyT

    def getCheckPoints(self, position):
        return self.shape.getCheckPoints(position)

    def transformSplines(self, variables, transformation):
        variables['cy'] = transformation(variables['cy'], self.knots, self.degree)
        return variables

    def getSplines(self, variables):
        return [BSpline(self.Basis, variables['cy',:,k]) for k in range(self.n_y)]

    def getSplineCoeffs(self, variables):
        return variables['cy'].toArray()

    def getVariableStruct(self):
        return struct([entry('cy', shape=(len(self.Basis), self.n_y))])

    def initVariables(self, variables):
        for k in range(self.n_y):
            variables['cy',:,k] = np.linspace(self.y0[k], self.yT[k], len(self.Basis))
        return variables

    def getParameterStruct(self):
        return struct([
            entry('y0',     shape=self.n_y),
            entry('dy0',    shape=self.n_dy),
            entry('yT',     shape=self.n_y),
            entry('dyT',    shape=self.n_dy)])

    def setParameters(self, parameters):
        parameters['y0']    = self.prediction['y']
        parameters['dy0']   = self.prediction['dy']
        parameters['yT']    = self.yT
        parameters['dyT']   = self.dyT
        return parameters

    def getLbgUbg(self, t = 0.):
        for d in range(1,max(self.boundary_smoothness['initial'],self.boundary_smoothness['internal'])+1):
            bc_state = 'initial' if (t == 0.) else 'internal'
            bc = 0 if (d <= self.boundary_smoothness[bc_state]) else inf
            for k in range(self.n_y):
                ind = (d-1)*self.n_y+k
                self.ubg['dy0_'+str(ind)] = bc
        return self.lbg, self.ubg

    # This implements the interpretation of a computed spline trajectory/prediction of next initial state/following of trajectory
    def update(self, y_coeffs, knots, update_time, horizon_time, sample_time):
        self.updateTrajectory(y_coeffs, knots, update_time, horizon_time, sample_time)
        self.predictState(update_time, sample_time)
        self.followTrajectory(update_time, sample_time)

    def updateTrajectory(self, y_coeffs, knots, update_time, horizon_time, sample_time):
        n_samp          = int(round(horizon_time*(1.-knots[0])/sample_time,3))
        current_time    = self.path['time'][-1]
        self.trajectory['time']             = np.linspace(current_time, current_time + n_samp*sample_time, n_samp + 1)
        self.trajectory['y']                = np.vstack([np.hstack(splev((self.trajectory['time']-current_time+knots[0]*horizon_time)/horizon_time, (knots, y_coeffs[:,k], self.degree), 0)) for k in range(self.n_y)])
        self.trajectory['dy']               = self.getDy(y_coeffs, knots, horizon_time, (self.trajectory['time']-current_time+knots[0]*horizon_time)/horizon_time)
        self.trajectory['input']            = self.getInput(self.trajectory['y'], self.trajectory['dy'])
        self.trajectory['state']            = self.getState(self.trajectory['y'], self.trajectory['dy'])
        self.trajectory['knots']['time']    = (np.r_[knots] - knots[0])*horizon_time + current_time
        self.trajectory['knots']['y']       = np.vstack([np.hstack(splev(np.r_[knots], (knots, y_coeffs[:,k], self.degree), 0)) for k in range(self.n_y)])
        self.trajectory['knots']['dy']      = self.getDy(y_coeffs, knots, horizon_time, np.r_[knots])
        self.trajectory['knots']['input']   = self.getInput(self.trajectory['knots']['y'], self.trajectory['knots']['dy'])
        self.trajectory['knots']['state']   = self.getState(self.trajectory['knots']['y'], self.trajectory['knots']['dy'])
        for ind in ['time','y','dy','input','state']:
            self.trajectories[ind].extend([self.trajectory[ind] for k in range(int(update_time/sample_time))])
            self.trajectories['knots'][ind].extend([self.trajectory['knots'][ind] for k in range(int(update_time/sample_time))])

    def predictState(self, predict_time, sample_time):
        n_samp          = int(predict_time/sample_time)
        if self.ideal:
            self.prediction['y']      = np.vstack([self.trajectory['y'][k,n_samp] for k in range(self.n_y)])
            self.prediction['dy']     = np.vstack([self.trajectory['dy'][k,n_samp] for k in range(self.n_dy)])
            self.prediction['input']  = self.getInput(self.prediction['y'], self.prediction['dy'])
            self.prediction['state']  = self.getState(self.prediction['y'], self.prediction['dy'])
        else:
            self.predicition = self.plant.simulate(self.state, self.trajectory['input'][:,:n_samp], sample_time)
        for ind in ['y','dy','input','state']:
            self.predictions[ind].extend([self.prediction[ind] for k in range(int(predict_time/sample_time))])

    def followTrajectory(self, follow_time, sample_time):
        n_samp     = int(follow_time/sample_time)
        if self.ideal:
            y      = np.vstack([self.trajectory['y'][k,1:n_samp+1] for k in range(self.n_y)])
            dy     = np.vstack([self.trajectory['dy'][k,1:n_samp+1] for k in range(self.n_dy)])
            input  = self.getInput(y, dy)
            state  = self.getState(y, dy)
            time   = np.linspace(sample_time,follow_time,n_samp)
        else:
            y, dy, input, state, time = self.plant.update(self.trajectory['input'][:,:n_samp], sample_time)
        self.path['time']   = np.append(self.path['time'], self.path['time'][-1] + time, axis = 1)
        self.path['y']      = np.append(self.path['y'], y, axis = 1)
        self.path['dy']     = np.append(self.path['dy'], dy, axis = 1)
        self.path['input']  = np.append(self.path['input'], input, axis = 1)
        self.path['state']  = np.append(self.path['state'], state, axis = 1)

    # This defines the required interface of inherriting vehicle classes
    def getConstraints(self, y, initial, terminal, boundary_smoothness, T, t = 0.):
        raise NotImplementedError('Please implement this method!')
    def getState(self, y, dy):
        raise NotImplementedError('Please implement this method!')
    def getInput(self, y, dy):
        raise NotImplementedError('Please implement this method!')
    def getPosition(self, y):
        raise NotImplementedError('Please implement this method!')
    def getDy(y_coeffs, knots, horizon_time, time_axis):
        raise NotImplementedError('Please implement this method!')
    def getDisturbance(self, sample_time):
        raise NotImplementedError('Please implement this method!')
    def updateModel(self, state, input, Ts):
        raise NotImplementedError('Please implement this method!')
    def updateReal(self, state, input, Ts):
        raise NotImplementedError('Please implement this method!')
    def draw(self, k = -1):
        raise NotImplementedError('Please implement this method!')

