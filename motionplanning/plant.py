
import numpy as np
import scipy.signal

class Plant:
    def __init__(self, updateReal, updateModel):
        self.updateReal = updateReal
        self.updateModel= updateModel
        self.dist_pntr = 0

    def setDisturbance(self, disturbance):
        # Create disturbance signal
        self.n_dist_samp = 3000
        fc      = disturbance['fc']     # cut-off frequency divided by Nyquist frequency
        stdev   = disturbance['stdev']
        n_in    = len(stdev)
        mean    = disturbance['mean'] if 'mean' in disturbance else np.zeros(n_in)
        b, a    = scipy.signal.butter(3, fc, 'low')
        self.disturbance = np.zeros((n_in, self.n_dist_samp))
        for k in range(n_in):
            if stdev[k] > 0.:
                self.disturbance[k,:] = scipy.signal.filtfilt(b, a, np.random.normal(mean[k], stdev[k], self.n_dist_samp))
            else:
                self.disturbance[k,:] = np.zeros(self.n_dist_samp)

    def setState(self, state):
        self.state = state

    def simulate(self, state0, input, sample_time):
        state = state0
        for k in range(input.shape[1]):
            y, dy, state = self.updateModel(state, input[:,k], sample_time)
        return {'y':y, 'dy':dy, 'input':input[:,-1], 'state':state}

    def update(self, input, sample_time):
        y_ = [], dy_ = [], input_ = [], state_ = [], time_ = [], t = 0.
        for k in range(input.shape[1]):

            y, dy, self.state = self.updateReal(self.state, input[:,k] + self.disturbance[:,self.dist_pntr], sample_time)
            y_.append(np.vstack(y))
            dy_.append(np.vstack(dy))
            input_.append(np.vstack(input[:,k] + self.disturbance[:,self.dist_pntr]))
            state_.append(np.vstack(self.state))
            t += sample_time
            time_.append(t)
            self.dist_pntr += 1
            if self.dist_pntr >= self.n_dist_samp:
                self.dist_pntr = 0
        return np.hstack(y_), np.hstack(dy_), np.hstack(dy_), np.hstack(state_), np.hstack(time_)
