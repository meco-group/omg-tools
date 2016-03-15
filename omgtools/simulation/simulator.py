from plots import Plots


class Simulator:

    def __init__(self, problem):
        self.problem = problem
        self.plot = Plots(problem)

    def run(self):
        current_time = 0.
        self.problem.initialize()
        stop = False
        while not stop:
            stop, current_time = self.update(current_time)
        self.problem.final()

    def update(self, current_time):
        # solve problem
        self.problem.solve(current_time)
        # update everything
        current_time = self.problem.update(current_time)
        self.plot.update()
        # check termination criteria
        stop = self.problem.stop_criterium()
        return stop, current_time
