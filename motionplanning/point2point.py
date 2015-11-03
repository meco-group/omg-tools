from problem import *
import time

class Point2point(Problem):
    def __init__(self, vehicle, environment, time, options):
        self.iteration      = 0
        self.update_times   = []
        Problem.__init__(self, vehicle, environment, time, options)

    def createStructs(self):
        self.var_struct = struct([
            entry('vehicle',    struct  =   self.vehicle.getVariableStruct()),
            entry('cg',         shape   =   (len(self.vehicle.Basis),self.vehicle.n_y)),
            entry('environment',struct  =   self.environment.getVariableStruct(self.knot_intervals, safety = self.col_safety))
        ])
        self.par_struct = struct([
            entry('vehicle',    struct  =   self.vehicle.getParameterStruct()),
            entry('environment',struct  =   self.environment.getParameterStruct()),
            entry('t'), entry('T')
        ])

    def solve(self, current_time):
        # Transform variables first
        self.transform(current_time)
        # Set parameters and initial guess
        self.problem.setInput(self.variables, 'x0')
        self.problem.setInput(self.setParameters(current_time), 'p')
        # Set lbg & ubg
        lbg_veh, ubg_veh = self.vehicle.getLbgUbg(current_time)
        lbg_env, ubg_env = self.environment.getLbgUbg()
        lbg = vertcat([self.lbg_obj,lbg_veh,lbg_env])
        ubg = vertcat([self.ubg_obj,ubg_veh,ubg_env])
        self.problem.setInput(lbg, 'lbg')
        self.problem.setInput(ubg, 'ubg')
        # Solve!
        t0 = time.time()
        self.problem.evaluate()
        t1 = time.time()
        t_upd = t1-t0
        self.variables  = self.var_struct(self.problem.getOutput('x'))
        stats           = self.problem.getStats()
        if stats.get("return_status") != "Solve_Succeeded":
            print stats.get("return_status")
        # Print
        self.iteration += 1
        if ((self.iteration-1)%20 == 0):
            print "----|------------|------------"
            print "%3s | %10s | %10s " % ("It","t upd","time")
            print "----|------------|------------"
        print "%3d | %.4e | %.4e " % (self.iteration, t_upd, current_time)
        self.update_times.append(t_upd)

    def final(self):
        obj = self.computeObjective()
        print 'We reached our target!'
        print 'Objective: %f' % (obj)
        print 'Max update time: %f' % (max(self.update_times))
        print 'Average update time: %f' % (sum(self.update_times)/len(self.update_times))

    def createProblem(self):
        var         = struct_symMX(self.var_struct)
        par         = struct_symMX(self.par_struct)

        # Splines
        y  = self.vehicle.getSplines(var.prefix['vehicle'])
        g  = [BSpline(self.vehicle.Basis, var['cg',:,k]) for k in range(self.vehicle.n_y)]

        # Objective
        t0      = par['t']/par['T']
        f       = sum([definite_integral(g[k], t0, 1.) for k in range(self.vehicle.n_y)])
        con_obj = []
        for k in range(self.vehicle.n_y):
            con_obj.append( y[k] - par['vehicle','yT',k] - g[k])
            con_obj.append(-y[k] + par['vehicle','yT',k] - g[k])
        con_obj = vertcat(list([c.coeffs for c in con_obj]))
        self.lbg_obj = -inf*np.ones(con_obj.size())
        self.ubg_obj = np.zeros(con_obj.size())

        # Constraints
        con_veh, lbg_veh, ubg_veh, obj_veh = self.vehicle.getConstraints(y, initial = {'y':par['vehicle','y0'], 'dy':par['vehicle','dy0']}, terminal = {'dy':par['vehicle','dyT']}, boundary_smoothness = {'initial': 1, 'internal': self.vehicle.degree-1, 'terminal': self.vehicle.degree}, T = par['T'], t = par['t'])
        con_env, lbg_env, ubg_env, obj_env = self.environment.getConstraints(self.vehicle, y, var.prefix['environment'], par.prefix['environment'], T = par['T'], t = par['t'])

        f += obj_veh + obj_env

        con = vertcat([con_obj,con_veh,con_env])
        lbg = vertcat([self.lbg_obj,lbg_veh,lbg_env])
        ubg = vertcat([self.ubg_obj,ubg_veh,ubg_env])

        # Create problem + code generation
        if self.codegen_opt['codegen']:
            if not self.codegen_opt['compileme']:
                problem = getNLPSolver('builds/'+self.codegen_opt['buildname'])
            elif self.codegen_opt['compileme']:
                nlp     = MXFunction(nlpIn(x = var, p = par), nlpOut(f = f, g = con))
                nlp.init()
                problem_ = NlpSolver('ipopt',nlp)
                problem, compile_time  = genCodeNLP(problem_,'builds/'+self.codegen_opt['buildname'])
        else:
            nlp     = MXFunction(nlpIn(x = var, p = par), nlpOut(f = f, g = con))
            nlp.init()
            problem = NlpSolver('ipopt',nlp)

        self.problem = problem

        self.problem.setOption("linear_solver","ma57")
        self.problem.setOption("tol", 1e-3)
        self.problem.setOption("warm_start_init_point", "yes")
        self.problem.setOption("print_level",0)
        self.problem.setOption("print_time",0)
        self.problem.init()

    def initVariables(self):
        self.variables = self.var_struct(0)
        self.variables['vehicle']        = self.vehicle.initVariables(self.variables.prefix['vehicle'])
        self.variables['environment']    = self.environment.initVariables(self.variables.prefix['environment'])

    def setParameters(self, current_time):
        parameters = self.par_struct(0)
        parameters['vehicle']       = self.vehicle.setParameters(parameters.prefix['vehicle'])
        parameters['environment']   = self.environment.setParameters(parameters.prefix['environment'])
        parameters['t']             = np.round(current_time,6)%self.knot_time
        parameters['T']             = self.horizon_time
        return parameters

    def transform(self, current_time):
        if np.round(current_time,6)%self.knot_time == 0:
            self.variables['cg']         = shiftOverKnot(self.variables['cg'], self.vehicle.knots, self.vehicle.degree, 1)
            self.variables['vehicle']    = self.vehicle.transformSplines(self.variables.prefix['vehicle'], lambda coeffs, knots, degree: shiftOverKnot(coeffs, knots, degree, 1))
            self.variables['environment']= self.environment.transformSplines(self.variables.prefix['environment'], lambda coeffs, knots, degree: shiftOverKnot(coeffs, knots, degree, 1))

    def update(self, current_time):
        y_coeffs    = self.vehicle.getSplineCoeffs(self.variables.prefix['vehicle'])
        knots2      = np.copy(self.vehicle.knots)
        knots2[:self.vehicle.degree+1] = (np.round(current_time,6)%self.knot_time)/self.horizon_time
        y_coeffs2   = shiftFirstKnotFwd(y_coeffs, self.vehicle.knots, self.vehicle.degree, knots2[0])
        self.vehicle.update(y_coeffs2, knots2, self.update_time, self.horizon_time, self.sample_time)
        self.environment.update(self.update_time, self.sample_time)

    def computeObjective(self):
        n_samp  = self.vehicle.path['y'].shape[1]
        err     = self.vehicle.path['y'] - self.vehicle.yT
        err_nrm1 = np.zeros(n_samp)
        for k in range(n_samp):
            err_nrm1[k] = np.linalg.norm(err[:,k],1)
        err_nrm1_int = 0.
        for k in range(n_samp-1):
            err_nrm1_int += 0.5*(err_nrm1[k] + err_nrm1[k+1])*self.sample_time
        return err_nrm1_int
