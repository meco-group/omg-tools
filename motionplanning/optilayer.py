from casadi import SX, inf, SXFunction, nlpIn, nlpOut, NlpSolver
from casadi import getSymbols, substitute
from casadi.tools import struct, struct_SX, struct_symSX, entry
from casadi.tools.structure import msymStruct
from codegeneration import get_nlp_solver, get_function
from codegeneration import gen_code_nlp, gen_code_function
from spline import BSpline
from itertools import groupby
import numpy as np
import copy


def evalf(fun, x):
    x = x if isinstance(x, list) else [x]
    return fun(x)


class OptiLayer:
    _children = {}
    _symbols = {}
    symbol_dict = {}

    def __init__(self, label):
        self.label = OptiLayer._make_label(label)
        self._constraints = {}
        self._variables = {}
        self._parameters = {}
        self._splines = {}
        self._objective = 0.
        self._constraint_cnt = 0
        OptiLayer._children.update({self.label: self})

    # ========================================================================
    # Auxiliary functions
    # ========================================================================

    @classmethod
    def _make_label(cls, label):
        label_split = [''.join(g) for _, g in groupby(label, str.isalpha)]
        index = label_split[-1]
        rest = ''.join(label_split[:-1])
        if index.isdigit():
            if label in cls._children.keys():
                return cls._make_label(rest+str(int(index)+1))
            else:
                return label
        else:
            return cls._make_label(label+str(0))

    @classmethod
    def _add_to_dict(cls, symbol, child, name):
        for index, sym in enumerate(getSymbols(symbol)):
            cls.symbol_dict[sym.getName()] = [child, name, index]

    def _add_label(self, name, label=None):
        if label is None:
            label = self.label
        return name+'_'+label

    # ========================================================================
    # Definition of symbols, variables, parameters, constraints, objective
    # ========================================================================

    def define_symbol(self, name, size0=1, size1=1):
        if not (name in OptiLayer._symbols):
            self._define_sx(name, size0, size1, OptiLayer._symbols)
        return OptiLayer._symbols[name]

    def define_variable(self, name, size0=1, size1=1):
        return self._define_sx(name, size0, size1, self._variables)

    def define_parameter(self, name, size0=1, size1=1):
        return self._define_sx(name, size0, size1, self._parameters)

    def define_spline_symbol(self, name, size0=1, size1=1, **kwargs):
        basis = kwargs['basis'] if 'basis' in kwargs else self.basis
        if not (name in OptiLayer._symbols):
            self._define_sx_spline(name, size0, size1, basis,
                                   OptiLayer._symbols)
        return OptiLayer._symbols[name]

    def define_spline_variable(self, name, size0=1, size1=1, **kwargs):
        basis = kwargs['basis'] if 'basis' in kwargs else self.basis
        return self._define_sx_spline(name, size0, size1,
                                      self._variables, basis)

    def define_spline_parameter(self, name, size0=1, size1=1, **kwargs):
        basis = kwargs['basis'] if 'basis' in kwargs else self.basis
        return self._define_sx_spline(name, size0, size1,
                                      self._parameters, basis)

    def _define_sx(self, name, size0, size1, dictionary):
        index = self._add_label(name)
        if index in OptiLayer.symbol_dict:
            raise ValueError('Name %s already used for parameter or variable!')
        dictionary[name] = SX.sym(index, size0, size1)
        OptiLayer._add_to_dict(dictionary[name], self, name)
        return dictionary[name]

    def _define_sx_spline(self, name, size0, size1, dictionary, basis):
        if size1 > 1:
            ret = []
            for l in range(size1):
                _name = name + str(l)
                ret.append(
                    self._define_sx_spline(_name, size0, 1, dictionary, basis))
            return ret
        else:
            coeffs = self._define_sx(name, len(basis), size0, dictionary)
            self._splines[name] = basis
            return [BSpline(basis, coeffs[:, k]) for k in range(size0)]

    def define_constraint(self, expr, lb, ub, shutdown=False, name=None):
        if name is None:
            name = 'c'+str(self._constraint_cnt)
            self._constraint_cnt += 1
        if name in self._constraints:
            raise ValueError('Name %s already used for constraint of %s!' %
                             (name, self.label))
        if isinstance(expr, BSpline):
            self._constraints[name] = (
                expr.coeffs, lb*np.ones(expr.coeffs.shape[0]),
                ub*np.ones(expr.coeffs.shape[0]), shutdown)
        else:
            self._constraints[name] = (expr, lb, ub, shutdown)

    def define_objective(self, expr):
        self._objective = expr

    def get_variable(self, name, **kwargs):
        variables = kwargs['variables'] if 'variables' in kwargs else None
        solution = kwargs['solution'] if 'solution' in kwargs else False
        spline = kwargs['spline'] if 'spline' in kwargs else True
        if solution:
            variables = OptiLayer._var_result
        if name in self._splines and spline:
            basis = self._splines[name]
            return self._get_symbol_spline(name, basis,
                                           variables, self._variables)
        else:
            return self._get_symbol(name, variables, self._variables)

    def get_parameter(self, name, **kwargs):
        parameters = kwargs['parameters'] if 'parameters' in kwargs else None
        solution = kwargs['solution'] if 'solution' in kwargs else False
        spline = kwargs['spline'] if 'spline' in kwargs else True
        if solution:
            parameters = OptiLayer._par
        if name in self._splines and spline:
            basis = self._splines[name]
            return self._get_symbol_spline(name, basis,
                                           parameters, self._parameters)
        else:
            return self._get_symbol(name, parameters, self._parameters)

    def _get_symbol(self, name, data, dictionary):
        if data is None:
            return dictionary[name]
        elif isinstance(data, msymStruct):
            return data[self.label, name]
        else:
            return data[self.label, name].toArray()

    def _get_symbol_spline(self, name, basis, data, dictionary):
        if data is None:
            coeffs = dictionary[name]
        elif isinstance(data, msymStruct):
            coeffs = data[self.label, name]
        else:
            coeffs = data[self.label, name].toArray()
        return [BSpline(basis, coeffs[:, k]) for k in range(coeffs.shape[1])]

    def get_constraint(self, name, variables=None, parameters=None):
        if (variables is None) and (parameters is None):
            return self._constraints[name][0]
        elif (isinstance(variables, msymStruct) and
              isinstance(parameters, msymStruct)):
            return OptiLayer._evaluate_symbols(self._constraints[name][0],
                                               variables,  parameters)
        else:
            return OptiLayer._evaluate_symbols(
                self._constraints[name][0], variables,
                parameters).toArray().ravel()

    def get_objective(self, variables=None, parameters=None):
        if (variables is None) and (parameters is None):
            return self._objective
        elif (isinstance(variables, msymStruct) and
              isinstance(parameters, msymStruct)):
            return OptiLayer._evaluate_symbols(self._objective, variables,
                                               parameters)
        else:
            return OptiLayer._evaluate_symbols(
                self._objective, variables, parameters).toArray().ravel()

    # ========================================================================
    # Problem composition
    # ========================================================================

    @classmethod
    def construct_problem(cls, options):
        cls.translate_symbols()
        variables = cls.construct_variables()
        parameters = cls.construct_parameters()
        constraints, lb, ub = cls.construct_constraints(variables, parameters)
        objective = cls.construct_objective(variables, parameters)
        problem, compile_time = cls.compile_nlp(
            SXFunction(nlpIn(x=variables, p=parameters),
                       nlpOut(f=objective, g=constraints)), options)
        for key, option in options['casadi'].items():
            if problem.hasOption(key):
                problem.setOption(key, option)
        problem.init()
        cls.init_variables()
        return problem, compile_time

    @classmethod
    def translate_symbols(cls, group=None):
        if group is None:
            group = cls._children
        for name, symbol in cls._symbols.items():
            sym_def = []
            for label, child in group.items():
                if name in child._variables or name in child._parameters:
                    sym_def.append(child)
            if len(sym_def) > 1:
                raise ValueError('Symbol %s is defined multiple times as'
                                 ' parameter or variable of %s!' %
                                (name, ','.join([sd.label for sd in sym_def])))
            else:
                cls._add_to_dict(symbol, sym_def[0], name)

    @classmethod
    def construct_variables(cls, group=None):
        if group is None:
            group = cls._children
        entries = []
        for label, child in group.items():
            entries_child = []
            for name, var in child._variables.items():
                entries_child.append(entry(name, shape=var.shape))
            entries.append(entry(label, struct=struct(entries_child)))
        cls._var_struct = struct(entries)
        return struct_symSX(cls._var_struct)

    @classmethod
    def construct_parameters(cls, group=None):
        if group is None:
            group = cls._children
        entries = []
        for label, child in group.items():
            entries_child = []
            for name, par in child._parameters.items():
                entries_child.append(entry(name, shape=par.shape))
            entries.append(entry(label, struct=struct(entries_child)))
        cls._par_struct = struct(entries)
        return struct_symSX(cls._par_struct)

    @classmethod
    def construct_constraints(cls, variables, parameters, group=None):
        if group is None:
            group = cls._children
        entries = []
        for child in group.values():
            for name, constraint in child._constraints.items():
                expression = cls._substitute_symbols(
                    constraint[0], variables, parameters)
                entries.append(entry(child._add_label(name), expr=expression))
        constraints = struct_SX(entries)
        cls._lb, cls._ub = constraints(0), constraints(0)
        cls._constraint_shutdown = {}
        for child in group.values():
            for name, constraint in child._constraints.items():
                cls._lb[child._add_label(name)] = constraint[1]
                cls._ub[child._add_label(name)] = constraint[2]
                if constraint[3]:
                    cls._constraint_shutdown[
                        child._add_label(name)] = constraint[3]
        return constraints, cls._lb, cls._ub

    @classmethod
    def construct_objective(cls, variables, parameters, group=None):
        if group is None:
            group = cls._children
        objective = 0.
        for child in group.values():
            objective += cls._substitute_symbols(
                child._objective, variables, parameters)
        return objective

    @classmethod
    def _substitute_symbols(cls, expr, variables, parameters):
        for sym in getSymbols(expr):
            [child, name, index] = cls.symbol_dict[sym.getName()]
            if name in child._variables:
                expr = substitute(expr, sym,
                                  variables[child.label, name, index])
            elif name in child._parameters:
                expr = substitute(expr, sym,
                                  parameters[child.label, name, index])
        return expr

    @classmethod
    def _evaluate_symbols(cls, expression, variables, parameters):
        symbols = getSymbols(expression)
        f = MXFunction(symbols, [expression])
        f.init()
        f_in = []
        for sym in symbols:
            [child, name] = cls.symbol_dict[sym.getName()]
            if name in child._variables:
                f_in.append(variables[child.label, name])
            elif name in child._parameters:
                f_in.append(parameters[child.label, name])
        return evalf(f, f_in)[0]

    # ========================================================================
    # Methods related to c code generation
    # ========================================================================

    @classmethod
    def compile_nlp(cls, nlp, options):
        codegen = options['codegen']
        compile_time = 0.
        if codegen['codegen']:
            if not codegen['compileme']:
                problem = get_nlp_solver('.builds/'+codegen['buildname'])
            else:
                nlp.init()
                problem, compile_time = gen_code_nlp(
                    NlpSolver(options['casadi']['solver'], nlp),
                    '.builds/'+codegen['buildname'])
            return problem, compile_time
        else:
            return NlpSolver(options['casadi']['solver'], nlp), 0.

    @classmethod
    def compile_function(cls, function, name, options):
        codegen = options['codegen']
        compile_time = 0.
        if codegen['codegen']:
            if not codegen['compileme']:
                function = get_function('.builds/'+codegen['buildname'], name)
            else:
                function, compile_time = gen_code_function(
                    function, '.builds/'+codegen['buildname'], name)
            return function, compile_time
        else:
            return function, 0.

    # ========================================================================
    # Problem evaluation
    # ========================================================================

    @classmethod
    def update_bounds(cls, current_time):
        lb, ub = copy.deepcopy(cls._lb), copy.deepcopy(cls._ub)
        for name, shutdown in cls._constraint_shutdown.items():
            if shutdown(current_time):
                lb[name], ub[name] = -inf, +inf
        return lb, ub

    @classmethod
    def init_variables(cls):
        variables = cls._var_struct(0.)
        for label, child in cls._children.items():
            var = child.init_variables()
            for name, v in var.items():
                variables[label, name] = v
        cls._var_result = variables

    @classmethod
    def get_parameters(cls, time):
        cls._par = cls._par_struct(0.)
        for label, child in cls._children.items():
            par = child.get_parameters(time)
            for name, p in par.items():
                cls._par[label, name] = p
        return cls._par

    @classmethod
    def get_variables(cls):
        return cls._var_result

    @classmethod
    def set_variables(cls, variables):
        cls._var_result = cls._var_struct(variables)

    # ========================================================================
    # Variable manipulation
    # ========================================================================

    @classmethod
    def transform_spline_variables(cls, transformation):
        for label, child in cls._children.items():
            for name, basis in child._splines.items():
                if name in child._variables:
                    cls._var_result[label, name] = transformation(
                        cls._var_result[child.label, name],
                        basis.knots, basis.degree)
