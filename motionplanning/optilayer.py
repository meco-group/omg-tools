from casadi import MX, inf, MXFunction, getSymbols, substitute
from casadi.tools import struct, struct_MX, struct_symMX, entry
from spline import BSpline
import numpy as np
import copy


def evalf(fun, x):
    x = x if isinstance(x, list) else [x]
    if not fun.isInit():
        fun.init()
    return fun(x)


class OptiLayer:
    _children = {}
    _symbols = {}
    _symbols_used = []

    def __init__(self, label):
        self._label = label
        self._constraints = {}
        self._variables = {}
        self._parameters = {}
        self._splines = {}
        self._objective = 0.
        self._constraint_cnt = 0
        OptiLayer._children.update({label: self})

    # ========================================================================
    # Auxiliary functions
    # ========================================================================

    def _add_label(self, name):
        return name+'_'+self._label

    @classmethod
    def _rmv_label(cls, index):
        return '_'.join(index.split('_')[:-1])

    @classmethod
    def _check_symbols(cls, name):
        if name in cls._symbols:
            if name in cls._symbols_used:
                raise ValueError(
                    """%s is a symbol which is already defined
                    as variable or parameter!""" % name)
            else:
                cls._symbols_used.append(name)

    @classmethod
    def _check_varpar(cls, name):
        cnt = 0
        for label, child in OptiLayer._children.items():
            if name in child._variables:
                cnt += 1
            if name in child._parameters:
                cnt += 1
        if cnt > 1:
            raise ValueError(
                """Symbol %s already used multiple times
                as variable or parameter!""" % name)
        else:
            return cnt

    # ========================================================================
    # Problem definition
    # ========================================================================

    def define_symbol(self, name, size0=1, size1=1):
        if not (name in OptiLayer._symbols):
            self._define_mx(name, size0, size1, OptiLayer._symbols, 'symbol')
        return self._symbols[name]

    def define_variable(self, name, size0=1, size1=1):
        OptiLayer._check_symbols(name)
        return self._define_mx(name, size0, size1,
                               self._variables, 'variable')

    def define_parameter(self, name, size0=1, size1=1):
        OptiLayer._check_symbols(name)
        return self._define_mx(name, size0, size1,
                               self._parameters, 'parameter')

    def define_spline_symbol(self, name, size0=1, size1=1, **kwargs):
        basis = kwargs['basis'] if 'basis' in kwargs else self.basis
        if not (name in OptiLayer._symbols):
            self._define_mx_spline(name, size0, size1, basis,
                                   OptiLayer._symbols, 'symbol')
        return self._symbols[name]

    def define_spline_variable(self, name, size0=1, size1=1, **kwargs):
        OptiLayer._check_symbols(name)
        basis = kwargs['basis'] if 'basis' in kwargs else self.basis
        return self._define_mx_spline(name, size0, size1,
                                      basis, self._variables, 'variable')

    def define_spline_parameter(self, name, size0=1, size1=1, **kwargs):
        OptiLayer._check_symbols(name)
        basis = kwargs['basis'] if 'basis' in kwargs else self.basis
        return self._define_mx_spline(name, size0, size1,
                                      basis, self._parameters, 'parameter')

    def _define_mx(self, name, size0, size1, dictionary, type):
        index = self._add_label((name))
        if index in dictionary:
            raise ValueError('Name %s already used for %s of %s!' %
                             (name, type, self._label))
        dictionary[name] = MX.sym(index, size0, size1)
        return dictionary[name]

    def _define_mx_spline(self, name, size0, size1, basis, dictionary, type):
        if size1 > 1:
            ret = []
            for l in range(size1):
                _name = name + str(l)
                ret.append(self._define_mx_spline(_name, size0, 1, basis,
                                                  dictionary, type))
            return ret
        else:
            coeffs = self._define_mx(name, len(basis), size0, dictionary, type)
        return [BSpline(basis, coeffs[:, k]) for k in range(size0)]

    def define_constraint(self, expr, lb, ub, shutdown=False, name=None):
        if name is None:
            name = 'c'+str(self._constraint_cnt)
            self._constraint_cnt += 1
        if name in self._constraints:
            raise ValueError('Name %s already used for constraint of %s!' %
                             (name, self._label))
        if isinstance(expr, BSpline):
            self._constraints[name] = (
                expr.coeffs, lb*np.ones(expr.coeffs.shape[0]),
                ub*np.ones(expr.coeffs.shape[0]), shutdown)
        else:
            self._constraints[name] = (expr, lb, ub, shutdown)

    def define_objective(self, expr):
        self._objective = expr

    def get_variable(self, name):
        return OptiLayer._var_result[self._label, name].toArray()

    def get_parameter(self, name):
        return OptiLayer._par[self._label, name].toArray()

    def get_constraint(self, name):
        return OptiLayer._evaluate_symbols(self._constraints[name][0],
                                           OptiLayer._var_result,
                                           OptiLayer._par).toArray().ravel()

    def get_objective(self):
        return OptiLayer._evaluate_symbols(self._objective,
                                           OptiLayer._var_result,
                                           OptiLayer._par).toArray().ravel()

    # ========================================================================
    # Problem composition
    # ========================================================================

    @classmethod
    def construct_variables(cls):
        entries = []
        for label, child in cls._children.items():
            entries_child = []
            for name, var in child._variables.items():
                entries_child.append(entry(name, shape=var.shape))
            entries.append(entry(label, struct=struct(entries_child)))
        cls._var_struct = struct(entries)
        return struct_symMX(cls._var_struct)

    @classmethod
    def construct_parameters(cls):
        entries = []
        for label, child in cls._children.items():
            entries_child = []
            for name, par in child._parameters.items():
                entries_child.append(entry(name, shape=par.shape))
            entries.append(entry(label, struct=struct(entries_child)))
        cls._par_struct = struct(entries)
        return struct_symMX(cls._par_struct)

    @classmethod
    def construct_constraints(cls, variables, parameters):
        entries = []
        for child in cls._children.values():
            for name, constraint in child._constraints.items():
                expression = cls._substitute_symbols(
                    constraint[0], variables, parameters)
                entries.append(entry(child._add_label(name), expr=expression))
        constraints = struct_MX(entries)
        cls._lb, cls._ub = constraints(0), constraints(0)
        cls._constraint_shutdown = {}
        for child in cls._children.values():
            for name, constraint in child._constraints.items():
                cls._lb[child._add_label(name)] = constraint[1]
                cls._ub[child._add_label(name)] = constraint[2]
                if constraint[3]:
                    cls._constraint_shutdown[
                        child._add_label(name)] = constraint[3]
        return constraints, cls._lb, cls._ub

    @classmethod
    def construct_objective(cls, variables, parameters):
        objective = 0.
        for child in cls._children.values():
            objective += cls._substitute_symbols(
                child._objective, variables, parameters)
        return objective

    @classmethod
    def _substitute_symbols(cls, expression, variables, parameters):
        for sym in getSymbols(expression):
            name = cls._rmv_label(sym.getName())
            for label, child in cls._children.items():
                if name in child._variables:
                    expression = substitute(
                        expression, sym, variables[label, name])
                elif name in child._parameters:
                    expression = substitute(
                        expression, sym, parameters[label, name])
        return expression

    @classmethod
    def _evaluate_symbols(cls, expression, variables, parameters):
        symbols = getSymbols(expression)
        f = MXFunction(symbols, [expression])
        f_in = []
        for sym in symbols:
            name = cls._rmv_label(sym.getName())
            for label, child in cls._children.items():
                if name in child._variables:
                    f_in.append(variables[label, name])
                elif name in child._parameters:
                    f_in.append(parameters[label, name])
        return evalf(f, f_in)[0]

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
    def transform_splines(cls, transformation):
        for label, child in cls._children.items():
            for name, basis in child._splines.items():
                cls._var_result[label, name] = transformation(
                    cls._var_result[child._label, name],
                    basis.knots, basis.degree)
