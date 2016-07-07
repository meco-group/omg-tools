# This file is part of OMG-tools.
#
# OMG-tools -- Optimal Motion Generation-tools
# Copyright (C) 2016 Ruben Van Parys & Tim Mercy, KU Leuven.
# All rights reserved.
#
# OMG-tools is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA

try:
    from casadi import Compiler
except:
    from casadi import Importer
    Compiler = Importer

from casadi import DM, MX, inf, Function, nlpsol, external


from casadi import symvar, substitute
from casadi.tools import struct, struct_MX, struct_symMX, entry
from spline import BSpline
from itertools import groupby
import time
import numpy as np
import copy
import os
import shutil
import collections as col

def evalf(fun, x):
    x = x if isinstance(x, list) else [x]
    return fun.call(x)


# ========================================================================
# Functions related to c code generation
# ========================================================================

def create_nlp(var, par, obj, con, options, name=''):
    codegen = options['codegen']
    if options['verbose'] >= 1:
        print 'Building nlp ... ',
    t0 = time.time()
    nlp = {'x': var, 'p': par, 'f': obj, 'g': con}
    slv_opt = options['solver_options'][options['solver']]
    opt = {}
    for key, value in slv_opt.items():
        opt[key] = value
    opt.update({'expand': True})
    solver = nlpsol('solver', options['solver'], nlp, opt)
    name = 'nlp' if name == '' else 'nlp_' + name
    if codegen['build'] == 'jit':
        if options['verbose'] >= 1:
            print('[jit compilation with flags %s]' % (codegen['flags'])),
        solver.generate_dependencies(name+'.c')
        compiler = Compiler(
            name+'.c', 'clang', {'flags': codegen['flags']})
        problem = nlpsol('solver', options['solver'], compiler, slv_opt)
        os.remove(name+'.c')
    elif codegen['build'] == 'shared':
        if os.name == 'nt':
            raise ValueError('Build option is not supported for Windows!')
        directory = os.path.join(os.getcwd(), 'build')
        if not os.path.isdir(directory):
            os.makedirs(directory)
        path = os.path.join(directory, name)
        if options['verbose'] >= 1:
            print('[compile to .so with flags %s]' % (codegen['flags'])),
        if os.path.isfile(path+'.so'):
            os.remove(path+'.so')
        solver.generate_dependencies(name+'.c')
        shutil.move(name+'.c', path+'.c')
        os.system('gcc -fPIC -shared %s %s.c -o %s.so' %
                  (codegen['flags'], path, path))
        problem = nlpsol('solver', options['solver'], path+'.so', slv_opt)
        os.remove(path+'.c')
    elif codegen['build'] == 'existing':
        if os.name == 'nt':
            raise ValueError('Build option is not supported for Windows!')
        directory = os.path.join(os.getcwd(), 'build')
        path = os.path.join(directory, name)
        if not os.path.isfile(path+'.so'):
            raise ValueError('%s.so does not exist!', path)
        if options['verbose'] >= 1:
            print('[using shared object %s.so]' % path),
        problem = nlpsol('solver', options['solver'], path+'.so', slv_opt)
    elif codegen['build'] is None:
        problem = solver
    else:
        raise ValueError('Invalid build option.')
    t1 = time.time()
    if options['verbose'] >= 1:
        print 'in %5f s' % (t1-t0)
    return problem, (t1-t0)


def create_function(name, inp, out, options):
    codegen = options['codegen']
    if options['verbose'] >= 1:
        print 'Building function %s ... ' % name,
    t0 = time.time()
    fun = Function(name, inp, out).expand()
    if codegen['build'] == 'jit':
        if options['verbose'] >= 1:
            print('[jit compilation with flags %s]' % (codegen['flags'])),
        fun.generate(name)
        compiler = Compiler(
            name+'.c', 'clang', {'flags': codegen['flags']})
        fun = external(name, compiler)
        os.remove(name+'.c')
    elif codegen['build'] == 'shared':
        if os.name == 'nt':
            raise ValueError('Build option is not supported for Windows!')
        directory = os.path.join(os.getcwd(), 'build')
        if not os.path.isdir(directory):
            os.makedirs(directory)
        path = os.path.join(directory, name)
        if options['verbose'] >= 1:
            print('[compile to .so with flags %s]' % (codegen['flags'])),
        if os.path.isfile(path+'.so'):
            os.remove(path+'.so')
        fun.generate(name+'.c')
        shutil.move(name+'.c', path+'.c')
        os.system('gcc -fPIC -shared %s %s.c -o %s.so' %
                  (codegen['flags'], path, path))
        fun = external(name, path+'.so')
        os.remove(path+'.c')
    elif codegen['build'] == 'existing':
        if os.name == 'nt':
            raise ValueError('Build option is not supported for Windows!')
        directory = os.path.join(os.getcwd(), 'build')
        path = os.path.join(directory, name)
        if not os.path.isfile(path+'.so'):
            raise ValueError('%s.so does not exist!', path)
        if options['verbose'] >= 1:
            print('[using shared object %s.so]' % path),
        fun = external(name, path+'.so')
    elif codegen['build'] is None:
        fun = fun
    else:
        raise ValueError('Invalid build option.')
    t1 = time.time()
    if options['verbose'] >= 1:
        print 'in %5f s' % (t1-t0)
    return fun, (t1-t0)


class OptiFather(object):

    def __init__(self, children=None):
        children = children or []
        self.children = col.OrderedDict()
        self.symbol_dict = col.OrderedDict()
        for child in children:
            self.add(child)

    def add(self, children):
        children = children if isinstance(children, list) else [children]
        for child in children:
            self.children.update({child.label: child})

    def add_to_dict(self, symbol, child, name):
        for sym in symvar(symbol):
            self.symbol_dict[sym.name()] = [child, name]

    # ========================================================================
    # Problem composition
    # ========================================================================

    def construct_problem(self, options, name='', problem=None):
        self.compose_dictionary()
        self.translate_symbols()
        variables = self.construct_variables()
        parameters = self.construct_parameters()
        self.construct_substitutes(variables, parameters)
        constraints, _, _ = self.construct_constraints(variables, parameters)
        objective = self.construct_objective(variables, parameters)
        self.problem_description = {'var': variables, 'par': parameters,
                                    'obj': objective, 'con': constraints,
                                    'opt': options}
        if problem is None:
            problem, buildtime = create_nlp(variables, parameters, objective,
                constraints, options, name)
        else:
            buildtime = 0.
        self.init_variables()
        self.init_parameters()
        return problem, buildtime

    def compose_dictionary(self):
        for child in self.children.values():
            self.symbol_dict.update(child.symbol_dict)

    def translate_symbols(self):
        for label, child in self.children.items():
            for name, symbol in child._symbols.items():
                sym_def = []
                for _, _child in self.children.items():
                    if (name in _child._variables or
                            name in _child._parameters):
                        sym_def.append(_child)
                if len(sym_def) > 1:
                    raise ValueError('Symbol %s, defined in %s, is defined'
                                     ' multiple times as parameter or'
                                     ' variable by %s!' %
                                     (name, label,
                                      ','.join([sd.label for sd in sym_def])))
                elif len(sym_def) == 0:
                    raise ValueError('Symbol %s, defined in %s, is not defined'
                                     ' as parameter or variable by any object' %
                                     (name, label))
                else:
                    self.add_to_dict(symbol, sym_def[0], name)

    def construct_variables(self):
        entries = []
        for label, child in self.children.items():
            entries_child = []
            for name, var in child._variables.items():
                entries_child.append(entry(name, shape=var.shape))
            entries.append(entry(label, struct=struct(entries_child)))
        self._var_struct = struct(entries)
        return struct_symMX(self._var_struct)

    def construct_parameters(self):
        entries = []
        for label, child in self.children.items():
            entries_child = []
            for name, par in child._parameters.items():
                entries_child.append(entry(name, shape=par.shape))
            entries.append(entry(label, struct=struct(entries_child)))
        self._par_struct = struct(entries)
        return struct_symMX(self._par_struct)

    def construct_substitutes(self, variables, parameters):
        self.substitutes = {}
        for child in self.children.values():
            self.substitutes[child] = {}
            for name, subst in child._substitutes.items():
                expr, _ = subst
                expression = self._substitute_symbols(expr, variables, parameters)
                self.substitutes[child][name] = Function(name, [variables, parameters], [expression])

    def construct_constraints(self, variables, parameters):
        entries = []
        for child in self.children.values():
            for name, constraint in child._constraints.items():
                expression = self._substitute_symbols(
                    constraint[0], variables, parameters)
                entries.append(entry(child._add_label(name), expr=expression))
        self._con_struct = struct(entries)
        constraints = struct_MX(entries)
        self._lb, self._ub = constraints(0), constraints(0)
        self._constraint_shutdown = {}
        for child in self.children.values():
            for name, constraint in child._constraints.items():
                self._lb[child._add_label(name)] = constraint[1]
                self._ub[child._add_label(name)] = constraint[2]
                if constraint[3]:
                    self._constraint_shutdown[
                        child._add_label(name)] = constraint[3]
        return constraints, self._lb, self._ub

    def construct_objective(self, variables, parameters):
        objective = 0.
        for child in self.children.values():
            objective += self._substitute_symbols(
                child._objective, variables, parameters)
        return objective

    def reset(self):
        for child in self.children.values():
            child.reset()

    def _substitute_symbols(self, expr, variables, parameters):
        if isinstance(expr, (int, float)):
            return expr
        for sym in symvar(expr):
            [child, name] = self.symbol_dict[sym.name()]
            if name in child._variables:
                expr = substitute(expr, sym, variables[child.label, name])
            elif name in child._parameters:
                expr = substitute(expr, sym, parameters[child.label, name])
        return expr

    def _evaluate_symbols(self, expression, variables, parameters):
        symbols = symvar(expression)
        f = Function(symbols, [expression])
        f.init()
        f_in = []
        for sym in symbols:
            [child, name] = self.symbol_dict[sym.name()]
            if name in child._variables:
                f_in.append(variables[child.label, name])
            elif name in child._parameters:
                f_in.append(parameters[child.label, name])
        return evalf(f, f_in)

    # ========================================================================
    # Problem evaluation
    # ========================================================================

    def update_bounds(self, current_time):
        lb, ub = copy.deepcopy(self._lb), copy.deepcopy(self._ub)
        for name, shutdown in self._constraint_shutdown.items():
            exec('shutdown_fun = lambda t: %s' % shutdown)
            if shutdown_fun(current_time):
                lb[name], ub[name] = -inf, +inf
        return lb, ub

    def init_variables(self):
        variables = self._var_struct(0.)
        for label, child in self.children.items():
            for name in child._variables.keys():
                variables[label, name] = child._values[name]
        self._var_result = variables
        self._dual_var_result = self._con_struct(0.)

    def init_parameters(self):
        self.set_parameters(0.)

    def set_variables(self, variables, child=None, name=None):
        if child is None:
            self._var_result = self._var_struct(variables)
        elif name is None:
            self._var_result[child.label] = variables
        else:
            self._var_result[child.label, name] = variables

    def get_variables(self, child=None, name=None, **kwargs):
        if child is None:
            return self._var_result
        elif name is None:
            return self._var_result.prefix(child.label)
        else:
            if name in child._substitutes:
                if name in child._splines_prim and not ('spline' in kwargs and not kwargs['spline']):
                    basis = child._splines_prim[name]['basis']
                    if 'symbolic' in kwargs and kwargs['symbolic']:
                        if 'substitute' in kwargs and not kwargs['substitute']:
                            coeffs = child._substitutes[name][1]
                        else:
                            coeffs = child._substitutes[name][0]
                    else:
                        fun = self.substitutes[child][name]
                        coeffs = np.array(fun(self._var_result, self._par_result))
                    return [BSpline(basis, coeffs[:, k]) for k in range(coeffs.shape[1])]
                else:
                    if 'symbolic' in kwargs and kwargs['symbolic']:
                        if 'substitute' in kwargs and not kwargs['substitute']:
                            return child._substitutes[name][1]
                        else:
                            return child._substitutes[name][0]
                    else:
                        fun = self.substitutes[child][name]
                        return np.array(fun(self._var_result, self._par_result))
            if name in child._splines_prim and not ('spline' in kwargs and not kwargs['spline']):
                basis = child._splines_prim[name]['basis']
                if 'symbolic' in kwargs and kwargs['symbolic']:
                    coeffs = child._variables[name]
                else:
                    coeffs = np.array(self._var_result[child.label, name])
                return [BSpline(basis, coeffs[:, k]) for k in range(coeffs.shape[1])]
            else:
                if 'symbolic' in kwargs and kwargs['symbolic']:
                    return child._variables[name]
                else:
                    return np.array(self._var_result[child.label, name])

    def get_parameters(self, child=None, name=None, **kwargs):
        if child is None:
            return self._par_result
        elif name is None:
            return self._par_result.prefix(child.label)
        else:
            if name in child._splines_prim and not ('spline' in kwargs and not kwargs['spline']):
                basis = child._splines_prim[name]['basis']
                if 'symbolic' in kwargs and kwargs['symbolic']:
                    coeffs = child._parameters[name]
                else:
                    coeffs = np.array(self._par_result[child.label, name])
                return [BSpline(basis, coeffs[:, k]) for k in range(coeffs.shape[1])]
            else:
                if 'symbolic' in kwargs and kwargs['symbolic']:
                    return child._parameters[name]
                else:
                    return np.array(self._par_result[child.label, name])

    def get_constraint(self, child, name, symbolic=False):
        if symbolic:
            return child._constraints[name][0]
        else:
            return self._evaluate_symbols(self.children[child.label]._constraints[name][0],
                self._var_result, self._par_result)

    def get_objective(self, child, name, symbolic=False):
        if symbolic:
            return child._objective
        else:
            return self._evaluate_symbols(self.children[child.label]._objective,
                self._var_result, self._par_result)

    def set_parameters(self, time):
        self._par_result = self._par_struct(0.)
        for label, child in self.children.items():
            par = child.set_parameters(time)
            for name in child._parameters.keys():
                if name in par:
                    self._par_result[label, name] = par[name]
                else:
                    self._par_result[label, name] = child._values[name]
        return self._par_result

    # ========================================================================
    # Spline tranformations
    # ========================================================================

    def init_transformations(self, init_primal_transform, init_dual_transform):
        # primal
        _init_tf = {}
        for child in self.children.values():
            for name, spl in child._splines_prim.items():
                if name in child._variables:
                    basis = spl['basis']
                    if basis not in _init_tf:
                        _init_tf[basis] = init_primal_transform(basis)
                    child._splines_prim[name]['init'] = _init_tf[basis]
        # dual
        _init_tf = {}
        for child in self.children.values():
            for name, spl in child._splines_dual.items():
                basis = spl['basis']
                if basis not in _init_tf:
                    _init_tf[basis] = init_dual_transform(basis)
                child._splines_dual[name]['init'] = _init_tf[basis]

    def transform_primal_splines(self, transform_fun):
        for label, child in self.children.items():
            for name, spl in child._splines_prim.items():
                if name in child._variables:
                    basis = spl['basis']
                    init = spl['init']
                    if init is not None:
                        self._var_result[label, name] = transform_fun(
                            self._var_result[label, name], basis, init)
                    else:
                        self._var_result[label, name] = transform_fun(
                            self._var_result[label, name], basis)

    def transform_dual_splines(self, transform_fun):
        for label, child in self.children.items():
            for name, spl in child._splines_dual.items():
                basis = spl['basis']
                init = spl['init']
                if init is not None:
                    init = spl['init']
                    self._dual_var_result[label, name] = transform_fun(
                        self._dual_var_result[label, name], basis, init)
                else:
                    self._dual_var_result[label, name] = transform_fun(
                        self._dual_var_result[label, name], basis)


class OptiChild(object):
    _labels = []

    def __init__(self, label):
        self.label = OptiChild._make_label(label)
        self._variables = col.OrderedDict()
        self._parameters = col.OrderedDict()
        self._symbols = col.OrderedDict()
        self._substitutes = col.OrderedDict()
        self._values = col.OrderedDict()
        self._splines_prim = col.OrderedDict()
        self._splines_dual = col.OrderedDict()
        self._constraints = col.OrderedDict()
        self.symbol_dict = col.OrderedDict()
        self._objective = 0.
        self._constraint_cnt = 0

    def __str__(self):
        return self.label

    __repr__ = __str__

    def add_to_dict(self, symbol, name):
        for sym in symvar(symbol):
            if sym in self.symbol_dict:
                raise ValueError('Symbol already added for %s' % self.label)
            self.symbol_dict[sym.name()] = [self, name]

    def _add_label(self, name):
        return name + '_' + self.label

    @classmethod
    def _make_label(cls, label):
        label_split = [''.join(g) for _, g in groupby(label, str.isalpha)]
        index = label_split[-1]
        rest = ''.join(label_split[:-1])
        if index.isdigit():
            if label in cls._labels:
                return cls._make_label(rest+str(int(index)+1))
            else:
                cls._labels.append(label)
                return label
        else:
            return cls._make_label(label+str(0))

    # ========================================================================
    # Definition of symbols, variables, parameters, constraints, objective
    # ========================================================================

    def define_symbol(self, name, size0=1, size1=1):
        return self._define_mx(name, size0, size1, self._symbols, None)

    def define_variable(self, name, size0=1, size1=1, **kwargs):
        value = kwargs['value'] if 'value' in kwargs else None
        return self._define_mx(name, size0, size1, self._variables, value)

    def define_parameter(self, name, size0=1, size1=1, **kwargs):
        value = kwargs['value'] if 'value' in kwargs else None
        return self._define_mx(name, size0, size1, self._parameters, value)

    def define_spline_symbol(self, name, size0=1, size1=1, **kwargs):
        basis = kwargs['basis'] if 'basis' in kwargs else self.basis
        value = kwargs['value'] if 'value' in kwargs else None
        return self._define_mx_spline(name, size0, size1,
                                      self._symbols, basis, value)

    def define_spline_variable(self, name, size0=1, size1=1, **kwargs):
        basis = kwargs['basis'] if 'basis' in kwargs else self.basis
        value = kwargs['value'] if 'value' in kwargs else None
        return self._define_mx_spline(name, size0, size1,
                                      self._variables, basis, value)

    def define_spline_parameter(self, name, size0=1, size1=1, **kwargs):
        basis = kwargs['basis'] if 'basis' in kwargs else self.basis
        value = kwargs['value'] if 'value' in kwargs else None
        return self._define_mx_spline(name, size0, size1,
                                      self._parameters, basis, value)

    def define_substitute(self, name, expr):
        if isinstance(expr, list):
            return [self.define_substitute(name+str(l), e) for l, e in enumerate(expr)]
        else:
            if name in self._substitutes:
                raise ValueError('Name %s already used for substitutes!' % (name))
            symbol_name = self._add_label(name)
            if isinstance(expr, BSpline):
                self._splines_prim[name] = {'basis': expr.basis}
                coeffs = MX.sym(symbol_name, expr.coeffs.shape[0], 1)
                subst = BSpline(expr.basis, coeffs)
                self._substitutes[name] = [expr.coeffs, subst.coeffs]
                inp_sym, inp_num = [], []
                for sym in symvar(expr.coeffs):
                    inp_sym.append(sym)
                    inp_num.append(DM(self._values[self.symbol_dict[sym.name()][1]]))
                fun = Function('eval', inp_sym, [expr.coeffs])
                self._values[name] = fun(*inp_num)
                self.add_to_dict(coeffs, name)
            else:
                subst = MX.sym(symbol_name, expr.shape[0], expr.shape[1])
                self._substitutes[name] = [expr, subst]
                self.add_to_dict(subst, name)
            return subst

    def _define_mx(self, name, size0, size1, dictionary, value=None):
        if value is None:
            value = np.zeros((size0, size1))
        symbol_name = self._add_label(name)
        dictionary[name] = MX.sym(symbol_name, size0, size1)
        self._values[name] = value
        self.add_to_dict(dictionary[name], name)
        return dictionary[name]

    def _define_mx_spline(self, name, size0, size1, dictionary, basis, value=None):
        if size1 > 1:
            return [self._define_mx_spline(name+str(l), size0,
                                           1, dictionary, basis, value)
                    for l in range(size1)]
        else:

            coeffs = self._define_mx(
                name, len(basis), size0, dictionary, value)
            self._splines_prim[name] = {'basis': basis}
            return [BSpline(basis, coeffs[:, k]) for k in range(size0)]

    def set_value(self, name, value):
        self._values[name] = value

    def define_constraint(self, expr, lb, ub, shutdown=False, name=None):
        if isinstance(expr, (float, int)):
            return
        if name is None:
            name = 'c'+str(self._constraint_cnt)
            self._constraint_cnt += 1
        if name in self._constraints:
            raise ValueError('Name %s already used for constraint!' % (name))
        if isinstance(expr, BSpline):
            self._constraints[name] = (
                expr.coeffs, lb*np.ones(expr.coeffs.shape[0]),
                ub*np.ones(expr.coeffs.shape[0]), shutdown)
            self._splines_dual[name] = {'basis': expr.basis}
        else:
            self._constraints[name] = (expr, lb, ub, shutdown)

    def define_objective(self, expr):
        self._objective += expr

    # ========================================================================
    # Reset variables, parameters, constraints, objectives
    # ========================================================================

    def reset(self):
        self._variables = col.OrderedDict()
        self._parameters = col.OrderedDict()
        self._symbols = col.OrderedDict()
        self._substitutes = col.OrderedDict()
        self._values = col.OrderedDict()
        self._splines_prim = col.OrderedDict()
        self._splines_dual = col.OrderedDict()
        self._constraints = col.OrderedDict()
        self.symbol_dict = col.OrderedDict()
        self._objective = 0.
        self._constraint_cnt = 0

    # ========================================================================
    # Methods required to override
    # ========================================================================

    def set_parameters(self, time):
        return {}
