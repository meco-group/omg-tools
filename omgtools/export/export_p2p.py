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

import os
import shutil
from casadi import nlpsol
from export import Export


class ExportP2P(Export):

    def __init__(self, problem, options):
        Export.__init__(self, problem, options)
        self.father = problem.father
        self.problem = problem
        if len(problem.vehicles) > 1:
            raise ValueError(('Only export for single vehicle ' +
                              'problems is supported.'))
        self.vehicle = problem.vehicles[0]
        src_files = ['Point2Point.cpp', 'Vehicle.cpp']
        self.export(['point2point', 'vehicles'], self.options['directory'], src_files, problem.father, problem)

    def set_default_options(self):
        Export.set_default_options(self)
        self.options['executable'] = 'Point2Point'

    def export_casadi_problems(self, destination, father, problem):
        filenames = Export.export_casadi_problems(self, destination, father, problem)
        obj = father.problem_description['obj']
        con = father.problem_description['con']
        var = father.problem_description['var']
        par = father.problem_description['par']
        opt = father.problem_description['opt']
        nlp = {'x': var, 'p': par, 'f': obj, 'g': con}
        options = {}
        for key, value in opt['solver_options']['ipopt'].items():
            options[key] = value
        options.update({'expand': True})
        solver = nlpsol('solver', 'ipopt', nlp, options)
        solver.generate_dependencies('nlp.c')
        cwd = os.getcwd()
        shutil.move(cwd+'/nlp.c', destination+'src/nlp.c')
        filenames.append('nlp.c')
        return filenames
