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


class ExportFormation(Export):

    def __init__(self, problem, options):
        Export.__init__(self, problem, options)
        source_dirs = ['formation', 'vehicles']
        if problem.options['separate_build']:
            for updater in problem.updaters:
                dest_dir = os.path.join(self.options['directory'], str(updater))
                self.export(source_dirs, dest_dir, updater.father_updx, updater, updater.problem)
        else:
            updaters = problem.separate_per_build()
            if (len(updaters.keys()) == 1 and len(updaters.values()[0].keys()) == 1):
                updater = updaters.values()[0].values()[0][0]
                self.export(source_dirs, self.options['directory'], updater.father_updx, updater, updater.problem)
            else:
                for veh_type, nghb_nr in updaters.items():
                    for nr, upd in nghb_nr.items():
                        dest_dir = os.path.join(self.options['directory'], veh_type+'_'+nr+'nghb')
                        self.export(source_dirs, dest_dir, upd.father_updx, upd, upd.problem)

    def set_default_options(self):
        Export.set_default_options(self)
        self.options['executable'] = 'FormationPoint2Point'

    def export_casadi_problems(self, destination, father, problem):
        cwd = os.getcwd()
        # updx
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
        solver.generate_dependencies('updx.c')
        # updz, updl, upd_res
        problem.problem_upd_z.generate('updz.c')
        problem.problem_upd_l.generate('updl.c')
        problem.problem_upd_res.generate('updres.c')
        # move files
        shutil.move(cwd+'/updx.c', destination+'src/updx.c')
        shutil.move(cwd+'/updz.c', destination+'src/updz.c')
        shutil.move(cwd+'/updl.c', destination+'src/updl.c')
        shutil.move(cwd+'/updres.c', destination+'src/updres.c')
