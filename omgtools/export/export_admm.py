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


class ExportADMM(Export):

    def __init__(self, problem, options, src_dir=[], src_files=[]):
        Export.__init__(self, problem, options)
        source_dirs = ['point2point', 'point2point/admm', 'vehicles'] + src_dir
        src_files = src_files + ['ADMMPoint2Point.cpp', 'Point2Point.cpp', 'Vehicle.cpp']
        if problem.options['separate_build']:
            for updater in problem.updaters:
                dest_dir = os.path.join(self.options['directory'], str(updater))
                self.export(source_dirs, dest_dir, src_files, updater.father_updx, updater, updater.problem)
        else:
            updaters = problem.separate_per_build()
            if (len(updaters.keys()) == 1 and len(updaters.values()[0].keys()) == 1):
                updater = updaters.values()[0].values()[0][0]
                self.export(source_dirs, self.options['directory'], src_files, updater.father_updx, updater, updater.problem)
            else:
                for veh_type, nghb_nr in updaters.items():
                    for nr, upd in nghb_nr.items():
                        dest_dir = os.path.join(self.options['directory'], veh_type+'_'+nr+'nghb')
                        self.export(source_dirs, dest_dir, src_files, upd.father_updx, upd, upd.problem)

    def set_default_options(self):
        Export.set_default_options(self)
        self.options['executable'] = 'FormationPoint2Point'

    def export_casadi_problems(self, destination, father, problem):
        cwd = os.getcwd()
        filenames = []
        # substitutes
        for child, subst in father.substitutes.items():
            for name, fun in subst.items():
                filenames.append('subst_'+name+'.c')
                fun.generate(filenames[-1])
                shutil.move(cwd+'/'+filenames[-1], destination+'src/'+filenames[-1])
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
        filenames.extend(['updx.c', 'updz.c', 'updl.c', 'updres.c'])
        # move files
        shutil.move(cwd+'/updx.c', destination+'src/updx.c')
        shutil.move(cwd+'/updz.c', destination+'src/updz.c')
        shutil.move(cwd+'/updl.c', destination+'src/updl.c')
        shutil.move(cwd+'/updres.c', destination+'src/updres.c')
        return filenames

    def create_defines(self, father, problem, point2point):
        defines = {}
        defines['ADMMLBL'] = '"' + problem.label + '"'
        defines['RHO'] = problem.options['rho']
        defines['INITITER'] = problem.options['init_iter']
        defines['N_SHARED'] = problem.q_i_struct(0).cat.size(1)
        defines['N_NGHB'] = len(problem.fleet.get_neighbors(problem.vehicle))
        defines['UPDZPROBLEM'] = '"' + problem.problem_upd_z.name() + '"'
        defines['UPDLPROBLEM'] = '"' + problem.problem_upd_l.name() + '"'
        defines['UPDRESPROBLEM'] = '"' + problem.problem_upd_res.name() + '"'
        data = Export.create_defines(self, father, problem, point2point)
        code = data['defines']
        for name, define in defines.items():
            code += '#define ' + str(name) + ' ' + str(define) + '\n'
        return {'defines': code}

    def _create_spline_tf(self, father, problem):
        defines = Export._create_spline_tf(self, father, problem)
        for child, q_i in problem.q_i.items():
            for name, ind in q_i.items():
                if name in child._splines_prim:
                    basis = child._splines_prim[name]['basis']
                    for l in range(len(basis)):
                        sl_min = l*len(basis)
                        sl_max = (l+1)*len(basis)
                        if set(range(sl_min, sl_max)) <= set(ind):
                            spl = child._splines_prim[name]
                            if spl['init'] is not None:
                                tf = '{'
                                for k in range(spl['init'].shape[0]):
                                    tf += '{'+','.join([str(t) for t in spl['init'][k].tolist()])+'},'
                                tf = tf[:-1]+'}'
                                defines.update({('XVAR_%s_TF') % name.upper(): tf})
                            break
        return defines

    def create_functions(self, father, problem, point2point):
        code = Export.create_functions(self, father, problem, point2point)
        code.update(self._create_retrieveSharedVariables(father, problem))
        code.update(self._create_transformSharedSplines(father, problem, point2point))
        return code

    def _create_retrieveSharedVariables(self, father, problem):
        code, cnt = '', 0
        code += '\tvector<vector<double>> subst;\n'
        code += '\tvector<vector<double>> args = {variables, parameters};\n'
        x = problem.q_i_struct(0)
        for child, q_i in problem.q_i.items():
            for name, ind in q_i.items():
                if name in child._substitutes:
                    code += '\tsubstitutes["'+name+'"](args, subst);\n'
                    for i in ind:
                        code += '\tvariables_admm["x_i"]['+str(cnt)+'] = subst[0]['+str(i)+'];\n'
                        cnt += 1
                else:
                    for i in ind:
                        code += '\tvariables_admm["x_i"]['+str(cnt)+'] = var_dict["'+child.label+'"]["'+name+'"]['+str(i)+'];\n'
                        cnt += 1
        return {'retrieveSharedVariables': code}

    def _create_initSplines(self, father, problem):
        data = Export._create_initSplines(self, father, problem)
        code = data['initSplines']
        for child, q_i in problem.q_i.items():
            for name, ind in q_i.items():
                if name in child._splines_prim:
                    basis = child._splines_prim[name]['basis']
                    for l in range(len(basis)):
                        sl_min = l*len(basis)
                        sl_max = (l+1)*len(basis)
                        if set(range(sl_min, sl_max)) <= set(ind):
                            code += '\tsplines_tf["xvar_'+name+'"] = XVAR_'+name.upper()+'_TF;\n'
        return {'initSplines': code}

    def _create_transformSharedSplines(self, father, problem, point2point):
        code = ('\tif(((current_time > 0) and ' +
                'fabs(fmod(round(current_time*1000.)/1000., ' +
                'horizon_time/' +
                str(point2point.vehicles[0].knot_intervals)+')) <1.e-6)){\n')
        for var in ['x_i', 'z_i', 'l_i', 'z_ji', 'l_ji', 'z_ij', 'l_ij']:
            code += '\t\tvector<double> ' + var + '_tf(variables_admm["' + var + '"]);\n'
        cnt = 0
        for child, q_i in problem.q_i.items():
            for name, ind in q_i.items():
                if name in child._splines_prim:
                    basis = child._splines_prim[name]['basis']
                    for l in range(len(basis)):
                        sl_min = l*len(basis)
                        sl_max = (l+1)*len(basis)
                        if set(range(sl_min, sl_max)) <= set(ind):
                            code += ('\t\tfor(int i=0; i<' + str(len(basis)) + '; i++){\n')
                            for var in ['x_i', 'z_i', 'l_i']:
                                code += ('\t\t\t' + var + '_tf['+str(cnt)+'+i] = 0;\n')
                            code += ('\t\t\tfor(int k=0; k<n_nghb; k++){\n')
                            for var in ['z_ji', 'l_ji', 'z_ij', 'l_ij']:
                                code += ('\t\t\t\t' + var + '_tf['+str(cnt)+'+i+k*n_shared] = 0;\n')
                            code += '\t\t\t}\n'
                            code += ('\t\t\tfor(int j=0; j<' + str(len(basis)) + '; j++){\n')
                            for var in ['x_i', 'z_i', 'l_i']:
                                code += ('\t\t\t\t'+var+'_tf['+str(cnt)+'+i] += splines_tf["xvar_'+name+'"][i][j]*variables_admm["'+var+'"]['+str(cnt)+'+j];\n')
                            code += ('\t\t\t\tfor(int k=0; k<n_nghb; k++){\n')
                            for var in ['z_ji', 'l_ji', 'z_ij', 'l_ij']:
                                code += ('\t\t\t\t\t'+var+'_tf['+str(cnt)+'+i+k*n_shared] += splines_tf["xvar_'+name+'"][i][j]*variables_admm["'+var+'"]['+str(cnt)+'+j+k*n_shared];\n')
                            code += '\t\t\t\t}\n'
                            code += '\t\t\t}\n'
                            code += '\t\t}\n'
                cnt += len(ind)
        for var in ['x_i', 'z_i', 'l_i', 'z_ji', 'l_ji', 'z_ij', 'l_ij']:
            code += '\t\tvariables_admm["' + var + '"] = ' + var + '_tf;\n'
        code += '\t}\n'
        return {'transformSharedSplines': code}
