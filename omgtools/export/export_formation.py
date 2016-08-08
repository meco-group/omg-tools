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


class ExportFormation(object):

    def __init__(self, problem, options):
        self.set_default_options()
        self.set_options(options)
        self.problem = problem
        self.updaters = problem.updaters
        self.vehicles = problem.vehicles
        if self.problem.options['separate_build']:
            for updater in problem.updaters:
                self.export(updater, str(updater))
        else:
            updaters = self.problem.separate_per_build()
            if (len(updaters.keys()) == 1 and len(updaters.values()[0].keys()) == 1):
                updater = updaters.values()[0].values()[0][0]
                self.export(updater)
            else:
                for veh_type, nghb_nr in updaters.items():
                    for nr, upd in nghb_nr.items():
                        self.export(upd, veh_type+'_'+nr+'nghb')

    def set_default_options(self):
        self.options = {}
        self.options['directory'] = os.path.join(os.getcwd(), 'export/')
        self.options['casadi_optiflags'] = ''  # '', '-O3', '-Os'
        self.options['casadilib'] = '/usr/local/lib/'
        self.options['casadiinc'] = '/usr/local/include/casadi/'
        self.options['casadiobj'] = '.'
        self.options['sourcefiles'] = 'example.cpp Holonomic.cpp'
        self.options['executable'] = 'FormationPoint2Point'

    def set_options(self, options):
        self.options.update(options)
        if not os.path.isdir(self.options['directory']):
            os.makedirs(self.options['directory'])

    def export(self, updater, subdir=None):
        print 'Exporting ...',
        if subdir is None:
            export_dir = self.options['directory']
        else:
            export_dir = os.path.join(self.options['directory'], subdir)
        # copy files
        this_path = os.path.dirname(os.path.realpath(__file__))
        subdirs = ['formation', 'vehicles']
        files = {}
        no_src = ['Makefile', 'instructions.txt']
        for directory in subdirs:
            dir_path = os.path.join(this_path, directory)
            subdir_files = os.listdir(dir_path)
            for f in subdir_files:
                if f in no_src:
                    files[os.path.join(dir_path, f)] = os.path.join(
                        export_dir, f)
                else:
                    files[os.path.join(dir_path, f)] = os.path.join(
                        export_dir, 'src/', f)
        for src, des in files.items():
            des_dir = os.path.dirname(des)
            if not os.path.isdir(des_dir):
                os.makedirs(des_dir)
            shutil.copy(src, des)
        # export casadi nlp
        self.create_src(updater, export_dir)
        # create data to fill in in c++ template
        data = {}
        data.update(self.get_make_options())
        # data.update(self.create_defines())
        # data.update(self.create_types())
        # data.update(self.create_functions())
        # # write data to template files
        # self.fill_template(data, files.values())
        print 'done.'
        print 'Check out instructions.txt for build and usage instructions.'

    def fill_template(self, data, files):
        if not isinstance(files, list):
            files = [files]
        for fil in files:
            with open(fil, 'r+') as f:
                if f is not None:
                    body = f.read()
                    for key, item in data.items():
                        body = body.replace('@'+key+'@', str(item))
                    f.seek(0)
                    f.truncate()
                    f.write(body)
                else:
                    raise ValueError('File '+os.path.join(self.src_dir, fil) +
                                     ' does not exist!')

    def get_make_options(self):
        make_opt = {}
        for key, option in self.options.items():
            make_opt[key] = option
        return make_opt

    def create_src(self, updater, destination):
        cwd = os.getcwd()
        # updx
        obj = updater.father_updx.problem_description['obj']
        con = updater.father_updx.problem_description['con']
        var = updater.father_updx.problem_description['var']
        par = updater.father_updx.problem_description['par']
        opt = updater.father_updx.problem_description['opt']
        nlp = {'x': var, 'p': par, 'f': obj, 'g': con}
        options = {}
        for key, value in opt['solver_options']['ipopt'].items():
            options[key] = value
        options.update({'expand': True})
        solver = nlpsol('solver', 'ipopt', nlp, options)
        solver.generate_dependencies('updx.c')
        # updz, updl, upd_res
        updater.problem_upd_z.generate('updz.c')
        updater.problem_upd_l.generate('updl.c')
        updater.problem_upd_res.generate('updres.c')
        # move files
        shutil.move(cwd+'/updx.c', destination+'src/updx.c')
        shutil.move(cwd+'/updz.c', destination+'src/updz.c')
        shutil.move(cwd+'/updl.c', destination+'src/updl.c')
        shutil.move(cwd+'/updres.c', destination+'src/updres.c')

    def create_defines(self):
        # create defines
        defines = {}
        defines['N_VAR'] = self.father._var_struct.size
        defines['N_PAR'] = self.father._par_struct.size
        defines['N_CON'] = self.father._con_struct.size
        defines['TOL'] = self.problem.options['solver_options']['ipopt']['ipopt.tol']
        defines['LINEAR_SOLVER'] = '"' + \
            self.problem.options['solver_options']['ipopt']['ipopt.linear_solver']+'"'
        defines['N_DIM'] = self.vehicle.n_dim
        defines['N_OBS'] = self.problem.environment.n_obs
        defines['VEHICLELBL'] = '"' + self.vehicle.label + '"'
        defines['PROBLEMLBL'] = '"' + self.problem.label + '"'
        defines['OBSTACLELBLS'] = '{' + ','.join(['"' + o.label + '"'
            for o in self.problem.environment.obstacles]) + '}'
        if self.problem.__class__.__name__ == 'FreeTPoint2point':
            defines['FREET'] = 'true'
        elif self.problem.__class__.__name__ == 'FixedTPoint2point':
            defines['FREET'] = 'false'
        else:
            raise ValueError('This type of point2point problem is not ' +
                             'supported for export')
        for child in self.father.children.values():
            for name in child._splines_prim:
                spl = child._splines_prim[name]
                if spl['init'] is not None:
                    tf = '{'
                    for k in range(spl['init'].shape[0]):
                        tf += '{'+','.join([str(t)
                                            for t in spl['init'][k].tolist()])+'},'
                    tf = tf[:-1]+'}'
                    defines.update({('%s_TF') % name.upper(): tf})
        # lbg & ubg
        lb, ub, cnt = '{', '{', 0
        for child in self.father.children.values():
            for name, con in child._constraints.items():
                if con[0].size(1) > 1:
                    for l in range(con[0].size(1)):
                        lb += str(con[1][l])+','
                        ub += str(con[2][l])+','
                else:
                    lb += str(con[1])+','
                    ub += str(con[2])+','
                cnt += con[0].size(1)
        lb = lb[:-1]+'}'
        ub = ub[:-1]+'}'
        defines.update({'LBG_DEF': lb, 'UBG_DEF': ub})
        code = ''
        for name, define in defines.items():
            code += '#define ' + str(name) + ' ' + str(define) + '\n'
        return {'defines': code}

    def create_types(self):
        return {}

    def create_functions(self):
        code = {}
        code.update(self._create_getParameterVector())
        code.update(self._create_getVariableVector())
        code.update(self._create_getVariableDict())
        code.update(self._create_updateBounds())
        code.update(self._create_initSplines())
        code.update(self._create_transformSplines())
        return code

    def _create_getParameterVector(self):
        code, cnt = '', 0
        for label, child in self.father.children.items():
            for name, par in child._parameters.items():
                if par.size(1) > 1:
                    code += '\tfor (int i=0; i<'+str(par.size(1))+'; i++){\n'
                    code += ('\t\tpar_vect['+str(cnt) +
                             '+i] = par_dict["'+label+'"]["'+name+'"][i];\n')
                    code += '\t}\n'
                else:
                    code += '\tpar_vect['+str(cnt) + \
                        '] = par_dict["'+label+'"]["'+name+'"][0];\n'
                cnt += par.size(1)
        return {'getParameterVector': code}

    def _create_getVariableVector(self):
        code, cnt = '', 0
        for label, child in self.father.children.items():
            code += '\tif (var_dict.find("'+label+'") != var_dict.end()){\n'
            for name, var in child._variables.items():
                code += '\t\tif (var_dict["'+label+'"].find("' + \
                    name+'") != var_dict["'+label+'"].end()){\n'
                if var.size(1) > 1:
                    code += '\t\t\tfor (int i=0; i<' + \
                        str(var.size(1))+'; i++){\n'
                    code += ('\t\t\t\tvar_vect['+str(cnt) +
                             '+i] = var_dict["'+label+'"]["'+name+'"][i];\n')
                    code += '\t\t\t}\n'
                else:
                    code += '\t\t\tvar_vect[' + \
                        str(cnt)+'] = var_dict["'+label+'"]["'+name+'"][0];\n'
                code += '\t\t}\n'
                cnt += var.size(1)
            code += '\t}\n'
        return {'getVariableVector': code}

    def _create_getVariableDict(self):
        code, cnt = '', 0
        code += '\tvector<double> vec;'
        for label, child in self.father.children.items():
            for name, var in child._variables.items():
                if var.size(1) > 1:
                    code += '\tvec.resize('+str(var.size(1))+');\n'
                    code += '\tfor (int i=0; i<'+str(var.size(1))+'; i++){\n'
                    code += '\t\tvec[i] = var_vect['+str(cnt)+'+i];\n'
                    code += '\t}\n'
                    code += '\tvar_dict["'+label+'"]["'+name+'"] = vec;\n'
                else:
                    code += '\tvar_dict["'+label+'"]["' + \
                        name+'"] = var_vect['+str(cnt)+'];\n'
                cnt += var.size(1)
        return {'getVariableDict': code}

    def _create_updateBounds(self):
        code, cnt = '', 0
        _lbg, _ubg = self.father._lb.cat, self.father._ub.cat
        for child in self.father.children.values():
            for name, con in child._constraints.items():
                if child._add_label(name) in self.father._constraint_shutdown:
                    shutdown = self.father._constraint_shutdown[
                        child._add_label(name)]
                    cond = shutdown.replace('t', 'current_time')
                    code += '\tif(' + cond + '){\n'
                    for i in range(con[0].size(1)):
                        code += '\t\tlbg['+str(cnt+i)+'] = -INF;\n'
                        code += '\t\tubg['+str(cnt+i)+'] = +INF;\n'
                    code += '\t}else{\n'
                    for i in range(con[0].size(1)):
                        code += ('\t\tlbg['+str(cnt+i)+'] = ' +
                                 str(_lbg[cnt+i]) + ';\n')
                        code += ('\t\tubg['+str(cnt+i)+'] = ' +
                                 str(_ubg[cnt+i]) + ';\n')
                    code += '\t}\n'
                cnt += con[0].size(1)
        return {'updateBounds': code}

    def _create_initSplines(self):
        code = ''
        for label, child in self.father.children.items():
            for name in child._splines_prim:
                code += '\tsplines_tf["'+name+'"] = '+name.upper()+'_TF;\n'
        return {'initSplines': code}

    def _create_transformSplines(self):
        code, cnt = '', 0
        if self.problem.__class__.__name__ == 'FixedTPoint2point':
            code += ('\tif(((current_time > 0) and ' +
                     'fabs(fmod(round(current_time*1000.)/1000., ' +
                     'horizon_time/' +
                     str(self.vehicle.knot_intervals)+')) <1.e-6)){\n')
            code += ('\t\tvector<double> spline_tf(' +
                     str(len(self.vehicle.basis))+');\n')
            for label, child in self.father.children.items():
                for name, var in child._variables.items():
                    if name in child._splines_prim:
                        tf = 'splines_tf["'+name+'"]'
                        code += ('\t\tfor(int k=0; k<' +
                                 str(var.shape[1])+'; k++){\n')
                        code += ('\t\t\tfor(int i=0; i<' +
                                 str(var.shape[0])+'; i++){\n')
                        code += '\t\t\tspline_tf[i] = 0.0;\n'
                        code += ('\t\t\t\tfor(int j=0; j<' +
                                 str(var.shape[0])+'; j++){\n')
                        code += ('\t\t\t\t\tspline_tf[i] += '+tf +
                                 '[i][j]*variables['+str(cnt)+'+k*' +
                                 str(var.shape[0])+'+j];\n')
                        code += '\t\t\t\t}\n'
                        code += '\t\t\t}\n'
                        code += ('\t\t\tfor(int i=0; i<'+str(var.shape[0]) +
                                 '; i++){\n')
                        code += ('\t\t\t\tvariables['+str(cnt)+'+k*' +
                                 str(var.shape[0])+'+i] = spline_tf[i];\n')
                        code += '\t\t\t}\n'
                        code += '\t\t}\n'
                    cnt += var.size(1)
            code += '\t}'
        elif self.problem.__class__.__name__ == 'FreeTPoint2point':
            raise Warning('Initialization for free time problem ' +
                          'not implemented (yet?).')
        return {'transformSplines': code}
