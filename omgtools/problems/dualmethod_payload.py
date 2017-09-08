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

from dualmethod import DualUpdater, DualProblem
from dualmethod import _create_struct_from_dict
from casadi.tools import struct, entry, structure
from casadi import SX, MX, DM

class DualUpdater_payload(DualUpdater):

    def __init__(self, index, vehicle, problem, environment, distr_problem,
                 label, options=None):
        DualUpdater.__init__(self,index, vehicle, problem, environment, distr_problem, label)

    # ========================================================================
    # Auxiliary methods
    # ========================================================================

    def _struct2dict_payload(self, var, dic):
        from admm_payload import ADMM_PAYLOAD
        from dualdecomposition import DDUpdater
        if isinstance(var, list):
            return [self._struct2dict_payload(v, dic) for v in var]
        elif isinstance(dic.keys()[0], (DDUpdater, ADMM_PAYLOAD)):
            ret = {}
            for nghb in dic.keys():
                ret[nghb.label] = {}
                for child, q in dic[nghb].items():
                    ret[nghb.label][child.label] = {}
                    for name in q.keys():
                        ret[nghb.label][child.label][name] = var[
                            nghb.label, child.label, name]
            return ret
        else:
            ret = {}
            for child, q in dic.items():
                ret[child.label] = {}
                for name in q.keys():
                    ret[child.label][name] = var[child.label, name]
            return ret

    def _dict2struct_payload(self, var, stru):
        if isinstance(var, list):
            return [self._dict2struct_payload(v, stru) for v in var]
        elif 'dd' in var.keys()[0] or 'admm_payload' in var.keys()[0]:
            chck = var.values()[0].values()[0].values()[0]
            if isinstance(chck, SX):
                ret = structure.SXStruct(stru)
            elif isinstance(chck, MX):
                ret = structure.MXStruct(stru)
            elif isinstance(chck, DM):
                ret = stru(0)
            for nghb in var.keys():
                for child, q in var[nghb].items():
                    for name in q.keys():
                        ret[nghb, child, name] = var[nghb][child][name]
            return ret
        else:
            chck = var.values()[0].values()[0]
            if isinstance(chck, SX):
                ret = structure.SXStruct(stru)
            elif isinstance(chck, MX):
                ret = structure.MXStruct(stru)
            elif isinstance(chck, DM):
                ret = stru(0)
            for child, q in var.items():
                for name in q.keys():
                    ret[child, name] = var[child][name]
            return ret

    def _transform_spline_payload(self, var, tf, dic):
        from admm_payload import ADMM_PAYLOAD
        from dualdecomposition import DDUpdater
        if isinstance(var, list):
            return [self._transform_spline_payload(v, tf, dic) for v in var]
        elif isinstance(var, struct):
            var = self._struct2dict_payload(var, dic)
            var = self._transform_spline_payload(var, tf, dic)
            return self._dict2struct_payload(var, _create_struct_from_dict(dic))
        elif isinstance(dic.keys()[0], (DDUpdater, ADMM_PAYLOAD)):
            ret = {}
            for nghb in dic.keys():
                ret[nghb.label] = self._transform_spline_payload(
                    var[nghb.label], tf, dic[nghb])
            return ret
        else:
            for child, q_i in dic.items():
                for name, ind in q_i.items():
                    if name in child._splines_prim:
                        basis = child._splines_prim[name]['basis']
                        for l in range(len(basis)):
                            sl_min = l*len(basis)
                            sl_max = (l+1)*len(basis)
                            if set(range(sl_min, sl_max)) <= set(ind):
                                sl = slice(sl_min-ind[0], sl_max-ind[0])
                                v = var[child.label][name][sl]
                                v = tf(v, basis)
                                var[child.label][name][sl] = v
            return var




class DualProblem_payload(DualProblem):

    def __init__(self, fleet, environment, problems, updater_type, options):
        DualProblem.__init__(self, fleet, environment, problems, updater_type, options)
