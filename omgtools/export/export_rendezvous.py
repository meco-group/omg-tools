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

from export_admm import ExportADMM


class ExportRendezVous(ExportADMM):

    def __init__(self, problem, options):
        ExportADMM.__init__(self, problem, options,
            ['point2point/admm/rendezvous', 'tests/rendezvous'], ['RendezVous.cpp'])

    def set_default_options(self):
        ExportADMM.set_default_options(self)
        self.options['executable'] = 'RendezVous'
