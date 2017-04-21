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
import sys

n_parts, part = int(sys.argv[1]), int(sys.argv[2])
directory = os.path.join(os.getcwd(), 'examples')
files = os.listdir(directory)
example_files = []
for f in files:
    if os.path.isfile(os.path.join(directory, f)) and f.endswith('.py'):
        example_files.append(os.path.join(directory, f))

n_files = len(example_files)/n_parts
if part == n_parts:
    test_files = example_files[(part-1)*n_files:]
else:
    test_files = example_files[(part-1)*n_files:part*n_files]
print ','.join(test_files)
