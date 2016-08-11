// This file is part of OMG-tools.

// OMG-tools -- Optimal Motion Generation-tools
// Copyright (C) 2016 Ruben Van Parys & Tim Mercy, KU Leuven.
// All rights reserved.

// OMG-tools is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA

#ifndef RENDEZVOUS
#define RENDEZVOUS

#include "ADMMPoint2Point.hpp"

namespace omg{

class RendezVous: public ADMMPoint2Point{
    private:
        std::vector<double> rel_pos_c;
        void fillParameterDict(std::vector<obstacle_t>&, std::map<std::string, std::map<std::string, std::vector<double>>>&);

    public:
        RendezVous(Vehicle* vehicle, double update_time, double sample_time, double horizon_time);
        RendezVous(Vehicle* vehicle, double update_time, double sample_time, double horizon_time, int trajectory_length);
        RendezVous(Vehicle* vehicle, double update_time, double sample_time, double horizon_time, int trajectory_length, int init_iter);
        bool update1(std::vector<double>&, std::vector<double>&, std::vector<std::vector<double>>&, std::vector<std::vector<double>>&, std::vector<double>&, std::vector<std::vector<double>>&, std::vector<std::vector<double>>&, std::vector<obstacle_t>&, std::vector<double> &);
        bool update1(std::vector<double>&, std::vector<double>&, std::vector<std::vector<double>>&, std::vector<std::vector<double>>&, std::vector<double>&, std::vector<std::vector<double>>&, std::vector<std::vector<double>>&, std::vector<obstacle_t>&, std::vector<double> &, int);
        bool update2(std::vector<std::vector<double>>&, std::vector<std::vector<double>>&, std::vector<std::vector<double>>&, std::vector<double>&);
    };
}

#endif
