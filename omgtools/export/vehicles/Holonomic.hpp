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

#ifndef HOLONOMIC
#define HOLONOMIC

#include "Vehicle.hpp"

namespace omg{

class Holonomic: public Vehicle{
    private:
        std::vector<double> poseT;

    public:
        Holonomic();

        void setInitialConditions(std::vector<double>& conditions);
        void setTerminalConditions(std::vector<double>& conditions);
        void setParameters(std::map<std::string,std::vector<double>>& par_dict);
        void ode(std::vector<double>& state, std::vector<double>& input, std::vector<double>& dstate);
        void getInitSplineValue(std::vector<std::vector<double>>& init_value);
        void splines2State(std::vector<std::vector<double>>& spline_coeffs, std::vector<double> time, std::vector<std::vector<double>>& state);
        void splines2Input(std::vector<std::vector<double>>& spline_coeffs, std::vector<double> time, std::vector<std::vector<double>>& input);
    };
}

#endif
