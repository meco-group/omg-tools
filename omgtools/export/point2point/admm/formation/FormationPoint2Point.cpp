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

#include "FormationPoint2Point.hpp"

using namespace std;
using namespace casadi;

namespace omg{

FormationPoint2Point::FormationPoint2Point(Vehicle* vehicle,
    double update_time, double sample_time, double horizon_time,
    int trajectory_length, int init_iter):
ADMMPoint2Point(vehicle, update_time, sample_time, horizon_time, trajectory_length, init_iter) {

}

FormationPoint2Point::FormationPoint2Point(Vehicle* vehicle,
    double update_time, double sample_time, double horizon_time):
ADMMPoint2Point(vehicle, update_time, sample_time, horizon_time, int(update_time/sample_time), INITITER){
}

FormationPoint2Point::FormationPoint2Point(Vehicle* vehicle,
    double update_time, double sample_time, double horizon_time,
    int trajectory_length):
ADMMPoint2Point(vehicle, update_time, sample_time, horizon_time, trajectory_length, INITITER){
}


bool FormationPoint2Point::update1(vector<double>& condition0, vector<double>& conditionT,
    vector<vector<double>>& state_trajectory, vector<vector<double>>& input_trajectory,
    vector<double>& x_var, vector<vector<double>>& z_ji_var, vector<vector<double>>& l_ji_var,
    vector<obstacle_t>& obstacles, vector<double>& rel_pos_c){
    update1(condition0, conditionT, state_trajectory, input_trajectory, x_var, z_ji_var, l_ji_var, obstacles, rel_pos_c, 0);
}

bool FormationPoint2Point::update1(vector<double>& condition0, vector<double>& conditionT,
    vector<vector<double>>& state_trajectory, vector<vector<double>>& input_trajectory,
    vector<double>& x_var, vector<vector<double>>& z_ji_var, vector<vector<double>>& l_ji_var,
    vector<obstacle_t>& obstacles, vector<double>& rel_pos_c, int predict_shift){
    this->rel_pos_c = rel_pos_c;
    ADMMPoint2Point::update1(condition0, conditionT, state_trajectory, input_trajectory,
        x_var, z_ji_var, l_ji_var, obstacles, predict_shift);
}

bool FormationPoint2Point::update2(vector<vector<double>>& x_j_var,
    vector<vector<double>>& z_ij_var, vector<vector<double>>& l_ij_var,
    vector<double>& residuals){
    ADMMPoint2Point::update2(x_j_var, z_ij_var, l_ij_var, residuals);
}

void FormationPoint2Point::fillParameterDict(vector<obstacle_t>& obstacles, map<string, map<string, vector<double>>>& par_dict){
    ADMMPoint2Point::fillParameterDict(obstacles, par_dict);
    par_dict[VEHICLELBL]["rel_pos_c"] = this->rel_pos_c;
}

}
