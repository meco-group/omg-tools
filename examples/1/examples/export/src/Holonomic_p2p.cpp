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

#include "Holonomic.hpp"

using namespace std;

namespace omg{

Holonomic::Holonomic() : Vehicle(2, 2, 2, 3), positionT(2){

}

void Holonomic::setInitialConditions(vector<double>& conditions){
    vector<double> zeros(2);
    setPrediction(conditions, zeros);
}

void Holonomic::setTerminalConditions(vector<double>& conditions){
    this->positionT = conditions;
}

void Holonomic::getInitSplineValue(vector<vector<double>>& init_value){
    vector<double> state0(2);
    vector<double> input0(2);
    int n_spl = getNSplines();
    int degree = getDegree();
    int len_basis = getLenBasis();
    getPrediction(state0, input0);
    // for(int k=0; k<n_spl; k++){
    //     for(int d=0; d<degree; d++){
    //         init_value[k][d] = state0[k];
    //         init_value[k][len_basis-degree+d] = this->positionT[k];
    //     }
    //     for (int j=0; j<len_basis-2*degree; j++){
    //         init_value[k][degree+j] = state0[k]+j*(this->positionT[k]-state0[k])/(len_basis-2*degree-1);
    //     }
    // }
    for(int k=0; k<n_spl; k++){
        for(int j=0; j<len_basis; j++){
            init_value[k][j] = state0[k]+j*(this->positionT[k]-state0[k])/(len_basis-1);
        }
    }
}

void Holonomic::setParameters(map<string,vector<double>>& parameters){
    vector<double> state0(2);
    vector<double> input0(2);
    getPrediction(state0, input0);
    parameters["state0"] = state0;
    parameters["input0"] = input0;
    parameters["positionT"] = this->positionT;
}

void Holonomic::splines2State(vector<vector<double>>& spline_coeffs, vector<double> time, vector<vector<double>>& state){
    this->sampleSplines(spline_coeffs, time, state);
}

void Holonomic::splines2Input(vector<vector<double>>& spline_coeffs, vector<double> time, vector<vector<double>>& input){
    this->sampleSplines(spline_coeffs, time, 1, input);
}

void Holonomic::ode(vector<double>& state, vector<double>& input, vector<double>& dstate){
    dstate = input;
}


}
