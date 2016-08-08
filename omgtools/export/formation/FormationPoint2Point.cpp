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
#ifdef DEBUG
#include <ctime>
#endif
#include <unistd.h>

using namespace std;
using namespace casadi;

namespace omg{

FormationPoint2Point::FormationPoint2Point(Vehicle* vehicle, double update_time, double sample_time, double horizon_time, int trajectory_length):
parameters(N_PAR), variables(N_VAR), lbg(LBG_DEF), ubg(UBG_DEF), time(trajectory_length+1),
state_trajectory(trajectory_length+1, vector<double>(vehicle->getNState())),
input_trajectory(trajectory_length+1, vector<double>(vehicle->getNInput())),
residuals(3) {
    if (trajectory_length > int(horizon_time/sample_time)){
        cerr << "trajectory_length too large!" << endl;
    }
    this->vehicle = vehicle;
    this->update_time = update_time;
    this->sample_time = sample_time;
    this->horizon_time = horizon_time;
    this->trajectory_length = trajectory_length;
    for (int k=0; k<time.size(); k++){
        time[k] = k*sample_time;
    }
    generateProblem();
    args["p"] = parameters;
    args["x0"] = variables;
    args["lbg"] = lbg;
    args["ubg"] = ubg;
    initSplines();
}

FormationPoint2Point::FormationPoint2Point(Vehicle* vehicle, double update_time, double sample_time, double horizon_time):FormationPoint2Point(vehicle, update_time, sample_time, horizon_time, int(update_time/sample_time)){
}

void FormationPoint2Point::generateProblem(){
    string obj_path = CASADIOBJ;
    // set nlp options
    Dict options;
    options["ipopt.print_level"] = 0;
    options["print_time"] = 0;
    options["ipopt.tol"] = TOL;
    options["ipopt.linear_solver"] = LINEAR_SOLVER;
    options["ipopt.warm_start_init_point"] = "yes";
    // create problems
    this->updx_problem = nlpsol("upd_x_problem", "ipopt", obj_path+"/updx.so", options);
    this->updz_problem = external("updz_problem", obj_path+"/updz.so");
    this->updl_problem = external("updl_problem", obj_path+"/updl.so");
    this->get_residuals = external("get_residuals", obj_path+"/res.so");
}

void FormationPoint2Point::initSplines(){
@initSplines@
}

void FormationPoint2Point::reset(){
    for (int k=0; k<input_trajectory.size(); k++){
        for (int j=0; j<input_trajectory[0].size(); j++){
            input_trajectory[k][j] = 0.0;
        }
    }
}

bool FormationPoint2Point::update1(vector<double>& condition0, vector<double>& conditionT,
    vector<vector<double>>& state_trajectory, vector<vector<double>>& input_trajectory,
    vector<double>& x_var, vector<vector<double>>& z_ji_var, vector<vector<double>>& l_ji_var,
    vector<obstacle_t>& obstacles){
    update1(condition0, conditionT, state_trajectory, input_trajectory, x_var, z_ji_var, l_ji_var, consensus_vars, obstacles, 0);
}

bool FormationPoint2Point::update1(vector<double>& state0, vector<double>& conditionT,
    vector<vector<double>>& state_trajectory, vector<vector<double>>& input_trajectory,
    vector<double>& x_var, vector<vector<double>>& z_ji_var, vector<vector<double>>& l_ji_var,
    vector<obstacle_t>& obstacles, int predict_shift){
    variables_admm["z_ji"] = z_ji_var;
    variables_admm["l_ji"] = l_ji_var;
    #ifdef DEBUG
    double tmeas;
    clock_t begin;
    clock_t end;
    #endif
    // transform splines: good init guess for this update + required transformation for ADMM updates
    #ifdef DEBUG
    begin = clock();
    #endif
    transformSplines(current_time);
    #ifdef DEBUG
    end = clock();
    tmeas = double(end-begin)/CLOCKS_PER_SEC;
    cout << "time in transformSplines: " << tmeas << "s" << endl;
    #endif
    // set target condition
    #ifdef DEBUG
    begin = clock();
    #endif
    vehicle->setTerminalConditions(conditionT);
    #ifdef DEBUG
    end = clock();
    tmeas = double(end-begin)/CLOCKS_PER_SEC;
    cout << "time in setTerminalConditions: " << tmeas << "s" << endl;
    #endif
    // predict initial state and input for problem
    #ifdef DEBUG
    begin = clock();
    #endif
    if (fabs(current_time)<=1.e-6){
        vehicle->setInitialConditions(state0);
    } else{
        vehicle->predict(state0, this->state_trajectory, this->input_trajectory, update_time, sample_time, predict_shift);
    }
    #ifdef DEBUG
    end = clock();
    tmeas = double(end-begin)/CLOCKS_PER_SEC;
    cout << "time in predict: " << tmeas << "s" << endl;
    #endif
    // solve updx problem
    #ifdef DEBUG
    begin = clock();
    #endif
    bool check = solveUpdx(obstacles);
    #ifdef DEBUG
    end = clock();
    tmeas = double(end-begin)/CLOCKS_PER_SEC;
    cout << "time in solveUpdx: " << tmeas << "s" << endl;
    #endif
    // extra data
    #ifdef DEBUG
    begin = clock();
    #endif
    extractData();
    #ifdef DEBUG
    end = clock();
    tmeas = double(end-begin)/CLOCKS_PER_SEC;
    cout << "time in extractData: " << tmeas << "s" << endl;
    #endif
    // ref state and input for system are one sample shorter!!
    for (int k=0; k<time.size()-1; k++){
        for (int j=0; j<state_trajectory[0].size(); j++){
            state_trajectory[k][j] = this->state_trajectory[k][j];
        }
        for (int j=0; j<input_trajectory[0].size(); j++){
            input_trajectory[k][j] = this->input_trajectory[k][j];
        }
    }
    z_ji_var = variables_admm["z_ji"];
    l_ji_var = variables_admm["l_ji"];
    // update current time
    current_time += update_time;

    return check;
}

bool FormationPoint2Point::update2(vector<vector<double>>& x_j_var,
    vector<vector<double>>& z_ij_var, vector<vector<double>>& l_ij_var,
    vector<double> residuals){
    variables_admm["x_j"] = x_j_var;
    #ifdef DEBUG
    double tmeas;
    clock_t begin;
    clock_t end;
    #endif
    // solve updz problem
    #ifdef DEBUG
    begin = clock();
    #endif
    solveUpdz();
    #ifdef DEBUG
    end = clock();
    tmeas = double(end-begin)/CLOCKS_PER_SEC;
    cout << "time in solveUpdz: " << tmeas << "s" << endl;
    #endif
    // solve updl problem
    #ifdef DEBUG
    begin = clock();
    #endif
    solveUpdl();
    #ifdef DEBUG
    end = clock();
    tmeas = double(end-begin)/CLOCKS_PER_SEC;
    cout << "time in solveUpdl: " << tmeas << "s" << endl;
    #endif
    // compute residuals
    #ifdef DEBUG
    begin = clock();
    #endif
    compute_residuals();
    #ifdef DEBUG
    end = clock();
    tmeas = double(end-begin)/CLOCKS_PER_SEC;
    cout << "time in compute_residuals: " << tmeas << "s" << endl;
    #endif
    z_ij_var = variables_admm["z_ij"];
    l_ij_var = variables_admm["l_ij"];
    residuals = this->residuals;
    return true;
}


bool FormationPoint2Point::solveUpdx(vector<obstacle_t>& obstacles){
    // init variables if first time
    if(fabs(current_time)<=1.e-6){
        initVariables();
    }
    setParameters(obstacles);
    args["p"] = parameters;
    args["x0"] = variables;
    args["lbg"] = lbg;
    args["ubg"] = ubg;
    sol = problem(args);
    solver_output = string(problem.stats().at("return_status"));
    vector<double> var(sol.at("x"));
    for (int k=0; k<n_var; k++){
        variables[k] = var[k];
    }
    if (solver_output.compare("Solve_Succeeded") != 0){
        cout << solver_output << endl;
        return false;
    } else{
        return true;
    }
}

bool FormationPoint2Point::solveUpdz(){

}

bool FormationPoint2Point::solveUpdl(){

}

bool FormationPoint2Point::computeResiduals(){

}

void FormationPoint2Point::initVariables(){
    map<string, map<string, vector<double>>> var_dict;
    int n_spl = vehicle->getNSplines();
    int len_basis = vehicle->getLenBasis();
    vector<vector<double>> init_var_veh (n_spl, vector<double>(len_basis));
    vehicle->getInitSplineValue(init_var_veh);
    vector<double> init_var_veh_vec(n_spl*len_basis);
    for (int k=0; k<n_spl; k++){
        for (int j=0; j<len_basis; j++){
            init_var_veh_vec[k*len_basis+j] = init_var_veh[k][j];
        }
    }
    var_dict[VEHICLELBL]["splines0"] = init_var_veh_vec;
    getVariableVector(variables, var_dict);
}

void FormationPoint2Point::setParameters(vector<obstacle_t>& obstacles){
    map<string, map<string, vector<double>>> par_dict;
    map<string, vector<double>> par_dict_veh;
    vehicle->setParameters(par_dict_veh);
    par_dict[VEHICLELBL] = par_dict_veh;
    if (!freeT){
        par_dict[PROBLEMLBL]["t"] = {fmod(round(current_time*1000.)/1000., horizon_time/(vehicle->getKnotIntervals()))};
        par_dict[PROBLEMLBL]["T"] = {horizon_time};
    } else{
        par_dict[PROBLEMLBL]["t"] = {0.0};
    }
    for (int k=0; k<n_obs; k++){
        vector<double> x_obs(n_dim);
        vector<double> v_obs(n_dim);
        vector<double> a_obs(n_dim);
        for (int i=0; i<n_dim; i++){
            x_obs[i] = obstacles[k].position[i];
            v_obs[i] = obstacles[k].velocity[i];
            a_obs[i] = obstacles[k].acceleration[i];
        }
        string obstacles [N_OBS] = OBSTACLELBLS;
        par_dict[obstacles[k]]["x"] = x_obs;
        par_dict[obstacles[k]]["v"] = v_obs;
        par_dict[obstacles[k]]["a"] = a_obs;
    }
    getParameterVector(parameters, par_dict);
}

void FormationPoint2Point::extractData(){
    map<string, map<string, vector<double>>> var_dict;
    getVariableDict(variables, var_dict);
    vector<double> spline_coeffs_vec(var_dict[VEHICLELBL]["splines0"]);
    vehicle->setKnotHorizon(horizon_time);
    int n_spl = vehicle->getNSplines();
    int len_basis = vehicle->getLenBasis();
    vector<vector<double>> spline_coeffs(n_spl, vector<double>(len_basis));
    for (int k=0; k<n_spl; k++){
        for (int j=0; j<len_basis; j++){
            spline_coeffs[k][j] = spline_coeffs_vec[k*len_basis+j];
        }
    }
    retrieveTrajectories(spline_coeffs);
    retrieveXVariables(spline_coeffs);
}

void FormationPoint2Point::retrieveTrajectories(vector<vector<double>>& spline_coeffs){
    vector<double> time(this->time);
    for (int k=0; k<time.size(); k++){
        time[k] += fmod(round(current_time*1000.)/1000., horizon_time/vehicle->getKnotIntervals());
    }
    vehicle->splines2State(spline_coeffs, time, state_trajectory);
    vehicle->splines2Input(spline_coeffs, time, input_trajectory);
}

void FormationPoint2Point::retrieveXVariables(vector<vector<double>>& spline_coeffs){
@retrieveXVariables@
}


void FormationPoint2Point::getParameterVector(vector<double>& par_vect, map<string, map<string, vector<double>>>& par_dict){
@getParameterVector@
}

void FormationPoint2Point::getVariableVector(vector<double>& var_vect, map<string, map<string, vector<double>>>& var_dict){
@getVariableVector@
}

void FormationPoint2Point::getVariableDict(vector<double>& var_vect, map<string, map<string, vector<double>>>& var_dict){
@getVariableDict@
}

void FormationPoint2Point::updateBounds(double current_time){
@updateBounds@
}

void FormationPoint2Point::transformSplines(double current_time){
@transformSplines@
}

}
