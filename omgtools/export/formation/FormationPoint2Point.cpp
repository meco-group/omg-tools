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
residuals(3), rho(RHO) {
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
    generateSubstituteFunctions();
    args["p"] = parameters;
    args["x0"] = variables;
    args["lbg"] = lbg;
    args["ubg"] = ubg;

    vector<double> zeros1(n_shared);
    vector<double> zeros2(n_nghb*n_shared);
    variables_admm["x_i"] = zeros1;
    variables_admm["z_i"] = zeros1;
    variables_admm["l_i"] = zeros1;
    variables_admm["x_j"] = zeros2;
    variables_admm["z_ij"] = zeros2;
    variables_admm["l_ij"] = zeros2;
    variables_admm["z_ji"] = zeros2;
    variables_admm["l_ji"] = zeros2;
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
    this->updz_problem = external(UPDZPROBLEM, obj_path+"/updz.so");
    this->updl_problem = external(UPDLPROBLEM, obj_path+"/updl.so");
    this->get_residuals = external(UPDRESPROBLEM, obj_path+"/updres.so");
}

void FormationPoint2Point::generateSubstituteFunctions(){
@generateSubstituteFunctions@
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
    vector<obstacle_t>& obstacles, vector<double>& rel_pos_c){
    update1(condition0, conditionT, state_trajectory, input_trajectory, x_var, z_ji_var, l_ji_var, obstacles, rel_pos_c, 0);
}

bool FormationPoint2Point::update1(vector<double>& condition0, vector<double>& conditionT,
    vector<vector<double>>& state_trajectory, vector<vector<double>>& input_trajectory,
    vector<double>& x_var, vector<vector<double>>& z_ji_var, vector<vector<double>>& l_ji_var,
    vector<obstacle_t>& obstacles, vector<double>& rel_pos_c, int predict_shift){
    // start with own initial guess
    if (fabs(current_time)>1.e-6){
        for (int i=0; i<n_nghb; i++){
            for (int j=0; j<n_shared; j++){
                variables_admm["z_ji"][i*n_shared+j] = z_ji_var[j][i];
                variables_admm["l_ji"][i*n_shared+j] = l_ji_var[j][i];
            }
        }
    }
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
        vehicle->setInitialConditions(condition0);
    } else{
        vehicle->predict(condition0, this->state_trajectory, this->input_trajectory, update_time, sample_time, predict_shift);
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
    bool check = solveUpdx(obstacles, rel_pos_c);
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
    // return x_i
    x_var = variables_admm["x_i"];
    // update current time
    current_time += update_time;

    return check;
}

bool FormationPoint2Point::update2(vector<vector<double>>& x_j_var,
    vector<vector<double>>& z_ij_var, vector<vector<double>>& l_ij_var,
    vector<double>& residuals){
    for (int i=0; i<n_nghb; i++){
        for (int j=0; j<n_shared; j++){
            variables_admm["x_j"][i*n_shared+j] = x_j_var[i][j];
        }
    }
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
    computeResiduals();
    #ifdef DEBUG
    end = clock();
    tmeas = double(end-begin)/CLOCKS_PER_SEC;
    cout << "time in compute_residuals: " << tmeas << "s" << endl;
    #endif
    // return z_ij, l_ij, residuals
    for (int i=0; i<n_nghb; i++){
        for (int j=0; j<n_shared; j++){
            z_ij_var[i][j] = variables_admm["z_ij"][i*n_shared+j];
            l_ij_var[i][j] = variables_admm["l_ij"][i*n_shared+j];
        }
    }
    residuals = this->residuals;
    return true;
}

bool FormationPoint2Point::solveUpdx(vector<obstacle_t>& obstacles, vector<double>& rel_pos_c){
    // init variables if first time
    if(fabs(current_time)<=1.e-6){
        initVariables();
        initVariablesADMM();
    }
    setParameters(obstacles, rel_pos_c);
    args["p"] = parameters;
    args["x0"] = variables;
    args["lbg"] = lbg;
    args["ubg"] = ubg;
    sol = updx_problem(args);
    solver_output = string(updx_problem.stats().at("return_status"));
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
    variables_admm["z_i_p"] = variables_admm["z_i"];
    variables_admm["z_ij_p"] = variables_admm["z_ij"];
    vector<vector<double>> res;
    updz_problem({variables_admm["x_i"], variables_admm["l_i"],
        variables_admm["l_ij"], variables_admm["x_j"],
        {fmod(round(current_time*1000.)/1000., horizon_time/(vehicle->getKnotIntervals()))},
        {horizon_time}, {rho}, {}}, res);
    variables_admm["z_i"] = res.at(0);
    variables_admm["z_ij"] = res.at(1);
}

bool FormationPoint2Point::solveUpdl(){
    variables_admm["l_i_p"] = variables_admm["l_i"];
    variables_admm["l_ij_p"] = variables_admm["l_ij_p"];
    vector<vector<double>> res;
    updl_problem({variables_admm["x_i"], variables_admm["z_i"],
        variables_admm["z_ij"], variables_admm["l_i"], variables_admm["l_ij"],
        variables_admm["x_j"], {rho}}, res);
    variables_admm["l_i"] = res.at(0);
    variables_admm["l_ij"] = res.at(1);
}

bool FormationPoint2Point::computeResiduals(){
    vector<vector<double>> res;
    get_residuals({variables_admm["x_i"], variables_admm["z_i"],
        variables_admm["z_i_p"], variables_admm["z_ij"], variables_admm["z_ij_p"],
        variables_admm["x_j"],
        {fmod(round(current_time*1000.)/1000., horizon_time/(vehicle->getKnotIntervals()))},
        {horizon_time}, {rho}}, res);
    residuals[0] = res.at(0).at(0);
    residuals[1] = res.at(1).at(0);
    residuals[2] = res.at(2).at(0);
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

void FormationPoint2Point::initVariablesADMM(){
    map<string, map<string, vector<double>>> var_dict;
    getVariableDict(variables, var_dict);
    // init x_i
    retrieveSharedVariables(var_dict);
    // init z variables
    variables_admm["z_i"] = variables_admm["x_i"];
    vector<double> z_ji(n_nghb*n_shared);
    for (int i=0; i<n_nghb; i++){
        for (int j=0; j<n_shared; j++){
            z_ji[i*n_shared+j] = variables_admm["x_i"][j];
        }
    }
    variables_admm["z_ji"] = z_ji;
}

void FormationPoint2Point::setParameters(vector<obstacle_t>& obstacles, vector<double>& rel_pos_c){
    map<string, map<string, vector<double>>> par_dict;
    map<string, vector<double>> par_dict_veh;
    vehicle->setParameters(par_dict_veh);
    par_dict[VEHICLELBL] = par_dict_veh;
    par_dict[VEHICLELBL]["rel_pos_c"] = rel_pos_c;
    if (!freeT){
        par_dict[P2PLBL]["t"] = {fmod(round(current_time*1000.)/1000., horizon_time/(vehicle->getKnotIntervals()))};
        par_dict[P2PLBL]["T"] = {horizon_time};
    } else{
        par_dict[P2PLBL]["t"] = {0.0};
    }
    par_dict[ADMMLBL]["z_i"] = variables_admm["z_i"];
    par_dict[ADMMLBL]["z_ji"] = variables_admm["z_ji"];
    par_dict[ADMMLBL]["l_i"] = variables_admm["l_i"];
    par_dict[ADMMLBL]["l_ji"] = variables_admm["l_ji"];
    par_dict[ADMMLBL]["rho"] = {rho};
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
    retrieveSharedVariables(var_dict);
}

void FormationPoint2Point::retrieveTrajectories(vector<vector<double>>& spline_coeffs){
    vector<double> time(this->time);
    for (int k=0; k<time.size(); k++){
        time[k] += fmod(round(current_time*1000.)/1000., horizon_time/vehicle->getKnotIntervals());
    }
    vehicle->splines2State(spline_coeffs, time, state_trajectory);
    vehicle->splines2Input(spline_coeffs, time, input_trajectory);
}

void FormationPoint2Point::retrieveSharedVariables(map<string, map<string, vector<double>>>& var_dict){
@retrieveSharedVariables@
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
