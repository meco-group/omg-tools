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

#include "ADMMPoint2Point.hpp"
#ifdef DEBUG
#include <ctime>
#endif
#include <unistd.h>

using namespace std;
using namespace casadi;

namespace omg{

ADMMPoint2Point::ADMMPoint2Point(Vehicle* vehicle,
    double update_time, double sample_time, double horizon_time,
    int trajectory_length, int init_iter, double rho):
Point2Point(vehicle, update_time, sample_time, horizon_time, trajectory_length, false),
residuals(3) {
    this->init_iter = init_iter;
    this->rho = rho;
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
    initialize();
}

ADMMPoint2Point::ADMMPoint2Point(Vehicle* vehicle,
    double update_time, double sample_time, double horizon_time):
ADMMPoint2Point(vehicle, update_time, sample_time, horizon_time, int(update_time/sample_time), INITITER, RHO){
}

ADMMPoint2Point::ADMMPoint2Point(Vehicle* vehicle,
    double update_time, double sample_time, double horizon_time,
    int trajectory_length):
ADMMPoint2Point(vehicle, update_time, sample_time, horizon_time, trajectory_length, INITITER, RHO){
}

ADMMPoint2Point::ADMMPoint2Point(Vehicle* vehicle,
    double update_time, double sample_time, double horizon_time,
    int trajectory_length, int init_iter):
ADMMPoint2Point(vehicle, update_time, sample_time, horizon_time, trajectory_length, init_iter, RHO){
}

void ADMMPoint2Point::generateProblem(){
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

void ADMMPoint2Point::reset(){
    Point2Point::reset();
}

void ADMMPoint2Point::resetTime(){
    Point2Point::resetTime();
    iteration = 0;
}

int ADMMPoint2Point::getIteration(){
    return iteration;
}

bool ADMMPoint2Point::update1(vector<double>& condition0, vector<double>& conditionT,
    vector<vector<double>>& state_trajectory, vector<vector<double>>& input_trajectory,
    vector<double>& x_var, vector<vector<double>>& z_ji_var, vector<vector<double>>& l_ji_var,
    vector<obstacle_t>& obstacles){
    update1(condition0, conditionT, state_trajectory, input_trajectory, x_var, z_ji_var, l_ji_var, obstacles, 0);
}

bool ADMMPoint2Point::update1(vector<double>& condition0, vector<double>& conditionT,
    vector<vector<double>>& state_trajectory, vector<vector<double>>& input_trajectory,
    vector<double>& x_var, vector<vector<double>>& z_ji_var, vector<vector<double>>& l_ji_var,
    vector<obstacle_t>& obstacles, int predict_shift){
    // start with own initial guess
    if (iteration > 0){
        for (int i=0; i<n_nghb; i++){
            for (int j=0; j<n_shared; j++){
                variables_admm["z_ji"][i*n_shared+j] = z_ji_var[i][j];
                variables_admm["l_ji"][i*n_shared+j] = l_ji_var[i][j];
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
    transformSharedSplines(current_time);
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
    // return x_i
    x_var = variables_admm["x_i"];
    // update current time
    if (iteration >= init_iter){
        current_time += update_time;
    }
    iteration++;
    return check;
}

bool ADMMPoint2Point::update2(vector<vector<double>>& x_j_var,
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

bool ADMMPoint2Point::solveUpdx(vector<obstacle_t>& obstacles){
    // first set parameters before initVariablesADMM!
    Point2Point::setParameters(obstacles);
    // init variables if first time
    if(iteration == 0){
        Point2Point::initVariables();
        initVariablesADMM();
        // set parameters again with init admm variables
        Point2Point::setParameters(obstacles);
    }
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

bool ADMMPoint2Point::solveUpdz(){
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

bool ADMMPoint2Point::solveUpdl(){
    variables_admm["l_i_p"] = variables_admm["l_i"];
    variables_admm["l_ij_p"] = variables_admm["l_ij_p"];
    vector<vector<double>> res;
    updl_problem({variables_admm["x_i"], variables_admm["z_i"],
        variables_admm["z_ij"], variables_admm["l_i"], variables_admm["l_ij"],
        variables_admm["x_j"], {rho}}, res);
    variables_admm["l_i"] = res.at(0);
    variables_admm["l_ij"] = res.at(1);
}

bool ADMMPoint2Point::computeResiduals(){
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

void ADMMPoint2Point::initVariablesADMM(){
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

void ADMMPoint2Point::fillParameterDict(vector<obstacle_t>& obstacles, map<string, map<string, vector<double>>>& par_dict){
    Point2Point::fillParameterDict(obstacles, par_dict);
    par_dict[ADMMLBL]["z_i"] = variables_admm["z_i"];
    par_dict[ADMMLBL]["z_ji"] = variables_admm["z_ji"];
    par_dict[ADMMLBL]["l_i"] = variables_admm["l_i"];
    par_dict[ADMMLBL]["l_ji"] = variables_admm["l_ji"];
    par_dict[ADMMLBL]["rho"] = {rho};
}

void ADMMPoint2Point::extractData(){
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

void ADMMPoint2Point::retrieveSharedVariables(map<string, map<string, vector<double>>>& var_dict){
@retrieveSharedVariables@
}

void ADMMPoint2Point::transformSharedSplines(double current_time){
@transformSharedSplines@
}

}
