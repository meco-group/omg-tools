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

#include "Point2Point.hpp"
#ifdef DEBUG
#include <ctime>
#endif
#include <unistd.h>

using namespace std;
using namespace casadi;

namespace omg{

Point2Point::Point2Point(Vehicle* vehicle, double update_time, double sample_time, double horizon_time, int trajectory_length):
parameters(N_PAR), variables(N_VAR), lbg(LBG_DEF), ubg(UBG_DEF), time(trajectory_length+1),
state_trajectory(trajectory_length+1, vector<double>(vehicle->getNState())),
input_trajectory(trajectory_length+1, vector<double>(vehicle->getNInput())) {
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

Point2Point::Point2Point(Vehicle* vehicle, double update_time, double sample_time, double horizon_time):Point2Point(vehicle, update_time, sample_time, horizon_time, int(update_time/sample_time)){
}

void Point2Point::generateProblem(){
    // change pwd to CASADIOBJ
    char cwd[1024];
    getcwd(cwd, sizeof(cwd));
    chdir(CASADIOBJ);
    // load object files
    ExternalFunction grad_f("grad_f");
    ExternalFunction jac_g("jac_g");
    ExternalFunction hess_lag("hess_lag");
    ExternalFunction nlp("nlp");
    // change back to original pwd
    chdir(cwd);
    // set options
    Dict options;
    options["grad_f"] = grad_f;
    options["jac_g"] = jac_g;
    options["jac_g"] = jac_g;
    options["hess_lag"] = hess_lag;
    options["print_level"] = 0;
    options["print_time"] = 0;
    options["tol"] = TOL;
    options["linear_solver"] = LINEAR_SOLVER;
    options["warm_start_init_point"] = "yes";
    // create nlp solver
    NlpSolver problem("problem", "ipopt", nlp, options);
    this->problem = problem;
}

void Point2Point::initSplines(){
@initSplines@
}

bool Point2Point::update(vector<double>& condition0, vector<double>& conditionT, vector<vector<double>>& state_trajectory, vector<vector<double>>& input_trajectory, vector<obstacle_t>& obstacles){
    update(condition0, conditionT, state_trajectory, input_trajectory, obstacles, 0);
}

bool Point2Point::update(vector<double>& state0, vector<double>& conditionT, vector<vector<double>>& state_trajectory, vector<vector<double>>& input_trajectory, vector<obstacle_t>& obstacles, int predict_shift){
    #ifdef DEBUG
    double tmeas;
    clock_t begin;
    clock_t end;
    #endif
    // transform splines: good init guess for this update
    #ifdef DEBUG
    begin = clock();
    #endif
    transformSplines(current_time);
    #ifdef DEBUG
    end = clock();
    tmeas = double(end-begin)/CLOCKS_PER_SEC;
    cout << "time in transformSplines: " << tmeas << "s" << endl;
    #endif
    // predict initial y for problem
    #ifdef DEBUG
    begin = clock();
    #endif
    if (fabs(current_time)<=1.e-6){
        vehicle->setInitialConditions(state0);
    } else{
        vehicle->predict(state0, state_trajectory, input_trajectory, update_time, sample_time, predict_shift);
    }
    #ifdef DEBUG
    end = clock();
    tmeas = double(end-begin)/CLOCKS_PER_SEC;
    cout << "time in predict: " << tmeas << "s" << endl;
    #endif
    // solve problem
    #ifdef DEBUG
    begin = clock();
    #endif
    bool check = solve(obstacles);
    #ifdef DEBUG
    end = clock();
    tmeas = double(end-begin)/CLOCKS_PER_SEC;
    cout << "time in solve: " << tmeas << "s" << endl;
    #endif
    // retrieve splines
    #ifdef DEBUG
    begin = clock();
    #endif
    retrieveTrajectories();
    #ifdef DEBUG
    end = clock();
    tmeas = double(end-begin)/CLOCKS_PER_SEC;
    cout << "time in retrieveTrajectories: " << tmeas << "s" << endl;
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
    // update current time
    current_time += update_time;

    return check;
}

bool Point2Point::solve(vector<obstacle_t>& obstacles){
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
    solver_output = string(problem.getStats().at("return_status"));
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

void Point2Point::initVariables(){
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
    var_dict["vehicle0"]["splines0"] = init_var_veh_vec;
    getVariableVector(variables, var_dict);
}

void Point2Point::setParameters(vector<obstacle_t>& obstacles){
    map<string, map<string, vector<double>>> par_dict;
    map<string, vector<double>> par_dict_veh;
    vehicle->setParameters(par_dict_veh);
    par_dict["vehicle0"] = par_dict_veh;
    if (!freeT){
        par_dict["p2p0"]["t"] = {fmod(round(current_time*1000.)/1000., horizon_time/(vehicle->getKnotIntervals()))};
        par_dict["p2p0"]["T"] = {horizon_time};
    } else{
        par_dict["p2p0"]["t"] = {0.0};
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
        par_dict["obstacle"+to_string(k)]["x"] = x_obs;
        par_dict["obstacle"+to_string(k)]["v"] = v_obs;
        par_dict["obstacle"+to_string(k)]["a"] = a_obs;
    }
    getParameterVector(parameters, par_dict);
}

void Point2Point::retrieveTrajectories(){
    map<string, map<string, vector<double>>> var_dict;
    getVariableDict(variables, var_dict);
    vector<double> spline_coeffs_vec(var_dict["vehicle0"]["splines"]);
    if (freeT){
        horizon_time = var_dict["p2p0"]["T"][0];
    }
    vehicle->setKnotHorizon(horizon_time);
    int n_spl = vehicle->getNSplines();
    int len_basis = vehicle->getLenBasis();
    vector<vector<double>> spline_coeffs(n_spl, vector<double>(len_basis));
    for (int k=0; k<n_spl; k++){
        for (int j=0; j<len_basis; j++){
            spline_coeffs[k][j] = spline_coeffs_vec[k*len_basis+j];
        }
    }
    if (!freeT){
        vector<double> time(this->time);
        for (int k=0; k<time.size(); k++){
            time[k] += fmod(round(current_time*1000.)/1000., horizon_time/vehicle->getKnotIntervals());
        }
    }
    vehicle->splines2State(spline_coeffs, time, state_trajectory);
    vehicle->splines2Input(spline_coeffs, time, input_trajectory);
}


void Point2Point::getParameterVector(vector<double>& par_vect, map<string, map<string, vector<double>>>& par_dict){
@getParameterVector@
}

void Point2Point::getVariableVector(vector<double>& var_vect, map<string, map<string, vector<double>>>& var_dict){
@getVariableVector@
}

void Point2Point::getVariableDict(vector<double>& var_vect, map<string, map<string, vector<double>>>& var_dict){
@getVariableDict@
}

void Point2Point::updateBounds(double current_time){
@updateBounds@
}

void Point2Point::transformSplines(double current_time){
@transformSplines@
}


}
