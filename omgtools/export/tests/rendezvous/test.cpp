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

#include "RendezVous.hpp"
#include "Holonomic.hpp"
#include <ctime>
#include <iostream>
#include <fstream>
#include <assert.h>
#include <math.h>

using namespace std;

int main()
{
    int n_iter = 30;
    int init_iter = 5;
    int N = 4;
    double horizon_time = 10;
    double sample_time = 0.01;
    double update_time = 0.1;
    int trajectory_length = 20;
    vector<omg::Holonomic*> vehicles(N);
    vector<omg::RendezVous*> problems(N);
    vector<vector<double>> state0(N, vector<double>(2));
    vector<vector<double>> stateT(N, vector<double>(2));
    // these will store the state and input trajectory
    vector<vector<vector<double>>> input_trajectory(N, vector<vector<double>>(trajectory_length, vector<double>(2)));
    vector<vector<vector<double>>> state_trajectory(N, vector<vector<double>>(trajectory_length, vector<double>(2)));
    vector<vector<double>> rel_pos_c(N, vector<double>(2));
    for (int v=0; v<N; v++){
        vehicles[v] = new omg::Holonomic();
        // ideal update: prediction of initial state based on spline extrapolation
        // non-ideal update: prediction based on current state0 and model integration
        vehicles[v]->setIdealPrediction(true);
        problems[v] = new omg::RendezVous(vehicles[v], update_time, sample_time, horizon_time, trajectory_length, init_iter);
    }
    int n_shared = problems[0]->n_shared;

    state0[0] = {0.0, 3.0};
    state0[1] = {3.0, 3.0};
    state0[2] = {3.0, 0.0};
    state0[3] = {0.0, 0.0};

    stateT[0] = {0.0, 0.0};
    stateT[1] = {0.0, 0.0};
    stateT[2] = {0.0, 0.0};
    stateT[3] = {0.0, 0.0};

    rel_pos_c[0] = {0.0, -0.2};
    rel_pos_c[1] = {-0.2, 0.0};
    rel_pos_c[2] = {0.0, 0.2};
    rel_pos_c[3] = {0.2, 0.0};

    // obstacles
    vector<omg::obstacle_t> obstacles(1);
    double width = 3.0;
    double height = 0.2;
    double radius = 0.001;

    for (int k=0; k<1; k++){
        obstacles[k].position.resize(2);
        obstacles[k].velocity.resize(2);
        obstacles[k].acceleration.resize(2);
        obstacles[k].checkpoints.resize(2*4);
        obstacles[k].radii.resize(4);
        obstacles[k].checkpoints[0] = 0.5*width;
        obstacles[k].checkpoints[1] = 0.5*height;
        obstacles[k].checkpoints[2] = 0.5*width;
        obstacles[k].checkpoints[3] = -0.5*height;
        obstacles[k].checkpoints[4] = -0.5*width;
        obstacles[k].checkpoints[5] = -0.5*height;
        obstacles[k].checkpoints[6] = -0.5*width;
        obstacles[k].checkpoints[7] = 0.5*height;
        for (int i=0; i<4; i++){
            obstacles[k].radii[i] = radius;
        }
    }
    obstacles[0].position[0] = 3.2;
    obstacles[0].position[1] = 1.0;

    vector<vector<double>> x_var(N, vector<double>(n_shared));
    vector<vector<vector<double>>> x_j_var(N, vector<vector<double>>(2, vector<double>(n_shared)));
    vector<vector<vector<double>>> z_ji_var(N, vector<vector<double>>(2, vector<double>(n_shared)));
    vector<vector<vector<double>>> l_ji_var(N, vector<vector<double>>(2, vector<double>(n_shared)));
    vector<vector<vector<double>>> z_ij_var(N, vector<vector<double>>(2, vector<double>(n_shared)));
    vector<vector<vector<double>>> l_ij_var(N, vector<vector<double>>(2, vector<double>(n_shared)));
    vector<vector<double>> residuals(N, vector<double>(3));
    vector<double> total_residuals(3);

    // compare with solution from Python
    vector<vector<vector<vector<double>>>> data_state(N, vector<vector<vector<double>>>(n_iter, vector<vector<double>>(trajectory_length, vector<double>(2))));
    vector<vector<vector<vector<double>>>> data_input(N, vector<vector<vector<double>>>(n_iter, vector<vector<double>>(trajectory_length, vector<double>(2))));

    ifstream file_state, file_input;
    file_state.open("../test/data_state.csv");
    file_input.open("../test/data_input.csv");

    for (int i=0; i<n_iter; i++){
        for (int v=0; v<N; v++){
            for (int k=0; k<2; k++){
                string line_state, line_input;
                getline(file_state, line_state);
                getline(file_input, line_input);
                stringstream iss_state(line_state);
                stringstream iss_input(line_input);
                for (int j=0; j<trajectory_length; j++){
                    string val_state;
                    string val_input;
                    getline(iss_state, val_state, ',');
                    getline(iss_input, val_input, ',');
                    stringstream converter_state(val_state);
                    stringstream converter_input(val_input);
                    converter_state >> data_state[v][i][j][k];
                    converter_input >> data_input[v][i][j][k];
                }
            }
        }
    }
    file_state.close();
    file_input.close();
    double time;
    double err;
    for (int i=0; i<n_iter; i++){
        total_residuals[0] = 0.0;
        total_residuals[1] = 0.0;
        total_residuals[2] = 0.0;
        // everyone executes update1
        time = 0;
        for (int v=0; v<N; v++){
            clock_t begin = clock();
            problems[v]->update1(state0[v], stateT[v], state_trajectory[v], input_trajectory[v], x_var[v], z_ji_var[v], l_ji_var[v], obstacles, rel_pos_c[v]);
            clock_t end = clock();
            if (time < double(end-begin)/CLOCKS_PER_SEC){
                time = double(end-begin)/CLOCKS_PER_SEC;
            }
        }
        cout << "it: " << i << " upd1, " << "time: " << time << "s, ";

        // communicate x_var
        for (int v=0; v<N; v++){
            x_j_var[v][0] = x_var[(N+v+1)%N];
            x_j_var[v][1] = x_var[(N+v-1)%N];
        }

        // everyone executes update2
        time = 0;
        for (int v=0; v<N; v++){
            clock_t begin = clock();
            problems[v]->update2(x_j_var[v], z_ij_var[v], l_ij_var[v], residuals[v]);
            clock_t end = clock();
            if (time < double(end-begin)/CLOCKS_PER_SEC){
                time = double(end-begin)/CLOCKS_PER_SEC;
            }
            total_residuals[0] += residuals[v][0];
            total_residuals[1] += residuals[v][1];
            total_residuals[2] += residuals[v][2];
        }
        cout << "upd2, " << "time: " << time << "s";
        cout << " prim r: " << sqrt(total_residuals[0]) << ", dual r: " << sqrt(total_residuals[1]) << endl;

        // communicate z_ij_var, l_ij_var
        for (int v=0; v<N; v++){
            z_ji_var[v][0] = z_ij_var[(N+v+1)%N][1];
            z_ji_var[v][1] = z_ij_var[(N+v-1)%N][0];
            l_ji_var[v][0] = l_ij_var[(N+v+1)%N][1];
            l_ji_var[v][1] = l_ij_var[(N+v-1)%N][0];
        }

        if (i >= init_iter) {
            // check with solution
            for (int v=0; v<N; v++){
                for (int k=0; k<2; k++){
                    for (int j=0; j<trajectory_length; j++){
                        if (data_state[v][i-init_iter][j][k] < 1e-14){
                            err = (data_state[v][i-init_iter][j][k] - state_trajectory[v][j][k]);
                        }
                        else {
                            err = (data_state[v][i-init_iter][j][k] - state_trajectory[v][j][k])/data_state[v][i-init_iter][j][k];
                        }
                        std::cout << err << std::endl;
                        assert(err < 1e-2);
                        if (data_input[v][i-init_iter][j][k] < 1e-14){
                            err = (data_input[v][i-init_iter][j][k] - input_trajectory[v][j][k]);
                        }
                        else {
                            err = (data_input[v][i-init_iter][j][k] - input_trajectory[v][j][k])/data_input[v][i-init_iter][j][k];
                        }
                        std::cout << err << std::endl;
                        assert(err < 1e-2);
                    }
                }
            }
        }
    }
}

