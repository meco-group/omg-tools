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
#include "Holonomic.hpp"
#include <ctime>
#include <iostream>
#include <fstream>
#include <assert.h>

using namespace std;

int main()
{
    int n_iter = 1;
    int N = 4;
    double horizon_time = 10;
    double sample_time = 0.01;
    double update_time = 0.1;
    int trajectory_length = 20;
    vector<omg::Holonomic*> vehicles(N);
    vector<omg::FormationPoint2Point*> problems(N);
    vector<vector<double>> state0(N, vector<double>(2));
    vector<vector<double>> stateT(N, vector<double>(2));
    // these will store the state and input trajectory
    vector<vector<vector<double>>> input_trajectory(N, vector<vector<double>>(trajectory_length, vector<double>(2)));
    vector<vector<vector<double>>> state_trajectory(N, vector<vector<double>>(trajectory_length, vector<double>(2)));
    vector<vector<double>> rel_pos_c(N, vector<double>(2));
    // ideal update: prediction of initial state based on spline extrapolation
    // non-ideal update: prediction based on current state0 and model integration
    for (int v=0; v<N; v++){
        vehicles[v] = new omg::Holonomic();
        vehicles[v]->setIdealPrediction(true);
        problems[v] = new omg::FormationPoint2Point(vehicles[0], update_time, sample_time, horizon_time, trajectory_length);
    }
    int n_shared = problems[0]->n_shared;

    state0[0] = {0.0, 0.2};
    state0[1] = {0.2, 0.0};
    state0[2] = {0.0, -0.2};
    state0[3] = {-0.2, 0.0};

    stateT[0] = {3.5, 3.7};
    stateT[1] = {3.7, 3.5};
    stateT[2] = {3.5, 3.3};
    stateT[3] = {3.3, 3.5};

    for (int v=0; v<N; v++){
        rel_pos_c[v][0] = -state0[v][0];
        rel_pos_c[v][1] = -state0[v][1];
    }

    // obstacles
    vector<omg::obstacle_t> obstacles(problems[0]->n_obs);
    obstacles[0].position[0] = -0.6;
    obstacles[0].position[1] = 1.0;
    obstacles[0].velocity[0] = 0.0;
    obstacles[0].velocity[1] = 0.0;
    obstacles[0].acceleration[0] = 0.0;
    obstacles[0].acceleration[1] = 0.0;

    obstacles[1].position[0] = 3.2;
    obstacles[1].position[1] = 1.0;
    obstacles[1].velocity[0] = 0.0;
    obstacles[1].velocity[1] = 0.0;
    obstacles[1].acceleration[0] = 0.0;
    obstacles[1].acceleration[1] = 0.0;

    vector<vector<double>> x_var(N, vector<double>(n_shared));
    vector<vector<vector<double>>> x_j_var(N, vector<vector<double>>(2, vector<double>(n_shared)));
    vector<vector<vector<double>>> z_ji_var(N, vector<vector<double>>(2, vector<double>(n_shared)));
    vector<vector<vector<double>>> l_ji_var(N, vector<vector<double>>(2, vector<double>(n_shared)));
    vector<vector<vector<double>>> z_ij_var(N, vector<vector<double>>(2, vector<double>(n_shared)));
    vector<vector<vector<double>>> l_ij_var(N, vector<vector<double>>(2, vector<double>(n_shared)));
    vector<vector<double>> residuals(N, vector<double>(3));

    // // compare with solution from Python
    // vector<vector<vector<vector<double>>>> data_state(N, vector<vector<vector<double>>>(n_iter, vector<vector<double>>(trajectory_length, vector<double>(2))));
    // vector<vector<vector<vector<double>>>> data_input(N, vector<vector<vector<double>>>(n_iter, vector<vector<double>>(trajectory_length, vector<double>(2))));
    // vector<string> file_data_state = {"../test/data_state_vehicle0", "../test/data_state_vehicle1", "../test/data_state_vehicle2", "../test/data_state_vehicle3"};
    // vector<string> file_data_input = {"../test/input_state_vehicle0", "../test/data_input_vehicle1", "../test/data_input_vehicle2", "../test/data_input_vehicle3"};
    // for (int v=0; v<N; v++){
    //     ifstream file_state, file_input;
    //     file_state.open(file_data_state[v]);
    //     file_input.open(file_data_input[v]);
    //     int k = 0;
    //     for (int i=0; i<2*n_iter; i++){
    //         string line_state, line_input;
    //         getline(file_state, line_state);
    //         getline(file_input, line_input);
    //         stringstream iss_state(line_state);
    //         stringstream iss_input(line_input);
    //         for (int j=0; j<trajectory_length; j++){
    //             string val_state;
    //             string val_input;
    //             getline(iss_state, val_state, ',');
    //             getline(iss_input, val_input, ',');
    //             stringstream converter_state(val_state);
    //             stringstream converter_input(val_input);
    //             converter_state >> data_state[v][i/2][j][k];
    //             converter_input >> data_input[v][i/2][j][k];
    //         }
    //         k++;
    //         if (k == 2){
    //             k = 0;
    //         }
    //     }

    // }

    double time;
    double err;
    for (int i=0; i<n_iter; i++){
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
        cout << "it: " << i << " upd1, " << "time: " << time << "s" << endl;

        // communicate x_var
        for (int v=0; v<N; v++){
            x_j_var[v][0] = x_var[(v+1)%N];
            x_j_var[v][1] = x_var[(v-1)%N];
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
        }
        cout << "it: " << i << " upd2, " << "time: " << time << "s" << endl;

        // // communicate z_ij_var, l_ij_var
        // for (int v=0; v<N; v++){
        //     z_ji_var[v][0] = z_ij_var[(v+1)%N][0];
        //     z_ji_var[v][1] = z_ij_var[(v-1)%N][1];
        //     l_ji_var[v][0] = l_ij_var[(v+1)%N][0];
        //     l_ji_var[v][1] = l_ij_var[(v-1)%N][1];
        // }

        // // check with solution
        // for (int v=0; v<N; v++){
        //     for (int k=0; k<2; k++){
        //         for (int j=0; j<trajectory_length; j++){
        //             if (data_state[v][i][j][k] < 1e-14){
        //                 err = (data_state[v][i][j][k] - state_trajectory[v][j][k]);
        //             }
        //             else {
        //                 err = (data_state[v][i][j][k] - state_trajectory[v][j][k])/data_state[v][i][j][k];
        //             }
        //             assert(err < 1e-4);
        //             if (data_input[v][i][j][k] < 1e-14){
        //                 err = (data_input[v][i][j][k] - input_trajectory[v][j][k]);
        //             }
        //             else {
        //                 err = (data_input[v][i][j][k] - input_trajectory[v][j][k])/data_input[v][i][j][k];
        //             }
        //             assert(err < 1e-4);
        //         }
        //     }
        // }
    }
}

