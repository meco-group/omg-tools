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
#include "Holonomic.hpp"
#include <ctime>
#include <iostream>
#include <fstream>
#include <assert.h>

using namespace std;

int main()
{
    int n_iter = 50;
    double horizon_time = 10;
    double sample_time = 0.01;
    double update_time = 0.1;
    int trajectory_length = 5;
    omg::Holonomic* vehicle = new omg::Holonomic();
    // ideal update: prediction of initial state based on spline extrapolation
    // non-ideal update: prediction based on current state0 and model integration
    vehicle->setIdealPrediction(true);
    omg::Point2Point p2p(vehicle, update_time, sample_time, horizon_time, trajectory_length);

    // set initial state and terminal state and input
    vector<double> state0(2);
    vector<double> stateT(2);
    for (int i=0; i<2; i++){
        state0.at(i) = 0.0;
        stateT.at(i) = 3.5;
    }

    // these will store the state and input trajectory
    vector<vector<double>> input_trajectory(trajectory_length, vector<double>(2));
    vector<vector<double>> state_trajectory(trajectory_length, vector<double>(2));

    // obstacles
    vector<omg::obstacle_t> obstacles(p2p.n_obs);
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

    // compare with solution from Python
    vector<vector<vector<double>>> data_state(n_iter, vector<vector<double>>(trajectory_length, vector<double>(2)));
    vector<vector<vector<double>>> data_input(n_iter, vector<vector<double>>(trajectory_length, vector<double>(2)));
    ifstream file_state, file_input;
    file_state.open("../test/data_state.csv");
    file_input.open("../test/data_input.csv");
    int k = 0;
    for (int i=0; i<2*n_iter; i++){
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
            converter_state >> data_state[i/2][j][k];
            converter_input >> data_input[i/2][j][k];
        }
        k++;
        if (k == 2){
            k = 0;
        }
    }
    double time;
    double err;
    for (int i=0; i<n_iter; i++){
        clock_t begin = clock();
        p2p.update(state0, stateT, state_trajectory, input_trajectory, obstacles);
        clock_t end = clock();
        time = double(end-begin)/CLOCKS_PER_SEC;
        cout << "it: " << i << ", " << "time: " << time << "s" << endl;
        int cnt = 0;
        for (int k=0; k<2; k++){
            for (int j=0; j<trajectory_length; j++){
                if (data_state[i][j][k] < 1e-14){
                    err = (data_state[i][j][k] - state_trajectory[j][k]);
                }
                else {
                    err = (data_state[i][j][k] - state_trajectory[j][k])/data_state[i][j][k];
                }
                assert(err < 1e-4);
                if (data_input[i][j][k] < 1e-14){
                    err = (data_input[i][j][k] - input_trajectory[j][k]);
                }
                else {
                    err = (data_input[i][j][k] - input_trajectory[j][k])/data_input[i][j][k];
                }
                assert(err < 1e-4);
            }
        }
    }
}
