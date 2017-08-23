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

#include "Point2Point_p2p.hpp"
#include "Holonomic_p2p.hpp"
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
    double width = 3.0;
    double height = 0.2;
    double radius = 0.001;

    for (int k=0; k<p2p.n_obs; k++){
        obstacles[k].position.resize(2);
        obstacles[k].velocity.resize(2);
        obstacles[k].acceleration.resize(2);
        obstacles[k].checkpoints.resize(2*4);
	obstacles[k].traj_coeffs.resize(2*13);
        obstacles[k].radii.resize(4);
        obstacles[k].checkpoints[0] = 0.5*width;
        obstacles[k].checkpoints[1] = 0.5*height;
        obstacles[k].checkpoints[2] = 0.5*width;
        obstacles[k].checkpoints[3] = -0.5*height;
        obstacles[k].checkpoints[4] = -0.5*width;
        obstacles[k].checkpoints[5] = -0.5*height;
        obstacles[k].checkpoints[6] = -0.5*width;
        obstacles[k].checkpoints[7] = 0.5*height;
        obstacles[k].avoid = true;
        for (int i=0; i<4; i++){
            obstacles[k].radii[i] = radius;
        }
    }
    obstacles[0].position[0] = -0.6;
    obstacles[0].position[1] = 1.0;
    for(int i=0;i<26;i++)
    {
	obstacles[1].traj_coeffs[i] = i;
    }

    double time;
    double err;
    for (int i=0; i<n_iter; i++){
        clock_t begin = clock();
        p2p.update(state0, stateT, state_trajectory, input_trajectory, obstacles);
        clock_t end = clock();
        time = double(end-begin)/CLOCKS_PER_SEC;
        cout << "it: " << i << ", " << "time: " << time << "s" << endl;
    }

}
