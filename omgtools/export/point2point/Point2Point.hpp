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

#ifndef POINT2POINT
#define POINT2POINT

#include "Vehicle.hpp"
#include <casadi/casadi.hpp>
#include <math.h>
#include <memory.h>
#include <iostream>

#define inf std::numeric_limits<double>::infinity()
@defines@

namespace omg{

typedef struct obstacle {
    double position[N_DIM];
    double velocity[N_DIM];
    double acceleration[N_DIM];
} __attribute__((packed)) obstacle_t;


class Point2Point{
    private:
        Vehicle* vehicle;
        double current_time=0.0;
        double horizon_time;
        double update_time;
        double sample_time;
        int trajectory_length;
        casadi::Function problem;
        std::map<std::string, casadi::Function> substitutes;
        std::map<std::string, casadi::DM> args, sol;
        std::vector<double> parameters;
        std::vector<double> variables;
        std::vector<double> lbg;
        std::vector<double> ubg;
        std::vector<double> time;
        std::vector<std::vector<double>> state_trajectory;
        std::vector<std::vector<double>> input_trajectory;
        std::map<std::string, std::vector<std::vector<double>>> splines_tf;
        std::string solver_output;

        const int n_var = N_VAR;
        const int n_par = N_PAR;
        const int n_con = N_CON;
        const int freeT = FREET;

        void generateProblem();
        void generateSubstituteFunctions();
        void initSplines();
        bool solve(std::vector<obstacle_t>&);
        void setParameters(std::vector<obstacle_t>&);
        void initVariables();
        void updateBounds(double);
        void extractData();
        void retrieveTrajectories(std::vector<std::vector<double>>&);
        void getParameterVector(std::vector<double>&, std::map<std::string, std::map<std::string, std::vector<double>>>&);
        void getVariableVector(std::vector<double>&, std::map<std::string, std::map<std::string, std::vector<double>>>&);
        void getVariableDict(std::vector<double>&, std::map<std::string, std::map<std::string, std::vector<double>>>&);
        void transformSplines(double);

    public:
        const int n_dim = N_DIM;
        const int n_obs = N_OBS;

        Point2Point(Vehicle* vehicle, double update_time, double sample_time, double horizon_time);
        Point2Point(Vehicle* vehicle, double update_time, double sample_time, double horizon_time, int trajectory_length);
        void reset();
        bool update(std::vector<double>&, std::vector<double>&, std::vector<std::vector<double>>&, std::vector<std::vector<double>>&, std::vector<obstacle_t>&);
        bool update(std::vector<double>&, std::vector<double>&, std::vector<std::vector<double>>&, std::vector<std::vector<double>>&, std::vector<obstacle_t>&, int);
    };
}

#endif
