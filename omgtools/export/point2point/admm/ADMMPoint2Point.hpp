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

#ifndef ADMMPOINT2POINT
#define ADMMPOINT2POINT

#include "Point2Point.hpp"

namespace omg{

class ADMMPoint2Point: public Point2Point{
    private:
        int iteration=0;
        int init_iter;
        casadi::Function updx_problem;
        casadi::Function updz_problem;
        casadi::Function updl_problem;
        casadi::Function get_residuals;
        std::map<std::string, std::vector<double>> variables_admm;
        std::vector<double> residuals;
        double rho;

        const int n_nghb = N_NGHB;

        bool solveUpdx(std::vector<obstacle_t>&);
        bool solveUpdz();
        bool solveUpdl();
        bool computeResiduals();
        void initVariablesADMM();
        void extractData();
        void retrieveSharedVariables(std::map<std::string, std::map<std::string, std::vector<double>>>&);
        void transformSharedSplines(double);
        void setParameters(std::vector<obstacle_t>&);
        virtual void generateProblem();

    protected:
        virtual void fillParameterDict(std::vector<obstacle_t>&, std::map<std::string, std::map<std::string, std::vector<double>>>&);
        virtual bool update1(std::vector<double>&, std::vector<double>&, std::vector<std::vector<double>>&, std::vector<std::vector<double>>&, std::vector<double>&, std::vector<std::vector<double>>&, std::vector<std::vector<double>>&, std::vector<obstacle_t>&);
        virtual bool update1(std::vector<double>&, std::vector<double>&, std::vector<std::vector<double>>&, std::vector<std::vector<double>>&, std::vector<double>&, std::vector<std::vector<double>>&, std::vector<std::vector<double>>&, std::vector<obstacle_t>&, int);
        virtual bool update2(std::vector<std::vector<double>>&, std::vector<std::vector<double>>&, std::vector<std::vector<double>>&, std::vector<double>&);

    public:
        const int n_shared = N_SHARED;
        ADMMPoint2Point(Vehicle* vehicle, double update_time, double sample_time, double horizon_time);
        ADMMPoint2Point(Vehicle* vehicle, double update_time, double sample_time, double horizon_time, int trajectory_length);
        ADMMPoint2Point(Vehicle* vehicle, double update_time, double sample_time, double horizon_time, int trajectory_length, int init_iter);
        ADMMPoint2Point(Vehicle* vehicle, double update_time, double sample_time, double horizon_time, int trajectory_length, int init_iter, double rho);
        virtual void reset();
        virtual void resetTime();
    };
}

#endif
