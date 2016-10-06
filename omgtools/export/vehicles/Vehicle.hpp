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

#ifndef VEHICLE
#define VEHICLE

#include <math.h>
#include <vector>
#include <map>

namespace omg{

class Vehicle{
    private:
        int n_st;
        int n_in;
        int n_spl;
        int degree;
        int len_basis;
        int knot_intervals;
        bool ideal_prediction;
        bool provide_prediction;
        double horizon_time;
        std::vector<double> knots;
        std::vector<double> predicted_state;
        std::vector<double> predicted_input;
        std::vector<std::vector<double>> state_trajectory;
        std::vector<std::vector<double>> input_trajectory;
        std::vector<std::vector<std::vector<double>>> derivative_T;

        void integrate(std::vector<double>& state0, std::vector<std::vector<double>>& input, std::vector<double>& stateT, double sample_time, int steps);
        double evalSpline(double x, std::vector<double>& knots, std::vector<double>& coeffs, int degree);
        void createDerivativeMatrices();

    protected:
        void sampleSplines(std::vector<std::vector<double>>& spline_coeffs, std::vector<double> time, int derivative, std::vector<std::vector<double>>& spline_sampled);
        void sampleSplines(std::vector<std::vector<double>>& spline_coeffs, std::vector<double> time, std::vector<std::vector<double>>& spline_sampled);
        void getPrediction(std::vector<double>& state, std::vector<double>& input);
        void setPrediction(std::vector<double>& state, std::vector<double>& input);

    public:
        virtual void setInitialConditions(std::vector<double>& conditions) = 0;
        virtual void setTerminalConditions(std::vector<double>& conditions) = 0;
        virtual void getInitSplineValue(std::vector<std::vector<double>>& init_value) = 0;
        virtual void setParameters(std::map<std::string,std::vector<double>>& par_dict) = 0;
        virtual void ode(std::vector<double>& state, std::vector<double>& input, std::vector<double>& dstate) = 0;
        virtual void splines2State(std::vector<std::vector<double>>& spline_coeffs, std::vector<double> time, std::vector<std::vector<double>>& state) = 0;
        virtual void splines2Input(std::vector<std::vector<double>>& spline_coeffs, std::vector<double> time, std::vector<std::vector<double>>& input) = 0;

        Vehicle(int n_st, int n_in, int n_spl, int degree, int knot_intervals);
        Vehicle(int n_st, int n_in, int n_spl, int degree);

        void predict(std::vector<double>& state0, std::vector<std::vector<double>>& state_trajectory, std::vector<std::vector<double>>& input_trajectory, double predict_time, double sample_time, int predict_shift);
        void setKnotHorizon(double horizon_time);
        void setIdealPrediction(bool ideal_prediction);
        void setProvidePrediction(bool provide_prediction);
        int getNSplines();
        int getNState();
        int getNInput();
        int getLenBasis();
        int getDegree();
        int getKnotIntervals();
    };
}

#endif
