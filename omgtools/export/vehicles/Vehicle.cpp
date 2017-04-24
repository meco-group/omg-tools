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

#include "Vehicle.hpp"

using namespace std;

namespace omg{

Vehicle::Vehicle(int n_st, int n_in, int n_spl, int degree, int knot_intervals):
ideal_prediction(false), provide_prediction(false), predicted_state(n_st), predicted_input(n_in){
    this->n_st = n_st;
    this->n_in = n_in;
    this->n_spl = n_spl;
    this->degree = degree;
    vector<double> knots(knot_intervals + 1 + 2*degree);
    this->knot_intervals = knot_intervals;
    this->len_basis = knots.size() - this->degree - 1;
    for (int d=0; d<degree; d++){
        knots[d] = 0.0;
        knots[d+knot_intervals+degree+1] = 1.0;
    }
    for (int j=0; j<knot_intervals+1; j++){
        knots[degree+j] = double(j)/knot_intervals;
    }
    this->knots = knots;
    this->derivative_T.resize(degree+1);
    createDerivativeMatrices();
}

Vehicle::Vehicle(int n_st, int n_in, int n_spl, int degree):Vehicle(n_st, n_in, n_spl, degree, 10){

}

void Vehicle::getPrediction(vector<double>& state, vector<double>& input){
    state = this->predicted_state;
    input = this->predicted_input;
}

void Vehicle::setPrediction(vector<double>& state, vector<double>& input){
    this->predicted_state = state;
    this->predicted_input = input;
}

void Vehicle::predict(vector<double>& state0, vector<vector<double>>& state_trajectory, vector<vector<double>>& input_trajectory, double predict_time, double sample_time, int predict_shift)
{
    if (ideal_prediction){
        vector<double> stateT = state_trajectory[int(predict_time/sample_time)+predict_shift];
        vector<double> inputT = input_trajectory[int(predict_time/sample_time)+predict_shift];
        setPrediction(stateT, inputT);
    } else if (provide_prediction){
        vector<double> inputT = input_trajectory[int(predict_time/sample_time)+predict_shift];
        setPrediction(state0, inputT);
    } else {
        int steps = int(predict_time/sample_time);
        vector<vector<double>> input(steps+1, vector<double>(input_trajectory[0].size()));
        for (int k=0; k<steps+1; k++){
            input[k] = input_trajectory[k+predict_shift];
        }
        vector<double> stateT(state0.size());
        integrate(state0, input, stateT, sample_time, steps);
        setPrediction(stateT, input[steps]);
    }
}

void Vehicle::integrate(vector<double>& state0, vector<vector<double>>& input, vector<double>& stateT, double sample_time, int steps){
    //integration via 4th order Runge-Kutta
    vector<double> k1(n_st);
    vector<double> k2(n_st);
    vector<double> k3(n_st);
    vector<double> k4(n_st);
    vector<double> st(n_st);
    for (int j=0; j<n_st; j++){
        stateT[j] = state0[j];
    }
    for (int i=0; i<steps; i++){
        this->ode(state0, input[i], k1);
        for (int j=0; j<n_st; j++){
            st[j] = state0[j] + 0.5*sample_time*k1[j];
        }
        this->ode(st, input[i], k2);
        for (int j=0; j<n_st; j++){
            st[j] = state0[j] + 0.5*sample_time*k2[j];
        }
        this->ode(st, input[i], k3);
        for (int j=0; j<n_st; j++){
            st[j] = state0[j] + sample_time*k3[j];
        }
        this->ode(st, input[i+1], k4);
        for (int j=0; j<n_st; j++){
            stateT[j] += (sample_time/6.0)*(k1[j]+2*k2[j]+2*k3[j]+k4[j]);
        }
    }
}

void Vehicle::sampleSplines(vector<vector<double>>& spline_coeffs, vector<double> time, int derivative, vector<vector<double>>& spline_sampled){
    vector<double> knots(this->knots.begin()+derivative, this->knots.end()-derivative);
    for (int k=0; k<time.size(); k++){
        for (int i=0; i<n_spl; i++){
            vector<double> coeffs(len_basis-derivative);
            for (int l=0; l<len_basis-derivative; l++){
                for (int m=0; m<len_basis; m++){
                    coeffs[l] += (1.0/pow(horizon_time, derivative))*this->derivative_T[derivative][l][m]*spline_coeffs[i][m];
                }
            }
            spline_sampled[k][i] = evalSpline(time[k]/horizon_time, knots, coeffs, this->degree - derivative);
        }
    }
}

void Vehicle::sampleSplines(vector<vector<double>>& spline_coeffs, vector<double> time, vector<vector<double>>& spline_samples){
    sampleSplines(spline_coeffs, time, 0, spline_samples);
}

void Vehicle::createDerivativeMatrices(){
    vector<vector<double>> derT(len_basis, vector<double>(len_basis));
    for (int j=0; j<len_basis; j++){
        derT[j][j] = 1.;
    }
    derivative_T[0] = derT;
    double delta_knots;
    for (int i=0; i<degree; i++){
        vector<double> knots(this->knots.begin()+i+1, this->knots.end()-i-1);
        derT.clear();
        derT.resize(len_basis-1-i, vector<double>(len_basis));
        vector<vector<double>> T(len_basis-1-i, vector<double>(len_basis-i));
        for (int j=0; j<len_basis-1-i; j++){
            delta_knots = knots[degree - i + j] - knots[j];
            T[j][j] = -1./delta_knots;
            T[j][j+1] = 1./delta_knots;
        }
        for (int l=0; l<len_basis-1-i; l++){
            for (int m=0; m<len_basis; m++){
                for (int k=0; k<len_basis-i; k++){
                    derT[l][m] += (degree-i)*T[l][k]*derivative_T[i][k][m];
                }
            }
        }
        derivative_T[i+1] = derT;
    }
}

double Vehicle::evalSpline(double x, vector<double>& knots, vector<double>& coeffs, int degree){
    int len_k = knots.size();
    double b;
    double bottom;
    double basis[degree+1][len_k-1];
    for (int i=0; i<(len_k-1); i++){
        if((i < degree+1) and (knots[0] == knots[i])){
            basis[0][i] = ((x >= knots[i]) and (x <= knots[i+1]));
        }else{
            basis[0][i] = ((x > knots[i]) and (x <= knots[i + 1]));
        }
    }
    for (int d=1; d<(degree+1); d++){
        for (int i=0; i<(len_k - d - 1); i++){
            b = 0;
            bottom = knots[i+d] - knots[i];
            if (bottom != 0){
                b = (x - knots[i])*basis[d-1][i]/bottom;
            }
            bottom = knots[i+d+1] - knots[i+1];
            if (bottom != 0){
                b += (knots[i+d+1] - x)*basis[d-1][i+1]/bottom;
            }
            basis[d][i] = b;
        }
    }
    double y = 0.0;
    for (int l=0; l<(len_k-degree-1); l++){
        y += coeffs[l]*basis[degree][l];
    }
    return y;
}

void Vehicle::setKnotHorizon(double horizon_time){
    this->horizon_time = horizon_time;
}

void Vehicle::setIdealPrediction(bool ideal_prediction){
    this->ideal_prediction = ideal_prediction;
}

void Vehicle::setProvidePrediction(bool provide_prediction){
    this->provide_prediction = provide_prediction;
}

int Vehicle::getNSplines(){
    return n_spl;
}

int Vehicle::getNState(){
    return n_st;
}

int Vehicle::getNInput(){
    return n_in;
}

int Vehicle::getLenBasis(){
    return len_basis;
}

int Vehicle::getDegree(){
    return degree;
}

int Vehicle::getKnotIntervals(){
    return knot_intervals;
}



}
