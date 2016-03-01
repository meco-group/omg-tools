#ifndef MOTIONPLANNING
#define MOTIONPLANNING

#include <casadi/casadi.hpp>
#include "MotionPlanning_def.hpp"
#include <math.h>
#include <memory.h>
#include <vector>
#include <iostream>

class MotionPlanning{
    private:
        double currentTime=0.0;
        double horizonTime;
        double updateTime;
        double sampleTime;
        casadi::NlpSolver problem;
        std::map<std::string, casadi::DMatrix> args, sol;
        std::vector<double> parameters;
        std::vector<double> variables;
        std::vector<double> lbg;
        std::vector<double> ubg;
        obstacle_t obstacles[n_obs];
        std::vector<std::vector<double>> y_coeffs;
        std::vector<double> time;
        std::vector<std::vector<double>> input;
        std::vector<std::vector<double>> y0;
        std::vector<std::vector<double>> yT;
        std::vector<std::vector<double>> ypred;
        std::vector<std::vector<std::vector<double>>> derivative_T;
        std::map<std::string, spline_t> splines;
        bool idealUpdate;

    public:
        MotionPlanning(double updateTime, double sampleTime, double horizonTime);
        void generateProblem();
        void initSplines();
        bool update(std::vector<double>&, std::vector<double>&, std::vector<double>&, std::vector<std::vector<double>>&);
        bool solve();
        void setParameters();
        void initVariables();
        void updateBounds(double);
        void interpreteVariables();
        void predict(std::vector<double>&, std::vector<std::vector<double>>&, std::vector<std::vector<double>>&, double);
        void integrate(std::vector<double>&, std::vector<std::vector<double>>&, std::vector<double>&, int);
        void sampleSplines(std::vector<std::vector<double>>&, std::vector<std::vector<double>>&);
        double evalSpline(double, std::vector<double>&, std::vector<double>&, int);
        void transformSplines(double);
        void updateModel(std::vector<double>&, std::vector<double>&, std::vector<double>&);
        void getY(std::vector<double>&, std::vector<double>&, std::vector<std::vector<double>>&);
        void getInput(std::vector<std::vector<double>>&, std::vector<double>&);
        void setIdealUpdate(bool);
};

#endif
