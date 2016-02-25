#ifndef MOTIONPLANNING
#define MOTIONPLANNING

#include <vector>
#include <iostream>
#include <casadi/casadi.hpp>
#include "MotionPlanning_def.hpp"

class MotionPlanning{
    private:
        double currentTime=0.0;
        double horizonTime;
        double updateTime;
        double sampleTime;
        casadi::NlpSolver problem;
        y_t prediction;
        y_t target;
        obstacle_t obstacles[n_obs];
        std::vector<double> parameters;
        std::vector<double> variables;
        std::vector<double> lbg;
        std::vector<double> ubg;
        std::map<std::string, casadi::DMatrix> args, sol;

    public:
        MotionPlanning(double updateTime, double sampleTime, double horizonTime);
        bool setParameters(std::vector<double>&, double, double, y_t, y_t, obstacle_t*);
        bool updateBounds(std::vector<double>&, std::vector<double>&, double);
        bool initVariables(std::vector<double>&, y_t, y_t);
        bool generateProblem();
        bool solve();
        bool update();
        std::vector<double> getVariables();
        void setPrediction(y_t);
        void setTarget(y_t);
        bool initialize();
        std::vector<double> updateModel(std::vector<double>&, std::vector<double>&);
        std::vector<std::vector<double>> getY(std::vector<double>&, std::vector<double>&);
        std::vector<double> getInput(std::vector<std::vector<double>>&);
        void predict(std::vector<double>&, std::vector<std::vector<double>>&, std::vector<std::vector<double>>&, double);
        void updateModel(std::vector<double>&, std::vector<double>&, std::vector<double>&);
        void getY(std::vector<double>&, std::vector<double>&, std::vector<std::vector<double>>&);
        void getInput(std::vector<std::vector<double>>&, std::vector<double>&);
};

#endif
