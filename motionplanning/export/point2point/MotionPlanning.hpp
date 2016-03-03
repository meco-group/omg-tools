#ifndef MOTIONPLANNING
#define MOTIONPLANNING

#include <casadi/casadi.hpp>
#include <math.h>
#include <memory.h>
#include <vector>
#include <iostream>

#define inf std::numeric_limits<double>::infinity()
@defines@

namespace mp{

typedef struct obstacle {
    double position[N_DIM];
    double velocity[N_DIM];
    double acceleration[N_DIM];
} __attribute__((packed)) obstacle_t;

@types@

class MotionPlanning{
    private:
        double current_time=0.0;
        double horizon_time;
        double update_time;
        double sample_time;
        casadi::NlpSolver problem;
        std::map<std::string, casadi::DMatrix> args, sol;
        std::vector<double> parameters;
        std::vector<double> variables;
        std::vector<double> lbg;
        std::vector<double> ubg;
        std::vector<std::vector<double>> y_coeffs;
        std::vector<double> time;
        std::vector<std::vector<double>> input_trajectory;
        std::vector<std::vector<double>> state_trajectory;
        std::vector<std::vector<double>> y0;
        std::vector<std::vector<double>> yT;
        std::vector<std::vector<double>> ypred;
        std::map<std::string, spline_t> splines;
        std::string solver_output;

        bool ideal_update;
        const int n_y = N_Y;
        const int n_var = N_VAR;
        const int n_par = N_PAR;
        const int n_con = N_CON;
        const int n_der = N_DER;
        const int order = ORDER;
        const int y_degree = Y_DEGREE;
        const std::vector<std::vector<std::vector<double>>> derivative_T = DERT_DEF;

        void generateProblem();
        void initSplines();
        bool solve(std::vector<obstacle_t>&);
        void setParameters(std::vector<obstacle_t>&);
        void initVariables();
        void updateBounds(double);
        void interpreteVariables();
        void predict(std::vector<double>&, std::vector<std::vector<double>>&, std::vector<std::vector<double>>&, double);
        void integrate(std::vector<double>&, std::vector<std::vector<double>>&, std::vector<double>&, int);
        void sampleSplines(std::vector<std::vector<double>>&, std::vector<std::vector<double>>&, std::vector<std::vector<double>>&);
        double evalSpline(double, std::vector<double>&, std::vector<double>&, int);
        void transformSplines(double);
        void updateModel(std::vector<double>&, std::vector<double>&, std::vector<double>&);
        void getY(std::vector<double>&, std::vector<double>&, std::vector<std::vector<double>>&);
        void getState(std::vector<std::vector<double>>&, std::vector<double>&);
        void getInput(std::vector<std::vector<double>>&, std::vector<double>&);

    public:
        const int n_dim = N_DIM;
        const int n_obs = N_OBS;
        const int n_in = N_IN;
        const int n_st = N_ST;

        MotionPlanning(double updateTime, double sampleTime, double horizonTime);
        bool update(std::vector<double>&, std::vector<double>&, std::vector<double>&, std::vector<std::vector<double>>&, std::vector<std::vector<double>>&, std::vector<obstacle_t>&);
        void setIdealUpdate(bool);
    };
}

#endif
