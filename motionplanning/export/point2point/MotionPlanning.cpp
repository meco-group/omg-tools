#include "MotionPlanning.hpp"
#include <math.h>

using namespace std;
using namespace casadi;

MotionPlanning::MotionPlanning(double updateTime, double sampleTime, double horizonTime):
lbg(lbg_def), ubg(ubg_def), parameters(n_par), variables(n_var){
    this->updateTime = updateTime;
    this->sampleTime = sampleTime;
    this->horizonTime = horizonTime;
}

bool MotionPlanning::initialize(){
    if(!generateProblem()){
        return false;
    }
    args["p"] = parameters;
    args["x0"] = variables;
    args["lbg"] = lbg;
    args["ubg"] = ubg;
}

bool MotionPlanning::generateProblem(){
    try{
        // load object files
        ExternalFunction grad_f("grad_f");
        ExternalFunction jac_g("jac_g");
        ExternalFunction hess_lag("hess_lag");
        ExternalFunction nlp("nlp");
        // set options
        Dict options = make_dict("grad_f", grad_f, "jac_g", jac_g,
                                 "hess_lag", hess_lag,
                                 // "print_level", 0, "print_time", 0,
                                 "warm_start_init_point", "yes", "tol", tol,
                                 "linear_solver", linear_solver);
        // create nlp solver
        NlpSolver problem("problem", "ipopt", nlp, options);
        this->problem = problem;
        return true;
    }
    catch (...)
    {
        cerr << "Error while generating problem" << endl;
        return false;
    }
}

bool MotionPlanning::update(){
    // what input is needed from user?
    // * current state ~> use a vector of states
    // * pos/vel/acc of obstacles ~> use predifined obstacle type
    // * target state ~> use vector of states


    // predict initial state
    // * based on current state + saved input trajectory
    // * ode model necessary
    // * possibility to predict 'ideal' (no disturbances) for debugging


    // solve problem
    solve();

    // generate input trajectories
    // * spline evaluation necessary
    // * only limited necessary sample points


    // transform all splines
    // * find a way to get all spline indices and their transformation
    // update current time
    currentTime += sampleTime;
}

bool MotionPlanning::solve(){
    setParameters(parameters, currentTime, horizonTime, prediction, target, obstacles);
    args["p"] = parameters;
    args["x0"] = variables;
    args["lbg"] = lbg;
    args["ubg"] = ubg;
    sol = this->problem(args);
    vector<double> var(sol.at("x"));
    variables = var;
    return true;
}

bool MotionPlanning::updateBounds(vector<double>& lbg, vector<double>& ubg, double currentTime){
@updateBounds@
    return true;
}

bool MotionPlanning::setParameters(vector<double>& parameters, double currentTime, double horizonTime, y_t prediction, y_t target, obstacle_t* obstacles){
@setParameters@
    return true;
}

bool MotionPlanning::initVariables(vector<double>& variables, y_t state, y_t target){
@initVariables@
    return true;
}

vector<double> MotionPlanning::getVariables(){
    return variables;
}

void MotionPlanning::setPrediction(y_t prediction){
    this->prediction = prediction;
    initVariables(variables, prediction, target);
}

void MotionPlanning::setTarget(y_t target){
    this->target = target;
    initVariables(variables, prediction, target);
}

vector<double> MotionPlanning::updateModel(vector<double>& state, vector<double>& input){
@updateModel@
}

vector<vector<double>> MotionPlanning::getY(vector<double>& state, vector<double>& input){
@getY@
}

vector<double> MotionPlanning::getInput(vector<vector<double>>& y){
@getInput@
}
