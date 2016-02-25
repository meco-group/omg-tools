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
void MotionPlanning::predict(vector<double>& state, vector<vector<double>>& input, vector<vector<double>>& y, double predictTime)
{
    int steps = int(predictTime/sampleTime);
    integrate(state, input, steps);
    getY(state, input[steps], y); //ok: input[steps] is the one which starts at updateTime
}

void MotionPlanning::integrate(vector<double>& state, vector<vector<double>>& input, int steps){
    //integration via 4th order Runge-Kutta
    vector<double> k1(n_st);
    vector<double> k2(n_st);
    vector<double> k3(n_st);
    vector<double> k4(n_st);
    vector<double> st(n_st);
    for (int i=0; i<steps; i++){
        updateModel(state, input[i], k1);
        for (int j=0; j<n_st; j++){
            st[j] = state[j] + 0.5*sampleTime*k1[j];
        }
        updateModel(st, input[i], k2);
        for (int j=0; j<n_st; j++){
            st[j] = state[j] + 0.5*sampleTime*k2[j];
        }
        updateModel(st, input[i], k3);
        for (int j=0; j<n_st; j++){
            st[j] = state[j] + sampleTime*k3[j];
        }
        updateModel(st, input[i+1], k4);
        for (int j=0; j<n_st; j++){
            state[j] += (sampleTime/6.0)*(k1[j]+2*k2[j]+2*k3[j]+k4[j]);
        }
    }
}

void MotionPlanning::sampleSplines(vector<vector<double>>& y, vector<vector<double>>& input){
@sampleSplines@
    int len_coeffs = splines["y"].knots.size() - y_degree - 1;
    for (int k=0; k<time.size(); k++){
        for (int i=0; i<n_y; i++){
            y[i][0] = evalSpline(time[k]/horizonTime, splines["y"].knots, y_coeffs[i], y_degree);
        }
        for (int d=1; d<n_der+1; d++){
            vector<double> dknots(splines["y"].knots.begin()+d, splines["y"].knots.end()-d);
            for (int i=0; i<n_y; i++){
                vector<double> dcoeffs(len_coeffs-d);
                for (int l=0; l<len_coeffs-d; l++){
                    for (int m=0; m<len_coeffs; m++){
                        dcoeffs[l] += (1.0/pow(horizonTime, d))*derivative_T[d-1][l][m]*y_coeffs[i][m];
                    }
                }
                y[i][d] = evalSpline(time[k]/horizonTime, dknots, dcoeffs, y_degree-d);
            }
        }
        getInput(y, input[k]);
    }
}

double MotionPlanning::evalSpline(double x, vector<double>& knots, vector<double>& coeffs, int degree){
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

void MotionPlanning::transformSplines(double currentTime){
@transformSplines@
}

void MotionPlanning::updateModel(vector<double>& state, vector<double>& input, vector<double>& dstate){
@updateModel@
}

void MotionPlanning::getY(vector<double>& state, vector<double>& input, vector<vector<double>>& y){
@getY@
}

void MotionPlanning::getInput(vector<vector<double>>& y, vector<double>& input){
@getInput@
}
