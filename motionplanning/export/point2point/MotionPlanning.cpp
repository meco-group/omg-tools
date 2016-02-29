#include "MotionPlanning.hpp"

using namespace std;
using namespace casadi;

MotionPlanning::MotionPlanning(double updateTime, double sampleTime, double horizonTime):
parameters(n_par), variables(n_var), lbg(lbg_def), ubg(ubg_def),
y_coeffs(n_y, vector<double>(len_y)), time(int(updateTime/sampleTime)+1),
input(int(updateTime/sampleTime)+1, vector<double>(n_in)),
y0(n_y, vector<double>(n_der+1)), yT(n_y, vector<double>(n_der+1)),
ypred(n_y, vector<double>(n_der+1)), derivative_T(derT_def){
    this->updateTime = updateTime;
    this->sampleTime = sampleTime;
    this->horizonTime = horizonTime;
    for (int k=0; k<time.size(); k++){
        time[k] = k*sampleTime;
    }
    generateProblem();
    args["p"] = parameters;
    args["x0"] = variables;
    args["lbg"] = lbg;
    args["ubg"] = ubg;
    initSplines();
}

void MotionPlanning::generateProblem(){
    // load object files
    ExternalFunction grad_f("grad_f");
    ExternalFunction jac_g("jac_g");
    ExternalFunction hess_lag("hess_lag");
    ExternalFunction nlp("nlp");
    // set options
    Dict options = make_dict("grad_f", grad_f, "jac_g", jac_g,
                             "hess_lag", hess_lag,
                             "print_level", 0, "print_time", 0,
                             // "tol", tol, "linear_solver", linear_solver,
                             "warm_start_init_point", "yes");
    // create nlp solver
    NlpSolver problem("problem", "ipopt", nlp, options);
    this->problem = problem;
}

void MotionPlanning::initSplines(){
@initSplines@
}

bool MotionPlanning::update(vector<double>& state0, vector<double>& stateT, vector<double>& inputT, vector<vector<double>>& input){
    // predict initial y for problem
    predict(state0, this->input, this->y0, updateTime);
    // translate terminal state/input
    getY(stateT, inputT, this->yT);
    // solve problem
    bool check = solve();
    // interprete variables (extract y_coeffs, T)
    interpreteVariables();
    // sample y splines and compute input and ideal ypred (y0 of next update = ypred in ideal cases)
    sampleSplines(ypred, this->input);
    // input for system is one sample shorter!!
    for (int k=0; k<time.size()-1; k++){
        input[k] = this->input[k];
    }
    // transform splines: good init guess for next update
    transformSplines(currentTime);
    // update current time
    currentTime += updateTime;

    return check;
}

bool MotionPlanning::solve(){
    // init variables if first time
    if(currentTime == 0.0){
        initVariables();
    }
    setParameters();
    args["p"] = parameters;
    args["x0"] = variables;
    args["lbg"] = lbg;
    args["ubg"] = ubg;
    sol = this->problem(args);
    vector<double> var(sol.at("x"));
    for (int k=0; k<n_var; k++){
        variables[k] = var[k];
    }
    return true;
}

void MotionPlanning::setParameters(){
    vector<double> y0(n_y);
    vector<double> dy0(n_y*n_der);
    vector<double> yT(n_y);
    vector<double> dyT(n_y*n_der);
    for (int i=0; i<n_y; i++){
        y0[i] = this->y0[i][0];
        yT[i] = this->yT[i][0];
        for (int j=0; j<n_der; j++){
            dy0[i*n_der+j] = this->y0[i][j+1];
            dyT[i*n_der+j] = this->yT[i][j+1];
        }
    }
@setParameters@
}

void MotionPlanning::initVariables(){
@initVariables@
}

void MotionPlanning::updateBounds(double currentTime){
@updateBounds@
}

void MotionPlanning::interpreteVariables(){
@interpreteVariables@
}

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
    cout << "in transformation" << endl;
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
