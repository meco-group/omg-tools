#include "MotionPlanning.hpp"
#include <ctime>

using namespace std;

int main()
{
    double horizon_time = 10;
    double sample_time = 0.01;
    double update_time = 0.1;
    int trajectory_length = int(update_time/sample_time);
    omg::MotionPlanning p2p(update_time, sample_time, horizon_time);

    // set initial state and terminal state and input
    vector<double> state0(p2p.n_st);
    vector<double> stateT(p2p.n_st);
    for (int i=0; i<p2p.n_st; i++){
        state0.at(i) = -1.5;
        stateT.at(i) = 2.0;
    }

    // these will store the state and input trajectory
    vector<vector<double>> input_trajectory(trajectory_length, vector<double>(p2p.n_in));
    vector<vector<double>> state_trajectory(trajectory_length, vector<double>(p2p.n_st));

    // obstacles
    vector<omg::obstacle_t> obstacles(p2p.n_obs);
    // obstacles[0].position[0] = 0.5;
    // obstacles[0].position[1] = 2.0;
    // obstacles[0].velocity[0] = 0.0;
    // obstacles[0].velocity[1] = 0.0;
    // obstacles[0].acceleration[0] = 0.0;
    // obstacles[0].acceleration[1] = 0.0;

    // obstacles[1].position[0] = 4.2;
    // obstacles[1].position[1] = 2.0;
    // obstacles[1].velocity[0] = 0.0;
    // obstacles[1].velocity[1] = 0.0;
    // obstacles[1].acceleration[0] = 0.0;
    // obstacles[1].acceleration[1] = 0.0;

    // ideal update: prediction of initial state based on spline extrapolation
    // non-ideal update: prediction based on current state0 and model integration
    p2p.setIdealUpdate(true);

    double time;
    for (int i=0; i<1; i++){
        clock_t begin = clock();
        p2p.update(state0, stateT, state_trajectory, input_trajectory, obstacles);
        clock_t end = clock();
        time = double(end-begin)/CLOCKS_PER_SEC;
        cout << "it: " << i << ", " << "time: " << time << "s" << endl;
    }
}

