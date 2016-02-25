#ifndef MOTIONPLANNING_DEF
#define MOTIONPLANNING_DEF

#include <vector>

#define n_par @n_par@
#define n_var @n_var@
#define n_con @n_con@
#define n_obs @n_obs@
#define n_dim @n_dim@
#define n_y @n_y@
#define n_der @n_der@
#define order @order@
#define n_st @n_st@
#define n_in @n_in@
#define lbg_def @lbg@
#define ubg_def @ubg@
#define tol @tol@
#define linear_solver "@linear_solver@"
#define inf std::numeric_limits<double>::infinity()

@spline_info@

typedef struct obstacle {
    double position[n_dim];
    double velocity[n_dim];
    double acceleration[n_dim];
} __attribute__((packed)) obstacle_t;

@spline_type@

#endif
