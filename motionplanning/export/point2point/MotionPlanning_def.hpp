#ifndef MOTIONPLANNING_DEF
#define MOTIONPLANNING_DEF

#include <vector>

#define inf std::numeric_limits<double>::infinity()
@defines@

typedef struct obstacle {
    double position[n_dim];
    double velocity[n_dim];
    double acceleration[n_dim];
} __attribute__((packed)) obstacle_t;

@types@

#endif
