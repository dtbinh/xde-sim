#include <Eigen/Dense>
#include "aiv/pathplanner/FlatoutputMonocycle.hpp"

const unsigned nctrlpts = 14;

typedef Eigen::Matrix<double, aiv::FlatoutputMonocycle::poseDim + aiv::FlatoutputMonocycle::velocityDim, nctrlpts> EqJacMatrix;

EqJacMatrix calcEqJac(double t, double Tp, const double *X);

// cmake:sourcegroup=PathPlanner