#include <Eigen/Dense>
#include "aiv/pathplanner/FlatoutputMonocycle.hpp"

typedef Eigen::Matrix<double, 1, Eigen::Dynamic> fJacMatrix;
typedef Eigen::Matrix<double, aiv::FlatoutputMonocycle::poseDim, Eigen::Dynamic> qJacMatrix;
typedef Eigen::Matrix<double, aiv::FlatoutputMonocycle::veloDim, Eigen::Dynamic> dqJacMatrix;
typedef Eigen::Matrix<double, aiv::FlatoutputMonocycle::accelDim, Eigen::Dynamic> ddqJacMatrix;
typedef Eigen::Matrix<double, 1, Eigen::Dynamic> obstDistJacMatrix;

fJacMatrix fJac(const double t, const double Tp, const int span, const double *X, const int Xdim, const int spld, const Eigen::Vector2d& targetPosition);
qJacMatrix qJac(const double t, const double Tp, const int span, const double *X, const int Xdim, const int spld);
dqJacMatrix dqJac(const double t, const double Tp, const int span, const double *X, const int Xdim, const int spld);
dqJacMatrix absdqJac(const double t, const double Tp, const int span, const double *X, const int Xdim, const int spld);
ddqJacMatrix ddqJac(const double t, const double Tp, const int span, const double *X, const int Xdim, const int spld);
ddqJacMatrix absddqJac(const double t, const double Tp, const int span, const double *X, const int Xdim, const int spld);
obstDistJacMatrix obstDistJac(const double t, const double Tp, const int span, const double *X, const int Xdim, const int spld, Eigen::Vector2d obstPosition);

// cmake:sourcegroup=PathPlanner