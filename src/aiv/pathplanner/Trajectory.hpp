#ifndef __AIV_TRAJECTORY_HPP__
#define __AIV_TRAJECTORY_HPP__
#pragma once

#include <unsupported/Eigen/Splines>
#include "aiv/pathplanner/FlatoutputMonocycle.hpp"

namespace aiv {

	class Trajectory
	{
	public:
		static const unsigned dim = 2; // 2D trajectory in XY plane
		static const unsigned derivDeg = aiv::FlatoutputMonocycle::flatDerivDeg + 1; // Support q dotq dotdotq

		void update(TrajectorySpline::ControlPointVectorType ctrlPts);
		void update(unsigned ncpts, const double *ctrlpts);

		void operator()(double time);
		void operator()(double time, unsigned deriv);
		
		//void setTrajectoryValue(const double *trajectoryValues);
		//void setTrajectoryValue(const double trajectoryValue,  const unsigned idx);
		Trajectory::Trajectory();
		Trajectory::~Trajectory();

	private:
		typedef Eigen::Spline< double, dim, derivDeg > TrajectorySpline;

		TrajectorySpline _trajecSpl;

		//Eigen::Matrix<double, _trajectoryDim, 1> _evaluatedTrajectoryValues;

		//bool _isSplUpToDate;

		TrajectorySpline::KnotVectorType _knots;
		//TrajectorySpline::ControlPointVectorType _ctrlpts;
	};

}

#endif // __AIV_TRAJECTORY_HPP__

// cmake:sourcegroup=PathPlanner