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
		typedef Eigen::Spline< double, dim, derivDeg > TrajectorySpline;

		void interpolate(const Eigen::Matrix<double, dim, Eigen::Dynamic>& points, const double parVarInterval);
		void setOption(std::string optionName, unsigned optionValue);
		template<class T>
		Eigen::Matrix<double, dim, Eigen::Dynamic> cArray2CtrlPtsMat(T ctrlpts);
		
		void update(const double *ctrlpts);
		void update(volatile double *ctrlpts);
		void update(const Eigen::Matrix<double, dim, Eigen::Dynamic>& ctrlpts);
		void update(const double *ctrlpts, const double parVarInterval);
		void update(volatile double *ctrlpts, volatile double parVarInterval);
		void update(const Eigen::Matrix<double, dim, Eigen::Dynamic>& ctrlpts, const double parVarInterval);

		void updateFromUniform(const double *ctrlpts);
		void updateFromUniform(const double *ctrlpts, const double parVarInterval);
		void updateFromUniform(const Eigen::Matrix<double, dim, Eigen::Dynamic>& ctrlpts);
		void updateFromUniform(const Eigen::Matrix<double, dim, Eigen::Dynamic>& ctrlpts, const double parVarInterval);

		Eigen::Matrix<double, dim, 1> operator()(const double evalTime) const;
		Eigen::Matrix<double, dim, Eigen::Dynamic> operator()(const double evalTime, const unsigned deriv) const;

		//void interpolate(const Eigen::MatrixXd& points, const double parVarInterval);

		void getParameters(double* params) const;
		//double getParVarInterv() const { return _trajecSpl.knots().tail(1)(0,0); };
		Eigen::Matrix<double, dim, Eigen::Dynamic> getCtrlPts() const { return _trajecSpl.ctrls(); };
		Eigen::Matrix<double, 1, Eigen::Dynamic> getKnots() const { return _trajecSpl.knots(); };

		int getSpan(const double t) const;

		int nParam() const { return _nCtrlPts; };

		Trajectory();
		Trajectory(const Eigen::Matrix<double, Trajectory::dim, Eigen::Dynamic>& points, const double parVarInterval);
		~Trajectory();

	private:

		TrajectorySpline _trajecSpl;

		//Eigen::Matrix<double, _trajectoryDim, 1> _evaluatedTrajectoryValues;

		//bool _isSplUpToDate;

		unsigned _nCtrlPts;
		unsigned _nIntervNonNull;
		TrajectorySpline::KnotVectorType _knots;
		//TrajectorySpline::ControlPointVectorType _ctrlpts;

		Eigen::Array< double, 1, Eigen::Dynamic > _genKnots(const double initT, const double finalT, const bool nonUniform, const unsigned nIntervNonNull) const;
	};

	template<class T>
	Eigen::Matrix<double, Trajectory::dim, Eigen::Dynamic> Trajectory::cArray2CtrlPtsMat(T ctrlpts)
	{
		Eigen::Matrix<double, dim, Eigen::Dynamic> ctrlPts(dim, _nCtrlPts);

		// Feed ctrlPts rows with values from the primal variables x
		for (int i = 0; i < int(_nCtrlPts*dim); ++i)
		{
			ctrlPts(i % dim, i / dim) = ctrlpts[i];
		}
		return ctrlPts;
	}

}

#endif // __AIV_TRAJECTORY_HPP__

// cmake:sourcegroup=PathPlanner