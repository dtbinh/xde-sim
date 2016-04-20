#ifndef __AIV_PATHPLANNER_HPP__
#define __AIV_PATHPLANNER_HPP__
#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Lgsm>
#include <map>
#include <unsupported/Eigen/Splines>
#include "aiv/pathplanner/FlatoutputMonocycle.hpp"

namespace aiv {

	//typedef Eigen::Spline< double, aiv::FlatoutputMonocycle::flatDim, aiv::FlatoutputMonocycle::flatDerivDeg + 1 > MySpline;
	class Obstacle;
	class AIV;

	class PathPlanner
	{
	protected:
		std::string name;
	public:
		PathPlanner(std::string name);
		virtual void update(std::map<std::string, Obstacle *> detectedObst,
		 		std::map<std::string, AIV *> otherVehicles,
		 		const Eigen::Displacementd & myRealPose) = 0;
		virtual double getLinVelocity() const = 0;
		virtual double getAngVelocity() const = 0;
		virtual double getLinAccel() const = 0;
		virtual double getAngAccel() const = 0;
		virtual double getMaxLinVelocity() const = 0;

		virtual double getXPosition() const = 0;
		virtual double getYPosition() const = 0;
		virtual double getOrientation() const = 0;

		//virtual double getSecRho() = 0;
		//virtual double getComRange() = 0;
		//irtual bool isDone() = 0;
		//virtual double getInitTimeOfCurrPlan() = 0;
		//virtual MySpline getSpline() = 0;
		//const Eigen::Displacementd & realPose, const Eigen::Twistd &realVelocity) = 0;
	};

}

#endif // __AIV_PATHPLANNER_HPP__

// cmake:sourcegroup=PathPlanner