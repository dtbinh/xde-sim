#ifndef __AIV__PATHPLANNER_HPP__
#define __AIV__PATHPLANNER_HPP__
#pragma once

#include <Eigen/Dense>
#include <Eigen/Lgsm>
#include <map>
//#include <unsupported/Eigen/Splines>
#include "aiv/pathplanner/FlatoutputMonocycle.hpp"
#include "aiv/pathplanner/Trajectory.hpp"

namespace aiv
{
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
			const Eigen::Displacementd & currentPose,
			const Eigen::Twistd & currentVelo,
			const double myRadius) = 0;

		virtual void lockIntendedSolMutex() = 0;
		virtual void unlockIntendedSolMutex() = 0;
		
		virtual double getIntendedBaseTime() const = 0;
		virtual double getIntendedPlanHor() const = 0;

		virtual const Eigen::Matrix<double, FlatoutputMonocycle::flatDim, 1> & getIntendedBasePos() const = 0;

		virtual const Trajectory & getIntendedTrajectory() const = 0;
		
		// virtual double getMaxLinAccel() const = 0;

		enum PlanStage {INIT, INTER, FINAL, DONE};
		
		virtual PlanStage getIntendedPlanStage() const = 0;

		virtual double getTargetedLinVelocity() const = 0;
		virtual const Eigen::Matrix<double, FlatoutputMonocycle::poseDim, 1> & getTargetedPose() const = 0;

		virtual double getLinVelocity() const = 0;
		virtual double getAngVelocity() const = 0;
		virtual double getLinAccel() const = 0;
		virtual double getAngAccel() const = 0;
		virtual double getMaxLinVelocity() const = 0;

		virtual double getXPosition() const = 0;
		virtual double getYPosition() const = 0;
		virtual double getOrientation() const = 0;
		virtual double getComRange() const = 0;
		virtual double getInterRobSafDist() const = 0;

	};
}

#endif // __AIV__PATHPLANNER_HPP__

// cmake:sourcegroup=PathPlanner