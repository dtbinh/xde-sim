#ifndef __AIV_CONTROLLER_HPP__
#define __AIV_CONTROLLER_HPP__
#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Lgsm>
#include "aiv/pathplanner/Trajectory.hpp"

namespace aiv {

	class Controller
	{

	public:
		Controller(std::string name);
		virtual void update(
			Eigen::Displacementd aivCurrPos,
			Eigen::Twistd aivCurrVel,
			const Trajectory & reference,
			const Eigen::Vector3d & translation,
			double refEvalTime,
			double planHorizon,
			unsigned planStage,
			double a1_r,
			double a2_r,
			double u1_r,
			double u2_r,
			double x_r,
			double y_r,
			double theta_r) = 0;
		std::string getName();

	protected:
		std::string name;

	};

}

#endif // __AIV_CONTROLLER_HPP__

// cmake:sourcegroup=Controller