#ifndef __AIV_CONTROLLERMONOCYCLE_HPP__
#define __AIV_CONTROLLERMONOCYCLE_HPP__
#pragma once

#include "Controller.hpp"
#include "PID.hpp"

namespace aiv {

	class ControllerMonocycle : public Controller
	{
	private:
		double _updateTimeStep;
		double  _u1, _u2, _k1, _k2, _k3;
		double _maxU1, _maxU2;
		//PID *pid;

	public:
		ControllerMonocycle(std::string name, double updateTimeStep, const double k1, const double k2, const double k3, const double maxU1, const double maxU2);
		~ControllerMonocycle();
		//void init();
		void update(Eigen::Displacementd, Eigen::Twistd, double, double, double, double, double);

		double getLinVelocity()	const	{return _u1;}	// m/s
		double getAngVelocity()	const	{return _u2;}	// rad/s

	};

}

#endif // __AIV_CONTROLLERMONOCYCLE_HPP__

// cmake:sourcegroup=ControllerMonocycle