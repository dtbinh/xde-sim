#ifndef __AIV_CONTROLLERMONOCYCLE_HPP__
#define __AIV_CONTROLLERMONOCYCLE_HPP__
#pragma once

#include "aiv/controller/Controller.hpp"
#include "aiv/pathplanner/FlatoutputMonocycle.hpp"
#include <boost/thread.hpp>
//#include "PID.hpp"

namespace aiv {

	class ControllerMonocycle : public Controller
	{
	private:
		static const unsigned _gainDim = 3;
		Gain _gain;
		Gain _auxGain;
		double _updateTimeStep;
		double  _u1, _u2, _k1, _k2, _k3;
		double _maxU1, _maxU2;
		double _firstPlanTimespan;

		unsigned long long updateCallCntr;
		//PID *pid;

		enum PlanStage { INIT, INTER, FINAL, DONE } _planStage;

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

// cmake:sourcegroup=Controller