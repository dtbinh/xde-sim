#ifndef __AIV_CONTROLLERMONOCYCLE_HPP__
#define __AIV_CONTROLLERMONOCYCLE_HPP__
#pragma once

#include "aiv/controller/Controller.hpp"
#include "aiv/pathplanner/FlatoutputMonocycle.hpp"
#include <boost/thread.hpp>
#include "aiv/controller/Gain.hpp"
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

		unsigned long long _updateCallCntr;
		//PID *pid;

		enum PlanStage { INIT, INTER, FINAL, DONE } _planStage;

		void ControllerMonocycle::NCGPCCompleteModel(
			double a1_r, double a2_r, double u1_r, double u2_r, double x_r, double y_r, double theta_r, double u, double w, double x, double y, double theta);

		void ControllerMonocycle::NCGPCKineModel(
			double u1_r, double u2_r, double x_r, double y_r, double theta_r, double x, double y, double theta);

		void ControllerMonocycle::gainOpt();

		boost::mutex _gainOptMutex;
		boost::thread _ctrlThread;
		bool _shouldUpdateGain;

	public:
		ControllerMonocycle(std::string name, double updateTimeStep, const double k1, const double k2, const double k3, const double maxU1, const double maxU2);
		~ControllerMonocycle();

		void setOption(std::string optionName, double optionValue);
		
		void update(Eigen::Displacementd, Eigen::Twistd, double, double, double, double, double);

		double getLinVelocity()	const	{return _u1;}	// m/s
		double getAngVelocity()	const	{return _u2;}	// rad/s

	};

}

#endif // __AIV_CONTROLLERMONOCYCLE_HPP__

// cmake:sourcegroup=Controller