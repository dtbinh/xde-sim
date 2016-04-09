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
		// NCGPCCompleteModel constants
		double _predictionTime;
		static const unsigned stateDim = 5;
		static const unsigned observDim = 5;
		static const unsigned outputDim = 2;
		static const unsigned dMParamDim = 6;
		typedef Eigen::Matrix< double, stateDim, 1 > StatVector;
		typedef Eigen::Matrix< double, observDim, 1 > ObservVector;
		typedef Eigen::Matrix< double, outputDim, 1 > OutputVector;
		typedef Eigen::Matrix< double, dMParamDim, 1 > DyModParamVector;

		static const StatVector relativeDegOfNonLinMIMO;
		static const unsigned relativeDegOfNonLinMIMOSum = 13; // equals to (relativeDegOfNonLinMIMO+1).sum(), but sum() is runtime, I want it at compitation time

		Eigen::Matrix<double, 1, 3> _K2;
		Eigen::Matrix<double, 1, 2> _K1;
		Eigen::Matrix< double, observDim, relativeDegOfNonLinMIMOSum > _K;
		//

		std::string _ctrllerType;

		static const unsigned _gainDim = 3;
		Gain _gain;
		Gain _auxGain;
		double _updateTimeStep;
		double  _u1, _u2;
		double _maxU1, _maxU2;
		double _firstPlanTimespan;

		unsigned long long _updateCallCntr;
		//PID *pid;

		enum PlanStage { INIT, INTER, FINAL, DONE } _planStage;

		void ControllerMonocycle::_NCGPCCompleteModel(
			double a1_r, double a2_r, double u1_r, double u2_r, double x_r, double y_r, double theta_r, double u, double w, double x, double y, double theta);

		void ControllerMonocycle::_NCGPCKineModel(
			double u1_r, double u2_r, double x_r, double y_r, double theta_r, double x, double y, double theta);

		void ControllerMonocycle::_TRP(
			double u1_r, double u2_r, double x_r, double y_r, double theta_r, double x, double y, double theta);

		void ControllerMonocycle::_gainOpt();

		boost::mutex _gainOptMutex;
		boost::thread _ctrlThread;
		bool _shouldUpdateGain;

	public:

		ControllerMonocycle(std::string name, double updateTimeStep, const double maxU1, const double maxU2);
		~ControllerMonocycle();

		void setOption(std::string optionName, double optionValue);
		void setOption(std::string optionName, std::string optionValue);
		
		void update(Eigen::Displacementd, Eigen::Twistd, double, double, double, double, double, double, double);

		double getLinVelocity()	const	{return _u1;}	// m/s
		double getAngVelocity()	const	{return _u2;}	// rad/s

	};

}

#endif // __AIV_CONTROLLERMONOCYCLE_HPP__

// cmake:sourcegroup=Controller