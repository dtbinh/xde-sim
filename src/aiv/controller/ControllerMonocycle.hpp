#ifndef __AIV_CONTROLLERMONOCYCLE_HPP__
#define __AIV_CONTROLLERMONOCYCLE_HPP__
#pragma once

#include "aiv/controller/Controller.hpp"
#include <boost/thread.hpp>

namespace aiv {

	class ControllerMonocycle : public Controller
	{
	private:
		// 
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
		DyModParamVector _dMParameters;
		//

		double  _u1, _u2;
		double _maxU1, _maxU2;

		Eigen::Quaternion<double> _auxQuaternion;
		Eigen::QuaternionBase<Eigen::Quaternion<double> >::Vector3 _coordXB;

		void _NCGPC(double a1_r, double a2_r, double u1_r, double u2_r, double x_r, double y_r, double theta_r, double u, double w, double x, double y, double theta);

	public:

		ControllerMonocycle(std::string name);
		~ControllerMonocycle();

		void setOption(std::string optionName, double optionValue);
		void setOption(std::string optionName, DyModParamVector optionValue);
		
		void update(Eigen::Displacementd, Eigen::Twistd, double, double, double, double, double, double, double);

		double getLinVelocity()	const	{return _u1;}	// m/s
		double getAngVelocity()	const	{return _u2;}	// rad/s
	};

}

#endif // __AIV_CONTROLLERMONOCYCLE_HPP__

// cmake:sourcegroup=Controller