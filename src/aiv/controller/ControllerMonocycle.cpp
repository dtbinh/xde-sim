
#include "aiv/controller/ControllerMonocycle.hpp"
#include <limits>
//#include <boost/timer.hpp>
#include "aiv/helpers/Common.hpp"

namespace aiv
{

	ControllerMonocycle::~ControllerMonocycle(){}

	ControllerMonocycle::ControllerMonocycle(std::string name):
		Controller(name),
		_u1(0.0),
		_u2(0.0),
		_maxU1(std::numeric_limits<double>::infinity()),
		_maxU2(std::numeric_limits<double>::infinity()),
		_predictionTime(0.6)
	{
		_KssInv << 20./std::pow(_predictionTime, 5), 0, 0,
				   0, 20./std::pow(_predictionTime, 5), 0,
				   0, 0, 20./std::pow(_predictionTime, 5);

		_Ks << std::pow(_predictionTime, 3)/6.,std::pow(_predictionTime, 4)/8.,std::pow(_predictionTime, 5)/20.,0,0,0,0,0,0,
			0,0,0,std::pow(_predictionTime,3)/6.,std::pow(_predictionTime,4)/8.,std::pow(_predictionTime,5)/20.,0,0,0,
			0,0,0,0,0,0,std::pow(_predictionTime, 3)/6.,std::pow(_predictionTime, 4)/8.,std::pow(_predictionTime, 5)/20.;

	}

	void ControllerMonocycle::setOption(std::string optionName, DyModParamVector optionValue)
	{
		if (optionName == "dynModelParam")
		{
			_dMParameters = optionValue;
			std::cout << "======  Set option [ " << optionName << " ] to value [ " << optionValue.transpose() << " ]" << std::endl;
		}
		else
		{
			std::cout << "Unknown option " << optionName << std::endl;
			std::cout << "======  Set option [ " << optionName << " ] to value [ " << optionValue << " ]" << std::endl;
		}
	}
	void ControllerMonocycle::setOption(std::string optionName, double optionValue)
	{
		if (optionName == "predictionHorizon")
		{
			_predictionTime = optionValue;
			//update Gain Matrix
			_KssInv << 20./std::pow(_predictionTime, 5), 0, 0,
				   0, 20./std::pow(_predictionTime, 5), 0,
				   0, 0, 20./std::pow(_predictionTime, 5);

			_Ks << std::pow(_predictionTime, 3)/6.,std::pow(_predictionTime, 4)/8.,std::pow(_predictionTime, 5)/20.,0,0,0,0,0,0,
				0,0,0,std::pow(_predictionTime,3)/6.,std::pow(_predictionTime,4)/8.,std::pow(_predictionTime,5)/20.,0,0,0,
				0,0,0,0,0,0,std::pow(_predictionTime, 3)/6.,std::pow(_predictionTime, 4)/8.,std::pow(_predictionTime, 5)/20.;
			
			std::cout << "======  Set option [ " << optionName << " ] to value [ " << optionValue << " ]" << std::endl;
		}
		else if (optionName == "maxu1")
		{
			_maxU1 = optionValue;
			std::cout << "======  Set option [ " << optionName << " ] to value [ " << optionValue << " ]" << std::endl;
		}
		else if (optionName == "maxu2")
		{
			_maxU2 = optionValue;
			std::cout << "======  Set option [ " << optionName << " ] to value [ " << optionValue << " ]" << std::endl;
		}
		else
		{
			std::cout << "Unknown option " << optionName << std::endl;
		}
	}

	void ControllerMonocycle::_NCGPC(
		const Trajectory & reference,
		const Eigen::Vector3d & translation,
		double refEvalTime,
		double planHorizon,
		unsigned planStage,
		// double a1_r,
		// double a2_r,
		// double u1_r,
		// double u2_r,
		// double x_r,
		// double y_r,
		// double theta_r,
		double u,
		double w,
		double x,
		double y,
		double theta)
	{

		if (planStage == 0 || planStage == 3)
		{
			_u1 = 0.0;
			_u2 = 0.0;
			return;
		}

		if (planStage == 2)
		{

			if (_predictionTime > planHorizon - refEvalTime)
			{
				_predictionTime = planHorizon - refEvalTime;

				//update Gain Matrix
				_KssInv << 20./std::pow(_predictionTime, 5), 0, 0,
								   0, 20./std::pow(_predictionTime, 5), 0,
								   0, 0, 20./std::pow(_predictionTime, 5);

				_Ks <<
					std::pow(_predictionTime, 3)/6.,std::pow(_predictionTime, 4)/8.,
					std::pow(_predictionTime, 5)/20.,0,
					0,0,
					0,0,
					0,0,
					0,0,
					std::pow(_predictionTime,3)/6.,std::pow(_predictionTime,4)/8.,
					std::pow(_predictionTime,5)/20.,0,
					0,0,
					0,0,
					0,0,
					0,0,
					std::pow(_predictionTime, 3)/6.,std::pow(_predictionTime, 4)/8.,
					std::pow(_predictionTime, 5)/20.;
			}
		}

		double integrationTimeStep = 0.1; //in seconds, may differ from 1/double(samplesNumber)*_predictionTime

		int samplesNumber = int(_predictionTime/integrationTimeStep);

		Eigen::Vector3d Rs = Eigen::Vector3d::Zero();
		Eigen::Vector3d previousF = Eigen::Vector3d::Zero();

		for (int i=1; i<=samplesNumber; ++i)
		{
			double tau = double(i)/samplesNumber * _predictionTime;

			Eigen::Matrix<double, FlatoutputMonocycle::flatDim, FlatoutputMonocycle::flatDerivDeg+1> derivFlat =
				reference(refEvalTime+tau, FlatoutputMonocycle::flatDerivDeg);

			Eigen::Vector3d poseOutput = FlatoutputMonocycle::flatToPose(derivFlat);
			poseOutput.block<FlatoutputMonocycle::posDim, 1>(FlatoutputMonocycle::posIdx, 0) =
				poseOutput.block<FlatoutputMonocycle::posDim, 1>(FlatoutputMonocycle::posIdx, 0) +
				translation.block<FlatoutputMonocycle::posDim, 1>(FlatoutputMonocycle::posIdx, 0);

			Rs += (poseOutput*tau*tau + previousF)/2. * _predictionTime/samplesNumber;

			previousF = poseOutput*tau*tau;
		}

		Eigen::Matrix<double, FlatoutputMonocycle::flatDim, FlatoutputMonocycle::flatDerivDeg+2> derivFlat =
			reference(refEvalTime, FlatoutputMonocycle::flatDerivDeg+1);

		ObservVector pose = FlatoutputMonocycle::flatToPose(derivFlat);
		pose.block<FlatoutputMonocycle::posDim, 1>(FlatoutputMonocycle::posIdx, 0) =
			pose.block<FlatoutputMonocycle::posDim, 1>(FlatoutputMonocycle::posIdx, 0) +
			translation.block<FlatoutputMonocycle::posDim, 1>(FlatoutputMonocycle::posIdx, 0);

		Eigen::Matrix<double, stateDim-observDim, 1> velo = FlatoutputMonocycle::flatToVelocity(derivFlat);
		Eigen::Matrix<double, stateDim-observDim, 1> accel = FlatoutputMonocycle::flatToAcceleration(derivFlat);

		double x_r = pose[0];
		double y_r = pose[1];
		double theta_r = pose[2];
		double u1_r = velo[0];
		double u2_r = velo[1];
		double a1_r = accel[0];
		double a2_r = accel[1];

		// A new explicit dynamic path tracking controller using Generalized Predictive Control
		// (Mohamed Krid, Faiz Benamar, and Roland Lenain)

		// Just need to be computed once thruout the whole simulation (does using 'static const" accomplish that?):

		double xdot_r = u1_r * cos(theta_r);
		double ydot_r = u1_r * sin(theta_r);
		double thetadot_r = u2_r;
		double xdotdot_r = a1_r * cos(theta_r);
		double ydotdot_r = a1_r * sin(theta_r);
		double thetadotdot_r = a2_r;

		double L2fy1 = cos(theta)*(_dMParameters[2]/_dMParameters[0]*w*w - _dMParameters[3]/_dMParameters[0]*u) -
			u*w*sin(theta);
		double L2fy2 = sin(theta)*(_dMParameters[2]/_dMParameters[0]*w*w - _dMParameters[3]/_dMParameters[0]*u) +
			u*w*cos(theta);
		double L2fy3 = -_dMParameters[4]/_dMParameters[1]*u*w - _dMParameters[5]/_dMParameters[1]*w;		
		double Lfy4 = _dMParameters[2]/_dMParameters[0]*w*w - _dMParameters[3]/_dMParameters[0]*u;
		double Lfy5 = -_dMParameters[4]/_dMParameters[1]*u*w - _dMParameters[5]/_dMParameters[1]*w;

		Eigen::Matrix < double, relativeDegOfNonLinMIMOSum, 1> E =
			(Eigen::Matrix < double, relativeDegOfNonLinMIMOSum, 1>() <<
			(x - x_r),
			u*cos(theta)-xdot_r,
			L2fy1 - xdotdot_r,
			(y - y_r),
			u*sin(theta)-ydot_r,
			L2fy2 - ydotdot_r,
			Common::wrapToPi(theta - theta_r),
			w - thetadot_r,
			L2fy3 - thetadotdot_r).finished();

		Eigen::Matrix < double, relativeDegOfNonLinMIMOSum, 1> Ly =
			(Eigen::Matrix < double, relativeDegOfNonLinMIMOSum, 1>() <<
			x,
			u*cos(theta),
			L2fy1,
			y,
			u*sin(theta),
			L2fy2,
			theta,
			w,
			L2fy3
			// u - u1_r,
			// Lfy4 - a1_r,
			// w - u2_r,
			// Lfy5 - a2_r
			).finished();

		Eigen::Matrix < double, relativeDegOfNonLinMIMOSum, 1> Lyref =
			(Eigen::Matrix < double, relativeDegOfNonLinMIMOSum, 1>() <<
			x_r,
			xdot_r,
			xdotdot_r,
			y_r,
			ydot_r,
			ydotdot_r,
			theta_r,
			thetadot_r,
			thetadotdot_r
			// u - u1_r,
			// Lfy4 - a1_r,
			// w - u2_r,
			// Lfy5 - a2_r
			).finished();

		Eigen::Matrix < double, outputDim, observDim> DtDInvDt =
			(Eigen::Matrix < double, outputDim, observDim>() <<
				cos(theta)*_dMParameters[0], sin(theta)*_dMParameters[0], 0,
				0, 0, _dMParameters[1]).finished();

		std::cout << "++++++++\n";
		std::cout << Rs - _Ks*Lyref;
		std::cout << "\n++++++++\n";

		Eigen::Vector2d cntrlOut = -DtDInvDt*_KssInv*(_Ks*Ly - Rs);

		double a = ydot_r;
		double b = -xdot_r;
		double c = xdot_r * y_r - ydot_r * x_r;

		double signedDist = (a*x + b*y + c) * Common::finvsqrt(a*a + b*b);


		_u1 = cntrlOut(0,0);
		_u2 = cntrlOut(1,0);// + signedDist;

		// limiting output
		_u1 = max( min( _u1, _maxU1 ), -1*_maxU1 );
		_u2 = max( min( _u2, _maxU2 ), -1*_maxU2 );
	}

	void ControllerMonocycle::update(
		Eigen::Displacementd aivCurrPos,
		Eigen::Twistd aivCurrVel,
		const Trajectory & reference,
		const Eigen::Vector3d & translation,
		double refEvalTime,
		double planHorizon,
		unsigned planStage)
		// double plannedXPos,
		// double plannedYPos,
		// double plannedOrientation,
		// double plannedLinVel,
		// double plannedAngVel,
		// double plannedLinAccel,
		// double plannedAngAccel
	{

		//double tic = Common::getRealTime();
		// Getting robot's orientation projected in the ground plane
		// (unless the vehicle is flying this is the right orientation)
		//  1. get the quaternion part of the displacement
		_auxQuaternion = Eigen::Quaternion<double>(aivCurrPos.qw(), aivCurrPos.qx(), aivCurrPos.qy(), aivCurrPos.qz());
		//  2. rotation of UnityX by the quaternion q (i.e. x of the robot frame wrt the absolute frame)
		_coordXB = _auxQuaternion._transformVector(Eigen::Vector3d::UnitX());

		_NCGPC(reference,
			translation,
			refEvalTime,
			planHorizon,
			planStage,
			// plannedLinAccel,
			// plannedAngAccel,
			// plannedLinVel,
			// plannedAngVel,
			// plannedXPos,
			// plannedYPos,
			// plannedOrientation,
			aivCurrVel.block<2, 1>(3, 0).norm(),
			aivCurrVel.topRows(3).z(),
			aivCurrPos.x(),
			aivCurrPos.y(),
			atan2(_coordXB.y(), _coordXB.x()));
		//std::cout << "controller elapsed time: " <<  Common::getRealTime() - tic << std::endl; 
	}
}
	

// cmake:sourcegroup=Controller