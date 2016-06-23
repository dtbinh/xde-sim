
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
		double a1_ref,
		double a2_ref,
		double u1_ref,
		double u2_ref,
		double x_ref,
		double y_ref,
		double theta_ref,
		double u,
		double w,
		double x,
		double y,
		double theta)
	{

		double xdot_ref = u1_ref * cos(theta_ref);
		double ydot_ref = u1_ref * sin(theta_ref);
		double thetadot_ref = u2_ref;
		double xdotdot_ref = a1_ref * cos(theta_ref);
		double ydotdot_ref = a1_ref * sin(theta_ref);
		double thetadotdot_ref = a2_ref;

		Eigen::Vector3d Rs;

		if (planStage == 0)
		{
			_u1 = 0.0;
			_u2 = 0.0;
			return;
		}
		// else if (planStage == 1)
		else if (planStage == 1 || planStage == 2)
		{

			if (planStage == 2 && _predictionTime > planHorizon - refEvalTime)
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

			double integrationTimeStep = 0.001; //in seconds, may differ from 1/double(samplesNumber)*_predictionTime

			int samplesNumber = int(_predictionTime/integrationTimeStep);

			Rs = Eigen::Vector3d::Zero();
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

				Rs += (poseOutput*tau*tau + previousF)/4. * _predictionTime/samplesNumber;

				previousF = poseOutput*tau*tau;
			}
		}
		else if (planStage == 3)
		// else if (planStage == 3 || planStage == 2)
		{
			// if (planStage == 2 && _predictionTime > planHorizon - refEvalTime)
			// {
			// 	_predictionTime = planHorizon - refEvalTime;

			// 	//update Gain Matrix
			// 	_KssInv << 20./std::pow(_predictionTime, 5), 0, 0,
			// 					   0, 20./std::pow(_predictionTime, 5), 0,
			// 					   0, 0, 20./std::pow(_predictionTime, 5);

			// 	_Ks <<
			// 		std::pow(_predictionTime, 3)/6.,std::pow(_predictionTime, 4)/8.,
			// 		std::pow(_predictionTime, 5)/20.,0,
			// 		0,0,
			// 		0,0,
			// 		0,0,
			// 		0,0,
			// 		std::pow(_predictionTime,3)/6.,std::pow(_predictionTime,4)/8.,
			// 		std::pow(_predictionTime,5)/20.,0,
			// 		0,0,
			// 		0,0,
			// 		0,0,
			// 		0,0,
			// 		std::pow(_predictionTime, 3)/6.,std::pow(_predictionTime, 4)/8.,
			// 		std::pow(_predictionTime, 5)/20.;
			// }
			// else
			// 	_predictionTime = 0.5;

			_predictionTime = 0.5;
			// TODO no magic number

			// update Gain Matrix
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

			Rs = _Ks* (Eigen::Matrix < double, relativeDegOfNonLinMIMOSum, 1>() <<
				x_ref,
				xdot_ref,
				xdotdot_ref,
				y_ref,
				ydot_ref,
				ydotdot_ref,
				theta_ref,
				thetadot_ref,
				thetadotdot_ref).finished();
		}


		// Eigen::Matrix<double, FlatoutputMonocycle::flatDim, FlatoutputMonocycle::flatDerivDeg+2> derivFlat =
		// 	reference(refEvalTime, FlatoutputMonocycle::flatDerivDeg+1);

		// ObservVector pose = FlatoutputMonocycle::flatToPose(derivFlat);
		// pose.block<FlatoutputMonocycle::posDim, 1>(FlatoutputMonocycle::posIdx, 0) =
		// 	pose.block<FlatoutputMonocycle::posDim, 1>(FlatoutputMonocycle::posIdx, 0) +
		// 	translation.block<FlatoutputMonocycle::posDim, 1>(FlatoutputMonocycle::posIdx, 0);

		// Eigen::Matrix<double, stateDim-observDim, 1> velo = FlatoutputMonocycle::flatToVelocity(derivFlat);
		// Eigen::Matrix<double, stateDim-observDim, 1> accel = FlatoutputMonocycle::flatToAcceleration(derivFlat);


		double L2fy1 = cos(theta)*(_dMParameters[2]/_dMParameters[0]*w*w - _dMParameters[3]/_dMParameters[0]*u) -
			u*w*sin(theta);
		double L2fy2 = sin(theta)*(_dMParameters[2]/_dMParameters[0]*w*w - _dMParameters[3]/_dMParameters[0]*u) +
			u*w*cos(theta);
		double L2fy3 = -_dMParameters[4]/_dMParameters[1]*u*w - _dMParameters[5]/_dMParameters[1]*w;		
		double Lfy4 = _dMParameters[2]/_dMParameters[0]*w*w - _dMParameters[3]/_dMParameters[0]*u;
		double Lfy5 = -_dMParameters[4]/_dMParameters[1]*u*w - _dMParameters[5]/_dMParameters[1]*w;

		// Eigen::Matrix < double, relativeDegOfNonLinMIMOSum, 1> E =
		// 	(Eigen::Matrix < double, relativeDegOfNonLinMIMOSum, 1>() <<
		// 	(x - x_ref),
		// 	u*cos(theta)-xdot_ref,
		// 	L2fy1 - xdotdot_ref,
		// 	(y - y_ref),
		// 	u*sin(theta)-ydot_ref,
		// 	L2fy2 - ydotdot_ref,
		// 	Common::wrapToPi(theta - theta_ref),
		// 	w - thetadot_ref,
		// 	L2fy3 - thetadotdot_ref).finished();

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
			// u - u1_ref,
			// Lfy4 - a1_ref,
			// w - u2_ref,
			// Lfy5 - a2_ref
			).finished();

		// Eigen::Matrix < double, relativeDegOfNonLinMIMOSum, 1> Lyref =
		// 	(Eigen::Matrix < double, relativeDegOfNonLinMIMOSum, 1>() <<
		// 	x_ref,
		// 	xdot_ref,
		// 	xdotdot_ref,
		// 	y_ref,
		// 	ydot_ref,
		// 	ydotdot_ref,
		// 	theta_ref,
		// 	thetadot_ref,
		// 	thetadotdot_ref
		// 	// u - u1_ref,
		// 	// Lfy4 - a1_ref,
		// 	// w - u2_ref,
		// 	// Lfy5 - a2_ref
		// 	).finished();

		Eigen::Matrix < double, outputDim, observDim> DtDInvDt =
			(Eigen::Matrix < double, outputDim, observDim>() <<
				cos(theta)*_dMParameters[0], sin(theta)*_dMParameters[0], 0,
				0, 0, _dMParameters[1]).finished();

		// std::cout << "++++++++\n";
		// std::cout << Rs/2. - _Ks*Lyref;
		// std::cout << "\n++++++++\n";

		Eigen::Vector2d cntrlOut = -DtDInvDt*_KssInv*(_Ks*Ly - Rs);

		double a = ydot_ref;
		double b = -xdot_ref;
		double c = xdot_ref * y_ref - ydot_ref * x_ref;

		double signedDist = (a*x + b*y + c) * Common::finvsqrt(a*a + b*b);


		_u1 = cntrlOut(0,0);
		_u2 = cntrlOut(1,0);//+ 2.*signedDist; //TODO no hardcoded constant, put on config.xml

		// limiting output
		_u1 = max( min( _u1, _maxU1 ), -1*_maxU1 );
		_u2 = max( min( _u2, _maxU2 ), -1*_maxU2 );

		// _u1 = max( min( _u1, 100*u1_ref ), -1*100*u1_ref );
		// _u2 = max( min( _u2, 100*u2_ref ), -1*100*u2_ref );
	}

	// void handbookOfRoboticsSprint()
	// {
	// 		//Treating input
	// 		double u1_r = plannedLinVel;
	// 		double u2_r = plannedAngVel;

	// 		double x_r = plannedXPos;
	// 		double y_r = plannedYPos;
	// 		double theta_r = plannedOrientation;

	// 		// // Getting robot's orientation projected in the ground plane (unless the vehicle is flying this is the right orientation)
	// 		// // 1. get the quaternion part of the displacement
	// 		Eigen::Quaternion<double> q = Eigen::Quaternion<double>(aivCurrPos.qw(), aivCurrPos.qx(), aivCurrPos.qy(), aivCurrPos.qz());
	// 		// // 2. rotation of UnityX by the quaternion q (i.e. x of the robot frame wrt the absolute frame)
	// 		Eigen::QuaternionBase<Eigen::Quaternion<double> >::Vector3 coordXB = q._transformVector(Eigen::Vector3d::UnitX());

	// 		double x = aivCurrPos.x();
	// 		double y = aivCurrPos.y();
	// 		double theta = atan2(coordXB.y(), coordXB.x());


	// 		//FROM HANDBOOK OF ROBOTICS SPRING

	// 		//------------------ Generating input w1 w2 --------------------

	// 		Eigen::Vector3d err = (Eigen::Vector3d() << x-x_r, y-y_r, theta-theta_r).finished(); // errors

	// 		Eigen::Matrix2d rotRef = (Eigen::Matrix2d() << cos(theta_r), sin(theta_r), -1*sin(theta_r), cos(theta_r)).finished();

	// 		Eigen::Vector2d err_r = rotRef*err.block<2,1>(0,0);

	// 		double xi1 = -1*std::abs(u1_r)*(err_r(0)+err_r(1)*tan(err(2)));
	// 		double xi2 = -1*u1_r*err_r(1);
	// 		double xi3 = -1*std::abs(u1_r)*tan(err(2));

	// 		// Only proportional gain
	// 		_gain(evalTime*_cntrlHorizon);
	// 		double w1 = _gain.k(0)*xi1;
	// 		double w2 = _gain.k(1)*xi2 + gain.k(2)(evalTime*_cntrlHorizon)*xi3;

	// 		//----------------- Back to u1, u2 from w1, w2 ---------------------

	// 		_u1 = (w1 + u1_r)/cos(err(2));
	// 		_u2 = w2*std::pow(cos(err(2)), 2)+u2_r;

	// 		// limiting output
	// 		_u1 = std::max(std::min(_u1, _maxU1), -1*_maxU1);
	// 		_u2 = std::max(std::min(_u2, _maxU2), -1*_maxU2);
	// }

	void ControllerMonocycle::update(
		Eigen::Displacementd aivCurrPos,
		Eigen::Twistd aivCurrVel,
		const Trajectory & reference,
		const Eigen::Vector3d & translation,
		double refEvalTime,
		double planHorizon,
		unsigned planStage,
		double plannedXPos,
		double plannedYPos,
		double plannedOrientation,
		double plannedLinVel,
		double plannedAngVel,
		double plannedLinAccel,
		double plannedAngAccel)
	{

		double tic = Common::getRealTime<double>();
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
			plannedLinAccel,
			plannedAngAccel,
			plannedLinVel,
			plannedAngVel,
			plannedXPos,
			plannedYPos,
			plannedOrientation,
			aivCurrVel.block<2, 1>(3, 0).norm(),
			aivCurrVel.topRows(3).z(),
			aivCurrPos.x(),
			aivCurrPos.y(),
			atan2(_coordXB.y(), _coordXB.x()));
		// std::cout << BG_L_RED << "controller elapsed time: " << RESET <<  Common::getRealTime<double>() - tic << std::endl;
	}
}
	

// cmake:sourcegroup=Controller