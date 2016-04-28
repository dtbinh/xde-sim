
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
		_K2 = (Eigen::Matrix<double, 1, 3>() << 10./(3.*_predictionTime*_predictionTime), 5./(2.*_predictionTime), 1.).finished();
		_K1 = (Eigen::Matrix<double, 1, 2>() << 3./(2.*_predictionTime), 1.).finished();
		_K = (Eigen::Matrix< double, ControllerMonocycle::observDim, ControllerMonocycle::relativeDegOfNonLinMIMOSum >() <<
			_K2, Eigen::Matrix<double, 1, 10>::Zero(),
			Eigen::Matrix<double, 1,  3>::Zero(), _K2, Eigen::Matrix<double, 1, 7>::Zero(),
			Eigen::Matrix<double, 1,  6>::Zero(), _K2, Eigen::Matrix<double, 1, 4>::Zero(),
			Eigen::Matrix<double, 1,  9>::Zero(), _K1, Eigen::Matrix<double, 1, 2>::Zero(),
			Eigen::Matrix<double, 1, 11>::Zero(), _K1).finished();
	}

	void ControllerMonocycle::setOption(std::string optionName, DyModParamVector optionValue)
	{
		if (optionName == "dynModelParam")
		{
			_dMParameters = optionValue;
			std::cout << "======  Set option [ " << optionName << " ] to value [ " << optionValue << " ]" << std::endl;
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
			_K2 << 10./(3.*_predictionTime*_predictionTime), 5./(2.*_predictionTime), 1.;
			_K1 << 3./(2.*_predictionTime), 1.;
			_K = (Eigen::Matrix< double, ControllerMonocycle::observDim, ControllerMonocycle::relativeDegOfNonLinMIMOSum >() <<
				_K2, Eigen::Matrix<double, 1, 10>::Zero(),
				Eigen::Matrix<double, 1,  3>::Zero(), _K2, Eigen::Matrix<double, 1, 7>::Zero(),
				Eigen::Matrix<double, 1,  6>::Zero(), _K2, Eigen::Matrix<double, 1, 4>::Zero(),
				Eigen::Matrix<double, 1,  9>::Zero(), _K1, Eigen::Matrix<double, 1, 2>::Zero(),
				Eigen::Matrix<double, 1, 11>::Zero(), _K1).finished();
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
		double a1_r,
		double a2_r,
		double u1_r,
		double u2_r,
		double x_r,
		double y_r,
		double theta_r,
		double u,
		double w,
		double x,
		double y,
		double theta)
	{
		// A new explicit dynamic path tracking controller using Generalized Predictive Control (Mohamed Krid, Faiz Benamar, and Roland Lenain)

		// Just need to be computed once thruout the whole simulation (does using 'static const" accomplish that?):

		double xdot_r = u1_r * cos(theta_r);
		double ydot_r = u1_r * sin(theta_r);
		double thetadot_r = u2_r;
		double xdotdot_r = a1_r * cos(theta_r);
		double ydotdot_r = a1_r * sin(theta_r);
		double thetadotdot_r = a2_r;

		double L2fy1 = cos(theta)*(_dMParameters[2]/_dMParameters[0]*w*w - _dMParameters[3]/_dMParameters[0]*u) - u*w*sin(theta);
		double L2fy2 = sin(theta)*(_dMParameters[2]/_dMParameters[0]*w*w - _dMParameters[3]/_dMParameters[0]*u) + u*w*cos(theta);
		double L2fy3 = -_dMParameters[4]/_dMParameters[1]*u*w - _dMParameters[5]/_dMParameters[1]*w;
		double Lfy4 = _dMParameters[2]/_dMParameters[0]*w*w - _dMParameters[3]/_dMParameters[0]*u;
		double Lfy5 = -_dMParameters[4]/_dMParameters[1]*u*w - _dMParameters[5]/_dMParameters[1]*w;

		Eigen::Matrix < double, relativeDegOfNonLinMIMOSum, 1> E =
			(Eigen::Matrix < double, relativeDegOfNonLinMIMOSum, 1>() <<
			x - x_r,
			u*cos(theta)-xdot_r,
			L2fy1 - xdotdot_r,
			y - y_r,
			u*sin(theta)-ydot_r,
			L2fy2 - ydotdot_r,
			Common::wrapToPi(theta - theta_r),
			w - thetadot_r,
			L2fy3 - thetadotdot_r,
			u - u1_r,
			Lfy4 - a1_r,
			w - u2_r,
			Lfy5 - a2_r ).finished();

		Eigen::Matrix < double, outputDim, observDim> DtDm1Dt =
			(Eigen::Matrix < double, outputDim, observDim>() <<
			cos(theta)*_dMParameters[0]/2., sin(theta)*_dMParameters[0]/2., 				   0, _dMParameters[0]/2., 				   0,
										0,							   0, _dMParameters[1]/2., 				   0, _dMParameters[1]/2.).finished();

		Eigen::Vector2d cntrlOut;
		cntrlOut = -2.*DtDm1Dt.block<outputDim, 3>(0,0)*_K.block<3, 3*3>(0,0)*E.block<3*3, 1>(0,0);

		_u1 = cntrlOut(0,0);
		_u2 = cntrlOut(1,0);

		// limiting output
		_u1 = max( min( _u1, _maxU1 ), -1*_maxU1 );
		_u2 = max( min( _u2, _maxU2 ), -1*_maxU2 );
	}

	void ControllerMonocycle::update(
		Eigen::Displacementd aivCurrPos,
		Eigen::Twistd aivCurrVel,
		double plannedXPos,
		double plannedYPos,
		double plannedOrientation,
		double plannedLinVel,
		double plannedAngVel,
		double plannedLinAccel,
		double plannedAngAccel)
	{

		//double tic = Common::getRealTime();
		// Getting robot's orientation projected in the ground plane (unless the vehicle is flying this is the right orientation)
		//  1. get the quaternion part of the displacement
		_auxQuaternion = Eigen::Quaternion<double>(aivCurrPos.qw(), aivCurrPos.qx(), aivCurrPos.qy(), aivCurrPos.qz());
		//  2. rotation of UnityX by the quaternion q (i.e. x of the robot frame wrt the absolute frame)
		_coordXB = _auxQuaternion._transformVector(Eigen::Vector3d::UnitX());

		_NCGPC(
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
		//std::cout << "controller elapsed time: " <<  Common::getRealTime() - tic << std::endl; 
	}
}
	

// cmake:sourcegroup=Controller