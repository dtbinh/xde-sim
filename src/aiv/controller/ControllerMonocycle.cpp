
#include "aiv/controller/ControllerMonocycle.hpp"
#include <nlopt.h>
#include <omp.h>

namespace aiv {


	// _NCGPCCompleteModel constants
	const ControllerMonocycle::StatVector ControllerMonocycle::relativeDegOfNonLinMIMO = (ControllerMonocycle::StatVector() << 2, 2, 2, 1, 1).finished();

	ControllerMonocycle::ControllerMonocycle(std::string name, double updateTimeStep, const double maxU1, const double maxU2)
		: Controller(name),
		_updateTimeStep(updateTimeStep),
		_u1(0.0),
		_u2(0.0),
		_maxU1(maxU1),
		_maxU2(maxU2),
		_updateCallCntr(0),
		_planStage(INIT),
		_shouldUpdateGain(false),
		_predictionTime(0.8),
		_ctrllerType("TRP"),
		_firstPlanTimespan(1.0)
	{
		//pid = new PID(updateTimeStep,1000000000,-1000000000,1,0.3,0.1);
		double gainValues[_gainDim] = {1, 1, 2};
		_gain.setGainValue(gainValues);
		_auxGain = _gain;

		_K2 = (Eigen::Matrix<double, 1, 3>() << 10./(3.*_predictionTime*_predictionTime), 5./(2.*_predictionTime), 1.).finished();
		_K1 = (Eigen::Matrix<double, 1, 2>() << 3./(2.*_predictionTime), 1.).finished();
		_K = (Eigen::Matrix< double, ControllerMonocycle::observDim, ControllerMonocycle::relativeDegOfNonLinMIMOSum >() <<
			_K2, Eigen::Matrix<double, 1, 10>::Zero(),
			Eigen::Matrix<double, 1,  3>::Zero(), _K2, Eigen::Matrix<double, 1, 7>::Zero(),
			Eigen::Matrix<double, 1,  6>::Zero(), _K2, Eigen::Matrix<double, 1, 4>::Zero(),
			Eigen::Matrix<double, 1,  9>::Zero(), _K1, Eigen::Matrix<double, 1, 2>::Zero(),
			Eigen::Matrix<double, 1, 11>::Zero(), _K1).finished();
	}

	void  ControllerMonocycle::setOption(std::string optionName, double optionValue)
	{
		if (optionName == "offsetTime")
		{
			_firstPlanTimespan = optionValue;
		}
		else if (optionName == "predictionHorizon")
		{
			_predictionTime = optionValue;
			_K2 = (Eigen::Matrix<double, 1, 3>() << 10./(3.*_predictionTime*_predictionTime), 5./(2.*_predictionTime), 1.).finished();
			_K1 = (Eigen::Matrix<double, 1, 2>() << 3./(2.*_predictionTime), 1.).finished();
			_K = (Eigen::Matrix< double, ControllerMonocycle::observDim, ControllerMonocycle::relativeDegOfNonLinMIMOSum >() <<
				_K2, Eigen::Matrix<double, 1, 10>::Zero(),
				Eigen::Matrix<double, 1,  3>::Zero(), _K2, Eigen::Matrix<double, 1, 7>::Zero(),
				Eigen::Matrix<double, 1,  6>::Zero(), _K2, Eigen::Matrix<double, 1, 4>::Zero(),
				Eigen::Matrix<double, 1,  9>::Zero(), _K1, Eigen::Matrix<double, 1, 2>::Zero(),
				Eigen::Matrix<double, 1, 11>::Zero(), _K1).finished();
		}
		else if (optionName == "k1")
		{
			_gain.setGainValue(optionValue, 0);
			_auxGain = _gain;
		}
		else if (optionName == "k2")
		{
			_gain.setGainValue(optionValue, 1);
			_auxGain = _gain;
		}
		else if (optionName == "k3")
		{
			_gain.setGainValue(optionValue, 2);
			_auxGain = _gain;
		}
	}
	void  ControllerMonocycle::setOption(std::string optionName, std::string optionValue)
	{
		if (optionName == "ctrllerType")
		{
			_ctrllerType = optionValue;
		}
	}

	//void ControllerMonocycle::init()
	//{
	//	// Public Variables
	//	this->v_r     = 0.0;	// Right wheel velocity (m/s)
	//	this->v_l     = 0.0;	// Left wheel velocity (m/s)
	//}

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

		//std::cout << "Say something\n" << std::endl;
		double currentTime = _updateTimeStep*_updateCallCntr;
		++_updateCallCntr;

		//double evalTime = std::max(((currentTime - (std::max((this->executingPlanIdx), 0)*this->compHorizon)) / this->planHorizon), 0.0);
		double evalTime = 0.0; //FIXME

		// STATE MACHINE 1 (managing threads)
		if (_planStage != DONE && _planStage != FINAL) //only necessery if the planning stage is INIT or INTER
		{
			// if plan 0 has not been computed yet
			if (currentTime < _firstPlanTimespan)
			{
				//do nothing
			}
			// if plan 0 has just been computed
			else if (_planStage == INIT)
			{
				std::cout << "------ Create first gain optimization -------" << std::endl;
				_gainOptMutex.lock();
				_ctrlThread = boost::thread(&ControllerMonocycle::_gainOpt, this); // Opt gain using P0

				_planStage = INTER;

				// reset time

			}
			// If we've already created the thread for optimizing gain using plan 0
			// 		Check if the optimization is still usefull (if none new plan from planner has arrived)
			//		If not usefull 'cause a new plan has arrived, kill thread and create another
			//		Else do nothing
			//
			// 		Check if the opt computation has finished
			//		If so, update gain with mutex locked
			else
			{
				//f (evalTime >= ratio) //FIXME
				if (evalTime >= 0.5) //FIXME
				{
					//evalTime -= ratio;
					evalTime -= 0.5;

					if (!_gainOptMutex.try_lock()) // if Tcc < Tc we are supposed to get this lock
					{
						_ctrlThread.interrupt(); // if not we interrupt opt so a new opt can be done
						_gainOptMutex.lock();
					}
					// _gainOptMutex is locked
					_ctrlThread = boost::thread(&ControllerMonocycle::_gainOpt, this);
				}

				if (!_gainOptMutex.try_lock())
				{
					// if we can't lock (some thread is computing the optimal gain) we whether:
					// do nothing or
					// see if Tcc is getting "too" close to Tc and do something else regarding the gain that will "probably" not be computed on time
				}
				else //successfully lock => no ctrl thread executing
				{
					//check if having no thread executing means that there is a new optimized gain ready
					if (_shouldUpdateGain)
					{
					// update gain spline with auxGain (with mutex locked)
						std::cout << "---------- Update Gain spline ------------" << std::endl;
						_gain = _auxGain;
						_shouldUpdateGain = false;
						_gainOptMutex.unlock();
					}
					// otherwise do nothing
				}
			}
		}
		else if (_planStage == FINAL && evalTime > 1.0)
		{
			////std::cout << "gone to DONE" << std::endl;
			_planStage = DONE;
		}

		// STATE MACHINE 2 (managing output)
		if (_planStage == INIT)
		{
			//DO NOTHING
			////std::cout << evalTime*planHorizon << ", " << initPose[0] << ", " << initPose[1] << std::endl;
			////std::cout << "plan STAGE INIT" << std::endl;
			//nextPoseRef = initPose;
			//nextVelocityRef = initVelocity;
		}
		else if (_planStage == DONE)
		{
			//std::cout << "Planning is over!" << std::endl;
			//std::cout << "Last step planning horizon: " << this->planHorizon << std::endl;
			//boost::this_thread::sleep(boost::posix_time::milliseconds(300));
			////std::cout << evalTime*planHorizon << ", " << targetedPose[0] << ", " << targetedPose[1] << std::endl;
			////std::cout << "plan STAGE DONE" << std::endl;
			_u1 = plannedLinVel;
			_u2 = plannedAngVel;
		}
		else //INTER or FINAL
		{
			//Treating input
			double u1_r = plannedLinVel;
			double u2_r = plannedAngVel;

			double x_r = plannedXPos;
			double y_r = plannedYPos;
			double theta_r = plannedOrientation;

			// // Getting robot's orientation projected in the ground plane (unless the vehicle is flying this is the right orientation)
			// // 1. get the quaternion part of the displacement
			Eigen::Quaternion<double> q = Eigen::Quaternion<double>(aivCurrPos.qw(), aivCurrPos.qx(), aivCurrPos.qy(), aivCurrPos.qz());
			// // 2. rotation of UnityX by the quaternion q (i.e. x of the robot frame wrt the absolute frame)
			Eigen::QuaternionBase<Eigen::Quaternion<double> >::Vector3 coordXB = q._transformVector(Eigen::Vector3d::UnitX());

			double x = aivCurrPos.x();
			double y = aivCurrPos.y();
			double theta = atan2(coordXB.y(), coordXB.x());

			//std::cout << _ctrllerType << std::endl;

			if (_ctrllerType == "TRP")
			{
				_TRP(
					u1_r,
					u2_r,
					x_r,
					y_r,
					theta_r,
					x,
					y,
					theta);
			}
			else if (_ctrllerType == "NCGPCDM")
			{
				_NCGPCCompleteModel(
					plannedLinAccel,
					plannedAngAccel,
					u1_r,
					u2_r,
					x_r,
					y_r,
					theta_r,
					aivCurrVel.block<2, 1>(3, 0).norm(),
					aivCurrVel.topRows(3).z(),
					x,
					y,
					theta);
			}
			else if (_ctrllerType == "NCGPCCM")
			{
				_NCGPCKineModel(
					u1_r,
					u2_r,
					x_r,
					y_r,
					theta_r,
					x,
					y,
					theta);
			}
			//FROM HANDBOOK OF ROBOTICS SPRING

		}
	}

	void ControllerMonocycle::_TRP(double u1_r, double u2_r, double x_r, double y_r, double theta_r, double x, double y, double theta)
	{
		//------------------ Generating input w1 w2 --------------------

		Eigen::Vector3d err = (Eigen::Vector3d() << x-x_r, y-y_r, theta-theta_r).finished(); // errors

		Eigen::Matrix2d rotRef = (Eigen::Matrix2d() << cos(theta_r), sin(theta_r), -1*sin(theta_r), cos(theta_r)).finished();

		Eigen::Vector2d err_r = rotRef*err.block<2,1>(0,0);

		double xi1 = -1*std::abs(u1_r)*(err_r(0)+err_r(1)*tan(err(2)));
		double xi2 = -1*u1_r*err_r(1);
		double xi3 = -1*std::abs(u1_r)*tan(err(2));

		// Only proportional gain
		// _gain(evalTime*_ctrlHorizon); //FIXME
		_gain(0.0);
		double w1 = _gain.k(0)*xi1;
		double w2 = _gain.k(1)*xi2 + _gain.k(2)*xi3;

		//----------------- Back to u1, u2 from w1, w2 ---------------------

		_u1 = (w1 + u1_r)/cos(err(2));
		_u2 = w2*std::pow(cos(err(2)), 2)+u2_r;

		// limiting output
		_u1 = std::max(std::min(_u1, _maxU1), -1*_maxU1);
		_u2 = std::max(std::min(_u2, _maxU2), -1*_maxU2);
	}


	void ControllerMonocycle::_NCGPCKineModel(double u1_r, double u2_r, double x_r, double y_r, double theta_r, double x, double y, double theta)
	{
		// A new explicit dynamic path tracking controller using Generalized Predictive Control (Mohamed Krid, Faiz Benamar, and Roland Lenain)

		Eigen::Matrix< double, 3, 6 > K = Eigen::Matrix< double, 3, 6 >::Zero();

		K(0,0) = 3/(2.*_predictionTime);
		K(1,2) = 3/(2.*_predictionTime);
		K(2,4) = 3/(2.*_predictionTime);
		K(0,1) = 1;
		K(1,3) = 1;
		K(2,5) = 1;

		double xdot_r = u1_r * cos(theta_r);
		double ydot_r = u1_r * sin(theta_r);

		Eigen::Matrix < double, 6, 1> E = (Eigen::Matrix < double, 6, 1>() << x - x_r, -xdot_r, y - y_r, -ydot_r, theta - theta_r, - u2_r).finished();

		Eigen::Matrix < double, 2, 3> DtDm1Dt = (Eigen::Matrix < double, 2, 3>() << cos(theta), sin(theta), 0, 0, 0, 1).finished();

		Eigen::Vector2d cntrlOut;
		cntrlOut = -DtDm1Dt*K*E;
		_u1 = cntrlOut(0,0);
		_u2 = cntrlOut(1,0);
	}

	void ControllerMonocycle::_NCGPCCompleteModel(
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
		std::cout << "Inside NCGPCCM" << std::endl;
		// A new explicit dynamic path tracking controller using Generalized Predictive Control (Mohamed Krid, Faiz Benamar, and Roland Lenain)

		// Just need to be computed once thruout the whole simulation (does using 'static const" accomplish that?):

		//DyModParamVector dMParameters = (DyModParamVector() << 1, 1, 1, 1, 1, 1).finished(); //TODO do system identification
		DyModParamVector dMParameters = (DyModParamVector() << 1.65353375e+01, 3.71645375e+00,-2.82408046e-03, 1.00003767e+00, 9.80505149e-01,-3.47528431e-02).finished();
		
		double xdot_r = u1_r * cos(theta_r);
		double ydot_r = u1_r * sin(theta_r);
		double thetadot_r = u2_r;
		double xdotdot_r = a1_r * cos(theta_r);
		double ydotdot_r = a1_r * sin(theta_r);
		double thetadotdot_r = a2_r;

		double L2fy1 = cos(theta)*(dMParameters[2]*dMParameters[0]*w*w - dMParameters[3]*dMParameters[0]*u) - u*w*sin(theta);
		double L2fy2 = sin(theta)*(dMParameters[2]*dMParameters[0]*w*w - dMParameters[3]*dMParameters[0]*u) + u*w*cos(theta);
		double L2fy3 = -dMParameters[4]*dMParameters[1]*u*w - dMParameters[5]*dMParameters[1]*w;
		double Lfy4 = dMParameters[2]*dMParameters[0]*w*w - dMParameters[3]*dMParameters[0]*u;
		double Lfy5 = -dMParameters[4]*dMParameters[1]*u*w - dMParameters[5]*dMParameters[1]*w;

		Eigen::Matrix < double, relativeDegOfNonLinMIMOSum, 1> E =
			(Eigen::Matrix < double, relativeDegOfNonLinMIMOSum, 1>() <<
			x - x_r,
			u*cos(theta)-xdot_r,
			L2fy1 - xdotdot_r,
			y - y_r,
			u*sin(theta)-ydot_r,
			L2fy2 - ydotdot_r,
			theta - theta_r,
			w - thetadot_r,
			L2fy3 - thetadotdot_r,
			u - u1_r,
			Lfy4 - a1_r,
			w - u2_r,
			Lfy5 - a2_r ).finished();

		Eigen::Matrix < double, outputDim, observDim> DtDm1Dt =
			(Eigen::Matrix < double, outputDim, observDim>() <<
			cos(theta)/dMParameters[0]/2., sin(theta)/dMParameters[0]/2., 				   0, 1./dMParameters[0]/2., 				   0,
										0,							   0, 1./dMParameters[1]/2., 				   0, 1./dMParameters[1]/2.).finished();

		Eigen::Vector2d cntrlOut;
		cntrlOut = -2.*DtDm1Dt.block<outputDim, 3>(0,0)*_K.block<3, 3*3>(0,0)*E.block<3*3, 1>(0,0);

		_u1 = cntrlOut(0,0);
		_u2 = cntrlOut(1,0);
	}

	void ControllerMonocycle::_gainOpt()
	{
		boost::this_thread::sleep(boost::posix_time::milliseconds(200));
		_shouldUpdateGain = true;
		_gainOptMutex.unlock();
	}

	ControllerMonocycle::~ControllerMonocycle()
	{
		//delete pid;
	}
}


// cmake:sourcegroup=Controller