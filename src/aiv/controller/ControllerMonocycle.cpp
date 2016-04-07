
#include "aiv/controller/ControllerMonocycle.hpp"
#include <nlopt.h>
#include <omp.h>

namespace aiv {

	ControllerMonocycle::ControllerMonocycle(std::string name, double updateTimeStep, const double k1, const double k2, const double k3, const double maxU1, const double maxU2)
		: Controller(name),
		_updateTimeStep(updateTimeStep),
		_u1(0.0),
		_u2(0.0),
		_maxU1(maxU1),
		_maxU2(maxU2),
		_gain(Gain(_gainDim)),
		_auxGain(Gain(_gainDim)),
		_updateCallCntr(0),
		_planStage(INIT)
	{
		//pid = new PID(updateTimeStep,1000000000,-1000000000,1,0.3,0.1);
		double gainValues[_gainDim] = {k1, k2, k3};
		_gain.setGainValues(gainValues);
		_auxGain = _gain;
	}

	void  PathPlannerRecHor::setOption(std::string optionName, double optionValue)
	{
		else if (optionName == "offsetTime")
		{
			_firstPlanTimespan = optionValue;
		}
	}

	//void ControllerMonocycle::init()
	//{
	//	// Public Variables
	//	this->v_r     = 0.0;	// Right wheel velocity (m/s)
	//	this->v_l     = 0.0;	// Left wheel velocity (m/s)
	//}

	void ControllerMonocycle::update(Eigen::Displacementd aivCurrPos, Eigen::Twistd aivCurrVel, double plannedLinVel, double plannedAngVel, double plannedXPos, double plannedYPos, double plannedOrientation)
	{

		double currentTime = _updateTimeStep*_updateCallCntr;
		++_updateCallCntr;

		double evalTime = std::max(((currentTime - (std::max((this->executingPlanIdx), 0)*this->compHorizon)) / this->planHorizon), 0.0);

		// STATE MACHINE
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
				this->_gainOptMutex.lock();
				_ctrlThread = boost::thread(&ControllerMonocycle::gainOpt, this); // Opt gain using P0

				_planStage = INTER;

				// reset time

			}
			// If we've already created the thread for optimizing gain using plan 0
			// let's check if it has alread found a solution.
			// If so, let's update the gain
			// otherwise, if evaltime became greater or equal to compHorizon / planHorizon
			else
			{
				if (evaltime >= ratio)
				{
					evalTime -= this->compHorizon / this->planHorizon;

					if (!_gainOptMutex.try_lock()) // if Tcc < Tc we are supposed to get this lock
					{
						_ctrlThread.interrupt(); // if not we interrupt opt so a new opt can be done
						_gainOptMutex.lock();
					}
					// _gainOptMutex is locked
					_ctrlThread = boost::thread(&ControllerMonocycle::gainOpt, this);
				}

				if (!_gainOptMutex.try_lock())
				{
					// if can't get lock we weather:
					// do nothing or
					// see if Tcc is getting "too" close to Tc and do something else regarding the gain
				}
				else //successfully lock => thread had finished
				{
					// update gain spline with auxGain
					_gain = _auxGain;

				}
			}
		}
		else if (_planStage == FINAL && evalTime > 1.0)
		{
			////std::cout << "gone to DONE" << std::endl;
			_planStage = DONE;
		}

		
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
			_u1 = u1_r;
			_u2 = u2_r;
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


			//FROM HANDBOOK OF ROBOTICS SPRING

			//------------------ Generating input w1 w2 --------------------

			Eigen::Vector3d err = (Eigen::Vector3d() << x-x_r, y-y_r, theta-theta_r).finished(); // errors

			Eigen::Matrix2d rotRef = (Eigen::Matrix2d() << cos(theta_r), sin(theta_r), -1*sin(theta_r), cos(theta_r)).finished();

			Eigen::Vector2d err_r = rotRef*err.block<2,1>(0,0);

			double xi1 = -1*std::abs(u1_r)*(err_r(0)+err_r(1)*tan(err(2)));
			double xi2 = -1*u1_r*err_r(1);
			double xi3 = -1*std::abs(u1_r)*tan(err(2));

			// Only proportional gain
			_gain(evalTime*_cntrlHorizon);
			double w1 = _gain.k(0)*xi1;
			double w2 = _gain.k(1)*xi2 + gain.k(2)(evalTime*_cntrlHorizon)*xi3;

			//----------------- Back to u1, u2 from w1, w2 ---------------------

			_u1 = (w1 + u1_r)/cos(err(2));
			_u2 = w2*std::pow(cos(err(2)), 2)+u2_r;

			// limiting output
			_u1 = std::max(std::min(_u1, _maxU1), -1*_maxU1);
			_u2 = std::max(std::min(_u2, _maxU2), -1*_maxU2);

		}
	}
	

		//double evalTime = std::max(((currentPlanningTime - (std::max((this->executingPlanIdx), 0)*this->compHorizon)) / this->planHorizon), 0.0);

		// See "Poursuite d'un vehicule de reference", item 4.2, equation 22 from Morin Palcal's book about control theory
		
		// double k1, k2, k3;
		
		// if (abs(plannedLinVel) > std::numeric_limits< double >::epsilon())
		// {
		// 	k1 = _k1/abs(plannedLinVel);
		// 	k2 = _k2/plannedLinVel;
		// 	k3 = _k3/abs(plannedLinVel);
		// }
		// //pid->calculate();


		// //------------------ Generating input w1 w2 --------------------

		// Eigen::Vector3d err = (Eigen::Vector3d() << x-x_r, y-y_r, theta-theta_r).finished(); // errors

		// Eigen::Matrix2d rotRef = (Eigen::Matrix2d() << cos(theta_r), sin(theta_r), -1*sin(theta_r), cos(theta_r)).finished();

		// Eigen::Vector2d err_r = rotRef*err.block<2,1>(0,0);

		// double xi1 = -1*std::abs(u1_r)*(err_r(0)+err_r(1)*tan(err(2)));
		// double xi2 = -1*u1_r*err_r(1);
		// double xi3 = -1*std::abs(u1_r)*tan(err(2));

		// // Only proportional gain
		// double w1 = k1*xi1;
		// double w2 = k2*xi2 + k3*xi3;

		// //----------------- Back to u1, u2 from w1, w2 ---------------------

		// _u1 = (w1 + u1_r)/cos(err(2));
		// _u2 = w2*std::pow(cos(err(2)), 2)+u2_r;

		// _u1 = std::max(std::min(_u1, _maxU1), -1*_maxU1);
		// _u2 = std::max(std::min(_u2, _maxU2), -1*_maxU2);

		// //----------------- Estimation: velocities -------------------------
		// _estq = Eigen::Vector3d::Zero();
		// _estqdot = Eigen::Vector3d::Zero();
		// _estqdotdot = Eigen::Vector3d::Zero();

		// Initialization
		//_estq = (Eigen::Vector3d() << plannedXPos, plannedYPos, plannedOrientation)
		//_estqdot = (Eigen::Vector3d() << plannedLinVel*cos(plannedOrientation), plannedLinVel*sin(plannedOrientation), plannedAngVel)
		// could get accelereation from planner

		// PREDICTIVE CONTROL. ONLY KINEMATIC MODEL


		// double predictionTime = 0.8;

		// Eigen::Matrix< double, 3, 6 > K = Eigen::Matrix< double, 3, 6 >::Zero();

		// K(0,0) = 3/(2.*predictionTime);
		// K(1,2) = 3/(2.*predictionTime);
		// K(2,4) = 3/(2.*predictionTime);
		// K(0,1) = 1;
		// K(1,3) = 1;
		// K(2,5) = 1;

		// double xdot_r = u1_r * cos(theta_r);
		// double ydot_r = u1_r * sin(theta_r);

		// Eigen::Matrix < double, 6, 1> E = (Eigen::Matrix < double, 6, 1>() << x - x_r, -xdot_r, y - y_r, -ydot_r, theta - theta_r, - u2_r).finished();

		// Eigen::Matrix < double, 2, 3> DtDm1Dt = (Eigen::Matrix < double, 2, 3>() << cos(theta), sin(theta), 0, 0, 0, 1).finished();

		// Eigen::Vector2d cntrlOut;
		// cntrlOut = -DtDm1Dt*K*E;
		// _u1 = cntrlOut(0,0);
		// _u2 = cntrlOut(1,0);


	void ControllerMonocycle::gainOpt()
	{
		boost::this_thread::sleep(boost::posix_time::milliseconds(100))
	}


	
	ControllerMonocycle::~ControllerMonocycle()
	{
		//delete pid;
	}
}


// cmake:sourcegroup=Controller