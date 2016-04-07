
#include "ControllerMonocycle.hpp"

namespace aiv {

	ControllerMonocycle::ControllerMonocycle(std::string name, double updateTimeStep, const double k1, const double k2, const double k3, const double maxU1, const double maxU2)
		: Controller(name),
		_updateTimeStep(updateTimeStep),
		_u1(0.0),
		_u2(0.0),
		_maxU1(maxU1),
		_maxU2(maxU2),
		_k1(k1),
		_k2(k2),
		_k3(k3)
	{
		//pid = new PID(updateTimeStep,1000000000,-1000000000,1,0.3,0.1);
	}

	//void ControllerMonocycle::init()
	//{
	//	// Public Variables
	//	this->v_r     = 0.0;	// Right wheel velocity (m/s)
	//	this->v_l     = 0.0;	// Left wheel velocity (m/s)
	//}

	void ControllerMonocycle::update(Eigen::Displacementd aivCurrPos, Eigen::Twistd aivCurrVel, double plannedLinVel, double plannedAngVel, double plannedXPos, double plannedYPos, double plannedOrientation)
	{
		// See "Poursuite d'un vehicule de reference", item 4.2, equation 22 from Morin Palcal's book about control theory
		
		// double k1, k2, k3;
		
		// if (abs(plannedLinVel) > std::numeric_limits< double >::epsilon())
		// {
		// 	k1 = _k1/abs(plannedLinVel);
		// 	k2 = _k2/plannedLinVel;
		// 	k3 = _k3/abs(plannedLinVel);
		// }
		// //pid->calculate();
	
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


		double predictionTime = 0.8;

		Eigen::Matrix< double, 3, 6 > K = Eigen::Matrix< double, 3, 6 >::Zero();

		K(0,0) = 3/(2.*predictionTime);
		K(1,2) = 3/(2.*predictionTime);
		K(2,4) = 3/(2.*predictionTime);
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
	ControllerMonocycle::~ControllerMonocycle()
	{
		//delete pid;
	}
}


// cmake:sourcegroup=ControllerMonocycle