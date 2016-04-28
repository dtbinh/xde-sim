/**
*
*
* \date 08/2015
* \author CEA/DRT/LIST/DIASI/LRI
* \author E. Lucet
* \author J. M. Mendes Filho
*
* @par Licence
* Copyright © 2015 CEA
*/
//==============================================================================
// MOBILE ROBOT PATH FOLLOWING APPLICATION
//==============================================================================

#include "sys/timer.h"
#include <iostream>
#include <math.h>
#include <cstdio>
//#include <boost/property_tree/ptree.hpp>
//#include <boost/property_tree/xml_parser.hpp>
//#include <boost/foreach.hpp>

#include "aiv/helpers/AppBuilder.hpp"
#include "aiv/robot/AIV.hpp"
#include "aiv/robot/AIVMonocycle.hpp"
#include "aiv/obstacle/Obstacle.hpp"
#include "aiv/controller/ControllerMonocycle.hpp"
#include "aiv/pathplanner/PathPlannerRecHor.hpp"
//#include "aiv/pathplanner/PathPlannerRecedingHoriz.hpp"

//#define foreach_ BOOST_FOREACH

using namespace Eigen;

/*
This main would be easily replaced in futur by a python script
*/

int main()
{
	xde::sys::Timer wallTimer;

	//aiv::ApplicationBuilder builder;
	//aiv::Application * app = builder.buildSimpleCollisionApp();
	aiv::Application * app;
	try
	{
		app = aiv::ApplicationBuilder::buildSimpleAdeptApp();
	}
	catch(std::exception& e)
	{
		std::cout << "main: Attempt to build simple App failed. " << e.what() << std::endl;
		return 0;
	}

	std::map<std::string, aiv::AIV*> vehicles = app->getVehicles();
	aiv::AIVMonocycle * vehicle;

	std::map<std::string, aiv::Obstacle*> obstacles = app->getObstacles();
	aiv::Obstacle * obstacle;

	//aiv::AIVMonocycle ** vehicles = new aiv::AIVMonocycle *[2];
	//vehicles[0] = dynamic_cast<aiv::AIVMonocycle*>(app->getVehicleByName("AdeptLynx0"));

	// Get vehicle path planner so we can initialize/configure it properly
	//aiv::PathPlannerRecHor2 ** vpathplanner = new aiv::PathPlannerRecHor2 *[2];
	//vpathplanner[0] = dynamic_cast<aiv:: PathPlannerRecHor2*>(vehicles[0]->getPathPlanner());

	// Get vehicle controller
	//aiv::ControllerMonocycle ** vcontroller = new aiv::ControllerMonocycle *[2];
	//vcontroller[0] = dynamic_cast<aiv::ControllerMonocycle*>(vehicles[0]->getController());

	//aiv::AIVMonocycle * vehicle2 = dynamic_cast<aiv::AIVMonocycle*>(app->getVehicleByName("AdeptLynx_2"));

	// TIME VARIABLES
	int index;
	double tic, toc, simTic;
	double appTimeStep = app->getTimeStep() * 1000.0;
	double appSimSpeed = app->getSimSpeed();

	// AUXILIAR VARIABLES
	double plLinVel, plAngVel, plX, plY, plTheta, plLinAccel, plAngAccel;
	double ctrlLinVel, ctrlAngVel;

	Eigen::Displacementd realPose;
	Eigen::Quaternion<double> q;
	Eigen::QuaternionBase<Eigen::Quaternion<double> >::Vector3 coordXB;

	Eigen::Twistd realVelocity;
	Eigen::Vector2d realAccel;
	double realLinVel, realAngVel;
	//double realLinAccel, realAngAccel;

	double wheelRadius, track;

	double remainingTime;

	// ARRAYS OF LOGGING FILES POINTERS
	std::ofstream * pl_ts = new std::ofstream[vehicles.size()];
	std::ofstream * ctrl_ts = new std::ofstream[vehicles.size()];
	std::ofstream * real_ts = new std::ofstream[vehicles.size()];
	std::ofstream * obst_ts = new std::ofstream[obstacles.size()];

	// REMOVE PREEXISTING LOG FILES
	/*std::string str;
	
		str = "pl_ts_" + it->second->getName() + ".csv";
		std::remove(str.c_str());
		str = "real_ts_" + it->second->getName() + ".csv";
		std::remove(str.c_str());
		str = "ctrl_ts_" + it->second->getName() + ".csv";
		std::remove(str.c_str());
	}
	for (std::map<std::string, aiv::Obstacle*>::iterator it = obstacles.begin(); it != obstacles.end(); ++it)
	{
		str = "obst_ts_" + it->second->getName() + ".csv";
		std::remove(str.c_str());
	}*/

	// OPEN FOR WRITING (OVERWRITES PREVIOUS CONTENT) TODO TEST
	for (std::map<std::string, aiv::AIV*>::iterator it = vehicles.begin(); it != vehicles.end(); ++it)
	{
		index = std::distance(vehicles.begin(), it);
		pl_ts[index].open("pl_ts_" + it->second->getName() + ".csv", std::iostream::out);
		real_ts[index].open("real_ts_" + it->second->getName() + ".csv", std::iostream::out);
		ctrl_ts[index].open("ctrl_ts_" + it->second->getName() + ".csv", std::iostream::out);
	}
	for (std::map<std::string, aiv::Obstacle*>::iterator it = obstacles.begin(); it != obstacles.end(); ++it)
	{
		index = std::distance(obstacles.begin(), it);
		obst_ts[index].open("obst_ts_" + it->second->getName() + ".csv", std::iostream::out);
	}
	

	wallTimer.ResetTime();
	for(int i=1; true; ++i)
	{
		tic = wallTimer.GetTime();

		simTic = appTimeStep / 1000. * i;

		try
		{
			app->update();
		}
		catch(std::exception& e)
		{
			std::cout << "main: Updating failed." << e.what();
			delete[] pl_ts;
			delete[] real_ts;
			delete[] ctrl_ts;
			delete[] obst_ts;
			return 0;
		}

		// APPLY CONTROLLER OUTPUT FOR EACH VEHICLE AND SAVE LOG
		for (std::map<std::string, aiv::AIV*>::iterator it = vehicles.begin(); it != vehicles.end(); ++it)
		{
			vehicle = dynamic_cast<aiv::AIVMonocycle*>(it->second);
			index = std::distance(vehicles.begin(), it);

			wheelRadius = vehicle->getWheelRadius();
			track = vehicle->getTrack();
			
			// GET PL OUTPUT (just for saving it in the log files)
			plLinAccel = vehicle->getPathPlanner()->getLinAccel();
			plAngAccel = vehicle->getPathPlanner()->getAngAccel();
			plLinVel = vehicle->getPathPlanner()->getLinVelocity();
			plAngVel = vehicle->getPathPlanner()->getAngVelocity();
			plX = vehicle->getPathPlanner()->getXPosition();
			plY = vehicle->getPathPlanner()->getYPosition();
			plTheta = vehicle->getPathPlanner()->getOrientation();

			// GET CONTROLLER OUTPUT
			ctrlLinVel = dynamic_cast<aiv::ControllerMonocycle*>(vehicle->getController())->getLinVelocity();
			ctrlAngVel = dynamic_cast<aiv::ControllerMonocycle*>(vehicle->getController())->getAngVelocity();

			//ctrlLinVel = 0.0;
			//ctrlAngVel = 0.0;

			//std::cout << "3\n";

			// Apply controller output
			vehicle->setDesiredRightWheelVelocity((ctrlLinVel + ctrlAngVel*track / 2) / wheelRadius);	//rad/s
			vehicle->setDesiredLeftWheelVelocity((ctrlLinVel - ctrlAngVel*track / 2) / wheelRadius);	//rad/s

			// vehicle->setDesiredRightWheelVelocity(0.0);	//rad/s
			// vehicle->setDesiredLeftWheelVelocity(0.0);	//rad/s

			//std::cout << "4\n";
			//std::cout << "5\n";

			// REAL POSE is [x, y, z, q], where q = [q1, q2, q3, q4] a quarternion representation of the orientation
			// Get it
			realPose = vehicle->getCurrentPosition();

			// Getting robot's orientation projected in the ground plane (unless the aiv is flying this is the correct orientation)
			// 1. get the quaternion part of the displacement
			q = Eigen::Quaternion<double>(realPose.qw(), realPose.qx(), realPose.qy(), realPose.qz());
			// 2. rotation of UnityX by the quaternion q (i.e. x of the robot frame wrt the absolute frame)
			coordXB = q._transformVector(Vector3d::UnitX());

			// REAL VELOCITY is [wx wy wz vx vy vz] wrt the robot frame
			realVelocity = vehicle->getCurrentVelocity();

			realLinVel = realVelocity.block<2, 1>(3, 0).norm(); // sqrt(vx^2 + vy^2) linear speed in the robot plane xy
			realAngVel = realVelocity.topRows(3).z(); // rotation around z of the robot frame

			// REAL ACCELERATION wrt the robot frame
			realAccel = vehicle->getCurrentAcceleration();
			//std::cout << "Acc:\n" << vehicle->getCurrentAcceleration() << std::endl; //

			//realLinAccel = realAccel.topRows(3).norm();
			// realAngAccel = realAccel.topRows(3).z(); // rotation around z of the robot frame
			//realAngAccel = 0.0;

			pl_ts[index] << simTic << ",";
			pl_ts[index] << plX << ",";
			pl_ts[index] << plY << ",";
			pl_ts[index] << plTheta << ",";
			pl_ts[index] << plLinVel << ",";
			pl_ts[index] << plAngVel << ",";
			pl_ts[index] << plLinAccel << ",";
			pl_ts[index] << plAngAccel << ",";
			pl_ts[index] << "\n";

			ctrl_ts[index] << simTic << ",";
			ctrl_ts[index] << ctrlLinVel << ",";
			ctrl_ts[index] << ctrlAngVel << ",";
			ctrl_ts[index] << "\n";

			real_ts[index] << simTic << ",";
			real_ts[index] << vehicle->getRad() << ","; // TODO Code method to get radius of circle envolving the frame of the robot in the z plane
			real_ts[index] << realPose.x() << ",";
			real_ts[index] << realPose.y() << ",";
			real_ts[index] << atan2(coordXB.y(), coordXB.x()) << ",";
			real_ts[index] << realLinVel << ",";
			real_ts[index] << realAngVel << ",";
/*			real_ts[index] << realAccel[0] << ",";
			real_ts[index] << realAccel[1] << ",";
			real_ts[index] << realAccel[2] << ",";
			real_ts[index] << realAccel[3] << ",";
			real_ts[index] << realAccel[4] << ",";
			real_ts[index] << realAccel[5] << ",";*/
			real_ts[index] << realAccel.x() << ",";
			real_ts[index] << realAccel.y() << ",";
			/*real_ts[index] << realAccel[0] << ",";
			real_ts[index] << realAccel[1] << ",";
			real_ts[index] << realAccel[2] << ",";*/
			real_ts[index] << "\n";
			
		}

		for (std::map<std::string, aiv::Obstacle*>::iterator it = obstacles.begin(); it != obstacles.end(); ++it)
		{
			//obstacle = dynamic_cast<aiv::Obstacle*>(it->second);
			obstacle = it->second;
			index = std::distance(obstacles.begin(), it);

			//obstacle.setDesiredVelocity(randomVx, randomVy)

			realPose =  obstacle->getCurrentPosition();

			// 1. get the quaternion part of the displacement
			q = Eigen::Quaternion<double>(realPose.qw(), realPose.qx(), realPose.qy(), realPose.qz());
			// 2. rotation of UnityX by the quaternion q (i.e. x of the robot frame wrt the absolute frame)
			coordXB = q._transformVector(Vector3d::UnitX());

			// REAL VELOCITY is [wx wy wz vx vy vz] wrt the robot frame
			realVelocity = obstacle->getCurrentVelocity();

			realLinVel = realVelocity.block<2, 1>(3, 0).norm(); // sqrt(vx^2 + vy^2) linear speed in the robot plane xy
			realAngVel = realVelocity.topRows(3).z(); // rotation around z of the robot frame


			obst_ts[index] << simTic << ",";
			obst_ts[index] << obstacle->getRad() << ",";
			obst_ts[index] << realPose.x() << ",";
			obst_ts[index] << realPose.y() << ",";
			obst_ts[index] << atan2(coordXB.y(), coordXB.x()) << ",";
			obst_ts[index] << realLinVel << ",";
			obst_ts[index] << realAngVel << ",";
			obst_ts[index] << "\n";

		}

		/*if(i++ > 200)
		{
		i = 0;
		app->printPerformanceReport(std::cout);
		}*/			

		toc = wallTimer.GetTime();

		//remainingTime = std::max( 0.0, appTimeStep/appSimSpeed - ( toc-tic ) ); // So we have a simulation time equals to the real time...
		remainingTime = appTimeStep/appSimSpeed - ( toc-tic );
		if ( remainingTime > 0.0 )
			xde::sys::Timer::Sleep( remainingTime ); // ... (it does not have impact on the physics of the simulation)

	}

	// CLOSING LOG FILES
	for (std::map<std::string, aiv::AIV*>::iterator it = vehicles.begin(); it != vehicles.end(); ++it)
	{
		index = std::distance(vehicles.begin(), it);
		pl_ts[index].close();
		real_ts[index].close();
		ctrl_ts[index].close();
	}
	for (std::map<std::string, aiv::Obstacle*>::iterator it = obstacles.begin(); it != obstacles.end(); ++it)
	{
		index = std::distance(obstacles.begin(), it);
		obst_ts[index].close();
	}

	// FREEING MEMORY
	delete[] pl_ts;
	delete[] real_ts;
	delete[] ctrl_ts;
	delete[] obst_ts;

	return 0;
}

//Affichage des trajectoires de consigne et réelle du véhicule : Déjà fait il y a un an mais sous python :
//
//# Visual lines of the vehicle trajectory
//line_blue = lgsm.vector(0.0, 0.0, 1.0, 1.0) ## R G B transparence (alpha)
//graph.s.Connectors.IConnectorLine.new("iclf", "i_line", "mainScene", line_blue, True)
//i_line = graph.getPort("i_line")
//o_line = controller.getPort("line")
//i_line.connectTo(o_line)
//
//Et ensuite dans une boucle Orocos OCL avec :
//
//this->addPort("line", out_line); 
//
//this->line = std::vector<double>(6);
//memset(&this->line[0], 0, 6*sizeof(double));
//…
//this->line[i] =…
//…
//out_line.write(this->line);