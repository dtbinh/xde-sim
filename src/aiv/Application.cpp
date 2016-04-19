#include "Application.hpp"

#include "aiv/robot/AIV.hpp"
#include "aiv/obstacle/Obstacle.hpp"

#include "xde/physics/gvm/SimulationEnum.h"

//static const double TIME_STEP = 0.01; // Really bad way of defining the timestep
static const std::string OGRE_RSC_PATH_ENV_VAR = "OGRE_RSC_PATH";

namespace aiv {

	Application::Application(double tStep)
		: gvmScene()
		, lmdScene()
		, timeStep(tStep)
		, xdeViewer(0x0)
		, si(0x0)
		, ci(0x0)
	{
		// build physic scene
		gvmScene = xde::gvm::SceneRef::createObject("gvmScene");
		gvmScene.setIntegratorFlags(xde::gvm::DYNAMIC_INTEGRATOR | xde::gvm::GAUSS_SEIDEL_SOLVER | xde::gvm::USE_NON_LINEAR_TERMS);
		gvmScene.setUcRelaxationFactor(.1);
		gvmScene.setFdvrFactor(.2);
		gvmScene.setTimeStep(timeStep);
		gvmScene.setVerticalDirectionUp(Eigen::Vector3d(0., 0., 1.));

		lmdScene = xde::lmd::SceneRef::createObject("lmdScene", .02, .05);

		gvmScene.setGeometricalScene(lmdScene);


		// build graphic scene
		// OGRE resources path is now an environement variable (can be set by batch run file before running exe)
		std::string ogre_rsc_path = "../../../../share/resources/ogre";
		char const* temp = getenv(OGRE_RSC_PATH_ENV_VAR.c_str());
		if(temp != NULL)
		{
			ogre_rsc_path = std::string(temp);            
		}

		xdeViewer = new xde::builder::SimpleViewer(ogre_rsc_path);
		si = &xdeViewer->getSceneInterface();
		ci = &xdeViewer->getCameraInterface();
	}


	Application::~Application()
	{
		if(xdeViewer)
		{
			delete xdeViewer;
			xdeViewer = 0x0;
		}

		for(std::map<std::string, AIV*>::iterator it = vehicles.begin() ; it != vehicles.end() ; ++it)
		{
			delete it->second;
		}
	}

	double Application::getTimeStep()
	{
		return timeStep;
	}

	double Application::getSimSpeed()
	{
		return simSpeed;
	}

	//void Application::setTimeStep(double ts)
	//{
	//  timeStep = ts;
	//}

	void Application::setSimSpeed(double ss)
	{
		simSpeed = ss;
	}

	xde::gvm::SceneRef & Application::getGVMScene()
	{
		return gvmScene;
	}

	xde::xcd::SceneRef & Application::getXCDScene()
	{
		return lmdScene;
	}

	xde::builder::SimpleViewer & Application::getViewer()
	{
		return *xdeViewer;
	}

	OGREViewer::SceneInterface & Application::getGraphicSceneInterface()
	{
		return *si;
	}


	OGREViewer::CameraInterface & Application::getCameraInterface()
	{
		return *ci;
	}

	void Application::update()
	{
		// here update robot path planners
		// get current time, add time step and calculate the next input vector.

		// update robots' sensors, planners, controllers
		for(std::map<std::string, AIV*>::iterator it = vehicles.begin() ; it != vehicles.end() ; ++it)
		{

			it->second->getSensor()->update(
				it->second->getCurrentPosition(),
				obstacles,
				vehicles);

			it->second->getPathPlanner()->update(
				it->second->getSensor()->getObstacles(),
				it->second->getSensor()->getVehicles(),
				it->second->getCurrentPosition());
			//std::cout << "after planner update [" << it->first << "]" << std::endl;
			//it->second->getCurrentPosition(), it->second->getCurrentVelocity()); // pplanner update call with state feedback ! THIS MAKES NO SENSE
			it->second->getController()->update(
				it->second->getCurrentPosition(),
				it->second->getCurrentVelocity(),
				it->second->getPathPlanner()->getXPosition(),
				it->second->getPathPlanner()->getYPosition(),
				it->second->getPathPlanner()->getOrientation(),
				it->second->getPathPlanner()->getLinVelocity(),
				it->second->getPathPlanner()->getAngVelocity(),
				it->second->getPathPlanner()->getLinAccel(),
				it->second->getPathPlanner()->getAngAccel());

			//std::cout << "after controller update [" << it->first << "]" << std::endl;

		}
		//std::cout << "after controllers sensors planners updates" << std::endl;

		// update simu
		gvmScene.detectCollisions();
		gvmScene.integrate();

		// update object positions in viewer from simu
		std::vector<std::string> bodyNames = gvmScene.getBodyNames();
		for(std::vector<std::string>::const_iterator it = bodyNames.begin(); it != bodyNames.end() ; ++it)
		{
			if(si->nodeExists(*it))
				si->setNodePosition( *it, gvmScene.getRigidBody(*it).getPosition() );
		}

		// update graphics
		xdeViewer->update();
		//std::cout << "end of Application update" << std::endl;
	}

	void Application::printPerformanceReport(std::ostream & os)
	{
		os << "===============================================================================" << std::endl;
		gvmScene.printPerformanceReport(os);
	}

	bool Application::addVehicle(AIV * v)
	{
		if(getVehicleByName(v->getName()) != 0x0)
		{
			return false;
		}
		vehicles[v->getName()] = v;
		return true;
	}

	AIV * Application::getVehicleByName(const std::string & n)
	{
		std::map<std::string, AIV*>::iterator it = vehicles.find(n);
		if(it == vehicles.end())
		{
			return 0x0;
		}
		return it->second;
	}

	std::map<std::string, AIV *> Application::getVehicles() const
	{
		return vehicles;
	}

	bool Application::addObstacle(Obstacle * obst)
	{
		if(getObstacleByName(obst->getName()) != 0x0)
		{
			return false;
		}
		obstacles[obst->getName()] = obst;
		return true;
	}

	Obstacle * Application::getObstacleByName(const std::string & n)
	{
		std::map<std::string, Obstacle*>::iterator it = obstacles.find(n);
		if(it == obstacles.end())
		{
			return 0x0;
		}
		return it->second;
	}

	std::map<std::string, Obstacle *> Application::getObstacles() const
	{
		return obstacles;
	}


}

// cmake:sourcegroup=Application