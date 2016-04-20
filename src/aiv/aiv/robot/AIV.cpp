#include "aiv/robot/AIV.hpp"

namespace aiv 
{

AIV::AIV(std::string n, Application * a)
  : name(n)
  , app(a)
{
}

std::string AIV::getName()
{
  return name;
}

PathPlanner * AIV::getPathPlanner()
{
  return planner;
}

Sensor * AIV::getSensor()
{
	return sensor;
}

Controller * AIV::getController()
{
  return ctrler;
}

}


// cmake:sourcegroup=Robot