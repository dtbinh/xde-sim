#ifndef __OBST_OBSTBUILDER_HPP__
#define __OBST_OBSTBUILDER_HPP__
#pragma once

#include <boost/property_tree/ptree.hpp>
#include "aiv/Application.hpp"
//#include "aiv/obstacle/Obstacle.hpp"
#include "aiv/obstacle/CircularObstacle.hpp"
#include "aiv/obstacle/PolygonObstacle.hpp"

namespace aiv
{

class ObstBuilder
{
public:
  ObstBuilder(Application *, xde::gvm::RigidBodyRef &);

  void addCircularObstToApplication(const std::string & name,
                                 const boost::property_tree::ptree::value_type &v);

  void addPolygonObstToApplication(const std::string & name,
                                 boost::property_tree::ptree::value_type &v);

protected:
  Application * app;
  xde::gvm::RigidBodyRef ground;
};


}

#endif // __OBST_OBSTBUILDER_HPP__
// cmake:sourcegroup=Helpers