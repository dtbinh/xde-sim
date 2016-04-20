#ifndef __AIV_APPBUILDER_HPP__
#define __AIV_APPBUILDER_HPP__
#pragma once

#include "aiv/Application.hpp"

namespace xde
{
  namespace gvm
  {
    class RigidBodyRef;
  }
}

namespace aiv {

// This class is used to avoid a mess in main.cpp
// This class knows how to construct and fill an Application.
// Various types of application can be constructed in future, with or without PathPlanner, Controller, Sensors...
class ApplicationBuilder
{
public:
  static Application * buildEmptyApp(double timeStep);

  static Application * buildSimpleCollisionApp();
  
  static Application * buildSimpleAdeptApp();

};

}

#endif // __AIV_APPBUILDER_HPP__

// cmake:sourcegroup=Helpers