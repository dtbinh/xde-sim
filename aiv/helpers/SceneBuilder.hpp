#ifndef __AIV_SCENEBUILDER_HPP__
#define __AIV_SCENEBUILDER_HPP__
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

// This class knows how to construct and fill a Scene (physics & graphics).
class SceneBuilder
{
public:
  SceneBuilder(Application *);

  xde::gvm::RigidBodyRef addFixedObject(const std::string & filePath, 
                                        const std::string & name, 
                                        const std::string & nodeName, 
                                        const Eigen::Displacementd & position,
                                        const Eigen::Vector3d & scales,
                                        double offset);

  xde::gvm::RigidBodyRef addFreeObject(const std::string & filePath, 
                                       const std::string & name, 
                                       const std::string & nodeName, 
                                       const Eigen::Displacementd & position, 
                                       double mass, 
                                       bool enableWeight,
                                       const Eigen::Vector3d & scales,
                                       double offset);

protected:
  Application * app;
};

}

#endif // __AIV_SCENEBUILDER_HPP__

// cmake:sourcegroup=Helpers