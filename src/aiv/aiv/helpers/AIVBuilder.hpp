#ifndef __AIV_AIVBUILDER_HPP__
#define __AIV_AIVBUILDER_HPP__
#pragma once

#include "aiv/Application.hpp"

#include "aiv/robot/AdeptLynx.hpp"

#include <boost/property_tree/ptree.hpp>

namespace aiv {

// This class knows how to an Autonomous Indoor Vehicle (physics & graphics).
class AIVBuilder
{
public:
  AIVBuilder(Application *, xde::gvm::RigidBodyRef &);

  void addAdeptLynxToApplication(const std::string & name,
                                 //const Eigen::Displacementd & position,
                                 const boost::property_tree::ptree &pt,
                                 const boost::property_tree::ptree::value_type &v
                                 //,const std::string & material = "rubber"
                                 );

protected:
  void createDriveWheel(const std::string & name, 
                        const std::string & daeFilePath, 
                        const std::string & nodeName, 
                        const Eigen::Displacementd & position,
                        double radius,
                        const Eigen::Vector3d & scales,
                        double offset,
                        double mass, 
                        bool enableWeight,
                        const Eigen::Vector3d & axis,
                        const xde::gvm::RigidBodyRef & frame, 
                        xde::gvm::RigidBodyRef & wheel, 
												xde::gvm::SerialJointRef & serial,
												xde::gvm::PrismaticJointRef & prisma,
                        xde::gvm::HingeJointRef & hinge);

  void createFreeBallWheel(const std::string & name, 
                           const std::string & daeFilePath, 
                           const std::string & nodeName, 
                           const Eigen::Displacementd & position,
                           double radius,
                           const Eigen::Vector3d & scales,
                           double offset,
                           double mass, 
                           bool enableWeight,
                           const xde::gvm::RigidBodyRef & frame, 
                           xde::gvm::RigidBodyRef & wheel,
													 xde::gvm::PrismaticJointRef & prisma,
                           xde::gvm::BallJointRef & balljoint);

protected:
  Application * app;
  xde::gvm::RigidBodyRef ground;
};

}

#endif // __AIV_AIVBUILDER_HPP__

// cmake:sourcegroup=Helpers