#ifndef __AIV_ADEPTLYNX_HPP__
#define __AIV_ADEPTLYNX_HPP__
#pragma once

#include "aiv\robot\AIVMonocycle.hpp"

#include "xde\physics\gvm\RigidBody.h"
#include "xde\physics\gvm\HingeJoint.h"
#include "xde\physics\gvm\PrismaticJoint.h"
#include "xde\physics\gvm\SerialJoint.h"
#include "xde\physics\gvm\BallJoint.h"

//#include "xde\physics\cml\GenericVariable.h"
//#include "C:\Users\JM246044\workspace\dev\xde\xde\xde\xde\modules\physics\core\src\internal\cml\GenericVariable.h"

namespace aiv 
{
class AIVBuilder;

class AdeptLynx : public AIVMonocycle
{
  friend class AIVBuilder;
  friend class ApplicationBuilder;

public:
  AdeptLynx(std::string name, Application * app);

public:
  double getCurrentRightWheelVelocity();
  double getCurrentLeftWheelVelocity();

  void setDesiredRightWheelVelocity(double radpersec);
  void setDesiredLeftWheelVelocity(double radpersec);

  /*!
    Get the (updated) position of the AdeptLynx (based on the position of its frame) wrt the global
    inertial frame, i.e. \f$H_{body}^0\f$.
  */
  Eigen::Displacementd getCurrentPosition();

  /*!
    Get the (updated) velocity of AdeptLynx (based on the velocity of its frame) wrt the global
    inertial frame, i.e. \f$T_{body}^{body, 0}(q,t)\f$.
  */
  Eigen::Twistd getCurrentVelocity();

  //const cml::GenericVariable<cml::SE3ColumnDesc>* AdeptLynx::getCurrentAcceleration();

  /*!
    Get the (updated) acceleration of AdeptLynx (based on the acceleration of its frame) wrt the global
    inertial frame, i.e. \f$T_{body}^{body, 0}(q,t)\f$.
  */
  Eigen::Vector2d getCurrentAcceleration();

  /*!
    Get the wheel radius of AdeptLynx.
  */
  double getWheelRadius();

  /*!
    Get the track of AdeptLynx.
  */
  double getTrack();

  /*!
    Get the radius of the smallest sphere containing the AdeptLynx.
  */
  double getRad();

protected:
  xde::gvm::RigidBodyRef  frame;
  xde::gvm::RigidBodyRef  leftDriveWheel;
  xde::gvm::RigidBodyRef  rightDriveWheel;
  xde::gvm::RigidBodyRef  leftFrontFreeWheel;
  xde::gvm::RigidBodyRef  rightFrontFreeWheel;
  xde::gvm::RigidBodyRef  leftBackFreeWheel;
  xde::gvm::RigidBodyRef  rightBackFreeWheel;

	xde::gvm::SerialJointRef leftDriveSerial;
	xde::gvm::SerialJointRef rightDriveSerial;
	xde::gvm::PrismaticJointRef	leftDrivePrisma;
	xde::gvm::PrismaticJointRef	rightDrivePrisma;
  xde::gvm::HingeJointRef leftDriveHinge;
  xde::gvm::HingeJointRef rightDriveHinge;

	xde::gvm::PrismaticJointRef  leftFrontPrisma;
  xde::gvm::PrismaticJointRef  rightFrontPrisma;
  xde::gvm::PrismaticJointRef  leftBackPrisma;
  xde::gvm::PrismaticJointRef  rightBackPrisma;
  xde::gvm::BallJointRef  leftFrontBallJoint;
  xde::gvm::BallJointRef  rightFrontBallJoint;
  xde::gvm::BallJointRef  leftBackBallJoint;
  xde::gvm::BallJointRef  rightBackBallJoint;

};

}

#endif // __AIV_ADEPTLYNX_HPP__

// cmake:sourcegroup=Robot