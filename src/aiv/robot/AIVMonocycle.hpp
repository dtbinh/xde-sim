#ifndef __AIV_AIVMONO_HPP__
#define __AIV_AIVMONO_HPP__
#pragma once

#include "aiv\robot\AIV.hpp"

namespace aiv 
{
class AIVMonocycle : public AIV
{
public:
  AIVMonocycle(std::string name, Application * app);
	virtual ~AIVMonocycle() {};

public:
  virtual double getCurrentRightWheelVelocity() = 0;
  virtual double getCurrentLeftWheelVelocity() = 0;

  virtual void setDesiredRightWheelVelocity(double radpersec) = 0;
  virtual void setDesiredLeftWheelVelocity(double radpersec) = 0;

    /*!
    Get the (updated) position of AIV (based on the position of its frame) wrt the global
    inertial frame, i.e. \f$H_{body}^0\f$.
  */
  virtual Eigen::Displacementd getCurrentPosition() = 0;

  /*!
    Get the (updated) velocity of AIV (based on the velocity of its frame) wrt the global
    inertial frame, i.e. \f$T_{body}^{body, 0}(q,t)\f$.
  */
  virtual Eigen::Twistd getCurrentVelocity() = 0;

  /*!
    Get the (updated) acceleration of AIV (based on the acceleration of its frame) wrt the global
    inertial frame.
  */
  virtual Eigen::Vector2d getCurrentAcceleration() = 0;
  

  /*!
    Get the wheel radius of AIV.
  */
  virtual double getWheelRadius() = 0;

  /*!
    Get the track of AIV.
  */
  virtual double getTrack() = 0;

  /*!
    Get the radius of the smallest sphere containing the AdeptLynx.
  */
  virtual double getRad() = 0;
};

}

#endif // __AIV_AIVMONO_HPP__

// cmake:sourcegroup=Robot