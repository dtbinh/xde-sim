#ifndef __AIV_AIV_HPP__
#define __AIV_AIV_HPP__
#pragma once

#include "aiv/Application.hpp"

#include "aiv/controller/Controller.hpp"
#include "aiv/pathplanner/PathPlanner.hpp"
#include "aiv/sensor/Sensor.hpp"

namespace aiv
{
class AIV
{
  friend class ApplicationBuilder;
public:
  AIV(std::string name, Application * app);
	virtual ~AIV() {};

public:
  std::string getName();
  PathPlanner * getPathPlanner();
  Sensor * getSensor();
  Controller * getController();
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


protected:
  Application * app;
  std::string   name;
  PathPlanner * planner;
  Controller *  ctrler;
  Sensor * sensor;

};

}

#endif // __AIV_AIV_HPP__

// cmake:sourcegroup=Robot