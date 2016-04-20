#ifndef __AIV_OBSTACLE_HPP__
#define __AIV_OBSTACLE_HPP__
#pragma once

#include "aiv/Application.hpp"

namespace aiv
{
class Obstacle
{
  friend class ApplicationBuilder;
public:
  Obstacle(std::string name, Application * app);
  virtual ~Obstacle(){};

public:
  std::string getName() const;

  Eigen::Displacementd getCurrentPosition();

  Eigen::Twistd getCurrentVelocity();

  /*!
    Get the (updated) velocity wrt the global
    inertial frame, i.e. \f$T_{body}^{body, 0}(q,t)\f$.
  */
  //virtual Eigen::Twistd getCurrentVelocity() = 0;

  /*!
    Give the distance of the projection of the obstacle CM's coordinates in the XY plan to a given point.
  */
  virtual inline double distToAIV(const Eigen::Vector2d & aiv_position, double aiv_radius)
      { return (aiv_position-getCurrentPosition().getTranslation().block<2,1>(0,0)).norm() - aiv_radius; };

  virtual inline double getRad() = 0;

protected:
  Application * app;
  std::string   name;
  xde::gvm::RigidBodyRef object;
};

}

#endif // __AIV_OBSTACLE_HPP__

// cmake:sourcegroup=Obstacle