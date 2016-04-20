#ifndef __AIV_POLYGONOBSTACLE_HPP__
#define __AIV_POLYGONOBSTACLE_HPP__
#pragma once

#include "aiv/Application.hpp"
#include "aiv/obstacle/Obstacle.hpp"

namespace aiv
{

class PolygonObstacle: public Obstacle
{
  friend class ObstBuilder;
public:
  PolygonObstacle(std::string name, Application * app);
  ~PolygonObstacle(){};

  double distToAIV(const Eigen::Vector2d & aiv_position, double aiv_radius);

  double getRad() { return 0.0; };

protected:
  std::vector< Eigen::Vector2d > vertices;
};

}
#endif //__AIV_POLYGONOBSTACLE_HPP__
// cmake:sourcegroup=Obstacle