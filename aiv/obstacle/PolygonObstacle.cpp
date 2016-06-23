#include "aiv/obstacle/PolygonObstacle.hpp"

namespace aiv 
{

PolygonObstacle::PolygonObstacle(std::string name, Application * app)
  : Obstacle(name, app)
{
}

double PolygonObstacle::distToAIV (const Eigen::Vector2d & aiv_position, double aiv_radius)
{

  return 4;
}

}
// cmake:sourcegroup=Obstacle