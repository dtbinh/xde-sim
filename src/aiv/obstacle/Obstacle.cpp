#include "aiv/obstacle/Obstacle.hpp"

namespace aiv 
{

Obstacle::Obstacle(std::string name, Application * app)
  : name(name)
  , app(app)
{
}

std::string Obstacle::getName() const
{
  return name;
}

Eigen::Displacementd Obstacle::getCurrentPosition()
{
  return object.getPosition();
}

Eigen::Twistd Obstacle::getCurrentVelocity()
{
  return object.getVelocity();
}

}
// cmake:sourcegroup=Obstacle