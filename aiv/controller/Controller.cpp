#include "Controller.hpp"

namespace aiv 
{

Controller::Controller(std::string n)
  : name(n)
{
}

//void Controller::update(Eigen::Displacementd, Eigen::Twistd, double, double, double, double, double)
//{
//}

std::string Controller::getName()
{
  return name;
}

}


// cmake:sourcegroup=Controller