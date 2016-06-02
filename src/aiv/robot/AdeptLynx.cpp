#include "aiv/robot/AdeptLynx.hpp"

namespace aiv
{

AdeptLynx::AdeptLynx(std::string name, Application * app)
  : AIVMonocycle(name, app)
{
}

double AdeptLynx::getCurrentRightWheelVelocity()
{
  return rightDriveHinge.getJointVelocities()[0];
}

double AdeptLynx::getCurrentLeftWheelVelocity()
{
  return leftDriveHinge.getJointVelocities()[0];
}

void AdeptLynx::setDesiredRightWheelVelocity(double radpersec)
{
  rightDriveHinge.setDesiredJointVelocity( radpersec );
}

void AdeptLynx::setDesiredLeftWheelVelocity(double radpersec)
{
  leftDriveHinge.setDesiredJointVelocity( radpersec );
}

Eigen::Displacementd AdeptLynx::getCurrentPosition()
{
  /*std::cout << "Position:" << std::endl << frame.getPosition() << std::endl;
  std::cout << "Center of gravity:" << std::endl << frame.getCenterOfGravity() << std::endl;
  std::cout << "Retrun:" <<  frame.getComPosition() << std::endl;*/
  return frame.getPosition();
}

Eigen::Twistd AdeptLynx::getCurrentVelocity()
{
  //frame.getPositionVariable();
  return frame.getVelocity();
}

Eigen::Vector2d AdeptLynx::getCurrentAcceleration()
{
  //cmlTinyVector<3> 
  //std::cout << "Hope to see a vector:" << std::endl << frame.getComNonlinearAcceleration() << std::endl;
  //return frame.getComNonlinearAcceleration();
  //std::cout << "JAc\n" << frame.getJacobian();
  double linAccelRight = rightDriveWheel.getComNonlinearAcceleration().y()*getWheelRadius();
  double linAccelLeft = leftDriveWheel.getComNonlinearAcceleration().y()*getWheelRadius();
  double angAccel = (linAccelRight - linAccelLeft)/getTrack();
  double linAccel = (linAccelRight + linAccelLeft)/2.;
  //std::cout << "Wheel non lienar acc\n" << linAccel << "\n" << angAccel << std::endl;
  return (Eigen::Vector2d() << rightDriveWheel.getComNonlinearAcceleration().x(), rightDriveWheel.getComNonlinearAcceleration().y()).finished();
}

double AdeptLynx::getWheelRadius()
{
  double collisionOffset = rightDriveWheel.getComposite().getOffset();
  
  double wheelRadius = (rightDriveWheel.getComposite().getTriMesh(
      name+std::string("_rightDriveWheel.mesh")).getDilation()+
      leftDriveWheel.getComposite().getTriMesh(
      name+std::string("_leftDriveWheel.mesh")).getDilation())/2.;

  return wheelRadius + collisionOffset;
}
 
double AdeptLynx::getTrack()
{
  double collisionOffset = rightDriveWheel.getComposite().getOffset();

  double track = (leftDrivePrisma.get_H_0().getTranslation()-rightDrivePrisma.get_H_0().getTranslation()).norm();

  return 2*collisionOffset + track;
}

double AdeptLynx::getRad()
{
  return 2.5*getTrack(); //TODO get biggest dimension of robot /2
}

}


// cmake:sourcegroup=Robot