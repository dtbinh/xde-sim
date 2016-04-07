#include "aiv\helpers\SceneBuilder.hpp"

#include "sys/timer.h"

#include "xde/data/Assets.h"
#include "xde/physics/builder/PhysicsBuilder.h"
#include "xde/graphics/builder/GraphicsBuilder.h"

#include "xde/physics/gvm/Scene.h"
#include "xde/physics/lmdpp/Scene.h"
#include "xde/physics/gvm/FixedJoint.h"
#include "xde/physics/gvm/FreeJoint.h"

#include <stdexcept> 

namespace aiv {

SceneBuilder::SceneBuilder(Application * a)
  : app(a)
{
}


xde::gvm::RigidBodyRef SceneBuilder::addFreeObject(const std::string & filePath, 
                                                   const std::string & name, 
                                                   const std::string & nodeName, 
                                                   const Eigen::Displacementd & position,
                                                   double mass,
                                                   bool  enableWeight,
                                                   const Eigen::Vector3d & scales,
                                                   double offset)
{
  std::cout << "======  Create Free Object [ " << name << " ] ================" << std::endl;

  try
  {
  xde::sys::Timer timer;

  // Parse Geometry
  timer.ResetTime();
  xde::data::Assets assets;
  if(filePath.substr(filePath.find_last_of(".") + 1) == "dae")
  {
    assets.importOpenColladaFile(filePath, scales);
  }
  else if(filePath.substr(filePath.find_last_of(".") + 1) == "3dxml")
  {
    assets.import3dxmlFile(filePath, scales);
  }
  else
  {
    throw std::string("unknow file extension : " + filePath);
  }
  std::cout << "Parsing data: " << timer.GetTime() * .001 << " sec." << std::endl;

  // Graphics
  timer.ResetTime();
  xde::builder::GraphicsBuilder graphBuilder(app->getViewer().getSceneComponents(), &assets);
  graphBuilder.loadAssets();
	const std::string& objId = assets.getIds().getLastId(filePath, nodeName);
  graphBuilder.createNodeFromTree(name, "", objId, objId, false);
  std::cout << "Building graphics: " << timer.GetTime() * .001 << " sec." << std::endl;

  // Collision
  timer.ResetTime();
  xde::builder::PhysicsBuilder phyBuilder(app->getXCDScene(), &assets);
  xde::xcd::CompositeRef composite = phyBuilder.createCompositeFromTree(name + std::string(".comp"), objId, objId, offset, 1., false, false);
  std::cout << "Building collision: " << timer.GetTime() * .001 << " sec." << std::endl;

  // GVM RigidBody
  timer.ResetTime();
  xde::gvm::RigidBodyRef rigidBody = xde::gvm::RigidBodyRef::createObject(name);
  xde::gvm::FreeJointRef joint = xde::gvm::FreeJointRef::createObject(name + std::string(".joint"));
  app->getGVMScene().addRigidBodyToGround(rigidBody, joint);
  rigidBody.setComposite(composite);
  rigidBody.setMass(mass);
  rigidBody.computePrincipalFrameAndMomentsOfInertiaUsingCompositeObbAndMass();
  joint.configure(position);
  if(enableWeight)
    rigidBody.enableWeight();
  else
    rigidBody.disableWeight();

  std::cout << "Building physics: " << timer.GetTime() * .001 << " sec." << std::endl;
  
  return rigidBody;

  }
  catch(std::exception & e)
  {
    std::cerr << e.what() << std::endl;
    throw e;
  }
}

xde::gvm::RigidBodyRef SceneBuilder::addFixedObject(const std::string & filePath, 
                                                    const std::string & name, 
                                                    const std::string & nodeName, 
                                                    const Eigen::Displacementd & position,
                                                    const Eigen::Vector3d & scales,
                                                    double offset)
{
  try
  {
  std::cout << "======  Create Fixed Object [ " << name << " ] ================" << std::endl;

  xde::sys::Timer timer;

  // Parse Geometry
  timer.ResetTime();
  xde::data::Assets assets;
  if(filePath.substr(filePath.find_last_of(".") + 1) == "dae")
  {
    assets.importOpenColladaFile(filePath, scales);
  }
  else if(filePath.substr(filePath.find_last_of(".") + 1) == "3dxml")
  {
    assets.import3dxmlFile(filePath, scales);
  }
  else
  {
    throw std::string("unknow file extension : " + filePath);
  }
  std::cout << "Parsing data: " << timer.GetTime() * .001 << " sec." << std::endl;

  // Graphics
  timer.ResetTime();
  xde::builder::GraphicsBuilder graphBuilder(app->getViewer().getSceneComponents(), &assets);
  graphBuilder.loadAssets();
	const std::string& objId = assets.getIds().getLastId(filePath, nodeName);
  graphBuilder.createNodeFromTree(name, "", objId, objId, false);
  std::cout << "Building graphics: " << timer.GetTime() * .001 << " sec." << std::endl;

  // Collision
  timer.ResetTime();
  xde::builder::PhysicsBuilder phyBuilder(app->getXCDScene(), &assets);
  xde::xcd::CompositeRef groundComposite = phyBuilder.createCompositeFromTree(name + std::string(".comp"), objId, objId, offset, 1., false, false);
  std::cout << "Building collision: " << timer.GetTime() * .001 << " sec." << std::endl;

  // GVM RigidBody
  timer.ResetTime();
  xde::gvm::RigidBodyRef rigidBody = xde::gvm::RigidBodyRef::createObject("collisionGround");
  xde::gvm::FixedJointRef joint = xde::gvm::FixedJointRef::createObject(name + std::string(".joint"));
  app->getGVMScene().addRigidBodyToGround(rigidBody, joint);
  rigidBody.setComposite(groundComposite);
  joint.configure(position);
  std::cout << "Building physics: " << timer.GetTime() * .001 << " sec." << std::endl;

  return rigidBody;

  }
  catch(std::exception & e)
  {
    std::cerr << e.what() << std::endl;
    throw e;
  }
}



}


// cmake:sourcegroup=Helpers