#include "xde/hardware/spacemouse/SpaceMouse.h"

#include "xde/data/Assets.h"

#include "xde/physics/human/XDESkeletonGraphics.h"
#include "xde/physics/human/UnitJointsBasedXDESkeleton.h"
#include "xde/physics/human/MorphologicalParameters.h"
#include "xde/physics/human/Bone.h"

#include "xde/graphics/builder/GraphicsBuilder.h"
#include "xde/graphics/builder/SimpleViewer.h"

#include "xde/physics/builder/PhysicsBuilder.h"
#include "xde/physics/manipulation/VelocityBasedManipulator.h"
#include "xde/physics/manipulation/PositionBasedManipulator.h"

#include "xde/physics/gvm/Scene.h"
#include "xde/physics/gvm/SimulationEnum.h"
#include "xde/physics/gvm/FreeJoint.h"
#include "xde/physics/gvm/FixedJoint.h"

#include "xde/physics/lmdpp/Scene.h"

#include "sys/timer.h"

#include <iostream>

/*void test_Human()
{
  xde::sys::Timer timer;

  xde::builder::SimpleViewer viewer("../../../../share/resources/ogre");
  OGREViewer::SceneInterface& si = viewer.getSceneInterface();
  OGREViewer::CameraInterface& ci = viewer.getCameraInterface();

  std::cout << "Creating viewer: " << timer.GetTime() * .001 << " sec." << std::endl;

  timer.ResetTime();

  xde::gvm::SceneRef scene = xde::gvm::SceneRef::createObject("main");
  scene.setIntegratorFlags(xde::gvm::DYNAMIC_INTEGRATOR | xde::gvm::GAUSS_SEIDEL_SOLVER | xde::gvm::USE_NON_LINEAR_TERMS);
  scene.setUcRelaxationFactor(.1);
  scene.setFdvrFactor(.2);
  scene.setTimeStep(.01);

  xde::xcd::SceneRef xcd = xde::lmd::SceneRef::createObject("lmd", .02, .05);

  scene.setGeometricalScene(xcd);

  std::cout << "Creating simulation: " << timer.GetTime() * .001 << " sec." << std::endl;

  timer.ResetTime();

  xde::data::Assets assets;
  assets.parseOpenColladaFile("../../../../share/resources/scenes/ground.dae", Eigen::Vector3d::Ones());

  std::cout << "Parsing data: " << timer.GetTime() * .001 << " sec." << std::endl;

  timer.ResetTime();

  xde::builder::GraphicsBuilder graphBuilder(viewer.getSceneComponents(), &assets);

  graphBuilder.loadAssets();

  xde::desc::core::Id groundId;
  groundId.set_id_str("node-ground");
  graphBuilder.createNodeFromTree("ground", "", groundId, groundId, false);

  std::cout << "Building graphics: " << timer.GetTime() * .001 << " sec." << std::endl;

  xde::builder::PhysicsBuilder phyBuilder(xcd, &assets);

  timer.ResetTime();

  xde::xcd::CompositeRef groundComposite = phyBuilder.createCompositeFromTree("ground.comp", groundId, groundId, .005, 1., false, false);

  std::cout << "Building collision: " << timer.GetTime() * .001 << " sec." << std::endl;

  timer.ResetTime();

  xde::gvm::RigidBodyRef groundBody = xde::gvm::RigidBodyRef::createObject("ground");
  xde::gvm::FixedJointRef groundJoint = xde::gvm::FixedJointRef::createObject("ground.joint");
  scene.addRigidBodyToGround(groundBody, groundJoint);
  groundBody.setComposite(groundComposite);
  groundJoint.configure(Eigen::Displacementd(Eigen::Vector3d(0., 0., -.11)));

  std::cout << "Building physics: " << timer.GetTime() * .001 << " sec." << std::endl;

  timer.ResetTime();

  xde::human::MorphologicalParameters humanParams;
  xde::human::UnitJointsBasedXDESkeleton skel("human", scene, xcd, humanParams);
  skel.build();
  
  std::vector<xde::human::Bone*> bones = skel.getBones();

  for(size_t i = 0; i < bones.size(); ++i)
  {
    scene.enableContactForBodyPair(bones[i]->getBody(), groundBody, true);
    bones[i]->getBody().disableWeight();
  }
  
  xde::human::SkeletonGraphics* skelGraphics = new xde::human::XDESkeletonGraphics(skel);
  skelGraphics->build();

  viewer.getSceneComponents().mergeGraphicalScene(skelGraphics->getScene(), skelGraphics->getLibrary());
  
  OGREViewer::MarkersInterface& mi = viewer.getMarkersInterface();

  for(size_t i = 0; i < bones.size(); ++i)
    mi.addMarker(bones[i]->getName(), false);

  std::cout << "Building human: " << timer.GetTime() * .001 << " sec." << std::endl;

  xde::gvm::FixedJointRef waistLocker = xde::gvm::FixedJointRef::createObject("waistLocker");
  waistLocker.configure(skel.getRootBone()->getJoint().getDirectGeometricModel());
  scene.replaceDgmJoint(skel.getRootBone()->getJoint(), waistLocker);

  std::vector<xde::hardware::SpaceMouseDesc> smds = xde::hardware::SpaceMouse::scan();
  xde::hardware::SpaceMouse sm(smds[0]);
  sm.setMaxLinearVelocity(1.);
  sm.setMaxAngularVelocity(3.14);
  sm.setDominatingModeOn();

  xde::manip::VelocityBasedManipulator vbm(scene);
  vbm.setTau(.01);
  vbm.attach(bones[6]->getBody());
  bones[6]->getBody().disableWeight();

  for(int n = 0; n < 3; ++n)
  {
    for(int i = 0; i < 100; ++i)
    {
      viewer.update();

      sm.update();

      vbm.setObservationFrame(ci.getCameraDisplacement(ci.getMainCameraName()));
      vbm.setInputVelocity(sm.getVelocity());
      vbm.preUpdate();

      scene.detectCollisions();
      scene.integrate();

      vbm.postUpdate();

      si.setNodePosition("ground", groundBody.getPosition());

      for(size_t i = 0; i < bones.size(); ++i)
      {
        si.setNodePosition(bones[i]->getName(), bones[i]->getBody().getPosition());
        mi.setMarker6DPosition(bones[i]->getName(), bones[i]->getBody().getPosition());
      }

      xde::sys::Timer::Sleep(10);
    }

    if(n != 2)
    {
      timer.ResetTime();

      vbm.detach();

      scene.replaceDgmJoint(waistLocker, skel.getRootBone()->getJoint());

      xde::human::MorphologicalParameters newHumanParams(90., 2.10);

      skel.setParameters(newHumanParams);

      bones = skel.getBones();

      for(size_t i = 0; i < bones.size(); ++i)
      {
        scene.enableContactForBodyPair(bones[i]->getBody(), groundBody, true);
        bones[i]->getBody().disableWeight();
      }

      xde::builder::removeNode(viewer.getSceneComponents(), skelGraphics->getScene().root_node(), true);
      skelGraphics->clear();
      delete skelGraphics;

      if(n == 0)
        skelGraphics = new xde::human::SkeletonGraphics(skel);
      else
        skelGraphics = new xde::human::XDESkeletonGraphics(skel);

      skelGraphics->build();

      viewer.getSceneComponents().mergeGraphicalScene(skelGraphics->getScene(), skelGraphics->getLibrary());

      vbm.attach(bones[6]->getBody());
      bones[6]->getBody().disableWeight();

      waistLocker.configure(skel.getRootBone()->getJoint().getDirectGeometricModel());
      scene.replaceDgmJoint(skel.getRootBone()->getJoint(), waistLocker);

      std::cout << "Re-building human: " << timer.GetTime() * .001 << " sec." << std::endl;
    }
  }

  vbm.detach();

  scene.replaceDgmJoint(waistLocker, skel.getRootBone()->getJoint());

  skel.clear();

  scene.printPerformanceReport(std::cout);

  system("PAUSE");
}
*/