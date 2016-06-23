#include "xde/hardware/spacemouse/SpaceMouse.h"

#include "xde/data/Assets.h"

#include "xde/graphics/builder/GraphicsBuilder.h"
#include "xde/graphics/builder/SimpleViewer.h"

#include "xde/physics/builder/PhysicsBuilder.h"
#include "xde/physics/manipulation/VelocityBasedManipulator.h"
#include "xde/physics/manipulation/HapticManipulator.h"

#include "xde/physics/gvm/Scene.h"
#include "xde/physics/gvm/SimulationEnum.h"
#include "xde/physics/gvm/FreeJoint.h"
#include "xde/physics/gvm/FixedJoint.h"

#include "xde/physics/lmdpp/Scene.h"

#include "sys/timer.h"

#include <iostream>

void test_HapticManipulator()
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
  assets.importOpenColladaFile("../../../../share/resources/scenes/ground.dae", Eigen::Vector3d::Ones());
  assets.importOpenColladaFile("../../../../share/resources/scenes/knob.dae", Eigen::Vector3d::Constant(.2));

  std::cout << "Parsing data: " << timer.GetTime() * .001 << " sec." << std::endl;

  timer.ResetTime();

  xde::builder::GraphicsBuilder graphBuilder(viewer.getSceneComponents(), &assets);

  graphBuilder.loadAssets();

  const std::string& knobId = assets.getIds().getLastId("../../../../share/resources/scenes/knob.dae", "knob");
  graphBuilder.createNodeFromTree("knob", "", knobId, knobId, false);

  graphBuilder.createNodeFromTree("knobProxy", "", knobId, knobId, false);
  si.setNodeMaterial("knobProxy", "xde/OrangeOpaque");

	const std::string& groundId = assets.getIds().getLastId("../../../../share/resources/scenes/ground.dae", "node-ground");
  graphBuilder.createNodeFromTree("ground", "", groundId, groundId, false);

  std::cout << "Building graphics: " << timer.GetTime() * .001 << " sec." << std::endl;

  xde::builder::PhysicsBuilder phyBuilder(xcd, &assets);

  timer.ResetTime();

  xde::xcd::CompositeRef groundComposite = phyBuilder.createCompositeFromTree("ground.comp", groundId, groundId, .005, 1., false, false);
  xde::xcd::CompositeRef knobComposite = phyBuilder.createCompositeFromTree("knob.comp", knobId, knobId, .005, .1, false, false);
  xde::xcd::CompositeRef knobProxyComposite = phyBuilder.createCompositeFromTree("knobProxy.comp", knobId, knobId, .005, .1, false, false);

  std::cout << "Building collision: " << timer.GetTime() * .001 << " sec." << std::endl;

  timer.ResetTime();

  xde::gvm::RigidBodyRef groundBody = xde::gvm::RigidBodyRef::createObject("ground");
  xde::gvm::FixedJointRef groundJoint = xde::gvm::FixedJointRef::createObject("ground.joint");
  scene.addRigidBodyToGround(groundBody, groundJoint);
  groundBody.setComposite(groundComposite);
  groundJoint.configure(Eigen::Displacementd::Identity());

  xde::gvm::RigidBodyRef knobBody = xde::gvm::RigidBodyRef::createObject("knob");
  xde::gvm::FreeJointRef knobJoint = xde::gvm::FreeJointRef::createObject("knob.joint");
  scene.addRigidBodyToGround(knobBody, knobJoint);
  knobBody.setComposite(knobComposite);
  knobBody.setMass(1.);
  knobBody.computePrincipalFrameAndMomentsOfInertiaUsingCompositeObbAndMass();
  knobJoint.configure(Eigen::Displacementd(0., 0., 1., .707, 0., .707, 0.));

  xde::gvm::RigidBodyRef knobProxyBody = xde::gvm::RigidBodyRef::createObject("knobProxy");
  xde::gvm::FreeJointRef knobProxyJoint = xde::gvm::FreeJointRef::createObject("knobProxy.joint");
  scene.addRigidBodyToGround(knobProxyBody, knobProxyJoint);
  knobProxyBody.setComposite(knobProxyComposite);
  knobProxyBody.setMass(1.);
  knobProxyBody.computePrincipalFrameAndMomentsOfInertiaUsingCompositeObbAndMass();
  knobProxyJoint.configure(Eigen::Displacementd(0., 0., 1., .707, 0., .707, 0.));

  scene.enableContactForBodyPair(knobBody, groundBody, true);

  std::cout << "Building physics: " << timer.GetTime() * .001 << " sec." << std::endl;

  timer.ResetTime();

  std::vector<xde::hardware::SpaceMouseDesc> smds = xde::hardware::SpaceMouse::scan();
  xde::hardware::SpaceMouse sm(smds[0]);
  sm.setMaxLinearVelocity(1.);
  sm.setMaxAngularVelocity(3.14);
  sm.setDominatingModeOn();

  xde::manip::VelocityBasedManipulator vbm(scene);
  vbm.setTau(.01);

  vbm.attach(knobProxyBody);
  knobProxyBody.disableWeight();

  std::cout << "Building spacemouse: " << timer.GetTime() * .001 << " sec." << std::endl;
  
  timer.ResetTime();
  
  xde::manip::HapticManipulator hm(scene);
  hm.set_kr(70.);
  hm.set_kt(1400.);
  hm.set_br(1.);
  hm.set_bt(40.);
  
  hm.setRigidBodyReferencePosition(knobProxyBody.getPosition());
  hm.setRigidBodyReferenceVelocity(knobProxyBody.getVelocity());
  hm.attach(knobBody);
  knobBody.disableWeight();

  xde::gvm::CartesianPDCouplingRef cpdProxy = xde::gvm::CartesianPDCouplingRef::createObject("cpdProxy");
  cpdProxy.setCoupledRigidBody(knobProxyBody);
  scene.add(cpdProxy);
  cpdProxy.disable();
  cpdProxy.set_kr(70.);
  cpdProxy.set_kt(1400.);
  cpdProxy.set_br(1.);
  cpdProxy.set_bt(40.);
  cpdProxy.setDesiredPosition(knobProxyBody.getPosition(), knobProxyBody.getPosition());
  cpdProxy.setDesiredVelocityInBodyFrame(knobProxyBody.getVelocity(), knobProxyBody.getVelocity());
  cpdProxy.enable();

  std::cout << "Building pseudo haptic: " << timer.GetTime() * .001 << " sec." << std::endl;

  for(int i = 0; i < 500; ++i)
  {
    viewer.update();

    sm.update();

    vbm.setObservationFrame(ci.getCameraDisplacement(ci.getMainCameraName()));
    vbm.setInputVelocity(sm.getVelocity());
    vbm.preUpdate();

    hm.setRigidBodyReferencePosition(knobProxyBody.getPosition());
    hm.setRigidBodyReferenceVelocity(knobProxyBody.getVelocity().changeFrame(knobProxyBody.getPosition().getRotation()));
    hm.preUpdate();

    scene.detectCollisions();
    scene.integrate();

    vbm.postUpdate();
    hm.postUpdate();

    cpdProxy.setDesiredPosition(hm.getDeviceReferencePosition(), hm.getDeviceReferencePosition());
    cpdProxy.setDesiredVelocityInKoenigFrame(hm.getDeviceReferenceVelocity(), hm.getDeviceReferenceVelocity());

    si.setNodePosition("ground", groundBody.getPosition());
    si.setNodePosition("knob", knobBody.getPosition());
    si.setNodePosition("knobProxy", knobProxyBody.getPosition());

    xde::sys::Timer::Sleep(10);
  }

  hm.detach();
  vbm.detach();

  cpdProxy.disable();
  scene.remove(cpdProxy);

  scene.removeRigidBody(groundBody);
  scene.removeRigidBody(knobBody);
  scene.removeRigidBody(knobProxyBody);

  scene.printPerformanceReport(std::cout);

  system("PAUSE");
}
