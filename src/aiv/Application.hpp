#ifndef __AIV_APPLICATION_HPP__
#define __AIV_APPLICATION_HPP_
#pragma once

#include <iostream>

#include "xde/physics/gvm/Scene.h"
#include "xde/physics/lmdpp/Scene.h"
#include "xde/graphics/builder/SimpleViewer.h"

namespace aiv {

class AIV;
class Obstacle;

class Application
{
  friend class ApplicationBuilder;

public:  // Cstor & Dstor
  Application(double);
  ~Application();

public:
  double getTimeStep();
  double getSimSpeed();
  //void setTimeStep(double);
  void setSimSpeed(double);

  xde::gvm::SceneRef & getGVMScene();
  xde::xcd::SceneRef & getXCDScene();

  xde::builder::SimpleViewer & getViewer();
  OGREViewer::SceneInterface & getGraphicSceneInterface();
  OGREViewer::CameraInterface & getCameraInterface();

public:
  bool addVehicle(AIV *);
  AIV * getVehicleByName(const std::string &);
  std::map<std::string, AIV *> Application::getVehicles() const;

  bool addObstacle(Obstacle *);
  Obstacle * getObstacleByName(const std::string &);
  std::map<std::string, Obstacle *> Application::getObstacles() const;

public:
  void update();
  void printPerformanceReport(std::ostream &);

protected:
  xde::gvm::SceneRef gvmScene;
  xde::xcd::SceneRef lmdScene;

  double timeStep;
  double simSpeed;

  xde::builder::SimpleViewer *  xdeViewer;
  OGREViewer::SceneInterface *  si;
  OGREViewer::CameraInterface * ci;

  std::map<std::string, AIV *> vehicles;
  std::map<std::string, Obstacle *> obstacles;

};

}

#endif // __AIV_APPLICATION_HPP_

// cmake:sourcegroup=Application