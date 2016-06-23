/**
 *
 *
 * \date 03/2015
 * \author CEA/DRT/LIST/DIASI/LRI
 * \author E.Lucet
 *
 * @par Licence
 * Copyright © 2015 CEA
 */
#pragma once
#define WIN32_LEAN_AND_MEAN

#include "xde/data/Assets.h"

#include "xde/graphics/builder/GraphicsBuilder.h"
#include "xde/graphics/builder/SimpleViewer.h"

#include "xde/physics/builder/PhysicsBuilder.h"
#include "xde/physics/manipulation/VelocityBasedManipulator.h"

#include "xde/physics/gvm/Scene.h"
#include "xde/physics/gvm/RigidBody.h"
#include "xde/physics/gvm/HingeJoint.h"
#include "xde/physics/gvm/PrismaticJoint.h"
#include "xde/physics/gvm/SerialJoint.h"
#include "xde/physics/gvm/SimulationEnum.h"
#include "xde/physics/xgl/ObjReader.h"
#include "xde/physics/gvm/FreeJoint.h"
#include "xde/physics/gvm/FixedJoint.h"
#include "xde/physics/gvm/ContactLawDesc.h"
#include "xde/physics/gvm/ExtWrench.h"

#include "xde/physics/lmdpp/Scene.h"

#include "sys/timer.h"
#include "xde/desc/core/geometry.pb.h"

#include <Eigen/Lgsm>
#include <iostream>

//==============================================================================
// ROBOT CREATOR
//==============================================================================
class RobotCreator
{
public:
RobotCreator();
void update();
void finalize();

private:
//// Physics
// functions
void createGround();
void createChassis();
void createWheel(	const std::string name, 
					xde::gvm::RigidBodyRef& wheel,
					xde::gvm::SerialJointRef& serial, 
					xde::gvm::HingeJointRef& hinge,
					Eigen::Displacementd pos,
					double radius);

// Bodies
xde::gvm::SceneRef mechanicalScene;
xde::gvm::RigidBodyRef ground;
xde::gvm::RigidBodyRef chassis;
xde::gvm::RigidBodyRef wheel0;
xde::gvm::RigidBodyRef wheel1;
xde::gvm::RigidBodyRef wheel2;
xde::gvm::RigidBodyRef wheel3;
xde::gvm::RigidBodyRef wheel4;
xde::gvm::RigidBodyRef wheel5;

// Joints
xde::gvm::HingeJointRef hinge0;
xde::gvm::HingeJointRef hinge1;
xde::gvm::HingeJointRef hinge2;
xde::gvm::HingeJointRef hinge3;
xde::gvm::HingeJointRef hinge4;
xde::gvm::HingeJointRef hinge5;

xde::gvm::SerialJointRef serial0;
xde::gvm::SerialJointRef serial1;
xde::gvm::SerialJointRef serial2;
xde::gvm::SerialJointRef serial3;
xde::gvm::SerialJointRef serial4;
xde::gvm::SerialJointRef serial5;

//// Collisions
xde::xcd::SceneRef lmdScene;

////Viewer
xde::builder::SimpleViewer viewer;
OGREViewer::SceneInterface& si;
OGREViewer::CameraInterface& ci;

//Recuperation de contenu de fichier sous forme de description protobuf
xde::data::Assets assets;

// Init
const int nbStepsByFrame;


};