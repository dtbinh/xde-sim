/**
 *
 *
 * \date 02/2015
 * \author CEA/DRT/LIST/DIASI/LSI
 * \author P.Evrard
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
#include "xde/physics/gvm/SimulationEnum.h"
#include "xde/physics/gvm/FreeJoint.h"
#include "xde/physics/gvm/FixedJoint.h"

#include "xde/physics/lmdpp/Scene.h"

#include "sys/timer.h"
#include "xde/desc/core/geometry.pb.h"

#include <iostream>

//==============================================================================
// GEOMETRIC OBJECTS
//==============================================================================
class GeometricObjects
{
public:
static xde::xcd::CompositeRef createCapsule(	xde::xcd::SceneRef scene,
												const std::string& name,
												const Eigen::Vector3d& p,
												const Eigen::Vector3d& q,
												double radius);

static xde::xcd::CompositeRef createDilatedPlane(	xde::xcd::SceneRef scene,
													const std::string& name,
													const Eigen::Vector3d& p0,
													const Eigen::Vector3d& p1,
													const Eigen::Vector3d& p2,
													const Eigen::Vector3d& p3,
													double radius);

static xde::xgl::TriMeshRef createImplicitSphere(const std::string& name, const Eigen::Vector3d& center, double radius);
static xde::xgl::TriMeshRef createBoxGeometry(const std::string& name, const Eigen::Vector3d& center, const Eigen::Vector3d& half_size);
static xde::desc::core::Mesh* createBoxGeometryDesc(const std::string& name, const Eigen::Vector3d& center, const Eigen::Vector3d& half_size);

static void createDiamondGeometry(xde::xgl::TriMeshRef mesh, const Eigen::Vector3d& u, int n);

};