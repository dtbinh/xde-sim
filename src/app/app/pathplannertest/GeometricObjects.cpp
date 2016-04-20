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
//==============================================================================
// GEOMETRIC OBJECTS
//==============================================================================
#include "xde/data/Assets.h"

#include "xde/graphics/builder/GraphicsBuilder.h"
#include "xde/graphics/builder/SimpleViewer.h"

#include "xde/physics/builder/PhysicsBuilder.h"
#include "xde/physics/manipulation/VelocityBasedManipulator.h"

#include "xde/physics/gvm/Scene.h"
#include "xde/physics/gvm/SimulationEnum.h"
#include "xde/physics/gvm/FreeJoint.h"
#include "xde/physics/gvm/FixedJoint.h"

//#include "xde/physics/xgl/SimplicialComplex.h"
//#include "xde/modules/physics/helper/src/xde/physics/human/SkeletonGraphics.cpp"
#include "xde/physics/human/XDESkeletonGraphics.h"

#include "xde/physics/lmdpp/Scene.h"

#include "sys/timer.h"

#include <iostream>

#include "GeometricObjects.hpp"

//---------------------------------------------
// Capsule object
//---------------------------------------------
xde::xcd::CompositeRef GeometricObjects::createCapsule(		xde::xcd::SceneRef	scene,
											const	std::string&		name,
											const	Eigen::Vector3d&	p,
											const	Eigen::Vector3d&	q,
													double				radius)
{
	xde::xcd::CompositeRef comp = scene.createRigidComposite(name);
	xde::xcd::TriMeshRef mesh = comp.createTriMesh(name + ".mesh");
	xde::xgl::SimplicialComplexRef simp = mesh.getSimplicialComplex();

	simp.setNumberOfVertices(2);
	simp.setNumberOfEdges(1);

	simp.setVertex(0, p.x(), p.y(), p.z());
	simp.setVertex(1, q.x(), q.y(), q.z());

	int* edges = simp.getEdges_dataArray();
	edges[0] = 0;
	edges[1] = 1;

	comp.addTriMesh(mesh);
	mesh.setDilation(radius);

	return comp;
}

//---------------------------------------------
// Dilated plane object
// 
// The plane is defined by 4 points: p0 .. p3,
// and 2 triangles: p0 p1 p2 and p1 p3 p2
//---------------------------------------------
xde::xcd::CompositeRef GeometricObjects::createDilatedPlane(			xde::xcd::SceneRef	scene,
													const	std::string&		name,
													const	Eigen::Vector3d&	p0,
													const	Eigen::Vector3d&	p1,
													const	Eigen::Vector3d&	p2,
													const	Eigen::Vector3d&	p3,
															double				radius)
{
	xde::xcd::CompositeRef comp = scene.createRigidComposite(name);
	xde::xcd::TriMeshRef mesh = comp.createTriMesh(name + ".mesh");
	xde::xgl::SimplicialComplexRef simp = mesh.getSimplicialComplex();

	simp.setNumberOfVertices(4);
	simp.setNumberOfTriangles(2);

	simp.setVertex(0, p0.x(), p0.y(), p0.z());
	simp.setVertex(1, p1.x(), p1.y(), p1.z());
	simp.setVertex(2, p2.x(), p2.y(), p2.z());
	simp.setVertex(3, p3.x(), p3.y(), p3.z());

	int* triangles = simp.getTriangles_dataArray();
	triangles[0] = 0;
	triangles[1] = 1;
	triangles[2] = 2;
	triangles[3] = 1;
	triangles[4] = 3;
	triangles[5] = 2;

	comp.addTriMesh(mesh);
	mesh.setDilation(radius);

	return comp;
}

//---------------------------------------------
// Box object
//---------------------------------------------
xde::xgl::TriMeshRef GeometricObjects::createBoxGeometry(const std::string& name, const Eigen::Vector3d& center, const Eigen::Vector3d& half_size)
{
	xde::xgl::TriMeshRef mesh = xde::xgl::TriMeshRef::createObject(name);
	xde::xgl::SimplicialComplexRef sc = mesh.getSimplicialComplex();

	sc.setNumberOfVertices(8);
	sc.setVertex(0,  half_size.x(), -half_size.y(),  half_size.z());
	sc.setVertex(1,  half_size.x(), -half_size.y(), -half_size.z());
	sc.setVertex(2, -half_size.x(), -half_size.y(), -half_size.z());
	sc.setVertex(3, -half_size.x(), -half_size.y(),  half_size.z());
	sc.setVertex(4,  half_size.x(),  half_size.y(),  half_size.z());
	sc.setVertex(5,  half_size.x(),  half_size.y(), -half_size.z());
	sc.setVertex(6, -half_size.x(),  half_size.y(), -half_size.z());
	sc.setVertex(7, -half_size.x(),  half_size.y(),  half_size.z());

	sc.setNumberOfTriangles(12);
	sc.setTriangle(0, 0, 2, 1); // front
	sc.setTriangle(1, 0, 3, 2);
	sc.setTriangle(2, 4, 1, 5); // right side
	sc.setTriangle(3, 4, 0, 1);
	sc.setTriangle(4, 7, 5, 6); // back
	sc.setTriangle(5, 7, 4, 5);
	sc.setTriangle(6, 3, 6, 2); // left side
	sc.setTriangle(7, 3, 7, 6);
	sc.setTriangle(8, 4, 3, 0); // top
	sc.setTriangle(9, 4, 7, 3);
	sc.setTriangle(10, 1, 6, 5); // bottom
	sc.setTriangle(11, 1, 2, 6);

	return mesh;
}

xde::desc::core::Mesh* GeometricObjects::createBoxGeometryDesc(const std::string& name, const Eigen::Vector3d& center, const Eigen::Vector3d& half_size)
{
	xde::desc::core::Mesh* mesh = new xde::desc::core::Mesh();
	mesh->set_name(name);
	mesh->mutable_id()->set_id_str(name);
	mesh->set_rep_type(xde::desc::core::Mesh::DOUBLE);
	xde::desc::core::Vertex* vertices = mesh->add_vertices();

	vertices->set_type(xde::desc::core::Vertex::POSITION);
	//add vertices
	//v0
	vertices->add_datad(half_size.x());
	vertices->add_datad(-half_size.y());
	vertices->add_datad(half_size.z());
	//v1
	vertices->add_datad(half_size.x());
	vertices->add_datad(-half_size.y());
	vertices->add_datad(-half_size.z());
	//v2
	vertices->add_datad(-half_size.x());
	vertices->add_datad(-half_size.y());
	vertices->add_datad(-half_size.z());
	//v3
	vertices->add_datad(-half_size.x());
	vertices->add_datad(-half_size.y());
	vertices->add_datad(half_size.z());
	//v4 
	vertices->add_datad(half_size.x());
	vertices->add_datad(half_size.y());
	vertices->add_datad(half_size.z());
    //v5 
	vertices->add_datad(half_size.x());
	vertices->add_datad(half_size.y());
	vertices->add_datad(-half_size.z());
	 //v6
	vertices->add_datad(-half_size.x());
	vertices->add_datad(half_size.y());
	vertices->add_datad(-half_size.z());
	 //v7
	vertices->add_datad(-half_size.x());
	vertices->add_datad(half_size.y());
	vertices->add_datad(half_size.z());

	//add triangles
	xde::desc::core::PrimitiveSet* primitives = mesh->add_primitives();
	primitives->add_triangles(0);
	primitives->add_triangles(2);
	primitives->add_triangles(1);

	primitives->add_triangles(0);
	primitives->add_triangles(3);
	primitives->add_triangles(2);

	primitives->add_triangles(4);
	primitives->add_triangles(1);
	primitives->add_triangles(5);

	primitives->add_triangles(4);
	primitives->add_triangles(0);
	primitives->add_triangles(1);

	primitives->add_triangles(7);
	primitives->add_triangles(5);
	primitives->add_triangles(6);

	primitives->add_triangles(7);
	primitives->add_triangles(4);
	primitives->add_triangles(5);

	primitives->add_triangles(3);
	primitives->add_triangles(6);
	primitives->add_triangles(2);

	primitives->add_triangles(3);
	primitives->add_triangles(7);
	primitives->add_triangles(6);

	primitives->add_triangles(4);
	primitives->add_triangles(3);
	primitives->add_triangles(0);

	primitives->add_triangles(4);
	primitives->add_triangles(7);
	primitives->add_triangles(3);

	primitives->add_triangles(1);
	primitives->add_triangles(6);
	primitives->add_triangles(5);

	primitives->add_triangles(1);
	primitives->add_triangles(2);
	primitives->add_triangles(6);

	return mesh;
}

//---------------------------------------------
// Sphere object
//---------------------------------------------
xde::xgl::TriMeshRef GeometricObjects::createImplicitSphere(const std::string& name, const Eigen::Vector3d& center, double radius)
{
	xde::xgl::TriMeshRef mesh = xde::xgl::TriMeshRef::createObject(name);
	xde::xgl::SimplicialComplexRef sc = mesh.getSimplicialComplex();

	sc.setNumberOfVertices(1);
	sc.setVertex(0, center.x(), center.y(), center.z());
	mesh.setDilation(radius);

	return mesh;
}

//---------------------------------------------
// Diamond object
//---------------------------------------------
namespace 
{
  Eigen::Quaterniond rotationThatProjectsEtoT(const Eigen::Vector3d& e, const Eigen::Vector3d& t);
  void mapVector3d(double* ptr, const Eigen::Vector3d& v);

  int argmin(const Eigen::Vector3d& v)
  {
    int res = 0;
    for(int i = 1; i < 3; ++i)
      if(v[i] < v[res])
        res = i;
    return res;
  }

  Eigen::Matrix3d tildeMat(const Eigen::Vector3d& v)
  {
    Eigen::Matrix3d res;
    res <<
      0., -v[2], v[1],
      v[2], 0., -v[0],
      -v[1], v[0], 0.;
    return res;
  }
  
  Eigen::Quaterniond rotationThatProjectsEtoT(const Eigen::Vector3d& e, const Eigen::Vector3d& t)
  {
    if((e+t).norm() < 1.e-8)
    {
      Eigen::Vector3d x = Eigen::Vector3d::Zero();
      x[argmin(e)] = 1.;
      Eigen::Vector3d n1 = e.cross(x);
      n1.normalize();
      Eigen::Vector3d n2 = e.cross(n1);
      return Eigen::Quaterniond(0., n1[0], n1[1], n1[2]);
    }
    else
    {
      double e_dot_t = e.transpose() * t;
      Eigen::Vector3d e_vect_t = e.cross(t);
      Eigen::Matrix3d evt_tens_evt = e_vect_t * e_vect_t.transpose();
      Eigen::Matrix3d R(e_dot_t * Eigen::Matrix3d::Identity() + tildeMat(e_vect_t) + evt_tens_evt / (1. + e_dot_t));
      Eigen::Quaterniond q(R);
      q.normalize();
      return q;
    }
  }

  inline void mapVector3d(double* ptr, const Eigen::Vector3d& v)
  {
    Eigen::Map<Eigen::Vector3d> m(ptr);
    m = v;
  }
}

void GeometricObjects::createDiamondGeometry(xde::xgl::TriMeshRef mesh, const Eigen::Vector3d& u, int n)
{
	using Eigen::Vector3d;
	const double BASE_RATIO = 1;
	const double HALF_BASE_SIZE = 1;
	const double u_norm = u.norm();
	const double z_base = BASE_RATIO * u_norm;
	const double angle = 2. * M_PI / static_cast<double>(n);
	const Eigen::Quaterniond R = rotationThatProjectsEtoT(Eigen::Vector3d::UnitZ(), u * (1. / u_norm));

	mesh.getSimplicialComplex().setNumberOfVertices(n + 2);
	mesh.getSimplicialComplex().setNumberOfTriangles(n * 2);

	double* verticesPtr = mesh.getSimplicialComplex().getVertices_dataArray();
	int* trianglesPtr = mesh.getSimplicialComplex().getTriangles_dataArray();

	mapVector3d(verticesPtr, Vector3d::Zero());
	mapVector3d(verticesPtr + 3, R * Vector3d(0., 0., u_norm));

	verticesPtr += 6;

	for(int i = 0; i < n; ++i) // base and triangles
	{
		const double theta = i * angle;
		const int i_base_cur = 2 + i;
		const int i_base_next = 2 + (i + 1) % n;

		mapVector3d(verticesPtr, R * Vector3d(HALF_BASE_SIZE * std::cos(theta), HALF_BASE_SIZE * std::sin(theta), z_base));

		//setTriangle(trianglesPtr  , 0, i_base_next, i_base_cur);
		//setTriangle(trianglesPtr+3, 1, i_base_cur, i_base_next);

		verticesPtr += 3;
		trianglesPtr += 6;
	}
}