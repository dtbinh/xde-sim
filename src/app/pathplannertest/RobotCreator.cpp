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
//==============================================================================
// ROBOT CREATOR
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

#include "xde/physics/human/XDESkeletonGraphics.h"

#include "xde/physics/lmdpp/Scene.h"

#include "sys/timer.h"

#include "GeometricObjects.hpp" // objets geometriques

#include <iostream>

#include "RobotCreator.hpp"

//---------------------------------------------
// Constructeur
//---------------------------------------------
RobotCreator::RobotCreator() : 
			 nbStepsByFrame(20)
			,viewer("../../../../share/resources/ogre")
			,si(viewer.getSceneInterface())
			,ci(viewer.getCameraInterface())
{
	xde::sys::Timer timer;
    /////////////////////////////////////
    //           Set viewer            //
    /////////////////////////////////////
	/*
	Creation d'un viewer Ogre pre-configure et recuperation de references sur les interfaces
	de manipulation de mechanicalScene et de camera.
	*/
	//OGREViewer::SceneInterface& si = viewer.getSceneInterface();
	//OGREViewer::CameraInterface& ci = viewer.getCameraInterface();


    /////////////////////////////////////
    //        Set meca & collisions    //
    /////////////////////////////////////
	/*
	Configuration de GVM (solveur mécanique, qui intègre les equations de la dynamique et resout les contacts) et d'XCD
	(detection de collisions). Ici, on utilise le moteur LMD d'XCD, qui base la detection de collision sur le calcul
	de distances minimales locales.
	*/
	mechanicalScene = xde::gvm::SceneRef::createObject("mechanicalScene");
	mechanicalScene.setIntegratorFlags(xde::gvm::DYNAMIC_INTEGRATOR | xde::gvm::GAUSS_SEIDEL_SOLVER | xde::gvm::USE_NON_LINEAR_TERMS);
	mechanicalScene.setUcRelaxationFactor(.1);
	mechanicalScene.setFdvrFactor(.2);
	mechanicalScene.setTimeStep(0.001);
	mechanicalScene.setVerticalDirectionUp(Eigen::Vector3d(0., 0., 1.0));


	lmdScene = xde::lmd::SceneRef::createObject("lmdScene", .02, .05);

	mechanicalScene.setGeometricalScene(lmdScene);

    createGround();
    createChassis();

	//enum Ideal_wheel_composites {
	//	"01.comp" = [ 0.196637, 0.096843, 0.0],
	//	"02.comp" = [-0.196637, 0.096843, 0.0]};
	//enum Ideal_castor_wheel_composites { 
	//	"03.comp" = [ 0.10245, 0.0375, 0.203167],
	//	"04.comp" = [-0.10245, 0.0375, 0.203167],
	//	"05.comp" = [ 0.151159, 0.0375, -0.222624],
	//	"06.comp" = [-0.151159, 0.0375, -0.222624]};
	double wheel_radius =  0.096843;
	double castor_wheel_radius =  0.036118;
    createWheel("wheel0", wheel0,
        serial0, hinge0,
        Eigen::Displacementd(0.196637, 0.096843, 0.0, 1.0, 0., 0., 0.), wheel_radius);
    createWheel("wheel1", wheel1,
        serial1, hinge1,
        Eigen::Displacementd(-0.196637, 0.096843, 0.0, 1.0, 0., 0., 0.), wheel_radius);
    createWheel("wheel2", wheel2,
        serial2, hinge2,
        Eigen::Displacementd(0.10245, 0.0375, 0.203167, 1.0, 0., 0., 0.), castor_wheel_radius);
    createWheel("wheel3", wheel3,
        serial3, hinge3,
        Eigen::Displacementd(-0.10245, 0.0375, 0.203167, 1.0, 0., 0., 0.), castor_wheel_radius);
	createWheel("wheel4", wheel4,
        serial4, hinge4,
        Eigen::Displacementd(0.151159, 0.0375, -0.222624, 1.0, 0., 0., 0.), castor_wheel_radius);
	createWheel("wheel5", wheel5,
        serial5, hinge5,
        Eigen::Displacementd(-0.151159, 0.0375, -0.222624, 1.0, 0., 0., 0.), castor_wheel_radius);
	
	std::cout << "Creating simulation: " << timer.GetTime() * .001 << " sec." << std::endl;
	timer.ResetTime();

    /////////////////////////////////////
    //          Set graphics           //
    /////////////////////////////////////
	//On commence a afficher des choses. On peuple le viewer avec les elements du fichier stockes dans les assets.
	xde::builder::GraphicsBuilder graphBuilder(viewer.getSceneComponents(), &assets);

	//load assets
	graphBuilder.loadAssets();

	//Charge les geometries et les materiaux (au sens rendu visuel) dans le viewer
	const std::string& groundId = assets.getIds().getLastId("../../../../share/resources/scenes/ground.dae", "node-ground");
	graphBuilder.createNodeFromTree("ground", "", groundId, groundId, false);

	const std::string& vehicule_coll_id = assets.getIds().getLastId("chassis_desc", "chassis.prim.visu");
	graphBuilder.createNodeFromTree("chassis.prim.visu", "", vehicule_coll_id, vehicule_coll_id, false);

	const std::string& wheel0_coll_id = assets.getIds().getLastId("../../../../share/resources/scenes/sphere.dae", "node");
	graphBuilder.createNodeFromTree("wheel0", "", wheel0_coll_id, wheel0_coll_id, false);

	const std::string& wheel1_coll_id = assets.getIds().getLastId("../../../../share/resources/scenes/sphere.dae", "node");
	graphBuilder.createNodeFromTree("wheel1", "", wheel1_coll_id, wheel1_coll_id, false);
	
	const std::string& wheel2_coll_id = assets.getIds().getLastId("../../../../share/resources/scenes/sphere.dae", "node");
	graphBuilder.createNodeFromTree("wheel2", "", wheel2_coll_id, wheel2_coll_id, false);
	
	const std::string& wheel3_coll_id = assets.getIds().getLastId("../../../../share/resources/scenes/sphere.dae", "node");
	graphBuilder.createNodeFromTree("wheel3", "", wheel3_coll_id, wheel3_coll_id, false);
	
	const std::string& wheel4_coll_id = assets.getIds().getLastId("../../../../share/resources/scenes/sphere.dae", "node");
	graphBuilder.createNodeFromTree("wheel4", "", wheel4_coll_id, wheel4_coll_id, false);
	
	const std::string& wheel5_coll_id = assets.getIds().getLastId("../../../../share/resources/scenes/sphere.dae", "node");
	graphBuilder.createNodeFromTree("wheel5", "", wheel5_coll_id, wheel5_coll_id, false);

	// charge chassis(Adept Lynx_InstanceRep_merged0.1)
	//xde::desc::core::Id rootId;
	//rootId.set_id_str("Adept Lynx");
	//graphBuilder.createNodeFromTree("vehicle", "", rootId, rootId, true);
	
	//Un peu de coloriage
	si.setNodeMaterial("chassis.prim.visu", "xde/OrangeOpaque");
	si.showNode("chassis.prim.visu");

	//si.setNodeMaterial("wheel1.prim.visu", "xde/OrangeOpaque");
	//si.showNode("wheel1.prim.visu");
	si.setNodeMaterial("wheel0", "xde/OrangeOpaque");
	si.showNode("wheel0");
	si.setNodeMaterial("wheel1", "xde/OrangeOpaque");
	si.showNode("wheel1");
	si.setNodeMaterial("wheel2", "xde/OrangeOpaque");
	si.showNode("wheel2");
	si.setNodeMaterial("wheel3", "xde/OrangeOpaque");
	si.showNode("wheel3");
	si.setNodeMaterial("wheel4", "xde/OrangeOpaque");
	si.showNode("wheel4");
	si.setNodeMaterial("wheel5", "xde/OrangeOpaque");
	si.showNode("wheel5");


    /////////////////////////////////////
    //          Set friction law       //
    /////////////////////////////////////
    xde::gvm::ContactLawDesc steelRubberLaw(xde::gvm::ContactLaw::Coulomb, 0.5);
    xde::gvm::ContactLawDesc steelSteelLaw(xde::gvm::ContactLaw::Coulomb, 0.01);
    mechanicalScene.setContactLawForMaterialPair("steel", "steel",
        steelSteelLaw);
    mechanicalScene.setContactLawForMaterialPair("steel", "rubber",
        steelRubberLaw);


}

//---------------------------------------------
// Create Ground
//---------------------------------------------
void RobotCreator::createGround()
{
	// Create Ground
	ground = xde::gvm::RigidBodyRef::createObject("ground");
	mechanicalScene.addFixedRigidBody(ground);
	ground.setPosition(
		Eigen::Displacementd(
			Eigen::Vector3d::Identity(),Eigen::AngleAxisd(0.0 * M_PI, Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(0.0 * M_PI, Eigen::Vector3d::UnitX())));

	ground.setMass(0.0); // ONLY fixed object can have a null mass.
	ground.setContactMaterial("steel");

	/////////////////////////////////////
	//     Create collision objects    //
	/////////////////////////////////////
	//ground collision
	//create ground composite collision mesh
	xde::xcd::CompositeRef groundComposite = lmdScene.createRigidComposite(
		"groundComposite");

	//create ground collision mesh
	xde::xcd::TriMeshRef groundTrimesh = groundComposite.createTriMesh(
		"groundTrimesh");

	//parse geometry from wavefront obj file
	xde::xgl::OBJReaderApi::readFromOBJ(groundTrimesh.getSimplicialComplex(),
	    "../../../../share/resources/scenes/plane.obj", 5., false);

	//set contact regularization offset
	groundTrimesh.setOffset(1e-3);

	//add mesh to composite mesh
	groundComposite.addTriMesh(groundTrimesh);

	//build precomputed collision detection data
	groundComposite.updatePreComputedData();

	//attach composite collision mesh to mechanical body
	ground.setComposite(groundComposite);

	/////////////////////////////////////
	//     Create graphical objects    //
	/////////////////////////////////////
	//Recuperation du contenu du fichier, sous forme de description protobuf,
	//organisees dans des dictionnaires pour acces rapide.
	assets.importOpenColladaFile("../../../../share/resources/scenes/ground.dae", Eigen::Vector3d::Ones());

}

//---------------------------------------------
// Create Chassis
//---------------------------------------------
void RobotCreator::createChassis()
{
	//Create chassis
	double chassisSizeX = 0.500;
	double chassisSizeY = 0.697;
	double chassisSizeZ = 0.372;

    chassis = xde::gvm::RigidBodyRef::createObject("chassis");
    mechanicalScene.addFreeRigidBody(chassis);

    chassis.setPosition(Eigen::Displacementd(0., 0., 0.8, 1., 0., 0., 0.));	// Displacementd(x, y, z, qw, qx, qy, qz)
    chassis.setMass(10.0);
    chassis.setPrincipalInertiaFrame(Eigen::Displacementd::Identity());
    chassis.setPrincipalMomentsOfInertia(Eigen::Vector3d(
		1.0 / 12.0 * chassis.getMass() * (chassisSizeY*chassisSizeY + chassisSizeZ*chassisSizeZ),
		1.0 / 12.0 * chassis.getMass() * (chassisSizeX*chassisSizeX + chassisSizeZ*chassisSizeZ),
		1.0 / 12.0 * chassis.getMass() * (chassisSizeX*chassisSizeX + chassisSizeY*chassisSizeY)));
	chassis.setContactMaterial("rubber");

	/////////////////////////////////////
	//     Create collision objects    //
	/////////////////////////////////////
/*
	//creation des geometries
	xde::xgl::TriMeshRef chassis_prim = GeometricObjects::createBoxGeometry("chassis.prim", 
		chassis.getPosition().getTranslation(), Eigen::Vector3d(chassisSizeX/2,chassisSizeY/2,chassisSizeZ/2));

	//creation de mesh de collision
	xde::xcd::CompositeRef chassis_comp = lmdScene.createRigidComposite("chassis.comp");
	xde::xcd::TriMeshRef chassis_mesh = chassis_comp.createTriMesh("chassis.mesh");
	chassis_mesh.copy(chassis_prim);

    //set contact regularization offset
    chassis_mesh.setOffset(1e-3);

	//ajout du mesh au composite
	chassis_comp.addTriMesh(chassis_mesh);

    //build precomputed collision detection data
    chassis_comp.updatePreComputedData();

    //attach composite collision mesh to mechanical body
    chassis.setComposite(chassis_comp);

    //enable collision
    mechanicalScene.enableContactForBodyPair(ground, chassis, true);
*/
	/////////////////////////////////////
	//     Create graphical objects    //
	/////////////////////////////////////
	//std::cout << "Create chassis: " << " Graph." << std::endl;
	//create visu pour le mesh de collision
	xde::desc::core::Mesh* box_mesh = GeometricObjects::createBoxGeometryDesc("chassis.prim.visu.ref", 
		chassis.getPosition().getTranslation(), Eigen::Vector3d(chassisSizeX/2,chassisSizeY/2,chassisSizeZ/2));
	xde::desc::graphic::GraphicalScene* box_mesh_sc = new xde::desc::graphic::GraphicalScene();
	xde::desc::graphic::GraphicalNode* box_mesh_node = box_mesh_sc->mutable_root_node();
	box_mesh_node->set_name("chassis.prim.visu");
	box_mesh_node->mutable_id()->set_id_str("chassis.prim.visu");
	box_mesh_node->add_position(0.);
	box_mesh_node->add_position(0.);
	box_mesh_node->add_position(0.);
	box_mesh_node->add_position(1.);
	box_mesh_node->add_position(0.);
	box_mesh_node->add_position(0.);
	box_mesh_node->add_position(0.);
	box_mesh_node->add_scale(1.);
	box_mesh_node->add_scale(1.);
	box_mesh_node->add_scale(1.);

	xde::desc::graphic::GraphicalMesh* box_mesh_instance = box_mesh_node->add_meshes();
	box_mesh_instance->set_name("chassis.prim.visu.i");
	box_mesh_instance->mutable_id()->set_id_str("chassis.prim.visu.i");
	box_mesh_instance->mutable_mesh_id()->set_id_str("chassis.prim.visu.ref");

	xde::desc::core::Library* box_mesh_lib = new xde::desc::core::Library();
	xde::desc::core::Geometry* box_geom = box_mesh_lib->add_geometries();
	box_geom->set_name("chassis.prim.visu.ref");
	box_geom->mutable_id()->set_id_str("chassis.prim.visu.ref");
	xde::desc::core::Mesh* box_geom_mesh = box_geom->add_meshes();
	box_geom_mesh->CopyFrom(*box_mesh);
	//std::cout<<"D "<<box_geom_mesh->name()<<std::endl;
	assets.importGraphicalNode("chassis_desc", *box_mesh_node,*box_mesh_lib,Eigen::Vector3d::Ones());
	/*
	Recuperation du contenu du fichier 3dxml, sous forme de description protobuf, organisees dans des dictionnaires pour
	acces rapide.
	*/
	//assets.parse3dxmlFile("../../../../share/resources/scenes/Adept_Lynx_simple.3dxml", Eigen::Vector3d::Constant(.001));

}

//---------------------------------------------
// Create Wheel
//---------------------------------------------
void RobotCreator::createWheel(	const std::string name, 
								xde::gvm::RigidBodyRef& wheel,
								xde::gvm::SerialJointRef& serial, 
								xde::gvm::HingeJointRef& hinge,
								Eigen::Displacementd pos,
								double radius)
{
	// create wheel
    wheel = xde::gvm::RigidBodyRef::createObject(name);
	
	// serial joint allows to introduce multiple joints between two rigid bodies without having to create intermediate bodies
    std::stringstream ss;
    ss << name << "serial";
	serial = xde::gvm::SerialJointRef::createObject(ss.str());
    serial.setNbJoints(2);

    ss.str("");
    ss << name << "hinge";
    hinge = xde::gvm::HingeJointRef::createObject(ss.str());

    ss.str("");
    ss << name << "prisma";
    xde::gvm::PrismaticJointRef prisma = xde::gvm::PrismaticJointRef::createObject(ss.str());

    serial.setDgmJoint(0, prisma);
    serial.setDgmJoint(1, hinge);
    serial.init();

    Eigen::Vector3d axis(0., 0., 1.);
    Eigen::Vector3d center = pos.getTranslation();

    prisma.configure(pos, Eigen::Vector3d::UnitY(), 0.);
    prisma.setDesiredJointVelocity(0.);
    prisma.setDesiredJointPosition(0.);
    prisma.setJointPDGains(1e5, 1e4);
    prisma.enableJointPDCoupling();

    hinge.configure(pos, center, axis, 0.);
    hinge.setJointPDGains(0., 1.e2);	// we do not control the wheel position, only its velocity
    hinge.setDesiredJointVelocity(0.);
    hinge.enableJointPDCoupling();

    mechanicalScene.addRigidBody(chassis, wheel, serial);

	/////////////////////////////////////
	//     Create collision objects    //
	/////////////////////////////////////
	//create geometries
	xde::xgl::TriMeshRef wheel_prim = GeometricObjects::createImplicitSphere("wheel.prim", 
		wheel.getPosition().getTranslation(), radius);

    //create wheel composite collision mesh
    ss.str("");
    ss << name << "wheel.comp";
    xde::xcd::CompositeRef wheel_comp = lmdScene.createRigidComposite(ss.str());

    //create collision mesh
    ss.str("");
    ss << name << "wheel.mesh";
    xde::xcd::TriMeshRef wheel_mesh = wheel_comp.createTriMesh(ss.str());
	wheel_mesh.copy(wheel_prim);

    //set contact regularization offset
    wheel_mesh.setOffset(1e-3);

    //add mesh to composite mesh
    wheel_comp.addTriMesh(wheel_mesh);

    //build precomputed collision detection data
    wheel_comp.updatePreComputedData();

    //attach composite collision mesh to mechanical body
    wheel.setComposite(wheel_comp);


    wheel.setMass(2.0);
    wheel.computePrincipalFrameAndMomentsOfInertiaUsingCompositeObbAndMass();
    wheel.setContactMaterial("rubber");

    //enable collision
    mechanicalScene.enableContactForBodyPair(ground, wheel, true);

	/////////////////////////////////////
	//     Create graphical objects    //
	/////////////////////////////////////
	//create visu pour le mesh de collision
/*
	xde::desc::core::Mesh* sphere_mesh = GeometricObjects::createBoxGeometryDesc(name+".prim.visu.ref", 
		wheel.getPosition().getTranslation(), Eigen::Vector3d(radius,radius,radius));

	xde::desc::graphic::GraphicalScene* sphere_mesh_sc = new xde::desc::graphic::GraphicalScene();

	xde::desc::graphic::GraphicalNode* sphere_mesh_node = sphere_mesh_sc->mutable_root_node();

	sphere_mesh_node->set_name(name+".prim.visu");
	sphere_mesh_node->mutable_id()->set_id_str(name+".prim.visu");

	std::cout << name+".prim.visu" << std::endl;

	sphere_mesh_node->add_position(0.);
	sphere_mesh_node->add_position(0.);
	sphere_mesh_node->add_position(0.);
	sphere_mesh_node->add_position(1.);
	sphere_mesh_node->add_position(0.);
	sphere_mesh_node->add_position(0.);
	sphere_mesh_node->add_position(0.);
	sphere_mesh_node->add_scale(1.);
	sphere_mesh_node->add_scale(1.);
	sphere_mesh_node->add_scale(1.);

	xde::desc::graphic::GraphicalMesh* sphere_mesh_instance = sphere_mesh_node->add_meshes();
	sphere_mesh_instance->set_name(name+".prim.visu.i");
	sphere_mesh_instance->mutable_id()->set_id_str(name+".prim.visu.i");
	sphere_mesh_instance->mutable_mesh_id()->set_id_str(name+".prim.visu.ref");

	xde::desc::core::Library* sphere_mesh_lib = new xde::desc::core::Library();
	xde::desc::core::Geometry* sphere_geom = sphere_mesh_lib->add_geometries();
	sphere_geom->set_name(name+".prim.visu.ref");
	sphere_geom->mutable_id()->set_id_str(name+".prim.visu.ref");
	xde::desc::core::Mesh* sphere_geom_mesh = sphere_geom->add_meshes();
	sphere_geom_mesh->CopyFrom(*sphere_mesh);
	//std::cout<<"D "<<sphere_geom_mesh->name()<<std::endl;
	assets.importGraphicalNode(*sphere_mesh_node,*sphere_mesh_lib,Eigen::Vector3d::Ones());
*/
	assets.importOpenColladaFile("../../../../share/resources/scenes/sphere.dae", Eigen::Vector3d(radius, radius, radius));
}

//---------------------------------------------
// Scene Update
//---------------------------------------------
void RobotCreator::update()
{
	viewer.update();
	/*
	sm.update();

	vbm.setObservationFrame(ci.getCameraDisplacement(ci.getMainCameraName()));
	vbm.setInputVelocity(sm.getVelocity());
	vbm.preUpdate();
	*/
	mechanicalScene.detectCollisions();
	mechanicalScene.integrate();

	//    vbm.postUpdate();
	//std::cout<<"ground pos "<<groundBody.getPosition()<<std::endl;
	si.setNodePosition("ground", ground.getPosition());
	//si.setNodePosition("vehicle", vehicleBody.getPosition());
	si.setNodePosition("chassis.prim.visu", /*vehicleBody*/chassis.getPosition());
	//std::cout << "Chassis Position: " << chassis.getPosition() << " m." << std::endl;
	si.setNodePosition("wheel0", wheel0.getPosition());
	si.setNodePosition("wheel1", wheel1.getPosition());
	si.setNodePosition("wheel2", wheel2.getPosition());
	si.setNodePosition("wheel3", wheel3.getPosition());
	si.setNodePosition("wheel4", wheel4.getPosition());
	si.setNodePosition("wheel5", wheel5.getPosition());
	xde::sys::Timer::Sleep(10);
}

//---------------------------------------------
// Scene Release
//---------------------------------------------
void RobotCreator::finalize()
{
	//vbm.detach();

	mechanicalScene.removeRigidBody(ground);
	mechanicalScene.removeRigidBody(chassis);
	mechanicalScene.removeRigidBody(wheel0);
	mechanicalScene.removeRigidBody(wheel1);
	mechanicalScene.removeRigidBody(wheel2);
	mechanicalScene.removeRigidBody(wheel3);
	mechanicalScene.removeRigidBody(wheel4);
	mechanicalScene.removeRigidBody(wheel5);

	mechanicalScene.printPerformanceReport(std::cout);
}