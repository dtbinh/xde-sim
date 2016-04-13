#include "aiv\helpers\AIVBuilder.hpp"

#include "sys/timer.h"

#include "GeometricObjects.hpp" // free wheel collision geometric shapes
#include "aiv/robot/AdeptLynx.hpp"
#include "aiv/helpers/SceneBuilder.hpp"
#include "aiv/controller/ControllerMonocycle.hpp"
#include "aiv/pathplanner/PathPlannerRecHor.hpp"
#include "aiv/sensor/Sensor.hpp"

#include "xde/data/Assets.h"
#include "xde/physics/builder/PhysicsBuilder.h"
#include "xde/graphics/builder/GraphicsBuilder.h"


namespace aiv {

	AIVBuilder::AIVBuilder(Application * a, xde::gvm::RigidBodyRef & ground)
		: app(a)
		, ground(ground)
	{
	}

	void AIVBuilder::addAdeptLynxToApplication(
		const std::string & name,
		//    const Eigen::Displacementd & position,
		const boost::property_tree::ptree &pt,
		const boost::property_tree::ptree::value_type &v
		//    ,const std::string & material
		)
	{
		AdeptLynx * lynx = new AdeptLynx(name, app);

		SceneBuilder builder(app);

		Eigen::Displacementd aivInitDisplacement(
			(Eigen::Matrix<double, 3, 1>() <<
			v.second.get<double>("initpose.x"),
			v.second.get<double>("initpose.y"),
			0.2).finished(),                                  // arbitrary translation in z
			Eigen::Quaternion<double>((Eigen::Matrix3d() <<
			cos(v.second.get<double>("initpose.theta")),-sin(v.second.get<double>("initpose.theta")),0,
			sin(v.second.get<double>("initpose.theta")),cos(v.second.get<double>("initpose.theta")),0,
			0,0,1).finished()));

		// build frame
		lynx->frame = builder.addFreeObject(v.second.get<std::string>("frame.cadmodel"), // dae file path
			name + std::string("_frame"),  // name
			"Frame",  // nodeName in dae
			aivInitDisplacement,  // position
			v.second.get<double>("frame.mass"), // mass
			true, // enableWeight
			Eigen::Vector3d::Constant(0.0254),  // scale
			0.005); // offset

		createDriveWheel(name + std::string("_leftDriveWheel"), 
			v.second.get<std::string>("leftdrivewheel.cadmodel"), 
			"DriveWheel",  // nodeName in dae
			Eigen::Displacementd(0., v.second.get<double>("track")/2., 0.04943,  1., 0., 0., 0.),
			v.second.get<double>("leftdrivewheel.radius"), 
			Eigen::Vector3d::Constant(0.0254),   // scales
			0.005, // offset
			v.second.get<double>("leftdrivewheel.mass"), // mass
			false,  // enableWeight
			Eigen::Vector3d::UnitY(),   // axis
			lynx->frame,
			lynx->leftDriveWheel,
			lynx->leftDriveSerial,
			lynx->leftDrivePrisma,
			lynx->leftDriveHinge);

		createDriveWheel(name + std::string("_rightDriveWheel"), 
			v.second.get<std::string>("rightdrivewheel.cadmodel"),
			"DriveWheel",  // nodeName in dae
			Eigen::Displacementd(0.,v.second.get<double>("track")/-2., 0.04943,  1., 0., 0., 0.),
			v.second.get<double>("rightdrivewheel.radius"), 
			Eigen::Vector3d::Constant(0.0254),   // scales
			0.005, // offset
			v.second.get<double>("rightdrivewheel.mass"), // mass
			false,  // enableWeight
			Eigen::Vector3d::UnitY(),   // axis
			lynx->frame, 
			lynx->rightDriveWheel,
			lynx->rightDriveSerial,
			lynx->rightDrivePrisma,
			lynx->rightDriveHinge);



		// free ball joints (should be replaced by caster wheels)
		createFreeBallWheel(name + std::string("_leftFrontFreeWheel"), 
			v.second.get<std::string>("leftfrontfreewheel.cadmodel"), 
			"FreeWheel",  // nodeName in dae
			Eigen::Displacementd(0.203, 0.102, -0.00992,  1., 0., 0., 0.),
			v.second.get<double>("leftfrontfreewheel.radius"),
			Eigen::Vector3d::Constant(0.0254),   // scales
			0.005, // offset
			v.second.get<double>("leftfrontfreewheel.mass"), // mass
			false,  // enableWeight
			lynx->frame, 
			lynx->leftFrontFreeWheel,
			lynx->leftFrontPrisma,
			lynx->leftFrontBallJoint);


		createFreeBallWheel(name + std::string("_leftBackFreeWheel"), 
			v.second.get<std::string>("leftbackfreewheel.cadmodel"), 
			"FreeWheel",  // nodeName in dae
			Eigen::Displacementd(-0.223, 0.151, -0.00992,  1., 0., 0., 0.),
			v.second.get<double>("leftbackfreewheel.radius"),
			Eigen::Vector3d::Constant(0.0254),   // scales
			0.005, // offset
			v.second.get<double>("leftbackfreewheel.mass"),
			false,  // enableWeight
			lynx->frame, 
			lynx->leftBackFreeWheel,
			lynx->leftBackPrisma,
			lynx->leftBackBallJoint);

		createFreeBallWheel(name + std::string("_rightFrontFreeWheel"), 
			v.second.get<std::string>("rightfrontfreewheel.cadmodel"),
			"FreeWheel",  // nodeName in dae
			Eigen::Displacementd( 0.203, -0.102, -0.00992,  1., 0., 0., 0.),
			v.second.get<double>("rightfrontfreewheel.radius"),
			Eigen::Vector3d::Constant(0.0254),   // scales
			0.005, // offset
			v.second.get<double>("rightfrontfreewheel.mass"),
			false,  // enableWeight
			lynx->frame, 
			lynx->rightFrontFreeWheel,
			lynx->rightFrontPrisma,
			lynx->rightFrontBallJoint);

		createFreeBallWheel(name + std::string("_rightBackFreeWheel"), 
			v.second.get<std::string>("rightbackfreewheel.cadmodel"),
			"FreeWheel",  // nodeName in dae
			Eigen::Displacementd(-0.223, -0.151, -0.00992,  1., 0., 0., 0.),
			v.second.get<double>("rightbackfreewheel.radius"),
			Eigen::Vector3d::Constant(0.0254),   // scales
			0.005, // offset
			v.second.get<double>("rightbackfreewheel.mass"), // mass
			false,  // enableWeight
			lynx->frame, 
			lynx->rightBackFreeWheel,
			lynx->rightBackPrisma,
			lynx->rightBackBallJoint);

		// change colors
		app->getGraphicSceneInterface().setNodeMaterial(name + std::string("_frame"), v.second.get<std::string>("frame.color"));
		app->getGraphicSceneInterface().setNodeMaterial(name + std::string("_leftDriveWheel"), v.second.get<std::string>("leftdrivewheel.color"));
		app->getGraphicSceneInterface().setNodeMaterial(name + std::string("_rightDriveWheel"), v.second.get<std::string>("rightdrivewheel.color"));
		app->getGraphicSceneInterface().setNodeMaterial(name + std::string("_leftFrontFreeWheel"), v.second.get<std::string>("leftfrontfreewheel.color"));
		app->getGraphicSceneInterface().setNodeMaterial(name + std::string("_leftBackFreeWheel"), v.second.get<std::string>("leftbackfreewheel.color"));
		app->getGraphicSceneInterface().setNodeMaterial(name + std::string("_rightFrontFreeWheel"), v.second.get<std::string>("rightfrontfreewheel.color"));
		app->getGraphicSceneInterface().setNodeMaterial(name + std::string("_rightBackFreeWheel"), v.second.get<std::string>("rightbackfreewheel.color"));

		// change contact material
		lynx->leftDriveWheel.setContactMaterial(v.second.get<std::string>("leftdrivewheel.material"));
		lynx->rightDriveWheel.setContactMaterial(v.second.get<std::string>("rightdrivewheel.material"));
		lynx->leftFrontFreeWheel.setContactMaterial(v.second.get<std::string>("leftfrontfreewheel.material"));
		lynx->leftBackFreeWheel.setContactMaterial(v.second.get<std::string>("leftbackfreewheel.material"));
		lynx->rightFrontFreeWheel.setContactMaterial(v.second.get<std::string>("rightfrontfreewheel.material"));
		lynx->rightBackFreeWheel.setContactMaterial(v.second.get<std::string>("rightbackfreewheel.material"));

		// manage collision with ground
		app->getGVMScene().enableContactForBodyPair(lynx->leftDriveWheel, ground, true);
		app->getGVMScene().enableContactForBodyPair(lynx->rightDriveWheel, ground, true);
		app->getGVMScene().enableContactForBodyPair(lynx->leftFrontFreeWheel, ground, true);
		app->getGVMScene().enableContactForBodyPair(lynx->leftBackFreeWheel, ground, true);
		app->getGVMScene().enableContactForBodyPair(lynx->rightFrontFreeWheel, ground, true);
		app->getGVMScene().enableContactForBodyPair(lynx->rightBackFreeWheel, ground, true);
		app->getGVMScene().enableContactForBodyPair(lynx->frame, ground, true);

		//-------------- Motor --------------
		lynx->leftDriveHinge.setJointPDGains(0., 10.);
		lynx->leftDriveHinge.setDesiredJointVelocity(0.);
		lynx->leftDriveHinge.enableJointPDCoupling();
		lynx->rightDriveHinge.setJointPDGains(0., 10.);
		lynx->rightDriveHinge.setDesiredJointVelocity(0.);
		lynx->rightDriveHinge.enableJointPDCoupling();

		//-------------- Sensor --------------
		std::cout << "======  Create Path Planner [ " << name +
			std::string("_Sensor") << " ] ================" << std::endl;

		lynx->sensor = new Sensor(name +
			std::string("_Sensor"), v.second.get<double>("detectionradius"));

		//-------------- Path Planner --------------
		if ( pt.get<std::string>("root.mpmethod.<xmlattr>.type") == "receding horizon with termination" )
		{
			std::cout << "======  Create Path Planner [ " << name +
				std::string("_PathPlannerRecHor") << " ] ================" << std::endl;
			// PathPlannerRecHor * pplanner = new   PathPlannerRecHor(name +
			//    std::string(" PathPlannerRecHor"), app->getTimeStep());
			PathPlannerRecHor * pplanner = new   PathPlannerRecHor(name +
				std::string("_PathPlannerRecHor"), app->getTimeStep());

			lynx->planner = pplanner;

			// This is needed in order to make the planner work (it gives initial and target poses etc)
			pplanner->init(
				(Eigen::Vector3d() <<
				v.second.get<double>("initpose.x"),
				v.second.get<double>("initpose.y"),
				v.second.get<double>("initpose.theta")).finished(),
				(Eigen::Vector2d() <<
				v.second.get<double>("initvelo.linear"),
				v.second.get<double>("initvelo.angular")).finished(),
				(Eigen::Vector3d() <<
				v.second.get<double>("goalpose.x"),
				v.second.get<double>("goalpose.y"),
				v.second.get<double>("goalpose.theta")).finished(),
				(Eigen::Vector2d() <<
				v.second.get<double>("goalvelo.linear"),
				v.second.get<double>("goalvelo.angular")).finished(),
				(Eigen::Vector2d() <<
				v.second.get<double>("maxvelo.linear"),
				v.second.get<double>("maxvelo.angular")).finished(),
				(Eigen::Vector2d() <<
				v.second.get<double>("maxacc.linear"),
				v.second.get<double>("maxacc.angular")).finished(),
				pt.get<double>("root.mpmethod.comphorizon"),
				pt.get<double>("root.mpmethod.planninghorizon"),
				pt.get<unsigned short>("root.mpmethod.sampling"),
				pt.get<unsigned short>("root.mpmethod.interknots"));

			// This is needed only if we don't want to use the default values (it's like "advanced options")
			//std::cout << "offsetTime " << pt.get<double>("root.timeoffset") << std::endl;
			pplanner->setOption("offsetTime", pt.get<double>("root.timeoffset"));

			pplanner->setOption("lastStepMinDist", pt.get<double>("root.mpmethod.terminationdist"));
			pplanner->setOption("conflictFreePathDeviation", pt.get<double>("root.mpmethod.conflictfreepathdeviation"));
			pplanner->setOption("interRobotSafetyDist", pt.get<double>("root.mpmethod.interrobotsafetydist"));
			pplanner->setOption("waitPlanning", pt.get<bool>("root.mpmethod.waitplanning"));
			pplanner->setOption("numderivativedelta", pt.get<double>("root.mpmethod.numderivativedelta"));

			pplanner->setOption("optimizerType", pt.get<std::string>("root.mpmethod.optimizer.<xmlattr>.type"));
			pplanner->setOption("xTol", pt.get<double>("root.mpmethod.optimizer.xtolerance"));
			pplanner->setOption("eqTol", pt.get<double>("root.mpmethod.optimizer.eqtolerance"));
			pplanner->setOption("ineqTol", pt.get<double>("root.mpmethod.optimizer.ineqtolerance"));
			pplanner->setOption("fTol", pt.get<double>("root.mpmethod.optimizer.ftolerance"));
			pplanner->setOption("lastMaxIteration", pt.get<unsigned>("root.mpmethod.optimizer.maxiteraction.last"));
			pplanner->setOption("firstMaxIteration", pt.get<unsigned>("root.mpmethod.optimizer.maxiteraction.first"));
			pplanner->setOption("interMaxIteration", pt.get<unsigned>("root.mpmethod.optimizer.maxiteraction.inter"));
		}
		else
		{
			std::cout << "======  Unknown path planning method [ " << pt.get<std::string>("root.mpmethod.<xmlattr>.type") << " ] ================" << std::endl;
		}

		//-------------- Controller --------------
		std::cout << "======  Create Controller [ " << name +
			std::string("_ControllerMonocycle") << " ] ================" << std::endl;

		ControllerMonocycle * ctrller = new ControllerMonocycle(name + std::string("_ControllerMonocycle"),
			app->getTimeStep(),
			pt.get<double>("root.controller.threshold.u1"),
			pt.get<double>("root.controller.threshold.u2"));

		lynx->ctrler = ctrller;

		ctrller->setOption("offsetTime", pt.get<double>("root.timeoffset"));
		ctrller->setOption("ctrllerType", pt.get<std::string>("root.controller.<xmlattr>.type"));

		if ( pt.get<std::string>("root.controller.<xmlattr>.type") == "TRP" )
		{
			ctrller->setOption("k1", pt.get<double>("root.controller.k1"));
			ctrller->setOption("k2", pt.get<double>("root.controller.k2"));
			ctrller->setOption("k3", pt.get<double>("root.controller.k3"));
			ctrller->setOption("dynModelParam",
				(Eigen::Matrix<double, 6, 1>() <<
				v.second.get<double>("dynmodelparam.p1"),
				v.second.get<double>("dynmodelparam.p2"),
				v.second.get<double>("dynmodelparam.p3"),
				v.second.get<double>("dynmodelparam.p4"),
				v.second.get<double>("dynmodelparam.p5"),
				v.second.get<double>("dynmodelparam.p6")).finished());
		}
		else if ( pt.get<std::string>("root.controller.<xmlattr>.type") == "NCGPCKM" )
		{
			ctrller->setOption("predictionHorizon", pt.get<double>("root.controller.predictionhorizon"));
		}
		else if ( pt.get<std::string>("root.controller.<xmlattr>.type") == "NCGPCCM" )
		{
			ctrller->setOption("predictionHorizon", pt.get<double>("root.controller.predictionhorizon"));
			ctrller->setOption("dynModelParam",
				(Eigen::Matrix<double, 6, 1>() <<
				v.second.get<double>("dynmodelparam.p1"),
				v.second.get<double>("dynmodelparam.p2"),
				v.second.get<double>("dynmodelparam.p3"),
				v.second.get<double>("dynmodelparam.p4"),
				v.second.get<double>("dynmodelparam.p5"),
				v.second.get<double>("dynmodelparam.p6")).finished());
		}
		else
		{
			std::cout << "======  Unknown controller type [ " << pt.get<std::string>("root.controller.<xmlattr>.type") << " ] ================" << std::endl;
		}

		app->addVehicle(lynx);
	}


	void AIVBuilder::createDriveWheel(
		const std::string & name, 
		const std::string & daeFilePath, 
		const std::string & nodeName, 
		const Eigen::Displacementd & position,
		double radius,
		const Eigen::Vector3d & scales,
		double offset,
		double mass, 
		bool enableWeight,
		const Eigen::Vector3d & axis,
		const xde::gvm::RigidBodyRef & frame, 
		xde::gvm::RigidBodyRef & wheel,
		xde::gvm::SerialJointRef & serial,
		xde::gvm::PrismaticJointRef & prisma,
		xde::gvm::HingeJointRef & hinge)
	{
		std::cout << "======  Create Drive Wheel [ " << name << " ] ================" << std::endl;

		xde::sys::Timer timer;

		// Parse Geometry
		timer.ResetTime();
		xde::data::Assets assets;
		assets.importOpenColladaFile(daeFilePath, scales);
		std::cout << "Parsing data: " << timer.GetTime() * .001 << " sec." << std::endl;

		// Graphics
		timer.ResetTime();
		xde::builder::GraphicsBuilder graphBuilder(app->getViewer().getSceneComponents(), &assets);
		graphBuilder.loadAssets();
		const std::string& objId = assets.getIds().getLastId(daeFilePath, nodeName);
		graphBuilder.createNodeFromTree(name, "", objId, objId, false);
		std::cout << "Building graphics: " << timer.GetTime() * .001 << " sec." << std::endl;

		//// Collision
		//timer.ResetTime();
		//xde::builder::PhysicsBuilder phyBuilder(app->getXCDScene(), &assets);
		//xde::xcd::CompositeRef composite = phyBuilder.createCompositeFromTree(name + std::string(".comp"), objId, objId, offset, 1., false, false);
		//std::cout << "Building collision: " << timer.GetTime() * .001 << " sec." << std::endl;

		// Collision
		//create geometries of collision
		timer.ResetTime();
		Eigen::Vector3d point1(0.000, -0.005, 0.000);
		Eigen::Vector3d point2(0.000,  0.005, 0.000);

		xde::xgl::TriMeshRef wheel_prim = GeometricObjects::createImplicitCapsule(name + std::string(".prim"), 
			point1, point2, radius);
		//create wheel composite collision mesh
		xde::xcd::CompositeRef wheel_comp = app->getXCDScene().createRigidComposite(name + std::string(".comp"));
		//create collision mesh
		xde::xcd::TriMeshRef wheel_mesh = wheel_comp.createTriMesh(name + std::string(".mesh"));
		wheel_mesh.copy(wheel_prim);
		//set contact regularization offset
		wheel_mesh.setOffset(offset);
		//add mesh to composite mesh
		wheel_comp.addTriMesh(wheel_mesh);
		//build precomputed collision detection data
		wheel_comp.updatePreComputedData();

		// GVM RigidBody
		timer.ResetTime();
		wheel = xde::gvm::RigidBodyRef::createObject(name);
		wheel.setComposite(wheel_comp);
		wheel.setMass(mass);
		wheel.computePrincipalFrameAndMomentsOfInertiaUsingCompositeObbAndMass();
		if(enableWeight)
			wheel.enableWeight();
		else
			wheel.disableWeight();

		// GVM Serial prisma & hinge Joints
		serial = xde::gvm::SerialJointRef::createObject(name + std::string(".joint"));
		app->getGVMScene().addRigidBody(frame, wheel, serial);
		serial.setNbJoints(2);
		hinge = xde::gvm::HingeJointRef::createObject(name + std::string(".hingejoint"));
		prisma = xde::gvm::PrismaticJointRef::createObject(name + std::string(".prismajoint"));
		serial.setDgmJoint(0, prisma);
		serial.setDgmJoint(1, hinge);
		serial.init();
		prisma.configure(position, Eigen::Vector3d::UnitZ(), 0.);
		Eigen::Displacementd hingePosition(0., 0., 0.0,  1., 0., 0., 0.);
		hinge.configure(hingePosition, hingePosition.getTranslation(), axis, 0.);

		// Shock absorber characteristics
		prisma.setDesiredJointVelocity(0.);
		prisma.setDesiredJointPosition(0.);
		prisma.setJointPDGains(1e5, 1e4);
		prisma.enableJointPDCoupling();

		std::cout << "Building physics: " << timer.GetTime() * .001 << " sec." << std::endl;
	}


	void AIVBuilder::createFreeBallWheel(const std::string & name, 
		const std::string & daeFilePath, 
		const std::string & nodeName, 
		const Eigen::Displacementd & position,
		double radius,
		const Eigen::Vector3d & scales,
		double offset,
		double mass, 
		bool enableWeight,
		const xde::gvm::RigidBodyRef & frame, 
		xde::gvm::RigidBodyRef & wheel,
		xde::gvm::PrismaticJointRef & prisma,
		xde::gvm::BallJointRef & balljoint)
	{
		std::cout << "======  Create Free (ball-joint) Wheel [ " << name << " ] ================" << std::endl;

		xde::sys::Timer timer;

		// Parse Geometry
		timer.ResetTime();
		xde::data::Assets assets;
		assets.importOpenColladaFile(daeFilePath, scales);
		std::cout << "Parsing data: " << timer.GetTime() * .001 << " sec." << std::endl;

		// Graphics
		timer.ResetTime();
		xde::builder::GraphicsBuilder graphBuilder(app->getViewer().getSceneComponents(), &assets);
		graphBuilder.loadAssets();
		const std::string& objId = assets.getIds().getLastId(daeFilePath, nodeName);
		graphBuilder.createNodeFromTree(name, "", objId, objId, false);
		std::cout << "Building graphics: " << timer.GetTime() * .001 << " sec." << std::endl;

		//// Collision
		//timer.ResetTime();
		//xde::builder::PhysicsBuilder phyBuilder(app->getXCDScene(), &assets);
		//xde::xcd::CompositeRef composite = phyBuilder.createCompositeFromTree(name + std::string(".comp"), objId, objId, offset, 1., false, false);
		//std::cout << "Building collision: " << timer.GetTime() * .001 << " sec." << std::endl;

		// Collision
		//create geometries of collision
		timer.ResetTime();
		Eigen::Vector3d center(0.000, 0.000, 0.000);

		xde::xgl::TriMeshRef wheel_prim = GeometricObjects::createImplicitSphere(name + std::string(".prim"), 
			center/*position.getTranslation()*/, radius);
		//create wheel composite collision mesh
		xde::xcd::CompositeRef wheel_comp = app->getXCDScene().createRigidComposite(name + std::string(".comp"));
		//create collision mesh
		xde::xcd::TriMeshRef wheel_mesh = wheel_comp.createTriMesh(name + std::string(".mesh"));
		wheel_mesh.copy(wheel_prim);
		//set contact regularization offset
		wheel_mesh.setOffset(offset);
		//add mesh to composite mesh
		wheel_comp.addTriMesh(wheel_mesh);
		//build precomputed collision detection data
		wheel_comp.updatePreComputedData();

		// GVM Wheel RigidBody
		timer.ResetTime();
		wheel = xde::gvm::RigidBodyRef::createObject(name);
		wheel.setComposite(wheel_comp);
		wheel.setMass(mass);
		wheel.computePrincipalFrameAndMomentsOfInertiaUsingCompositeObbAndMass();
		if(enableWeight)
			wheel.enableWeight();
		else
			wheel.disableWeight();

		// GVM Phantom RigidBody
		xde::gvm::RigidBodyRef phantomBody = xde::gvm::RigidBodyRef::createObject(name + std::string(".phantomBody"));
		phantomBody.setMass(mass/100);
		if(enableWeight)
			phantomBody.enableWeight();
		else
			phantomBody.disableWeight();

		// GVM Prisma Joint
		prisma = xde::gvm::PrismaticJointRef::createObject(name + std::string(".prismaJoint"));
		app->getGVMScene().addRigidBody(frame, phantomBody, prisma);
		prisma.configure(position, Eigen::Vector3d::UnitZ(), 0.);
		prisma.setDesiredJointVelocity(0.);	// Shock absorber characteristics
		prisma.setDesiredJointPosition(0.);
		prisma.setJointPDGains(1e5, 1e4);
		prisma.enableJointPDCoupling();

		// GVM Ball Joint
		balljoint = xde::gvm::BallJointRef::createObject(name + std::string(".ballJoint"));
		app->getGVMScene().addRigidBody(phantomBody, wheel, balljoint);
		Eigen::Displacementd balljointPosition(0., 0., 0.0,  1., 0., 0., 0.);
		balljoint.configure(balljointPosition.getTranslation(), balljointPosition);

		std::cout << "Building physics: " << timer.GetTime() * .001 << " sec." << std::endl;
	}

}


// cmake:sourcegroup=Helpers