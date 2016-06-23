#include "aiv\helpers\AppBuilder.hpp"

#include "aiv/helpers/AIVBuilder.hpp"
#include "aiv/helpers/ObstBuilder.hpp"
#include "aiv/helpers/SceneBuilder.hpp"
#include "xde/physics/gvm/Scene.h"

#include "xde/physics/gvm/ContactLawDesc.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>      // std::ostringstream
#include <iomanip>
#include <locale>

#define foreach_ BOOST_FOREACH

namespace aiv {

Application * ApplicationBuilder::buildEmptyApp(double timeStep)
{
  Application * app = new Application(timeStep);
  return app;
}


Application * ApplicationBuilder::buildSimpleCollisionApp()
{
  Application * app = buildEmptyApp(0.01);
  SceneBuilder builder(app);

  // build ground
  xde::gvm::RigidBodyRef ground;
  ground = builder.addFixedObject("../../../../share/resources/scenes/ground.dae", // dae file path
                                  "ground",  // name
                                  "node-ground",  // nodeName in dae
                                  Eigen::Displacementd::Identity(),  // position
                                  Eigen::Vector3d::Ones(), // scale
                                  0.005);  // offset 
  
  // build knob
  xde::gvm::RigidBodyRef knob;
  knob = builder.addFreeObject("../../../../share/resources/scenes/knob.dae", // dae file path
                               "knob",  // name
                               "knob",  // nodeName in dae
                               Eigen::Displacementd(0., 0., 0.5,  1., 0., 0., 0.),  // position
                               1., // mass
                               true, // enableWeight
                               Eigen::Vector3d::Constant(.2), // scale
                               0.005); // offset

  
  // build knob
  xde::gvm::RigidBodyRef knob2;
  knob2 = builder.addFreeObject("../../../../share/resources/scenes/knob.dae", // dae file path
                               "knob2",  // name
                               "knob",  // nodeName in dae
                               Eigen::Displacementd(0., -1., 0.5,  1., 0., 0., 0.),  // position
                               1., // mass
                               false, // enableWeight
                               Eigen::Vector3d::Constant(.1), // scale
                               0.005); // offset

  // manage collision between them
  app->getGVMScene().enableContactForBodyPair(knob, ground, true);
  app->getGVMScene().enableContactForBodyPair(knob2, ground, true);

  // here is place for scene customization
  app->getGraphicSceneInterface().setNodeMaterial("knob", "xde/OrangeOpaque");

  return app;
}

Application * ApplicationBuilder::buildSimpleAdeptApp()
{

  // --------- Get application property tree from XML configuration file ---------
  using boost::property_tree::ptree;
  ptree pt;
  read_xml("../../../../src/aiv/config.xml", pt);

  // build application
  Application * app = buildEmptyApp(pt.get<double>("root.timestep"));
  SceneBuilder scenebuilder(app);

  app->setSimSpeed(pt.get<double>("root.simspeed"));

  // set contact material law for interaction between ground and wheels
  xde::gvm::ContactLawDesc floorRubberLaw(xde::gvm::ContactLaw::Coulomb, 0.7);
  app->getGVMScene().setContactLawForMaterialPair("floor", "rubber", floorRubberLaw);

  // build ground
  xde::gvm::RigidBodyRef ground;
  ground = scenebuilder.addFixedObject("../../../../share/resources/scenes/ground.dae", // dae file path
                                       "ground",  // name
                                       "node-ground",  // node id in dae
                                       Eigen::Displacementd(0., 0., -0.1,  1., 0., 0. ,0.),  // position
                                       Eigen::Vector3d::Ones(),  // scale
                                       0.); // offset
  ground.setContactMaterial("floor");


  // Add AIVs

  AIVBuilder aivbuilder(app, ground);

  unsigned long long aivId = 0;
  std::string aivBaseName = "AdeptLynx";

  foreach_(ptree::value_type &v, pt.get_child("root.aivs"))
  {
    if ( v.first == "aiv" )
    {
      aivbuilder.addAdeptLynxToApplication( aivBaseName+std::to_string(aivId), pt, v );
      ++aivId;
    }
    else
    {} //TODO error, unkown kind of robot
  }

  // Add Obstacles

  ObstBuilder obstbuilder(app, ground);

  unsigned long long obstId = 0;
  std::string obstBaseName = "Obst";

  foreach_(ptree::value_type &v, pt.get_child("root.obstacles"))
  {
    if ( v.first == "circular" )
    {
      obstbuilder.addCircularObstToApplication( obstBaseName+std::to_string(obstId), v );
      ++obstId;
    }
    else if ( v.first == "polygon" )
    {
      obstbuilder.addPolygonObstToApplication( obstBaseName+std::to_string(obstId), v );
      ++obstId;
    }
    else
    {} //TODO error, unkown kind of robot
  }

  // COLLISIONS
  // typdefs needed for using boost for each or just for simplifying
  typedef std::map<std::string, AIV *> MapAIV;
  typedef std::map<std::string, Obstacle *> MapObst;

  //vehicle-to-vehicle and vehicle-to-obstacle
  // for ( MapAIV::iterator it_v1 = app->vehicles.begin(); it_v1 != app->vehicles.end(); ++it_v1 )
  // {
  // //foreach_ ( MapAIV::value_type& v1, app->vehicles )
  // //{
  //   std::cout << "Vehicle A ----------- " << it_v1->first << std::endl;
  //   for ( MapObst::iterator it_obst = app->obstacles.begin(); it_obst != app->obstacles.end(); ++it_obst )
  //   {
  //   //foreach_ ( MapObst::value_type& obst, app->obstacles )
  //   //{
  //     std::cout << "OBST: "<< it_obst->first << std::endl;
  //     //app->getGVMScene().enableContactForBodyPair(obst.second->object, static_cast<AdeptLynx*>(v1.second)->frame, true);
  //     app->getGVMScene().enableContactForBodyPair(it_obst->second->object, static_cast<AdeptLynx*>(it_v1->second)->frame, true);
  //   }

  //   //MapAIV::iterator it_v1 = (app->vehicles).find(v1.first);
  //   for ( MapAIV::iterator it_v2 = std::next(it_v1); it_v2 != app->vehicles.end(); ++it_v2 )
  //   {
  //     std::cout << "Vehicle B - " << it_v2->first << std::endl;
  //     app->getGVMScene().enableContactForBodyPair(static_cast<AdeptLynx*>(it_v1->second)->frame,
  //         static_cast<AdeptLynx*>(it_v2->second)->frame, true);
  //   }
  // }

  // // obstacle-to-obstacle
  // for ( MapObst::iterator it_obst1 = app->obstacles.begin(); it_obst1 != app->obstacles.end(); ++it_obst1 )
  // {
  //   std::cout << "OBST A - " << it_obst1->first << std::endl;
  //   for ( MapObst::iterator it_obst2 = ++it_obst1; it_obst2 != app->obstacles.end(); ++it_obst2 )
  //   {
  //     std::cout << "OBST B - " << it_obst2->first << std::endl;
  //     app->getGVMScene().enableContactForBodyPair(it_obst1->second->object, it_obst2->second->object, true);
  //   }
  // }

  app->getGraphicSceneInterface().hideNode("ground");

  return app;

}


}


// cmake:sourcegroup=Helpers