#include "aiv/helpers/ObstBuilder.hpp"
#include "aiv/helpers/SceneBuilder.hpp"
#include <boost/foreach.hpp>

#define foreach_ BOOST_FOREACH

namespace aiv {

ObstBuilder::ObstBuilder(Application *app, xde::gvm::RigidBodyRef &ground)
  : app(app)
  , ground(ground)
{
}

void ObstBuilder::addCircularObstToApplication(const std::string & name,
                                 const boost::property_tree::ptree::value_type &v)
{
  CircularObstacle * obst = new CircularObstacle(name, app);

  SceneBuilder sceneBuilder(app);

  Eigen::Displacementd obstInitDisplacements(
      v.second.get<double>("cmposition.x"),
      v.second.get<double>("cmposition.y"),
      v.second.get<double>("cmposition.z")+0.2,  // arbitrary translation in z
      1., 0., 0., 0.); // circular obstacle is orientation invariant

  obst->object = sceneBuilder.addFreeObject(v.second.get<std::string>("cadmodel"), // dae file path
                                      name + std::string("_circularObst"),  // name
                                      "CircularObst",  // node id in dae
                                      //"Frame",  // node id in dae
                                      obstInitDisplacements,  // position
                                      v.second.get<double>("mass"), // mass
                                      true, // enableWeight
                                      v.second.get<double>("radius")*Eigen::Vector3d::Constant(0.0254),  // scale
                                      0.005); // offset
  
  obst->radius = v.second.get<double>("radius");

  app->getGraphicSceneInterface().setNodeMaterial(name + std::string("_circularObst"), v.second.get<std::string>("color"));

  obst->object.setContactMaterial(v.second.get<std::string>("material"));

  app->getGVMScene().enableContactForBodyPair(obst->object, ground, true);

  app->addObstacle(obst);
}

void ObstBuilder::addPolygonObstToApplication(const std::string & name,
                                 boost::property_tree::ptree::value_type &v)
{
  PolygonObstacle * obst = new PolygonObstacle(name, app);

  SceneBuilder sceneBuilder(app);

  Eigen::Displacementd obstInitDisplacements(
      (Eigen::Matrix<double, 3, 1>() <<
      v.second.get<double>("cmposition.x"),
      v.second.get<double>("cmposition.y"),
      v.second.get<double>("cmposition.z")+0.2).finished(),  // arbitrary translation in z
      Eigen::Quaternion<double>((Eigen::Matrix3d() <<
      cos(v.second.get<double>("orientation")),-sin(v.second.get<double>("orientation")),0,
      sin(v.second.get<double>("orientation")),cos(v.second.get<double>("orientation")),0,
      0,0,1).finished())); // Rotation in z

  obst->object = sceneBuilder.addFreeObject(v.second.get<std::string>("cadmodel"), // dae file path
                                      name + std::string("_polygonObst"),  // name
                                      "Cube",  // node id in dae
                                      obstInitDisplacements,  // position
                                      v.second.get<double>("mass"), // mass
                                      true, // enableWeight
                                      v.second.get<double>("radius")*Eigen::Vector3d::Constant(0.0254),  // scale
                                      0.005); // offset

  // get polygon vertices' position in the XY plan
  using boost::property_tree::ptree;

  foreach_ ( ptree::value_type &vertex, v.second.get_child("vertices") )
  {
    obst->vertices.push_back( (Eigen::Vector2d() << vertex.second.get<double>("x"), vertex.second.get<double>("y") ).finished() );
  }

  // color
  app->getGraphicSceneInterface().setNodeMaterial(name + std::string("_polygonObst"), v.second.get<std::string>("color"));

  //material
  obst->object.setContactMaterial(v.second.get<std::string>("material"));

  //collision
  app->getGVMScene().enableContactForBodyPair(obst->object, ground, true);

  app->addObstacle(obst);
}


}

// cmake:sourcegroup=Helpers