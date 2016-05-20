#include "aiv/pathplanner/ImaginaryForbiddenSpace.hpp"

namespace aiv 
{

	ImaginaryForbiddenSpace::ImaginaryForbiddenSpace(std::string name, double radius, const Eigen::Vector2d& position):
		Obstacle(name),
		_radius(radius)
	{
		Eigen::Displacementd aux (position.x(),
			position.y(),
			0.0,  // arbitrary translation in z
			1., 0., 0., 0.); // circular obstacle is orientation invariant
		_displ = aux;
	}
}
// cmake:sourcegroup=pathplanner