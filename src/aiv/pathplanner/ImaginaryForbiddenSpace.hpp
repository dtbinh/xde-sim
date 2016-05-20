#ifndef __AIV_IMAGINARYFORBIDDENSPACE_HPP__
#define __AIV_IMAGINARYFORBIDDENSPACE_HPP__
#pragma once

#include "aiv/obstacle/Obstacle.hpp"

namespace aiv
{

	class ImaginaryForbiddenSpace: public Obstacle
	{
		public:
			ImaginaryForbiddenSpace(std::string name, double radius, const Eigen::Vector2d& position);
			~ImaginaryForbiddenSpace(){};

			inline double distToAIV (const Eigen::Vector2d & aivPosition, double aivRadius)
			{
				std::cout << "ImaginaryForbiddenSpace distToAIV\n";
				return (aivPosition-_displ.getTranslation().block<2,1>(0,0)).norm() - _radius - aivRadius;
			};

			inline double getRad() { return _radius; };

			// Eigen::Displacementd getCurrentPosition();

			// Eigen::Twistd getCurrentVelocity();
			
			inline Eigen::Displacementd getCurrentPosition() { return _displ; };

			inline Eigen::Twistd getCurrentVelocity() { return Eigen::Twistd::Zero(); };

		private:
			double _radius;
			Eigen::Displacementd _displ;

	};

}
#endif //__AIV_IMAGINARYFORBIDDENSPACE_HPP__
// cmake:sourcegroup=PathPlanner