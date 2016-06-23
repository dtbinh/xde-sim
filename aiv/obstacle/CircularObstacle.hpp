#ifndef __AIV_CIRCULAROBSTACLE_HPP__
#define __AIV_CIRCULAROBSTACLE_HPP__
#pragma once

#include "aiv/Application.hpp"
#include "aiv/obstacle/Obstacle.hpp"

namespace aiv
{

	class CircularObstacle: public Obstacle
	{
		friend class ObstBuilder;
		public:
			CircularObstacle(std::string name, Application * app);
			~CircularObstacle(){};

			inline double distToAIV (const Eigen::Vector2d & aiv_position, double aiv_radius)
			{
				return (aiv_position-getCurrentPosition().getTranslation().block<2,1>(0,0)).norm() - radius - aiv_radius;
			};

			inline double getRad() { return radius; };

		protected:
			double radius;
	};

}
#endif //__AIV_CIRCULAROBSTACLE_HPP__
// cmake:sourcegroup=Obstacle