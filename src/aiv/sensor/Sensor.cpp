#include "Sensor.hpp"

#include "aiv/robot/AIV.hpp"
#include "aiv/obstacle/Obstacle.hpp"

namespace aiv
{

	Sensor::Sensor(std::string n, double detectionRadius)
		: name(n),
		detecRad(detectionRadius)
	{
	}

	void Sensor::update(Eigen::Displacementd currPos, std::map<std::string, Obstacle *> obstacles, std::map<std::string, AIV *> vehicles)
	{
		detectedObstacles.clear();
		otherVehicles.clear();

		Eigen::Vector2d curr2dPosition;
		curr2dPosition << currPos.x(), currPos.y();

		for (std::map<std::string, Obstacle *>::iterator it = obstacles.begin(); it != obstacles.end(); ++it)
		{
			if ((it->second->getCurrentPosition().getTranslation().head(2) - curr2dPosition).norm() < detecRad)
			{
				detectedObstacles[it->first] = it->second;
			}
		}

		for (std::map<std::string, AIV *>::iterator it = vehicles.begin(); it != vehicles.end(); ++it)
		{
			if (name.find(it->first) != std::string::npos)
				continue;

			otherVehicles[it->first] = it->second;
		}

	}
}


// cmake:sourcegroup=Sensor