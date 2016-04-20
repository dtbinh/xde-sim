#ifndef __AIV_SENSOR_HPP__
#define __AIV_SENSOR_HPP__
#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Lgsm>
#include <map>
#include "aiv/pathplanner/FlatoutputMonocycle.hpp"

namespace aiv {

	class AIV;
	class Obstacle;

	class Sensor
	{

	public:
		Sensor(std::string n, double detectionRadius);
		void update(Eigen::Displacementd currPos, std::map<std::string, Obstacle *> obstacles, std::map<std::string, AIV *> vehicles);
		inline std::string getName() const { return name; }
		inline std::map<std::string, Obstacle *> getObstacles() const { return detectedObstacles; };
		inline std::map<std::string, AIV *> getVehicles() const { return otherVehicles; };

	protected:
		std::string name;
		const double detecRad;
		std::map<std::string, Obstacle *> detectedObstacles;
		std::map<std::string, AIV *> otherVehicles;
	};

}

#endif // __AIV_SENSOR_HPP__

// cmake:sourcegroup=Sensor