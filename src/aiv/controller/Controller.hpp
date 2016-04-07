#ifndef __AIV_CONTROLLER_HPP__
#define __AIV_CONTROLLER_HPP__
#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Lgsm>

namespace aiv {

	class Controller
	{

	public:
		Controller(std::string name);
		virtual void update(Eigen::Displacementd, Eigen::Twistd, double, double, double, double, double) = 0;
		std::string getName();

	protected:
		std::string   name;

	};

}

#endif // __AIV_CONTROLLER_HPP__

// cmake:sourcegroup=Controller