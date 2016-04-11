#ifndef __SYSTEMID_HPP__
#define __SYSTEMID_HPP__
#pragma once

#include <Eigen/Dense>

class SystemID
{
	private:
	public:
		SystemID();
		~SystemID();
		const Eigen::Matrix<double, 6, 1> getSysParameters();
};

#endif // __SYSTEMID_HPP__