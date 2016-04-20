//#include "C:\Users\JM246044\workspace\dev\xde\xde\xde\xde\src\app\sysidentification\SystemID.hpp"
#include "SystemID.hpp"
#include <iostream>
//#include <nlopt.h>
//#include <Eigen/Dense>


int main()
{
	//Eigen::Matrix<double, 6, 1> systemParameters;
	SystemID systemIDfier;
	std::cout << systemIDfier.getSysParameters() << std::endl;

	system("pause");
	return 0;
}

