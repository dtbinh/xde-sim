#include "aiv/pathplanner/Trajectory.hpp"
#include <iostream>
#include <Eigen/Dense>
using namespace aiv;

int main()
{
	const unsigned splDim = 2;
	const unsigned splDeg = 3;
	const unsigned nIntervNonNull = 5;
	const unsigned nCtrlPts = nIntervNonNull + splDeg;

	Trajectory traj;
	traj.setOption("nCtrlPts", nCtrlPts);
	traj.setOption("nIntervNonNull", nIntervNonNull);


	//EXAMPLE
	double parVarInterv = 1.00;

	Eigen::Matrix<double, splDim, Eigen::Dynamic> points (splDim, nCtrlPts);
	for (auto i = 0; i < splDim; ++i)
		points.row(i) = Eigen::RowVectorXd::LinSpaced(nCtrlPts, 0.0, 1.0);
		//points.row(i) = Eigen::RowVectorXd::Random();
	std::cout << points << std::endl;

	//Trajectory traj(points, parVarInterv);
	//traj.updateFromUniform(points);

	traj.interpolate(points, parVarInterv);


	// //double * optParam = new double[nParam];
	double optPar[20];
	traj.getParameters(optPar);

	for (auto i=0; i<splDim*nCtrlPts; ++i)
		std::cout << optPar[i] << std::endl;

	//Eigen::RowVectorXd t(Eigen::RowVectorXd::LinSpaced(nCtrlPts, 0.0, parVarInterv));
	for (auto i=0; i < nCtrlPts; ++i)
	{
		std::cout << traj(double(i)/(nCtrlPts-1)*parVarInterv) << std::endl << std::endl;
	}

	// traj.updateFromUniform(points);

	// traj.getParameters(optPar);

	// for (auto i=0; i<splDim*nCtrlPts; ++i)
	// 	std::cout << optPar[i] << std::endl;

	// std::cout << "traj(0.0)\n--\n" << traj(0.0) << "\n--" << std::endl;
	// std::cout << "traj(1.0)\n--\n" << traj(1.0) << "\n--" << std::endl;
	// std::cout << "traj(0.0, 1)\n--\n" << traj(0.0, 3) << "\n--" << std::endl;
	// std::cout << "traj(parVarInterv, 1)\n--\n" << traj(parVarInterv, 3) << "\n--" << std::endl;
	// std::cout << "traj(parVarInterv*1./7., 1)\n--\n" << traj(parVarInterv*1./7., 3) << "\n--" << std::endl;
	// std::cout << "traj(parVarInterv*5./7., 1)\n--\n" << traj(parVarInterv*5./7., 3) << "\n--" << std::endl;


	//double nPar[] = {1, 1, 2.33333, 2.33333, 4.33333, 4.33333, 7, 7, 9, 9, 11.6667, 11.6667, 13.6667, 13.6667, 15, 15};
	// traj.update(optPar, 1.35);

	// traj.getParameters(optPar);

	// for (auto i=0; i<splDim*nCtrlPts; ++i)
	// 	std::cout << optPar[i] << std::endl;

	// std::cout << "traj(0.0)\n--\n" << traj(0.0) << "\n--" << std::endl;
	// std::cout << "traj(1.0)\n--\n" << traj(1.0) << "\n--" << std::endl;
	// std::cout << "traj(0.0, 1)\n--\n" << traj(0.0, 1) << "\n--" << std::endl;
	// std::cout << "traj(1.0, 1)\n--\n" << traj(1.0, 1) << "\n--" << std::endl;
	// std::cout << "traj(1./7., 1)\n--\n" << traj(1./7., 1) << "\n--" << std::endl;
	// std::cout << "traj(5./7., 1)\n--\n" << traj(5./7., 1) << "\n--" << std::endl;


	return 0;

}