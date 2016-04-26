#include "aiv/pathplanner/Trajectory.hpp"
#include "aiv/helpers/MyException.hpp"
#include <iostream>

namespace aiv {

	Trajectory::Trajectory()
	{

	}

	Trajectory::Trajectory(const Eigen::Matrix<double, Trajectory::dim, Eigen::Dynamic>& ctrlPts, const double parVarInterval)
	{
		_nCtrlPts = ctrlPts.cols();
		_nIntervNonNull = ctrlPts.cols()-derivDeg;
		_trajecSpl.~TrajectorySpline();
		new (&_trajecSpl) TrajectorySpline(_genKnots(0.0, parVarInterval, true, ctrlPts.cols()-derivDeg), ctrlPts);
	}

	void Trajectory::interpolate(const Eigen::Matrix<double, Trajectory::dim, Eigen::Dynamic>& points, const double parVarInterval)
	{
		Eigen::RowVectorXd parVariable = Eigen::RowVectorXd::LinSpaced(_nCtrlPts, 0.0, parVarInterval);
		TrajectorySpline::KnotVectorType chordLengths;
		Eigen::ChordLengths(parVariable, chordLengths);
		//std::cout << "parVariable\n" << parVariable << std::endl;
		//std::cout << "chordLengths\n" << chordLengths << std::endl;
		_trajecSpl = Eigen::SplineFitting<TrajectorySpline>::Interpolate(points, derivDeg, chordLengths);

		TrajectorySpline::ControlPointVectorType ctrlPts(dim, _nCtrlPts);
		ctrlPts = _trajecSpl.ctrls();

		_trajecSpl.~TrajectorySpline();
		new (&_trajecSpl) TrajectorySpline(_genKnots(0.0, parVarInterval, true, _nIntervNonNull), ctrlPts);

		//std::cout << "knots\n" << _trajecSpl.knots() << std::endl;
		//std::cout << "genKnots\n" << _genKnots(0.0, parVarInterval, true, _nIntervNonNull) << std::endl;
	}

	void Trajectory::setOption(std::string optionName, unsigned optionValue)
	{
		if (optionName == "nCtrlPts")
		{
			_nCtrlPts = optionValue;
		}
		else if (optionName == "nIntervNonNull")
		{
			_nIntervNonNull = optionValue;
		}
	}

	void Trajectory::update(const double *ctrlpts)
	{
		_knots = _trajecSpl.knots();
		_trajecSpl.~TrajectorySpline();
		new (&_trajecSpl) TrajectorySpline(_knots, cArray2CtrlPtsMat(ctrlpts));
	}

	void Trajectory::update(volatile double *ctrlpts)
	{
		_knots = _trajecSpl.knots();
		_trajecSpl.~TrajectorySpline();
		new (&_trajecSpl) TrajectorySpline(_knots, cArray2CtrlPtsMat(ctrlpts));
	}

	void Trajectory::update(const Eigen::Matrix<double, Trajectory::dim, Eigen::Dynamic>& ctrlpts)
	{
		_knots = _trajecSpl.knots();
		_trajecSpl.~TrajectorySpline();
		new (&_trajecSpl) TrajectorySpline(_knots, ctrlpts);
	}

	void Trajectory::update(const double *ctrlpts, const double parVarInterval)
	{
		_trajecSpl.~TrajectorySpline();
		new (&_trajecSpl) TrajectorySpline(_genKnots(0.0, parVarInterval, true, _nIntervNonNull), cArray2CtrlPtsMat(ctrlpts));
	}

	void Trajectory::update(volatile double *ctrlpts, volatile double parVarInterval)
	{
		_trajecSpl.~TrajectorySpline();
		new (&_trajecSpl) TrajectorySpline(_genKnots(0.0, parVarInterval, true, _nIntervNonNull), cArray2CtrlPtsMat(ctrlpts));
	}

	void Trajectory::update(const Eigen::Matrix<double, Trajectory::dim, Eigen::Dynamic>& ctrlpts, const double parVarInterval)
	{
		_trajecSpl.~TrajectorySpline();
		new (&_trajecSpl) TrajectorySpline(_genKnots(0.0, parVarInterval, true, _nIntervNonNull), ctrlpts);
	}

	void Trajectory::updateFromUniform(const double *ctrlpts)
	{
		double parVarInterval = _trajecSpl.knots().tail(1)(0,0);

		TrajectorySpline auxSpline(_genKnots(0.0, parVarInterval, false, _nIntervNonNull), cArray2CtrlPtsMat(ctrlpts));

		Eigen::RowVectorXd parVariable = Eigen::RowVectorXd::LinSpaced(_nCtrlPts, 0.0, parVarInterval);

		Eigen::Matrix<double, Trajectory::dim, Eigen::Dynamic> points(dim, _nCtrlPts);

		for (auto i=0; i < _nCtrlPts; ++i)
		{
			points.col(i) = auxSpline(parVariable(i));
		}

		interpolate(points, parVarInterval);
	}

	void Trajectory::updateFromUniform(const double *ctrlpts, const double parVarInterval)
	{
		TrajectorySpline auxSpline(_genKnots(0.0, parVarInterval, false, _nIntervNonNull), cArray2CtrlPtsMat(ctrlpts));

		Eigen::RowVectorXd parVariable = Eigen::RowVectorXd::LinSpaced(_nCtrlPts, 0.0, parVarInterval);

		Eigen::Matrix<double, Trajectory::dim, Eigen::Dynamic> points(dim, _nCtrlPts);

		for (auto i=0; i < _nCtrlPts; ++i)
		{
			points.col(i) = auxSpline(parVariable(i));
		}

		interpolate(points, parVarInterval);
	}

	void Trajectory::updateFromUniform(const Eigen::Matrix<double, Trajectory::dim, Eigen::Dynamic>& ctrlpts)
	{
		double parVarInterval = _trajecSpl.knots().tail(1)(0,0);

		TrajectorySpline auxSpline(_genKnots(0.0, parVarInterval, false, _nIntervNonNull), ctrlpts);

		Eigen::RowVectorXd parVariable = Eigen::RowVectorXd::LinSpaced(_nCtrlPts, 0.0, parVarInterval);

		Eigen::Matrix<double, Trajectory::dim, Eigen::Dynamic> points(dim, _nCtrlPts);

		for (auto i=0; i < _nCtrlPts; ++i)
		{
			points.col(i) = auxSpline(parVariable(i));
		}

		interpolate(points, parVarInterval);
	}

	void Trajectory::updateFromUniform( const Eigen::Matrix<double, Trajectory::dim, Eigen::Dynamic>& ctrlpts, const double parVarInterval)
	{
		TrajectorySpline auxSpline(_genKnots(0.0, parVarInterval, false, _nIntervNonNull), ctrlpts);

		Eigen::RowVectorXd parVariable = Eigen::RowVectorXd::LinSpaced(_nCtrlPts, 0.0, parVarInterval);

		Eigen::Matrix<double, Trajectory::dim, Eigen::Dynamic> points(dim, _nCtrlPts);

		for (auto i=0; i < _nCtrlPts; ++i)
		{
			points.col(i) = auxSpline(parVariable(i));
		}

		interpolate(points, parVarInterval);
	}

	// void Trajectory::updateFromUniform(const double *ctrlpts, const double parVarInterval)
	// {
	// 	TrajectorySpline::ControlPointVectorType ctrlPts(dim, _nCtrlPts);
	// 	// Feed ctrlPts rows with values from the primal variables x
	// 	for (int i = 0; i < _nCtrlPts*dim; ++i)
	// 	{
	// 		ctrlPts(i % dim, i / dim) = ctrlpts[i];
	// 	}
	// 	_trajecSpl.~TrajectorySpline();
	// 	new (&_trajecSpl) TrajectorySpline(_genKnots(0.0, parVarInterval, true, _nIntervNonNull), ctrlPts);
	// }

	Eigen::Matrix<double, Trajectory::dim, 1> Trajectory::operator()(const double evalTime) const
	{	
		//TODO CHECK IF TIME IS within limits
		return _trajecSpl(evalTime);
	}

	Eigen::Matrix<double, Trajectory::dim, Eigen::Dynamic> Trajectory::operator()(const double evalTime, const unsigned deriv) const
	{
		//TODO CHECK IF TIME IS within limits
		return _trajecSpl.derivatives(evalTime, deriv);
	}


	void Trajectory::getParameters(double* params) const
	{
		if (params == NULL)
		{
			std::stringstream ss;
			ss << "Trajectory::getParameters: invalid C-like array (points to NULL). Was memory allocated?";
			throw(MyException(ss.str()));
		}
		//unsigned nParam = _trajecSpl.ctrls().size(); // size() = total of coefficients (= rows() + cols())
		for (auto i = 0; i < _nCtrlPts*dim; ++i)
		{
			params[i] = _trajecSpl.ctrls()(i % dim, i / dim);
		}
	}

	Eigen::Array< double, 1, Eigen::Dynamic > Trajectory::_genKnots(const double initT, const double finalT, const bool nonUniform, const unsigned nIntervNonNull) const
	{
		// TODO throw error
		if (nIntervNonNull < 2)
		{
			std::stringstream ss;
			ss << "Trajectory::_genKnots: number of non null intervals is too low [ " << nIntervNonNull << " ].";
			throw(MyException(ss.str()));
		}

		double d = (finalT - initT) / (4 + (nIntervNonNull - 2));
		// d is the nonuniform interval base value (spacing produce intervals like this: 2*d, d,... , d, 2*d)

		Eigen::Array< double, 1, Eigen::Dynamic > knots(derivDeg * 2 + nIntervNonNull + 1);

		// first and last knots
		knots.head(derivDeg) = Eigen::Array< double, 1, Eigen::Dynamic >::Constant(derivDeg, initT);
		knots.tail(derivDeg) = Eigen::Array< double, 1, Eigen::Dynamic >::Constant(derivDeg, finalT);

		// intermediaries knots
		if (nonUniform)
		{
			knots(derivDeg) = initT;
			knots(derivDeg + 1) = initT + 2 * d;

			unsigned i = 0;
			for (i = 0; i < nIntervNonNull - 2; ++i)
			{
				knots(derivDeg + i + 2) = knots(derivDeg + i + 1) + d;
			}

			knots(derivDeg + 2 + i) = finalT; // = knots(derivDeg+2+i-1) + 2*d
		}
		else // uniform
		{
			knots.segment(derivDeg, nIntervNonNull + 1) = Eigen::Array< double, 1, Eigen::Dynamic >::LinSpaced(nIntervNonNull + 1, initT, finalT);
		}
		return knots;
	}

	Trajectory::~Trajectory(){}
	
}

// cmake:sourcegroup=PathPlanner