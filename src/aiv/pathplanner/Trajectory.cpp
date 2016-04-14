#include "aiv/pathplanner/Trajectory.hpp"

namespace aiv {

	Trajectory::Trajectory():
	{

	}

	void Trajectory::update(TrajectorySpline::ControlPointVectorType ctrlPts)
	{
		_knots = _trajecSpl.knots();
		_trajecSpl.~TrajectorySpline();
		new (&_trajecSpl) TrajectorySpline(_knots, ctrlPts);
	}
	void Trajectory::update(unsigned ncpts, const double *ctrlpts)
	{
		TrajectorySpline::ControlPointVectorType ctrlPts(dim, ncpts);
		// Feed ctrlPts rows with values from the primal variables x
		for (int i = 0; i < ncpts; ++i)
		{
			ctrlPts(i % dim, i / dim) = ctrlpts[i];
		}
		_knots = _trajecSpl.knots();
		_trajecSpl.~TrajectorySpline();
		new (&_trajecSpl) TrajectorySpline(_knots, ctrlPts);
	}
	void Trajectory::update(TrajectorySpline::ControlPointVectorType ctrlPts, TrajectorySpline::KnotVectorType knots)
	{
		_trajecSpl.~TrajectorySpline();
		new (&_trajecSpl) TrajectorySpline(knots, ctrlPts);
	}
	void Trajectory::update(unsigned ncpts, const double *ctrlpts, TrajectorySpline::KnotVectorType knots)
	{
		TrajectorySpline::ControlPointVectorType ctrlPts(dim, ncpts);
		// Feed ctrlPts rows with values from the primal variables x
		for (int i = 0; i < ncpts; ++i)
		{
			ctrlPts(i % dim, i / dim) = ctrlpts[i];
		}
		_trajecSpl.~TrajectorySpline();
		new (&_trajecSpl) TrajectorySpline(knots, ctrlPts);
	}

	void Trajectory::operator()(const double time)
	{
		if (_isSplUpToDate == true)
		{
			_evaluatedTrajectoryValues = _trajecSpl(time);
			//CHECK IF TIME IS OK IF NOT LIMITE IT TO MAXTIME
		}
	}

	const double Trajectory::k(const unsigned idx)
	{
		if (idx >= dim)
		{
			// TODO assert error
			return 0.0;
		}
		return _evaluatedTrajectoryValues(idx,0);
	}

	void Trajectory::setTrajectoryValue(const double *trajectoryValues)
	{
		_isSplUpToDate = false;

		for (auto i = 0; i < dim; ++i)
		{
			_evaluatedTrajectoryValues(i,0) = trajectoryValues[i];
		}

	}
	void Trajectory::setTrajectoryValue(const double trajectoryValue, const unsigned idx)
	{
		_isSplUpToDate = false;
		_evaluatedTrajectoryValues(idx,0) = trajectoryValue;
	}

	Eigen::Array< double, 1, Eigen::Dynamic >  Trajectory::genKnots(const double initT, const double finalT, bool nonUniform)
	{
		// TODO throw error
		if (this->noIntervNonNull < 2)
		{
			std::stringstream ss;
			ss << "Trajectory::genKnots: number of non null intervals is too low [ " << noIntervNonNull << " ]. Check configuration file.";
			throw(MyException(ss.str()));
		}

		double d = (finalT - initT) / (4 + (this->noIntervNonNull - 2));
		// d is the nonuniform interval base value (spacing produce intervals like this: 2*d, d,... , d, 2*d)

		Eigen::Array< double, 1, Eigen::Dynamic > knots(this->splDegree * 2 + this->noIntervNonNull + 1);

		// first and last knots
		knots.head(this->splDegree) = Eigen::Array< double, 1, Eigen::Dynamic >::Constant(this->splDegree, initT);
		knots.tail(this->splDegree) = Eigen::Array< double, 1, Eigen::Dynamic >::Constant(this->splDegree, finalT);

		// intermediaries knots
		if (nonUniform)
		{
			knots(this->splDegree) = initT;
			knots(this->splDegree + 1) = initT + 2 * d;

			unsigned i = 0;
			for (i = 0; i < this->noIntervNonNull - 2; ++i)
			{
				knots(this->splDegree + i + 2) = knots(this->splDegree + i + 1) + d;
			}

			knots(this->splDegree + 2 + i) = finalT; // = knots(this->splDegree+2+i-1) + 2*d
		}
		else // uniform
		{
			knots.segment(this->splDegree, this->noIntervNonNull + 1) = Eigen::Array< double, 1, Eigen::Dynamic >::LinSpaced(this->noIntervNonNull + 1, initT, finalT);
		}
		return knots;
	}

	Trajectory::~Trajectory(){}
	
}

// cmake:sourcegroup=PathPlanner