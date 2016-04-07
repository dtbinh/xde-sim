#include "aiv/controller/Gain.hpp"

namespace aiv {

	Gain::Gain(const unsigned gainDim):
		_isSplRepValid(false),
		_gainDim(gainDim)
	{

	}

	void Gain::update(Eigen::Spline< double, _gainDim, _derivDeg >::ControlPointVectorType ctrlPts)
	{
		_gainSpline.~MySpline();
		new (&_gainSpline) MySpline(_knots, ctrlPts);
		_isSplUpToDate = true;
	}

	void Gain::operator()(const double time)
	{
		if (_isSplUpToDate == true)
		{
			_evaluatedGainValues = _gainSpline(time);
			//CHECK IF TIME IS OK IF NOT LIMITE IT TO MAXTIME
		}
	}

	const double Gain::k(const unsigned idx)
	{
		if (idx >= _gainDim)
		{
			// TODO assert error
			return 0.0;
		}
		return _evaluatedGainValues(idx,0);
	}

	void Gain::setGainValues(const double *gainValues)
	{
		_isSplUpToDate = false;

		for (auto i; i < _gainDim; ++i)
		{
			_evaluatedGainValues(i,0) = gainValues[i];
		}

	}

	Gain::~Gain(){}
	


// cmake:sourcegroup=Controller