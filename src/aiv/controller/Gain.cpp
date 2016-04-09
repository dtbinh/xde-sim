#include "aiv/controller/Gain.hpp"

namespace aiv {

	Gain::Gain():
		_isSplUpToDate(false)
	{

	}

	void Gain::update(GainSpline::ControlPointVectorType ctrlPts)
	{
		_gainSpline.~GainSpline();
		new (&_gainSpline) GainSpline(_knots, ctrlPts);
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

	void Gain::setGainValue(const double *gainValues)
	{
		_isSplUpToDate = false;

		for (auto i = 0; i < _gainDim; ++i)
		{
			_evaluatedGainValues(i,0) = gainValues[i];
		}

	}
	void Gain::setGainValue(const double gainValue, const unsigned idx)
	{
		_isSplUpToDate = false;
		_evaluatedGainValues(idx,0) = gainValue;
	}

	Gain::~Gain(){}
	
}

// cmake:sourcegroup=Controller