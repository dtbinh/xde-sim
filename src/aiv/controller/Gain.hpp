#ifndef __AIV_GAIN_HPP__
#define __AIV_GAIN_HPP__
#pragma once

#include <unsupported/Eigen/Splines>

namespace aiv {

	class Gain
	{
	private:
		static const unsigned _gainDim = 3;
		static const unsigned _derivDeg = 3;
		typedef Eigen::Spline< double, _gainDim, _derivDeg > GainSpline;

		GainSpline _gainSpline;

		Eigen::Matrix<double, _gainDim, 1> _evaluatedGainValues;

		bool _isSplUpToDate;

		GainSpline::KnotVectorType _knots;

	public:
		
		void update(Eigen::Spline< double, _gainDim, _derivDeg >::ControlPointVectorType ctrlPts);
		void operator()(double);
		const double k(const unsigned idx);
		void setGainValue(const double *gainValues);
		void setGainValue(const double gainValue,  const unsigned idx);
		Gain::Gain();
		Gain::~Gain();
	};

}

#endif // __AIV_GAIN_HPP__

// cmake:sourcegroup=Controller