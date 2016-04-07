#ifndef __AIV_GAIN_HPP__
#define __AIV_GAIN_HPP__
#pragma once

#include <unsupported/Eigen/Splines>

extern const _gainDim;
typedef Eigen::Spline< double, _gainDim, _derivDeg > GainSpline;

namespace aiv {

	class Gain
	{
	private:
		static const unsigned _gainDim;
		static const unsigned _derivDeg = 3;


		bool _isSplRepValid;
		Eigen::Vector<double, _gainDim, 1> _evaluatedGainValues;

		typedef Eigen::Spline< double, _gainDim, _derivDeg > GainSpline;
		GainSpline _gainSpline;

	public:
		
		void update(Eigen::Spline< double, _gainDim, _derivDeg >::ControlPointVectorType ctrlPts);
		void operator()(double);
		const double k(const unsigned idx);
		void setGainValues(const double *gainValues);

	};

}

#endif // __AIV_GAIN_HPP__

// cmake:sourcegroup=Controller