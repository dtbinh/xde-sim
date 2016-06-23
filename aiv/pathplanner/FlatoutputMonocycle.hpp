#ifndef __AIV_FLATOUTPUTMONOCYCLE_HPP__
#define __AIV_FLATOUTPUTMONOCYCLE_HPP__
#pragma once

#include "Eigen/Dense"
//#include <adolc.h>

namespace aiv {

	class FlatoutputMonocycle
	{
	public:
		static const unsigned flatDim = 2;
		static const unsigned poseDim = 3;
		static const unsigned posDim = 2;
		static const unsigned veloDim = 2;
		static const unsigned accelDim = 2;
		static const unsigned flatDerivDeg = 2;
		static const unsigned linSpeedIdx = 0;
		static const unsigned angSpeedIdx = 1;
		static const unsigned linAccelIdx = 0;
		static const unsigned angAccelIdx = 1;
		static const unsigned posIdx = 0;
		static const unsigned oriIdx = 2;

		/*!
			\brief
			Return z given q.

			Return \f$[x\\ y\\ \\theta]^T\f$ given \f$[z\\ \dot{z}\\ \dotsc\\ z^{(l)}]\f$
			(only \f$z\f$ and \f$\dot{z}\f$ are used). \f$\\theta\f$ is in the range
			\f$(-\pi, \pi]\f$.

			\f[
				\begin{array}{l}
				\varphi_1(z(t_k),\dotsc,z^{(l)}(t_k))=\\
				\left[\begin{array}{c}
				x\\
				y\\
				\omega
				\end{array}\right]
				\left[\begin{array}{c}
				z_1\\
				z_2\\
				\arctan(\dot{z}_2/\dot{z}_1)\\
				\end{array}\right]
				\end{array}
			\f]
			\param q
			State vector.

			\returns
			The flatoutput.
		 */
		template<class T>
		static Eigen::Matrix< T, flatDim, 1 > poseToFlat(const Eigen::Matrix< T, poseDim, 1 > &pose)
		{
			return pose.block< aiv::FlatoutputMonocycle::flatDim, 1 >(0, 0);
		}

		template<class T>
		static Eigen::Matrix< T, poseDim, 1 > flatToPose(const Eigen::Matrix< T, flatDim, flatDerivDeg + 1 > &dFlat)
		{
			// double vx = dFlat(0, 1) < 0.0 ? 0.0 : dFlat(0, 1); // THIS IS CRUTIAL FOR THE OPTIMIZATION TO WORK
			double vx = dFlat(0,1);

			return (Eigen::Matrix< T, aiv::FlatoutputMonocycle::poseDim, 1 >() <<
				dFlat.col(0),
				atan2(dFlat(1, 1), vx)
				).finished();
		}

		template<class T>
		static Eigen::Matrix< T, poseDim, 1 > flatToPose(const Eigen::Matrix< T, flatDim, flatDerivDeg + 2 > &dFlat)
		{
			// double vx = dFlat(0, 1) < 0.0 ? 0.0 : dFlat(0, 1); // THIS IS CRUTIAL FOR THE OPTIMIZATION TO WORK
			double vx = dFlat(0,1);

			return (Eigen::Matrix< T, aiv::FlatoutputMonocycle::poseDim, 1 >() <<
				dFlat.col(0),
				atan2(dFlat(1, 1), vx)
				).finished();
		}

		template<class T>
		static Eigen::Matrix< T, posDim, 1 > flatToPosition(const Eigen::Matrix< T, flatDim, 1> &flat)
		{
			return flat;
		}
		
		template<class T>
		static Eigen::Matrix< T, posDim, 1 > flatToPosition(const Eigen::Matrix< T, flatDim, flatDerivDeg + 1> &dFlat)
		{
			return dFlat.col(0);
		}

		template<class T>
		static Eigen::Matrix< T, posDim, 1 > flatToPosition(const Eigen::Matrix< T, flatDim, flatDerivDeg + 2> &dFlat)
		{
			return dFlat.col(0);
		}

		template<class T>
		static Eigen::Matrix< T, veloDim, 1 > flatToVelocity(const Eigen::Matrix< T, flatDim, flatDerivDeg + 1 > &dFlat)
		{
			// double vx = dFlat(0, 1) < 0.0 ? 0.0 : dFlat(0, 1);
			double vx = dFlat(0,1);

			T den = vx*vx + dFlat(1, 1)*dFlat(1, 1); //+eps so no /0 + static_cast<T>(std::numeric_limits< float >::epsilon())
			den = den < static_cast<T>(std::numeric_limits< double >::epsilon()) ? static_cast<T>(std::numeric_limits< double >::epsilon()) : den;

			return (Eigen::Matrix< T, aiv::FlatoutputMonocycle::veloDim, 1>() <<
				dFlat.block< aiv::FlatoutputMonocycle::flatDim, 1>(0, 1).norm(),
				(vx*dFlat(1, 2) - dFlat(1, 1)*dFlat(0, 2)) / den
				).finished();
		}

		template<class T>
		static Eigen::Matrix< T, veloDim, 1 > flatToVelocity(const Eigen::Matrix< T, flatDim, flatDerivDeg + 2 > &dFlat)
		{
			// double vx = dFlat(0, 1) < 0.0 ? 0.0 : dFlat(0, 1);
			double vx = dFlat(0,1);

			T den =vx*vx + dFlat(1, 1)*dFlat(1, 1); //+eps so no /0 + static_cast<T>(std::numeric_limits< float >::epsilon())
			den = den < static_cast<T>(std::numeric_limits< double >::epsilon()) ? static_cast<T>(std::numeric_limits< double >::epsilon()) : den;

			return (Eigen::Matrix< T, aiv::FlatoutputMonocycle::veloDim, 1>() <<
				dFlat.block< aiv::FlatoutputMonocycle::flatDim, 1>(0, 1).norm(),
				(vx*dFlat(1, 2) - dFlat(1, 1)*dFlat(0, 2)) / den
				).finished();
		}
// 			   eps = np.finfo(float).eps
//             pquartereps = eps**(.25)
//             # Prevent division by zero
//             dz_norm = LA.norm(zl[:, 1])
//             # min_den_norm = np.finfo(float).eps**(-4)
//             # den = dz_norm if dz_norm >= min_den_norm else min_den_norm
//             den = dz_norm if abs(dz_norm) > pquartereps else pquartereps
		template<class T>
		static Eigen::Matrix< T, veloDim, 1 > flatToAcceleration(const Eigen::Matrix< T, flatDim, flatDerivDeg + 2 > &dFlat)
		{
			// double vx = dFlat(0, 1) < 0.0 ? 0.0 : dFlat(0, 1);
			double vx = dFlat(0,1);

			T dflat_norm = dFlat.block< aiv::FlatoutputMonocycle::flatDim, 1 >(0, 1).norm();

			T min_den_norm_dw = pow(static_cast<T>(std::numeric_limits< double >::epsilon()), 0.25);
			T min_den_norm_dv = static_cast<T>(std::numeric_limits< double >::epsilon());

			T dflat_norm_den1 = dflat_norm < min_den_norm_dv ? min_den_norm_dv : dflat_norm;
			T dflat_norm_den2 = dflat_norm < min_den_norm_dw ? min_den_norm_dw : dflat_norm;
			
			T dv = (vx*dFlat(0, 2) + dFlat(1, 1)*dFlat(1, 2)) / dflat_norm_den1;
			T dw = ((dFlat(0, 2)*dFlat(1, 2) + dFlat(1, 3)*vx -
				(dFlat(1, 2)*dFlat(0, 2) + dFlat(0, 3)*dFlat(1, 1)))*(pow(dflat_norm, 2)) -
				(vx*dFlat(1, 2) - dFlat(1, 1)*dFlat(0, 2)) * 2 * dflat_norm*dv) / pow(dflat_norm_den2, 4);

			return (Eigen::Matrix< T, aiv::FlatoutputMonocycle::veloDim, 1 >() <<
				dv,
				dw
				).finished();
		}

	};

	// typedef Eigen::Matrix< double, aiv::FlatoutputMonocycle::poseDim, 1 > PoseVector;
	// typedef Eigen::Matrix< double, aiv::FlatoutputMonocycle::veloDim, 1 > VelVector;
	// typedef Eigen::Matrix< double, aiv::FlatoutputMonocycle::accelDim, 1 > AccVector;

}
#endif // __AIV_FLATOUTPUTMONOCYCLE_HPP__

// cmake:sourcegroup=PathPlanner