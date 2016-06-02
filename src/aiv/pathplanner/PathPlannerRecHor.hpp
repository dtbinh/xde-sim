#ifndef __AIV_PATHPLANNERRECHOR_HPP__
#define __AIV_PATHPLANNERRECHOR_HPP__
#pragma once

#include "aiv/pathplanner/PathPlanner.hpp"
#include "aiv/pathplanner/FlatoutputMonocycle.hpp"
#include "aiv/pathplanner/Trajectory.hpp"
#include "aiv/helpers/Common.hpp"
#include <boost/thread.hpp>
#include <fstream>

#include <nlopt.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <set>


namespace aiv
{

	typedef Eigen::Matrix< double, FlatoutputMonocycle::flatDim, FlatoutputMonocycle::flatDerivDeg + 1 > NDerivativesMatrix;
	typedef Eigen::Matrix< double, FlatoutputMonocycle::flatDim, FlatoutputMonocycle::flatDerivDeg + 2 > Np1DerivativesMatrix;
	typedef Eigen::Matrix< double, FlatoutputMonocycle::poseDim, 1 > PoseVector;
	typedef Eigen::Matrix< double, FlatoutputMonocycle::flatDim, 1 > FlatVector;
	typedef Eigen::Matrix< double, FlatoutputMonocycle::veloDim, 1 > VeloVector;
	typedef Eigen::Matrix< double, FlatoutputMonocycle::accelDim, 1 > AccelVector;

	typedef std::map< std::string, Obstacle* > MapObst;
	typedef std::map< std::string, AIV* > MapAIV;

	typedef std::map< std::string, Eigen::Matrix < double, FlatoutputMonocycle::posDim, Eigen::Dynamic > > MapSharedPath;


	class Obstacle;
	class AIV;

	class CompObj
	{
	private:
		std::map<std::string, Common::CArray3d> _avoidanceInfo;
		
	public:
		
		CompObj(std::map<std::string, Common::CArray3d> avoidInfo):_avoidanceInfo(avoidInfo){}

		bool operator() (const std::string& smaller, const std::string& greater)
		{
			double s0 = _avoidanceInfo[smaller][0];
			double s1 = _avoidanceInfo[smaller][1];
			double g0 = _avoidanceInfo[greater][0];
			double g1 = _avoidanceInfo[greater][1];

			if ((s0 < g0 && s0 < g1) ||
				(s1 < g0 && s1 < g1))
			{
				// if (s1 < s0)
				// {
				// 	double aux = s1;
				// 	_avoidanceInfo[smaller][1] = s0;
				// 	_avoidanceInfo[smaller][0] = aux;
				// }
				return true;
			}
			else return false;
		}

		template < template < typename, typename > class C >
		bool operator() (C<std::string, std::allocator<std::string> > smaller, C<std::string, std::allocator<std::string> > greater)
		{
			double sf0 = abs(_avoidanceInfo[smaller.front()][0]);
			double sf1 = abs(_avoidanceInfo[smaller.front()][1]);
			double sb0 = abs(_avoidanceInfo[smaller.back()][0]);
			double sb1 = abs(_avoidanceInfo[smaller.back()][1]);
			double gf0 = abs(_avoidanceInfo[greater.front()][0]);
			double gf1 = abs(_avoidanceInfo[greater.front()][1]);
			double gb0 = abs(_avoidanceInfo[greater.back()][0]);
			double gb1 = abs(_avoidanceInfo[greater.back()][1]);

			if ((sf0 < gf0 && sf0 < gf1 && sf0 < gb0 && sf0 < gb1) ||
				(sf1 < gf0 && sf1 < gf1 && sf1 < gb0 && sf1 < gb1) ||
				(sb0 < gf0 && sb0 < gf1 && sb0 < gb0 && sb0 < gb1) ||
				(sb1 < gf0 && sb1 < gf1 && sb1 < gb0 && sb1 < gb1))
			{
				return true;
			}
			else return false;
		}
	};


	class PathPlannerRecHor : public PathPlanner
	{

		/*____________________________________________ Members ____________________________________________*/
	public:
		static const unsigned splDegree = FlatoutputMonocycle::flatDerivDeg + 1; // spline degree defined by Defoort is = splDegree + 1    
		static const unsigned splDim = FlatoutputMonocycle::flatDim; // spline dimension
	private:

		// Trajectory representation ======================================================
		Trajectory _trajectory;
		Trajectory _optTrajectory; // For using by the optimization process
		//unsigned _nIntervNonNull;   // number of non null knots intervals
		//unsigned _nCtrlPts;         // number of control points

		// Initial and target conditions ==================================================
		PoseVector _initPose;
		FlatVector _initFlat;
		VeloVector _initVelocity;
		PoseVector _targetedPose;
		PoseVector _sharedTargetedPose;
		FlatVector _targetedFlat;
		VeloVector _targetedVelocity;
		VeloVector _maxVelocity;
		AccelVector _maxAcceleration;

		FlatVector _latestFlat;
		PoseVector _latestPose;
		Eigen::Displacementd _currentPose;
		Eigen::Twistd _currentVelo;
		Eigen::QuaternionBase<Eigen::Quaternion<double> >::Vector3 _currentDirec;

		VeloVector _latestVelocity;
		PoseVector _initPoseForFuturePlan; // buffer for base position
		PoseVector _initPoseForCurrPlan; // buffer for base position

		// Planning outputs ===============================================================
		VeloVector _velocityOutput;
		AccelVector _accelOutput;
		PoseVector _poseOutput;

		// Planning parameters ============================================================
		double _compHorizon;         // computation horizon
		double _planHorizon;      // planning horizon
		double _refPlanHorizon;      // planning horizon
		double _optPlanHorizon;    // planning horizon for using by the optimization process

		bool _planLastPlan; // last planning flag
		double _lastStepMinDist; // parameter for stop condition

		double _conflictFreePathDeviation;
		double _interRobotSafetyDist;
		double _robotObstacleSafetyDist;
		double _maxStraightDist;

		PlanStage _planStage;
		PlanStage _sharedPlanStage;
		bool _waitForThread;

		//double _estTime;
		
		MapObst _detectedObstacles;
		MapObst _knownObstacles;

		//std::map<std::string, std::vector<int> > conflictInfo;
		MapAIV _knownRobots;
		
		MapAIV _collisionAIVs;
		MapAIV _comOutAIVs;
		MapSharedPath _pathsFromConflictualAIVs;

		std::set< std::string > _comAIVsSet;

		double _comRange;
		double _radius;
		double _sharedRadius;
		//double secRho; // secure distance from other robots and obstacles

		unsigned _nTimeSamples;     // number of  time samples taken within a planning horizon
		int _executingPlanIdx;
		bool _spawnP0;

		Eigen::Matrix2d _rotMat2WRef;
		Eigen::Matrix2d _rotMat2RRef;
		FlatVector _wayPt;
		//int _ongoingPlanIdx;

		std::map< std::string, Common::CArray3d > _avoidanceInfo;

		//enum ConflictEnum { NONE, COLL, COM, BOTH } _isThereConfl;

		// Optimization parameters ========================================================
		std::string _optimizerType;
		double _xTol;
		double _fTol;
		double _eqTol;
		double _ineqTol;
		unsigned _lastMaxIteration;
		unsigned _firstMaxIteration;
		unsigned _interMaxIteration;
		double _numDerivativeFactor;
		// double _h; // for numeric derivation

		// xde simulation related =========================================================
		double _updateTimeStep;
		double _firstPlanTimespan;
		//unsigned long long _updateCallCntr;
		double _currentPlanningTime;
		double _baseTime;
		double _timeOffset;

		// multithreading management ======================================================
		boost::mutex _planOngoingMutex;
		boost::thread _planThread;

		boost::mutex _comAIVsSetMutex;
		boost::mutex _knownRobotsMutex;
		boost::mutex _sharedSolMutex;

		unsigned _optIterCnter;

		nlopt_opt _opt;

		// In out streamming ==============================================================
		// std::ofstream _opt_log;
		// std::ofstream _eq_log;
		boost::property_tree::ptree _property_tree;


		Trajectory _sharedTraj;
		FlatVector _sharedBasePos;
		double _sharedPlanHor;
		double _sharedBaseTime;
		//double nbPointsCloseInTime;

		//double initTimeOfCurrPlan;


		/*____________________________________________ Methods ____________________________________________*/

	private:
		void _plan();
		//int findspan(const double t);
		bool _solveOptPbl();
		void _conflictEval();

		// int nbPointsCloseInTimeEval();
		bool _isAnyForbSpacInRobotsWayToTarget(MapObst forbiddenSpaces);
		bool _isForbSpaceInRobotsWay(const std::string& fsName, MapObst forbiddenSpaces);
		Common::CArray3d _getAngularVariationAndDistForAvoidance(const std::string& fsName, MapObst forbiddenSpaces);
		Common::CArray3d _estimateCollisionCenterAndRadius(const std::string& othRobName);	


	public:
		PathPlannerRecHor(std::string name, double updateTimeStep);

		~PathPlannerRecHor();

		void init(
			const PoseVector &initPose, // x, y, theta
			const VeloVector &initVelocity, // v, w
			const PoseVector &targetedPose, // x, y, theta
			const VeloVector &targetedVelocity, // v, w
			const VeloVector &maxVelocity, // v, w
			double compHorizon,
			double refPlanHorizon,
			unsigned noTimeSamples,
			unsigned noIntervNonNull);

		void init(
			const PoseVector &initPose, // x, y, theta
			const VeloVector &initVelocity, // v, w
			const PoseVector &targetedPose, // x, y, theta
			const VeloVector &targetedVelocity, // v, w
			const VeloVector &maxVelocity, // v, w
			const VeloVector &maxAcceleration, // dv, dw
			double compHorizon,
			double refPlanHorizon,
			unsigned noTimeSamples,
			unsigned noIntervNonNull);

		void setOption(const std::string& optionName, const double optionValue);
		void setOption(const std::string& optionName, const bool optionValue);
		void setOption(const std::string& optionName, const unsigned optionValue);
		void setOption(const std::string& optionName, const std::string& optionValue);

		void update(MapObst detectedObst, MapAIV otherVehicles, const Eigen::Displacementd & currentPose, const Eigen::Twistd & currertVelo, const double myRadius);

		inline void lockSharedSolMutex() { _sharedSolMutex.lock(); }
		inline void unlockSharedSolMutex() { _sharedSolMutex.unlock(); }

		// getters
		inline double getSharedBaseTime() const { return _sharedBaseTime; }
		inline PlanStage getSharedPlanStage() const { return _sharedPlanStage; }
		inline double getSharedPlanHor() const { return _sharedPlanHor; }
		inline const FlatVector& getSharedBasePos() const { return _sharedBasePos; }
		inline const Trajectory& getSharedTrajectory() const { return _sharedTraj; }
		
		inline const PoseVector& getSharedTargetedPose() const { return _sharedTargetedPose; }

		inline double getSharedRadius() const { return _sharedRadius; }
		// inline double getMaxLinAccel() const { return _maxAcceleration(FlatoutputMonocycle::linAccelIdx); }

		inline double getLinVelocity() const { return _velocityOutput(FlatoutputMonocycle::linSpeedIdx); }
		inline double getTargetedLinVelocity() const { return _targetedVelocity(FlatoutputMonocycle::linSpeedIdx); }
		inline double getMaxLinVelocity() const { return _maxVelocity(FlatoutputMonocycle::linSpeedIdx); }
		inline double getAngVelocity() const { return _velocityOutput(FlatoutputMonocycle::angSpeedIdx); }
		inline double getLinAccel() const { return _accelOutput(FlatoutputMonocycle::linSpeedIdx); }
		inline double getAngAccel() const { return _accelOutput(FlatoutputMonocycle::angSpeedIdx); }
		inline double getRobotObstacleSafetyDist() const { return _robotObstacleSafetyDist; }
		inline double getComRange() const { return _comRange; }
		inline double getInterRobSafDist() const { return _interRobotSafetyDist; }

		inline double getXPosition() const { return _poseOutput(FlatoutputMonocycle::posIdx); }
		inline double getYPosition() const { return _poseOutput(FlatoutputMonocycle::posIdx + 1); }
		inline double getOrientation() const { return _poseOutput(FlatoutputMonocycle::posIdx + 2); }

		static double objectFunc(unsigned n, const double *x, double *grad, void *my_func_data);
		static double objectFuncLS(unsigned n, const double *x, double *grad, void *my_func_data);
		static void eqFunc(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data);
		static void eqFuncLS(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data);
		static void ineqFunc(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data);
		static void ineqFuncLS(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data);
		
		template<class T>
		static void evalObj(double *result, unsigned n, T x, PathPlannerRecHor *context);
		template<class T>
		static void evalEq(double *result, unsigned n, T x, PathPlannerRecHor *context);
		template<class T>
		static void evalIneq(double *result, unsigned n, T x, PathPlannerRecHor *context);
		template<class T>
		static void evalObjLS(double *result, unsigned n, T x, PathPlannerRecHor *context);
		template<class T>
		static void evalEqLS(double *result, unsigned n, T x, PathPlannerRecHor *context);
		template<class T>
		static void evalIneqLS(double *result, unsigned n, T x, PathPlannerRecHor *context);

		void computeNumGrad(unsigned m, unsigned n, const double* x, double* grad, void (*eval)(double *, unsigned, volatile double*, PathPlannerRecHor*));

		void _findNextWayPt();

		//static void eval_eqHalf(double t, double *result, unsigned n, const double* x, void* data);
		//static void eval_ineq_coll(double *result, unsigned n, const double* x, void* data);
		// void eval_dobst_ineq(double *result, unsigned n, const double* x, void* data);
		// void ineqFuncColl(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data);
		// void eval_eqLS(double *result, unsigned n, const double* x, void* data);
		// void eval_ineqLS(double *result, unsigned n, const double* x, void* data);

		//inline bool isDone () {if (planStage == DONE) return true; else return false;}

		//inline double getInitTimeOfCurrPlan() const { return initTimeOfCurrPlan; }

		//inline MySpline getSpline() const { return auxSpline; }
		
		/*static double old_objectFunc(unsigned n, const double *x, double *grad, void *my_func_data);
		static void old_eqFunc(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data);
		static void old_ineqFunc(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data);*/
	};

	template<class T>
	void PathPlannerRecHor::evalObj(double *result, unsigned n, T x, PathPlannerRecHor *context)
	{
		try { boost::this_thread::interruption_point(); }
		catch (boost::thread_interrupted&) { nlopt_force_stop(context->_opt); }

		// Increment number of eval obj calls
		context->_optIterCnter++;

		// Update optimization trajectory with x
		context->_optTrajectory.update(x);

		// Create a Matrix consisting of the flat output and its needed derivatives for the instant Tp (1.0)
		NDerivativesMatrix derivFlat = context->_optTrajectory(context->_optPlanHorizon, FlatoutputMonocycle::flatDerivDeg);
		
		// Get pose at Tp from flatoutput
		PoseVector poseAtTp = FlatoutputMonocycle::flatToPose(derivFlat);

		// Position at Tp wrt robot at T0 to flat output wrt world frame of ref
		FlatVector flatAtTp = context->_rotMat2WRef*poseAtTp.block<FlatoutputMonocycle::posDim, 1>(FlatoutputMonocycle::posIdx, 0) + context->_latestFlat;

		// Compute euclidian distance to square from position at Tp and goal position which will be the cost
		double fx = pow((context->_wayPt - flatAtTp).norm(), 2);

		*result = fx;
	}

	template<class T>
	void PathPlannerRecHor::evalEq(double *result, unsigned n, T x, PathPlannerRecHor *context)
	{
		try { boost::this_thread::interruption_point(); }
		catch (boost::thread_interrupted&) { nlopt_force_stop(context->_opt); }

		// Update optimization trajectory with x
		context->_optTrajectory.update(x);

		// Create a Matrix consisting of the flat output and its needed derivatives for the instant T0 (0.0)
		NDerivativesMatrix derivFlatEq = context->_optTrajectory(1e-6, FlatoutputMonocycle::flatDerivDeg);

		// Get error from pose at T0 and pose at the end of the previous plan (or initial position if this is the very first plan)
		PoseVector diffPoseAtT0 = FlatoutputMonocycle::flatToPose(derivFlatEq);
		diffPoseAtT0.tail<1>()(0,0) = Common::wrapToPi(diffPoseAtT0.tail<1>()(0,0));

		VeloVector diffVelocityAtT0 = FlatoutputMonocycle::flatToVelocity(derivFlatEq) - context->_latestVelocity;

		int i;
		for (i = 0; i < FlatoutputMonocycle::poseDim; ++i)
		{
			result[i] = diffPoseAtT0[i];
		}
		for (int j = i; j - i < FlatoutputMonocycle::veloDim; ++j)
		{
			result[j] = diffVelocityAtT0[j - i];
		}
	}

	template<class T>
	void PathPlannerRecHor::evalIneq(double *result, unsigned n, T x, PathPlannerRecHor *context)
	{
		try { boost::this_thread::interruption_point(); }
		catch (boost::thread_interrupted&) { nlopt_force_stop(context->_opt); }

		// Update optimization trajectory with x
		context->_optTrajectory.update(x);

		// Matrix for storing flat output derivatives		
		Np1DerivativesMatrix derivFlat;
		NDerivativesMatrix derivFlatSmall;

		// Create Matrices for storing velocity and acceleration
		PoseVector pose;
		VeloVector velocity;
		AccelVector acceleration;

		// Get first "flatDerivDeg + 1" derivatives
		derivFlat = context->_optTrajectory(0.0, FlatoutputMonocycle::flatDerivDeg + 1);

		// ACCELERATION AT 0.0
		acceleration = FlatoutputMonocycle::flatToAcceleration(derivFlat);

		unsigned i, j, k;
		for (i = 0; i < FlatoutputMonocycle::accelDim; ++i)
		{
			result[i] = abs(acceleration(i,0)) - context->_maxAcceleration(i, 0);
		}

		unsigned nAcc = i;
		unsigned ieqPerSample = (FlatoutputMonocycle::veloDim + FlatoutputMonocycle::accelDim + context->_detectedObstacles.size() + context->_collisionAIVs.size() + context->_comOutAIVs.size());

		//#pragma omp parallel for <== DO NOT USE IT, BREAKS THE EVALUATION OF CONSTRAINTS SOMEHOW
		for (i = 1; i <= int(context->_nTimeSamples); ++i)
		{
			derivFlat = context->_optTrajectory(double(i) / context->_nTimeSamples * context->_optPlanHorizon, FlatoutputMonocycle::flatDerivDeg + 1);

			derivFlatSmall = derivFlat.block<FlatoutputMonocycle::flatDim, FlatoutputMonocycle::flatDerivDeg + 1>(0, 0);

			pose = FlatoutputMonocycle::flatToPose(derivFlatSmall);
			velocity = FlatoutputMonocycle::flatToVelocity(derivFlatSmall);
			acceleration = FlatoutputMonocycle::flatToAcceleration(derivFlat);

			for (j = 0; j < FlatoutputMonocycle::accelDim; ++j)
			{
				result[nAcc + j + (i - 1)*ieqPerSample] = abs(acceleration(j,0)) - context->_maxAcceleration(j, 0);
			}
			for (k = j; k - j < FlatoutputMonocycle::veloDim; ++k)
			{
				result[nAcc + k + (i - 1)*ieqPerSample] = abs(velocity(k - j, 0)) - context->_maxVelocity(k - j, 0);
			}

			MapObst::iterator obsIt;
			for (obsIt = context->_detectedObstacles.begin(), j = k; j - k < context->_detectedObstacles.size(); ++j, ++obsIt)
			{
				result[nAcc + j + (i - 1)*ieqPerSample] =
					-1. * obsIt->second->distToAIV(context->_rotMat2WRef*pose.head<2>() + context->_latestPose.head<2>(), context->_robotObstacleSafetyDist + context->_radius);
			}

			MapAIV::iterator aivIt;
			for (aivIt = context->_collisionAIVs.begin(), k = j; k - j < context->_collisionAIVs.size(); ++k, ++aivIt)
			{
				// std::cout << "Insinde conllision\n";
				double distanceInterRobots = (context->_rotMat2WRef*derivFlat.col(0) +context->_latestPose.head<2>() - context->_pathsFromConflictualAIVs[aivIt->first].col(i-1)).norm();

				// std::cout << "distanceInterRobots " << distanceInterRobots << std::endl;

				aivIt->second->getPathPlanner()->lockSharedSolMutex();
				double radii = aivIt->second->getPathPlanner()->getSharedRadius() + context->_radius;
				aivIt->second->getPathPlanner()->unlockSharedSolMutex();

				double dSecurity = max(context->_interRobotSafetyDist, aivIt->second->getPathPlanner()->getInterRobSafDist());

				result[nAcc + k + (i - 1)*ieqPerSample] = - distanceInterRobots + radii + dSecurity;

				 // - distanceInterRobots + ;
				// std::cout << "result[nAcc + k + (i - 1)*ieqPerSample] " << result[nAcc + k + (i - 1)*ieqPerSample] << std::endl;

			}
			
			// for (aivIt = context->_comOutAIVs.begin(), j = k; j - k < context->_comOutAIVs.size(); ++j, ++aivIt)
			// {


			// }

		}
	}

	template<class T>
	void PathPlannerRecHor::evalObjLS(double *result, unsigned n, T x, PathPlannerRecHor *context)
	{
		try { boost::this_thread::interruption_point(); }
		catch (boost::thread_interrupted&) { nlopt_force_stop(context->_opt); }

		// Increment number of eval obj calls
		context->_optIterCnter++;

		*result = x[0]*x[0];
	}

	template<class T>
	void PathPlannerRecHor::evalEqLS(double *result, unsigned n, T x, PathPlannerRecHor *context)
	{
		try { boost::this_thread::interruption_point(); }
		catch (boost::thread_interrupted&) { nlopt_force_stop(context->_opt); }

		// Update optimization trajectory with x
		context->_optTrajectory.update(&(x[1]), x[0]);

		// Create a Matrix consisting of the flat output and its needed derivatives for the instant T0 (0.0)
		NDerivativesMatrix derivFlatEq = context->_optTrajectory(1e-6, FlatoutputMonocycle::flatDerivDeg);

		PoseVector diffPose = FlatoutputMonocycle::flatToPose(derivFlatEq);
		diffPose.tail<1>()(0,0) = Common::wrapToPi(diffPose.tail<1>()(0,0));

		VeloVector diffVelocity = FlatoutputMonocycle::flatToVelocity(derivFlatEq) - context->_latestVelocity;

		int i, j;
		for (i = 0; i < FlatoutputMonocycle::poseDim; ++i)
		{
			result[i] = diffPose[i];
		}
		for (j = i; j - i < FlatoutputMonocycle::veloDim; ++j)
		{
			result[j] = diffVelocity[j - i];
		}

		// Create a Matrix consisting of the flat output and its needed derivatives for the instant T0 (0.0)
		derivFlatEq = context->_optTrajectory(x[0]-1e-6, FlatoutputMonocycle::flatDerivDeg);

		PoseVector poseAtTfWRTMySelf = FlatoutputMonocycle::flatToPose(derivFlatEq);
		
		// 2 equivalent things:
		// diff = [Rw*p + latest_p, theta + latest_theta]T - targetpose
		// diff = [p, theta]T - [Rr*(target_p - latest_p), target_theta - latest_theta]T
		diffPose = poseAtTfWRTMySelf - (PoseVector() << context->_rotMat2RRef*(context->_targetedPose.head<2>() - context->_latestFlat), Common::wrapToPi(context->_targetedPose.tail<1>()(0,0) - context->_latestPose.tail<1>()(0,0))).finished();
		//diffPose = (PoseVector() << context->_rotMat2WRef * poseAtTfWRTMySelf.head<2>() + context->_latestFlat, _signed_angle(poseAtTfWRTMySelf.tail<1>()(0,0)+context->_latestPose.tail<1>()(0,0))).finished() - context->_targetedPose;
		//std::cout << context->_rotMat2RRef * context->_rotMat2WRef << std::endl;

		diffVelocity = FlatoutputMonocycle::flatToVelocity(derivFlatEq) - context->_targetedVelocity;

		for (i = j; i - j < FlatoutputMonocycle::poseDim; ++i)
		{
			result[i] = diffPose[i - j];
		}
		for (j = i; j - i < FlatoutputMonocycle::veloDim; ++j)
		{
			result[j] = diffVelocity[j - i];
		}
	}

	template<class T>
	void PathPlannerRecHor::evalIneqLS(double *result, unsigned n, T x, PathPlannerRecHor *context)
	{
		try { boost::this_thread::interruption_point(); }
		catch (boost::thread_interrupted&) { nlopt_force_stop(context->_opt); }

		//std::cout << "::evalIneqLS call" << std::endl;
		// Update optimization trajectory with x
		context->_optTrajectory.update(&(x[1]), x[0]);
		
		// INEQUATIONS
		Np1DerivativesMatrix derivFlat;
		NDerivativesMatrix derivFlatSmall;

		// Create Matrices for storing velocity and acceleration
		VeloVector velocity;
		AccelVector acceleration;
		PoseVector pose;

		result[0] = -x[0]; // time has to be positive

		// ACCELERATION AT 0.0
		derivFlat = context->_optTrajectory(0.0, FlatoutputMonocycle::flatDerivDeg + 1);

		acceleration = FlatoutputMonocycle::flatToAcceleration(derivFlat);

		unsigned i, j, k;
		for (i = 1; i < FlatoutputMonocycle::accelDim; ++i)
		{
			result[i] = abs(acceleration(i,0)) - context->_maxAcceleration(i, 0);
			// std::cout << "r[" << i << "]=" << result[i] << std::endl;
		}

		// ACCELERATION AT Tf
		derivFlat = context->_optTrajectory(x[0], FlatoutputMonocycle::flatDerivDeg + 1);

		acceleration = FlatoutputMonocycle::flatToAcceleration(derivFlat);

		for (j = i; j - i < FlatoutputMonocycle::accelDim; ++j)
		{
			result[j] = abs(acceleration(j,0)) - context->_maxAcceleration(j,0);
			// std::cout << "r[" << j << "]=" << result[j] << std::endl;
		}

		unsigned nAcc = j;

		unsigned ieqPerSample = (FlatoutputMonocycle::veloDim + FlatoutputMonocycle::accelDim + context->_detectedObstacles.size() + context->_collisionAIVs.size() + context->_comOutAIVs.size());

		for (int i = 1; i < int(context->_nTimeSamples); ++i)
		{
			derivFlat = context->_optTrajectory(double(i) / context->_nTimeSamples * context->_optPlanHorizon, FlatoutputMonocycle::flatDerivDeg + 1);

			derivFlatSmall = derivFlat.block<FlatoutputMonocycle::flatDim, FlatoutputMonocycle::flatDerivDeg + 1>(0, 0);

			pose = FlatoutputMonocycle::flatToPose(derivFlatSmall);
			velocity = FlatoutputMonocycle::flatToVelocity(derivFlatSmall);
			acceleration = FlatoutputMonocycle::flatToAcceleration(derivFlat);

			for (j = 0; j < FlatoutputMonocycle::accelDim; ++j)
			{
				result[nAcc + j + (i - 1)*ieqPerSample] = abs(acceleration(j,0)) - context->_maxAcceleration(j, 0);
				// std::cout << "r[" << nAcc + j + (i - 1)*ieqPerSample << "]=" << result[nAcc + j + (i - 1)*ieqPerSample] << std::endl;
			}

			for (k = j; k - j < FlatoutputMonocycle::veloDim; ++k)
			{
				result[nAcc + k + (i - 1)*ieqPerSample] = abs(velocity(k - j, 0)) - context->_maxVelocity(k - j, 0);
				// std::cout << "r[" << nAcc + k + (i - 1)*ieqPerSample << "]=" << result[nAcc + k + (i - 1)*ieqPerSample] << std::endl;
			}

			MapObst::iterator it;
			for (it = context->_detectedObstacles.begin(), j = k; j - k < context->_detectedObstacles.size(); ++j, ++it)
			{
				result[nAcc + j + (i - 1)*ieqPerSample] =
					-1. * it->second->distToAIV(context->_rotMat2WRef*pose.head<2>() + context->_latestPose.head<2>(), context->_robotObstacleSafetyDist + context->_radius);
				// std::cout << "r[" << nAcc + j + (i - 1)*ieqPerSample << "]=" << result[nAcc + j + (i - 1)*ieqPerSample] << std::endl;
			}

			
			MapAIV::iterator aivIt;
			for (aivIt = context->_collisionAIVs.begin(), k = j; k - j < context->_collisionAIVs.size(); ++k, ++aivIt)
			{
				// std::cout << "Insinde conllision\n";
				double distanceInterRobots = (context->_rotMat2WRef*derivFlat.col(0) +context->_latestPose.head<2>() - context->_pathsFromConflictualAIVs[aivIt->first].col(i-1)).norm();

				// std::cout << "distanceInterRobots " << distanceInterRobots << std::endl;

				aivIt->second->getPathPlanner()->lockSharedSolMutex();
				double radii = aivIt->second->getPathPlanner()->getSharedRadius() + context->_radius;
				aivIt->second->getPathPlanner()->unlockSharedSolMutex();

				double dSecurity = max(context->_interRobotSafetyDist, aivIt->second->getPathPlanner()->getInterRobSafDist());

				result[nAcc + k + (i - 1)*ieqPerSample] = - distanceInterRobots + radii + dSecurity;

			}
		}
	}
	

}

#endif // __AIV_PATHPLANNERRECHOR_HPP__

// cmake:sourcegroup=PathPlanner
