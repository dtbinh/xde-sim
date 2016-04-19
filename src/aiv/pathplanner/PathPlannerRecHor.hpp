#ifndef __AIV_PATHPLANNERRECHOR_HPP__
#define __AIV_PATHPLANNERRECHOR_HPP__
#pragma once

#include "aiv/pathplanner/PathPlanner.hpp"
#include "aiv/pathplanner/FlatoutputMonocycle.hpp"
#include "aiv/pathplanner/Trajectory.hpp"
#include <boost/thread.hpp>
#include <fstream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <set>

//Spline< dim, degree >

namespace aiv {

	typedef Eigen::Matrix< double, FlatoutputMonocycle::flatDim, FlatoutputMonocycle::flatDerivDeg + 1 > NDerivativesMatrix;
	typedef Eigen::Matrix< double, FlatoutputMonocycle::flatDim, FlatoutputMonocycle::flatDerivDeg + 2 > Np1DerivativesMatrix;
	typedef Eigen::Matrix< double, FlatoutputMonocycle::poseDim, 1 > PoseVector;
	typedef Eigen::Matrix< double, FlatoutputMonocycle::flatDim, 1 > FlatVector;
	typedef Eigen::Matrix< double, FlatoutputMonocycle::veloDim, 1 > VeloVector;
	typedef Eigen::Matrix< double, FlatoutputMonocycle::accelDim, 1 > AccelVector;

	class Obstacle;
	class AIV;

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
		FlatVector _targetedFlat;
		VeloVector _targetedVelocity;
		VeloVector _maxVelocity;
		AccelVector _maxAcceleration;
		FlatVector _latestFlat;
		PoseVector _latestPose;
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
		double _optPlanHorizon;    // planning horizon for using by the optimization process

		bool _planLastPlan; // last planning flag
		double _lastStepMinDist; // parameter for stop condition

		double _conflictFreePathDeviation;
		double _interRobotSafetyDist;
		double _robotObstacleSafetyDist;
		double _maxStraightDist;

		enum PlanStage { INIT, INTER, FINAL, DONE } _planStage;
		bool _waitForThread;

		//double _estTime;
		
		std::map< std::string, Obstacle* > _detectedObstacles;
		//std::map<std::string, std::vector<int> > conflictInfo;
		// std::map< std::string, AIV* > _collisionAIVs;
		// std::map< std::string, AIV* > _comOutAIVs;
		// std::set< std::string > _comAIVsSet;

		double _comRange;
		//double secRho; // secure distance from other robots and obstacles

		unsigned _nTimeSamples;     // number of  time samples taken within a planning horizon
		int _executingPlanIdx;
		int _ongoingPlanIdx;

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
		// double _h; // for numeric derivation

		// xde simulation related =========================================================
		double _updateTimeStep;
		double _firstPlanTimespan;
		unsigned long long _updateCallCntr;

		// multithreading management ======================================================
		boost::mutex _planOngoingMutex;
		boost::thread _planThread;


		// In out streamming ==============================================================
		// std::ofstream _opt_log;
		// std::ofstream _eq_log;
		boost::property_tree::ptree _property_tree;

		//double nbPointsCloseInTime;

		//double initTimeOfCurrPlan;


		/*____________________________________________ Methods ____________________________________________*/

	private:
		void _plan();
		//int findspan(const double t);
		void _solveOptPbl();
		//void _conflictEval(std::map<std::string, AIV *> otherVehicles, const Eigen::Displacementd & myRealPose);
		// int nbPointsCloseInTimeEval();

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

		void update(std::map<std::string, Obstacle *> detectedObst,
				std::map<std::string, AIV *> otherVehicles,
				const Eigen::Displacementd & myRealPose); //const Eigen::Displacementd & realPose, const Eigen::Twistd &realVelocity);

		// getters
		inline double getLinVelocity() const { return _velocityOutput(FlatoutputMonocycle::linSpeedIdx); }
		inline double getMaxLinVelocity() const { return _maxVelocity(FlatoutputMonocycle::linSpeedIdx); }
		inline double getAngVelocity() const { return _velocityOutput(FlatoutputMonocycle::angSpeedIdx); }
		inline double getLinAccel() const { return _accelOutput(FlatoutputMonocycle::linSpeedIdx); }
		inline double getAngAccel() const { return _accelOutput(FlatoutputMonocycle::angSpeedIdx); }
		inline double getRobotObstacleSafetyDist() const { return _robotObstacleSafetyDist; }
		inline double getComRange() const { return _comRange; }

		inline double getXPosition() const { return _poseOutput(FlatoutputMonocycle::posIdx); }
		inline double getYPosition() const { return _poseOutput(FlatoutputMonocycle::posIdx + 1); }
		inline double getOrientation() const { return _poseOutput(FlatoutputMonocycle::posIdx + 2); }

		static double objectFunc(unsigned n, const double *x, double *grad, void *my_func_data);
		static double objectFuncLS(unsigned n, const double *x, double *grad, void *my_func_data);
		static void eval_eq(double *result, unsigned n, const double* x, void* data);
		//static void eval_eqHalf(double t, double *result, unsigned n, const double* x, void* data);
		static void eqFunc(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data);
		static void eval_ineq(double *result, unsigned n, const double* x, void* data);
		static void eval_ineq_coll(double *result, unsigned n, const double* x, void* data);
		static void eval_dobst_ineq(double *result, unsigned n, const double* x, void* data);
		static void ineqFunc(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data);
		static void ineqFuncColl(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data);
		static void eval_eqLS(double *result, unsigned n, const double* x, void* data);
		static void eqFuncLS(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data);
		static void eval_ineqLS(double *result, unsigned n, const double* x, void* data);
		static void ineqFuncLS(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data);

		//inline bool isDone () {if (planStage == DONE) return true; else return false;}

		//inline double getInitTimeOfCurrPlan() const { return initTimeOfCurrPlan; }

		//inline MySpline getSpline() const { return auxSpline; }
		
		/*static double old_objectFunc(unsigned n, const double *x, double *grad, void *my_func_data);
		static void old_eqFunc(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data);
		static void old_ineqFunc(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data);*/
	};

}

#endif // __AIV_PATHPLANNERRECHOR_HPP__

// cmake:sourcegroup=PathPlanner
