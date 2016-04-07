#ifndef __AIV_PATHPLANNERRECHOR_HPP__
#define __AIV_PATHPLANNERRECHOR_HPP__
#pragma once

#include "aiv/pathplanner/PathPlanner.hpp"
#include "aiv/pathplanner/FlatoutputMonocycle.hpp"
#include <unsupported/Eigen/Splines>
#include <boost/thread.hpp>
#include <fstream>
//#include "sys/timer.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <set>

//Spline< dim, degree >

typedef Eigen::Spline< double, aiv::FlatoutputMonocycle::flatDim, aiv::FlatoutputMonocycle::flatDerivDeg + 1 > MySpline;
typedef Eigen::Matrix< double, aiv::FlatoutputMonocycle::flatDim, aiv::FlatoutputMonocycle::flatDerivDeg + 1 > NDerivativesMatrix;
typedef Eigen::Matrix< double, aiv::FlatoutputMonocycle::flatDim, aiv::FlatoutputMonocycle::flatDerivDeg + 2 > Np1DerivativesMatrix;
typedef Eigen::Matrix< double, aiv::FlatoutputMonocycle::poseDim, 1 > PoseVector;
typedef Eigen::Matrix< double, aiv::FlatoutputMonocycle::velocityDim, 1 > VelVector;
typedef Eigen::Matrix< double, aiv::FlatoutputMonocycle::accelerationDim, 1 > AccVector;

namespace aiv {

	class Obstacle;
	class AIV;

	class PathPlannerRecHor : public PathPlanner
	{

		/*____________________________________________ Members ____________________________________________*/
	public:
		static const unsigned splDegree = aiv::FlatoutputMonocycle::flatDerivDeg + 1; // spline degree defined by Defoort is = splDegree + 1    
		static const unsigned splDim = aiv::FlatoutputMonocycle::flatDim; // spline dimension
	private:

		// Solution "flat output" b-spline representation
		//typedef Eigen::Spline< double, splDim, splDegree > MySpline;

		// Planning parameters
		Eigen::Matrix< double, aiv::FlatoutputMonocycle::poseDim, 1 > initPose;
		Eigen::Matrix< double, aiv::FlatoutputMonocycle::flatDim, 1 > initFlat;
		Eigen::Matrix< double, aiv::FlatoutputMonocycle::velocityDim, 1 > initVelocity;
		Eigen::Matrix< double, aiv::FlatoutputMonocycle::poseDim, 1 > targetedPose;
		Eigen::Matrix< double, aiv::FlatoutputMonocycle::flatDim, 1 > targetedFlat;
		Eigen::Matrix< double, aiv::FlatoutputMonocycle::velocityDim, 1 > targetedVelocity;
		Eigen::Matrix< double, aiv::FlatoutputMonocycle::velocityDim, 1 > maxVelocity;
		Eigen::Matrix< double, aiv::FlatoutputMonocycle::velocityDim, 1 > maxAcceleration;
		Eigen::Matrix< double, aiv::FlatoutputMonocycle::flatDim, 1 > latestFlat;
		Eigen::Matrix< double, aiv::FlatoutputMonocycle::poseDim, 1 > latestPose;
		Eigen::Matrix< double, aiv::FlatoutputMonocycle::poseDim, 1 > finalPoseForFuturePlan;
		Eigen::Matrix< double, aiv::FlatoutputMonocycle::poseDim, 1 > initPoseForCurrPlan;
		Eigen::Matrix< double, aiv::FlatoutputMonocycle::velocityDim, 1 > latestVelocity;
		bool mindAcceleration;

		double compHorizon;         // computation horizon
		double refPlanHorizon;      // planning horizon
		double finalPlanHorizon;    // planning horizon
		double planHorizon;         // planning horizon
		bool lastPlanComputed;
		unsigned noTimeSamples;     // number of  time samples taken within a planning horizon
		unsigned noIntervNonNull;   // number of non null knots intervals
		unsigned noCtrlPts;         // number of control points

		double lastStepMinDist;
		double xTol;
		double fTol;
		double eqTol;
		double ineqTol;
		unsigned lastMaxIteration;
		unsigned firstMaxIteration;
		unsigned interMaxIteration;
		double conflictFreePathDeviation;
		double interRobotSafetyDist;
		double maxStraightDist;

		//xde::sys::Timer plTimer;
		double updateTimeStep;
		double firstPlanTimespan;
		unsigned long long updateCallCntr;

		boost::mutex planOngoingMutex;
		enum ConflictEnum { NONE, COLL, COM, BOTH } isThereConfl;

		enum PlanStage { INIT, INTER, FINAL, DONE } planStage;
		bool waitPlanning;

		MySpline solSpline;
		MySpline auxSpline;
		double estTime;

		boost::thread planThread;

		int ongoingPlanIdx;
		int executingPlanIdx;

		// Planning outputs
		Eigen::Matrix< double, aiv::FlatoutputMonocycle::velocityDim, 1 > nextVelocityRef;
		Eigen::Matrix< double, aiv::FlatoutputMonocycle::accelerationDim, 1 > nextAccRef;
		Eigen::Matrix< double, aiv::FlatoutputMonocycle::poseDim, 1 > nextPoseRef;

		std::ofstream opt_log;
		std::ofstream eq_log;

		std::string optimizerType;

		double h;

		boost::property_tree::ptree property_tree;

		std::map< std::string, Obstacle* > detectedObstacles;
		std::map< std::string, AIV* > collisionAIVs;
		std::map< std::string, AIV* > comOutAIVs;
		std::set< std::string > comAIVsSet;

		double comRange;
		double secRho;

		double nbPointsCloseInTime;

		double initTimeOfCurrPlan;

		std::map<std::string, std::vector<int> > conflictInfo;

		/*____________________________________________ Methods ____________________________________________*/

	private:
		void plan();
		Eigen::Array< double, 1, Eigen::Dynamic > genKnots(const double initT, const double finalT, const bool nonUniform);
		int findspan(const double t);
		void solveOptPbl();
		void conflictEval(std::map<std::string, AIV *> otherVehicles,
				const Eigen::Displacementd & myRealPose);
		int nbPointsCloseInTimeEval();

	public:
		PathPlannerRecHor(std::string name, double updateTimeStep);

		~PathPlannerRecHor();

		void init(
			const Eigen::Matrix< double, aiv::FlatoutputMonocycle::poseDim, 1 > &initPose, // x, y, theta
			const Eigen::Matrix< double, aiv::FlatoutputMonocycle::velocityDim, 1 > &initVelocity, // v, w
			const Eigen::Matrix< double, aiv::FlatoutputMonocycle::poseDim, 1 > &targetedPose, // x, y, theta
			const Eigen::Matrix< double, aiv::FlatoutputMonocycle::velocityDim, 1 > &targetedVelocity, // v, w
			const Eigen::Matrix< double, aiv::FlatoutputMonocycle::velocityDim, 1 > &maxVelocity, // v, w
			double compHorizon,
			double refPlanHorizon,
			unsigned noTimeSamples,
			unsigned noIntervNonNull);

		void init(
			const Eigen::Matrix< double, aiv::FlatoutputMonocycle::poseDim, 1 > &initPose, // x, y, theta
			const Eigen::Matrix< double, aiv::FlatoutputMonocycle::velocityDim, 1 > &initVelocity, // v, w
			const Eigen::Matrix< double, aiv::FlatoutputMonocycle::poseDim, 1 > &targetedPose, // x, y, theta
			const Eigen::Matrix< double, aiv::FlatoutputMonocycle::velocityDim, 1 > &targetedVelocity, // v, w
			const Eigen::Matrix< double, aiv::FlatoutputMonocycle::velocityDim, 1 > &maxVelocity, // v, w
			const Eigen::Matrix< double, aiv::FlatoutputMonocycle::velocityDim, 1 > &maxAcceleration, // dv, dw
			double compHorizon,
			double refPlanHorizon,
			unsigned noTimeSamples,
			unsigned noIntervNonNull);

		void setOption(std::string optionName, double optionValue);
		void setOption(std::string optionName, std::string optionValue);

		void update(std::map<std::string, Obstacle *> detectedObst,
				std::map<std::string, AIV *> otherVehicles,
				const Eigen::Displacementd & myRealPose); //const Eigen::Displacementd & realPose, const Eigen::Twistd &realVelocity);

		inline double getLinVelocity() { return this->nextVelocityRef(aiv::FlatoutputMonocycle::linSpeedIdx); }
		inline double getMaxLinVelocity() { return this->maxVelocity(aiv::FlatoutputMonocycle::linSpeedIdx); }
		inline double getAngVelocity() { return this->nextVelocityRef(aiv::FlatoutputMonocycle::angSpeedIdx); }
		inline double getLinAccel() { return this->nextAccRef(aiv::FlatoutputMonocycle::linSpeedIdx); }
		inline double getAngAccel() { return this->nextAccRef(aiv::FlatoutputMonocycle::angSpeedIdx); }
		inline double getSecRho() { return this->secRho; }
		inline double getComRange() { return this->comRange; }

		inline double getXPosition() { return this->nextPoseRef(aiv::FlatoutputMonocycle::posIdx); }
		inline double getYPosition() { return this->nextPoseRef(aiv::FlatoutputMonocycle::posIdx + 1); }
		inline double getOrientation() { return this->nextPoseRef(aiv::FlatoutputMonocycle::posIdx + 2); }

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

		inline bool isDone () {if (planStage == DONE) return true; else return false;}

		inline double getInitTimeOfCurrPlan() { return initTimeOfCurrPlan; }

		inline MySpline getSpline() { return auxSpline; }
		
		/*static double old_objectFunc(unsigned n, const double *x, double *grad, void *my_func_data);
		static void old_eqFunc(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data);
		static void old_ineqFunc(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data);*/
	};


}

#endif // __AIV_PATHPLANNERRECHOR_HPP__

// cmake:sourcegroup=PathPlanner
