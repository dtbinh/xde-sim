#include "aiv/pathplanner/PathPlannerRecHor.hpp"
#include "aiv/pathplanner/FlatoutputMonocycle.hpp"
#include "aiv/obstacle/Obstacle.hpp"
#include "aiv/robot/AIV.hpp"
#include "aiv/pathplanner/constrJac.hpp"
#include "aiv/pathplanner/eq_cons_jac.hpp"
#include <nlopt.h>
#include <omp.h>
#include <boost/foreach.hpp>
#include <algorithm>
#include <cstdlib>
#include <cmath>

#define foreach_ BOOST_FOREACH

#ifndef HAVE_CONFIG_H
#define HAVE_CONFIG_H
#endif

using boost::property_tree::ptree;

namespace aiv
{

	PathPlannerRecHor::PathPlannerRecHor(std::string name, double updateTimeStep)
		: PathPlanner(name)
		, updateTimeStep(updateTimeStep)
		, updateCallCntr(0)
		, lastStepMinDist(0.5)
		, xTol(0.001)
		, fTol(0.001)
		, eqTol(0.001)
		, ineqTol(0.001)
		, lastMaxIteration(25)
		, firstMaxIteration(50)
		, interMaxIteration(20)
		, conflictFreePathDeviation(4.0)
		, interRobotSafetyDist(0.1)
		, nextVelocityRef(Eigen::Matrix< double, aiv::FlatoutputMonocycle::velocityDim, 1 >::Zero())
		, nextAccRef(Eigen::Matrix< double, aiv::FlatoutputMonocycle::accelerationDim, 1 >::Zero())
		, nextPoseRef(Eigen::Matrix< double, aiv::FlatoutputMonocycle::poseDim, 1 >::Zero())
		, planStage(INIT)
		, ongoingPlanIdx(-1)
		, executingPlanIdx(-1)
		, isThereConfl(NONE)
		, lastPlanComputed(false)
		, optimizerType("SLSQP")
		, h(1e-15)
		, comRange(15)
		, secRho(0.55)
	{
		this->planOngoingMutex.initialize();
		//this->opt_log.open("optlog///opt_log.txt");
		this->eq_log.open("optlog/eq_log.txt");

		read_xml("../../../../src/aiv/output.xml", this->property_tree);
	}

	PathPlannerRecHor::~PathPlannerRecHor()
	{
		//this->opt_log.close();
		this->eq_log.close();
	}

	void  PathPlannerRecHor::init(
		const Eigen::Matrix< double, aiv::FlatoutputMonocycle::poseDim, 1 > &initPose, // x, y, theta
		const Eigen::Matrix< double, aiv::FlatoutputMonocycle::velocityDim, 1 > &initVelocity, // v, w
		const Eigen::Matrix< double, aiv::FlatoutputMonocycle::poseDim, 1 > &targetedPose, // x, y, theta
		const Eigen::Matrix< double, aiv::FlatoutputMonocycle::velocityDim, 1 > &targetedVelocity, // v, w
		const Eigen::Matrix< double, aiv::FlatoutputMonocycle::velocityDim, 1 > &maxVelocity, // v, w
		double compHorizon,
		double refPlanHorizon,
		unsigned noTimeSamples,
		unsigned noIntervNonNull)
	{
		this->initPose = initPose;
		this->initFlat = aiv::FlatoutputMonocycle::poseToFlat(this->initPose);
		this->latestFlat = this->initFlat;
		this->initVelocity = initVelocity;
		this->targetedPose = targetedPose;

		this->nextPoseRef = initPose;
		this->nextVelocityRef = initVelocity;

		this->targetedFlat = aiv::FlatoutputMonocycle::poseToFlat(this->targetedPose);
		this->targetedVelocity = targetedVelocity;
		this->maxVelocity = maxVelocity;
		this->mindAcceleration = false;

		this->compHorizon = compHorizon; // computation horizon
		this->refPlanHorizon = refPlanHorizon; // planning horizon
		this->finalPlanHorizon = refPlanHorizon; // planning horizon
		this->planHorizon = refPlanHorizon; // planning horizon
		this->noTimeSamples = noTimeSamples; // number of  time samples taken within a planning horizon
		this->noIntervNonNull = noIntervNonNull; // number of non null knots intervals
		this->noCtrlPts = noIntervNonNull + this->splDegree;
		this->maxStraightDist = refPlanHorizon * maxVelocity(aiv::FlatoutputMonocycle::linSpeedIdx);

		this->latestFlat = this->initFlat;
		this->latestPose = initPose;
		this->initPoseForCurrPlan = initPose;
		this->latestVelocity = initVelocity;

		//opt_log << "init: PP Initializing..." << std::endl;
		//plTimer.ResetTime();
	}

	void  PathPlannerRecHor::init(
		const Eigen::Matrix< double, aiv::FlatoutputMonocycle::poseDim, 1 > &initPose, // x, y, theta
		const Eigen::Matrix< double, aiv::FlatoutputMonocycle::velocityDim, 1 > &initVelocity, // v, w
		const Eigen::Matrix< double, aiv::FlatoutputMonocycle::poseDim, 1 > &targetedPose, // x, y, theta
		const Eigen::Matrix< double, aiv::FlatoutputMonocycle::velocityDim, 1 > &targetedVelocity, // v, w
		const Eigen::Matrix< double, aiv::FlatoutputMonocycle::velocityDim, 1 > &maxVelocity, // v, w
		const Eigen::Matrix< double, aiv::FlatoutputMonocycle::velocityDim, 1 > &maxAcceleration, // dv, dw
		double compHorizon,
		double refPlanHorizon,
		unsigned noTimeSamples,
		unsigned noIntervNonNull)
	{
		init(initPose, initVelocity, targetedPose, targetedVelocity, maxVelocity, compHorizon, refPlanHorizon, noTimeSamples, noIntervNonNull);
		this->maxAcceleration = maxAcceleration;
		//opt_log << "init: Using acceleration." << std::endl;
	}

	void  PathPlannerRecHor::setOption(std::string optionName, std::string optionValue)
	{
		if (optionName == "optimizerType")
		{
			this->optimizerType = optionValue;
			//opt_log << "init: Using \"" << optionValue << "\" optimizer" << std::endl;
			//std::cout << "init: Using \"" << optionValue << "\" optimizer" << std::endl;
		}
	}

	void  PathPlannerRecHor::setOption(std::string optionName, double optionValue)
	{
		if (optionName == "lastStepMinDist")
		{
			this->lastStepMinDist = optionValue;
		}
		else if (optionName == "xTol")
		{
			this->xTol = optionValue;
		}
		else if (optionName == "fTol")
		{
			this->fTol = optionValue;
		}
		else if (optionName == "eqTol")
		{
			this->eqTol = optionValue;
		}
		else if (optionName == "ineqTol")
		{
			this->ineqTol = optionValue;
		}
		else if (optionName == "conflictFreePathDeviation")
		{
			this->conflictFreePathDeviation = optionValue;
		}
		else if (optionName == "interRobotSafetyDist")
		{
			this->interRobotSafetyDist = optionValue;
		}
		else if (optionName == "lastMaxIteration")
		{
			this->lastMaxIteration = unsigned(optionValue);
		}
		else if (optionName == "firstMaxIteration")
		{
			this->firstMaxIteration = unsigned(optionValue);
		}
		else if (optionName == "interMaxIteration")
		{
			this->interMaxIteration = unsigned(optionValue);
		}
		else if (optionName == "offsetTime")
		{
			this->firstPlanTimespan = optionValue;
		}
		else if (optionName == "waitPlanning")
		{
			this->waitPlanning = bool(optionValue);
		}
		else if (optionName == "numderivativedelta")
		{
			this->h = optionValue;
		}
	}

	void PathPlannerRecHor::conflictEval(std::map<std::string, AIV *>
			otherVehicles, const Eigen::Displacementd & myRealPose)
	{
		// Clear conflicts robots maps
		this->collisionAIVs.clear();
		this->comOutAIVs.clear();

		Eigen::Vector2d curr2dPosition;
		curr2dPosition << myRealPose.x(), myRealPose.y();

		Eigen::Vector2d otherRobPosition;

		for (std::map<std::string, AIV *>::iterator it = otherVehicles.begin();
				it != otherVehicles.end(); ++it)
		{
			otherRobPosition << it->second->getCurrentPosition().x(),
					it->second->getCurrentPosition().y();

			double dInterRobot = (curr2dPosition - otherRobPosition).norm();

			double maxLinSpeed = std::max(this->maxVelocity[
					aiv::FlatoutputMonocycle::linSpeedIdx],
					it->second->getPathPlanner()->getMaxLinVelocity());

			double dSecutiry = this->secRho +
					it->second->getPathPlanner()->getSecRho();

			if (dInterRobot <= dSecutiry + maxLinSpeed*this->planHorizon)
			{
				this->collisionAIVs[it->first] = it->second;
			}

			double minComRange = std::min(this->comRange,
					it->second->getPathPlanner()->getComRange());

			if (this->comAIVsSet.find(it->first) != this->comAIVsSet.end() &&
				dInterRobot + maxLinSpeed*this->planHorizon >= minComRange)
			{
				this->comOutAIVs[it->first] = it->second;
			}
		}
	}

	void PathPlannerRecHor::update(std::map<std::string, Obstacle *> detectedObst,
		std::map<std::string, AIV *> otherVehicles,
		const Eigen::Displacementd & myRealPose) //const Eigen::Displacementd & realPose, const Eigen::Twistd &realVelocity)
	{
		// std::cout << "update" << std::endl;
		double currentPlanningTime = updateTimeStep*updateCallCntr;
		++updateCallCntr;

		double evalTime = std::max(((currentPlanningTime - (std::max((this->executingPlanIdx), 0)*this->compHorizon)) / this->planHorizon), 0.0);

		// --- REAL COMPUTATION

		if (planStage != DONE && planStage != FINAL) //only necessery if the planning stage is INIT or INTER
		{

			// First update call
			if (this->ongoingPlanIdx == -1)
			{
				this->detectedObstacles = detectedObst;
				// std::cout << "conflictEval 1" << std::endl;
				this->conflictEval(otherVehicles, myRealPose);
				// std::cout << "conflictEval 2" << std::endl;
				this->initTimeOfCurrPlan = currentPlanningTime;
				this->planOngoingMutex.lock();
				planThread = boost::thread(&PathPlannerRecHor::plan, this); // Do P0
				++this->ongoingPlanIdx;
				//opt_log << "update: _______ (C1) Spawn the first plan thread!!! ________ " << this->ongoingPlanIdx << std::endl;
			}

			// If we are in P0 stage
			else if (this->ongoingPlanIdx == 0 && currentPlanningTime >= firstPlanTimespan) // Time so robot touch the floor
			{
				////std::cout << "firstPlanTimespan " <<  firstPlanTimespan << std::endl;
				if (!this->planOngoingMutex.try_lock()) // We are supposed to get this lock
				{
					// "kill" planning thread putting something reasonable in the aux spline
					// OR pause the simulation for a while, waiting for the plan thread to finish (completly unreal) => planThread.join()
					//opt_log << "update: ####### HECK! Plan didn't finish before computing time expired #######" << std::endl;
					//std::cout << "update: ####### HECK! Plan didn't finish before computing time expired #######" << std::endl;
					if (waitPlanning == true)
					{
						planThread.join();
					}
					else
					{
						planThread.interrupt();
					}
					this->planOngoingMutex.lock();
				}

				planStage = INTER;

				// update solution spline with the auxliar spline find in planning 0;
				// no need to lock a mutex associated to the auxiliar spline because we are sure there is no ongoing planning
				this->solSpline = this->auxSpline;
				this->finalPoseForFuturePlan = this->latestPose;

				if (lastPlanComputed)
				{
					planStage = FINAL;
					this->planHorizon = this->finalPlanHorizon;
					this->planOngoingMutex.unlock();
					//opt_log << "update: !!!!! Now goint to execute last plan !!!!!" << std::endl;
				}
				else
				{
					// plan next section!
					this->detectedObstacles = detectedObst;
					this->initTimeOfCurrPlan = currentPlanningTime;
					// std::cout << "conflictEval 1" << std::endl;
					this->conflictEval(otherVehicles, myRealPose);
					// std::cout << "conflictEval 2" << std::endl;
					planThread = boost::thread(&PathPlannerRecHor::plan, this); // Do PX with X in (1, 2, ..., indefined_finit_value)
					++this->ongoingPlanIdx;
					//opt_log << "update: _______ (C2) Spawn the second plan thread!!! ________ " << this->ongoingPlanIdx << std::endl;
				}

				// P0 will begin to be executed
				++this->executingPlanIdx;

				// Reset update call counter so evaluation time be corret
				updateCallCntr = 1; // the update for evalTime zero will be done next, during this same "update" call. Thus counter should be 1 for the next call.
				currentPlanningTime = 0.0;
				evalTime = 0.0;
			}

			// If the robot started to execute the motion:
			else if (this->ongoingPlanIdx > 0 && evalTime >= this->compHorizon / this->planHorizon)
			{
				////std::cout << "Eval time before fix " << evalTime << std::endl;
				////std::cout << "Current before fix " << secCurrentTime << std::endl;
				////std::cout << "CompHorizon " << this->compHorizon << std::endl;
				evalTime -= this->compHorizon / this->planHorizon; // "Fix" evalTime
																   ////std::cout << "Eval time fixed " << evalTime << std::endl;

				if (!this->planOngoingMutex.try_lock()) // We are supposed to get this lock
				{
					//opt_log << "update: ####### HECK! Plan didn't finish before computing time expired #######" << std::endl;
					//std::cout << "update: ####### HECK! Plan didn't finish before computing time expired #######" << std::endl;
					// "kill" planning thread putting something reasonable in the aux spline
					// OR pause the simulation for a while, waiting for the plan thread to finish (completly unreal) => planThread.join()
					//xde::sys::Timer::Sleep( 0.02 );
					if (waitPlanning == true)
					{
						planThread.join();
					}
					else
					{
						planThread.interrupt();
					}
					this->planOngoingMutex.lock();
				}
				// Now we are sure that there is no ongoing planning!

				// update solution spline with the auxliar spline;
				// no need to lock a mutex associated to the auxiliar spline because we are sure there is no ongoing planning

				this->solSpline = this->auxSpline; // update solution
				this->initPoseForCurrPlan = this->finalPoseForFuturePlan; // update base pose
				this->finalPoseForFuturePlan = this->latestPose; // get latest position found by the plan that just ended (will be the next base pos)

				if (lastPlanComputed)
				{
					planStage = FINAL;
					this->planHorizon = this->finalPlanHorizon;
					this->planOngoingMutex.unlock();
					//opt_log << "update: !!!!! Now goint to execute last plan !!!!!" << std::endl;
				}
				else
				{
					// plan next section!
					this->detectedObstacles = detectedObst;
					this->initTimeOfCurrPlan = currentPlanningTime;
					// std::cout << "conflictEval 1" << std::endl;
					this->conflictEval(otherVehicles, myRealPose);
					// std::cout << "conflictEval 2" << std::endl;
					planThread = boost::thread(&PathPlannerRecHor::plan, this); // Do PX with X in (1, 2, ..., indefined_finit_value)
					++this->ongoingPlanIdx;
					//opt_log << "update: _______ (C3) Spawn new plan thread!!! ________ " << this->ongoingPlanIdx << std::endl;
				}
				++this->executingPlanIdx;
			}
		}
		else if (planStage == FINAL && evalTime > 1.0)
		{
			////std::cout << "gone to DONE" << std::endl;
			planStage = DONE;
		}

		// --- APPARENT UPDATE

		if (planStage == INIT)
		{
			////std::cout << evalTime*planHorizon << ", " << initPose[0] << ", " << initPose[1] << std::endl;
			////std::cout << "plan STAGE INIT" << std::endl;
			//nextPoseRef = initPose;
			//nextVelocityRef = initVelocity;
		}
		else if (planStage == DONE)
		{
			std::cout << "Planning is over!" << std::endl;
			std::cout << "Last step planning horizon: " << this->planHorizon << std::endl;
			boost::this_thread::sleep(boost::posix_time::milliseconds(300));
			////std::cout << evalTime*planHorizon << ", " << targetedPose[0] << ", " << targetedPose[1] << std::endl;
			////std::cout << "plan STAGE DONE" << std::endl;
			nextPoseRef = targetedPose;
			nextVelocityRef = targetedVelocity;
			nextAccRef = Eigen::Matrix< double, aiv::FlatoutputMonocycle::accelerationDim, 1 >::Zero();
		}
		else //INTER or FINAL
		{
			////std::cout << "plan STAGE INTER or FINAL" << std::endl;
			////std::cout << evalTi
			// just use solSpline to get the nextReferences values

			NDerivativesMatrix derivFlat =
				solSpline.derivatives(evalTime*planHorizon, aiv::FlatoutputMonocycle::flatDerivDeg);

			Np1DerivativesMatrix derivFlat2 =
				solSpline.derivatives(evalTime*planHorizon, aiv::FlatoutputMonocycle::flatDerivDeg+1);

			this->nextPoseRef = aiv::FlatoutputMonocycle::flatToPose(derivFlat);

			this->nextPoseRef.block<aiv::FlatoutputMonocycle::posDim, 1>(aiv::FlatoutputMonocycle::posIdx, 0) =
				this->nextPoseRef.block<aiv::FlatoutputMonocycle::posDim, 1>(aiv::FlatoutputMonocycle::posIdx, 0) +
				this->initPoseForCurrPlan.block<aiv::FlatoutputMonocycle::posDim, 1>(aiv::FlatoutputMonocycle::posIdx, 0);

			////std::cout << evalTime*planHorizon << ", " << this->nextPoseRef[0] << ", " << this->nextPoseRef[1] << std::endl;

			this->nextVelocityRef = aiv::FlatoutputMonocycle::flatToVelocity(derivFlat);

			this->nextAccRef = aiv::FlatoutputMonocycle::flatToAcceleration(derivFlat2);

			/*if (evalTime == 0)
			{
				//std::cout << "EVALTIME ZERO AND CurrentPlanningTime = " << currentPlanningTime << ":\n" << this->nextPoseRef << this->nextVelocityRef << std::endl;
			}*/
		}

		return;
	}

	// update auxSpline
	void  PathPlannerRecHor::plan()
	{
		// std::cout << "enter plan\n";
		//boost::this_thread::sleep(boost::posix_time::milliseconds(300));

		/* INITIALIZATION OF CTRL PTS
		_______________________________________*/
		// TODO Use auxSpline with old solution to initiate the new one
		// Estimate the position of the point

		// GET ESTIMATE POSITION OF LAST POINT
		Eigen::Matrix< double, aiv::FlatoutputMonocycle::flatDim, 1 > remainingDistVectorUni = (this->targetedFlat - this->latestFlat);
		double remainingDist = remainingDistVectorUni.norm();
		remainingDistVectorUni /= remainingDist;
		Eigen::Matrix< double, aiv::FlatoutputMonocycle::flatDim, 1 > lastCtrlPt = this->maxStraightDist*remainingDistVectorUni;

		this->finalPlanHorizon = this->refPlanHorizon;

		if (remainingDist < this->lastStepMinDist + this->compHorizon*this->maxVelocity[aiv::FlatoutputMonocycle::linSpeedIdx])
		{
			//std::cout << "CONDITION FOR TERMINATION PLAN REACHED!" << std::endl;
			lastPlanComputed = true;
			//estimate last planning horizon
			this->finalPlanHorizon = remainingDist / this->maxVelocity[aiv::FlatoutputMonocycle::linSpeedIdx];
			// TODO recalc n_ctrlpts n_knots
		}

		////opt_log << "plan: PLAN STAGE: " << planStage << std::endl;

		// Interpolation time
		Eigen::RowVectorXd interpTime = Eigen::RowVectorXd::LinSpaced(this->noCtrlPts, 0.0, this->finalPlanHorizon);

		// Interpolation points ( noCtrlPts points )
		MySpline::ControlPointVectorType points(this->splDim, this->noCtrlPts);

		// Populate points with linear spaced points from zero to estimate last ctrl point
		switch (lastPlanComputed)
		{
		case false:
			if (planStage == INIT)
			{
				/*for ( auto i = 1; i < splDim; ++i )
				{
				for ( auto j = 0; j < noCtrlPts; ++j )
				{
				points(i,j) = -100.0 + (double)rand() / RAND_MAX * (200.0);
				}
				}*/
				for (auto i = 0; i < splDim; ++i)
				{
					points.row(i) = Eigen::RowVectorXd::LinSpaced(noCtrlPts, 0.0, lastCtrlPt(i));
					//points.row(i) = Eigen::RowVectorXd::Random();
				}
				/*bearing = acos(remainingDistVectorUni.x())-latestPose(2,0);
				linSpeed = refPlanHorizon/noCtrlPts * maxAcceleration.x();
				angSpeed = refPlanHorizon/noCtrlPts * maxAcceleration.y();*/ //refPlanHorizon/noCtrlPts*bearing/(2*3.1415)*angSpeed/2.+
				//points(1,1) = cos(latestPose(2,0))*(refPlanHorizon/noCtrlPts*maxVelocity.x()/2.) + points(1,0);// + points(1,1))/2.;
				//points(2,1) = sin(latestPose(2,0))*(refPlanHorizon/noCtrlPts*maxVelocity.x()/2.) + points(2,0);// + points(2,1))/2.;
				//////std::cout << "___________________PTSinit________________________\n" << points.block<2,1>(0,1) << std::endl;
			}
			else
			{
				/*for ( auto i = 1; i < splDim; ++i )
				{
				for ( auto j = 0; j < noCtrlPts; ++j )
				{
				points(i,j) = -100.0 + (double)rand() / RAND_MAX * (200.0);
				}
				}*/
				for (auto i = 0; i < splDim; ++i)
				{
					points.row(i) = Eigen::RowVectorXd::LinSpaced(this->noCtrlPts, 0.0, lastCtrlPt(i));
					//points.row(i) = Eigen::RowVectorXd::Random();
				}
			}
			break;
		case true:
			for (auto i = 0; i < splDim; ++i)
			{
				points.row(i) = Eigen::RowVectorXd::LinSpaced(this->noCtrlPts, 0.0, lastCtrlPt(i));
				//points.row(i) = Eigen::RowVectorXd::Random();

			}
			break;
		}

		// Get chords lengths from interpolation time (appears to be the same as interpTime/finalPlanningHorizon)
		MySpline::KnotVectorType chordLengths;
		Eigen::ChordLengths(interpTime, chordLengths);

		// Call fitting static method
		this->auxSpline = Eigen::SplineFitting<MySpline>::Interpolate(points, splDegree, chordLengths);

		// Change the indexation which is \in [0.0, 1.0] to \in [0.0, finalPlanningHorizon]. Useful when computing spline derivatives
		MySpline::ControlPointVectorType ctrlpts(this->splDim, this->noCtrlPts);
		ctrlpts = auxSpline.ctrls();
		this->auxSpline.~MySpline();
		new (&this->auxSpline) MySpline(genKnots(0.0, this->finalPlanHorizon, false), ctrlpts); // has to be false, otherwise hardcoded jacobian will be wrong

		////std::cout << this->auxSpline.ctrls() << std::endl;
		//system("pause");

		/*Eigen::ArrayXd optParam(this->splDim-1, this->noCtrlPts);
		optParam = this->auxSpline.ctrls().block(1, 0, this->splDim-1, this->noCtrlPts);
		for ( unsigned i = 0; i < this->noCtrlPts*aiv::FlatoutputMonocycle::flatDim; ++i )
		{
		//std::cout << optParam(i/(this->splDim-1), i%(this->splDim-1)) << std::endl;
		}*/

		/* CALL OPT SOLVER (in stand alone mode)
		_______________________________________*/

		this->isThereConfl = NONE;
		// std::cout << "Call solveopt 1\n";
		this->solveOptPbl(); // !!!! after this call auxSpline has the new 
		// std::cout << "Call solveopt 2\n";
		this->initTimeOfCurrPlan += this->compHorizon;

		/* CHECK ABOUT CONFLICTS
		_______________________________________*/
		// if (this->collisionAIVs.size() != 0 && this->comOutAIVs.size() != 0)
		// {
		// 	this->isThereConfl = BOTH;
		// }
		// else if (this->collisionAIVs.size() != 0)
		// {
		// 	this->isThereConfl = COLL;
		// }
		// else if (this->comOutAIVs.size() != 0)
		// {
		// 	this->isThereConfl = COM;
		// }
		//else rest unchanged

		// if (this->isThereConfl != NONE)
		// {
		// 	this->solveOptPbl();
		// }

		/* GET RESULTS
		_______________________________________*/


		/* UPDATES
		_______________________________________*/
		// update latest flatoutput and pose for the new planning

		NDerivativesMatrix derivFlat =
			auxSpline.derivatives(this->compHorizon, aiv::FlatoutputMonocycle::flatDerivDeg);

		this->latestFlat += derivFlat.col(0);
		////std::cout << "__________\nLatest flat\n" << this->latestFlat << "\n_____________\n";

		/*this->nextPoseRef = aiv::FlatoutputMonocycle::flatToPose(derivFlat);

		this->nextPoseRef.block<aiv::FlatoutputMonocycle::posDim, 1>(aiv::FlatoutputMonocycle::posIdx, 0) =
		this->nextPoseRef.block<aiv::FlatoutputMonocycle::posDim, 1>(aiv::FlatoutputMonocycle::posIdx, 0) +
		this->initPoseForCurrPlan.block<aiv::FlatoutputMonocycle::posDim, 1>(aiv::FlatoutputMonocycle::posIdx, 0);*/

		PoseVector auxLatestPose = this->latestPose;

		this->latestPose = aiv::FlatoutputMonocycle::flatToPose(derivFlat);
		this->latestPose.block<aiv::FlatoutputMonocycle::posDim, 1>(aiv::FlatoutputMonocycle::posIdx, 0) +=
			auxLatestPose.block<aiv::FlatoutputMonocycle::posDim, 1>(aiv::FlatoutputMonocycle::posIdx, 0);

		this->latestVelocity = aiv::FlatoutputMonocycle::flatToVelocity(derivFlat);

		//switch (this->planStage)
		//{
		//case FINAL:
		//	break;
		//case INIT:
		//	this->planStage = INTER;
		//	break;
		//	// REDO TIME AND KNOTS
		//}
		//opt_log << "plan:\n__________\nLATEST POSE\n" << this->latestPose << "\n_____________\n";
		//boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
		this->planOngoingMutex.unlock();
		return;
	}

	int PathPlannerRecHor::nbPointsCloseInTimeEval()
	{
		int nbpts = 0;
		std::vector<int> auxVec(2, -1);
		conflictInfo.clear();

		for (std::map<std::string, AIV* >::const_iterator
				it = this->collisionAIVs.begin();
				it != this->collisionAIVs.end();
				++it)
		{
			if (it->second->getPathPlanner()->isDone())
			{
				nbpts += this->noTimeSamples;
				conflictInfo.insert(std::pair<std::string, std::vector<int> >(it->first, auxVec));
			}
			else
			{
				// std::cout << this->initTimeOfCurrPlan << ", " << it->second->getPathPlanner()->getInitTimeOfCurrPlan() << std::endl;

				int discount = abs((this->initTimeOfCurrPlan - it->second->getPathPlanner()->getInitTimeOfCurrPlan())/(planHorizon/noTimeSamples));

				nbpts += this->noTimeSamples - discount;
				conflictInfo.insert(std::pair<std::string, std::vector<int> >(it->first, auxVec));
				conflictInfo[it->first].clear();

				if (this->initTimeOfCurrPlan < it->second->getPathPlanner()->getInitTimeOfCurrPlan())
				{
					conflictInfo[it->first].push_back(discount);
					conflictInfo[it->first].push_back(0);
				}
				else
				{
					conflictInfo[it->first].push_back(0);
					conflictInfo[it->first].push_back(discount);
				}
			}
		}
		return nbpts;
	}

	void PathPlannerRecHor::solveOptPbl()
	{
		unsigned nParam;
		unsigned nEq;
		unsigned nIneq;
		double(*objF) (unsigned, const double *, double *, void *);
		void(*eqF) (unsigned, double *, unsigned, const double*, double*, void*);
		void(*ieqF) (unsigned, double *, unsigned, const double*, double*, void*);

		double *optParam;
		double *tolEq;
		double *tolIneq;

		switch (lastPlanComputed)
		{
		case false:
			// if (isThereConfl == NONE)
			// {
				objF = PathPlannerRecHor::objectFunc;
				eqF = PathPlannerRecHor::eqFunc;
				ieqF = PathPlannerRecHor::ineqFunc;
				nParam = this->noCtrlPts*aiv::FlatoutputMonocycle::flatDim;
				nEq = aiv::FlatoutputMonocycle::poseDim + aiv::FlatoutputMonocycle::velocityDim;
				nIneq = aiv::FlatoutputMonocycle::velocityDim * this->noTimeSamples +
					aiv::FlatoutputMonocycle::accelerationDim * (this->noTimeSamples + 1) +
					this->noTimeSamples * detectedObstacles.size();
				////std::cout << "nIEQ: " << nIneq << std::endl;

				optParam = new double[nParam];
				for (unsigned i = 0; i < nParam; ++i)
				{
					optParam[i] = this->auxSpline.ctrls()(i % this->splDim, i / this->splDim);
				}

				tolEq = new double[nEq];
				for (unsigned i = 0; i < nEq; ++i)
				{
					tolEq[i] = this->eqTol;
				}

				tolIneq = new double[nIneq];
				for (unsigned i = 0; i < nIneq; ++i)
				{
					tolIneq[i] = this->ineqTol;
				}
			// }
			// else //if (isThereConfl == COLL)
			// {
			// 	objF = PathPlannerRecHor::objectFunc;
			// 	eqF = PathPlannerRecHor::eqFunc;
			// 	ieqF = PathPlannerRecHor::ineqFuncColl;
			// 	nParam = this->noCtrlPts*aiv::FlatoutputMonocycle::flatDim;
			// 	nEq = aiv::FlatoutputMonocycle::poseDim + aiv::FlatoutputMonocycle::velocityDim;
			// 	nIneq = aiv::FlatoutputMonocycle::velocityDim * this->noTimeSamples +
			// 		aiv::FlatoutputMonocycle::accelerationDim * (this->noTimeSamples + 1) +
			// 		this->noTimeSamples * detectedObstacles.size() +
			// 		this->nbPointsCloseInTimeEval();
				
			// 		// std::cout << "nbPointsToAdd\n" << this->nbPointsCloseInTimeEval() << std::endl;
			// 	////std::cout << "nIEQ: " << nIneq << std::endl;

			// 	optParam = new double[nParam];
			// 	for (unsigned i = 0; i < nParam; ++i)
			// 	{
			// 		optParam[i] = this->auxSpline.ctrls()(i % this->splDim, i / this->splDim);
			// 	}

			// 	tolEq = new double[nEq];
			// 	for (unsigned i = 0; i < nEq; ++i)
			// 	{
			// 		tolEq[i] = this->eqTol;
			// 	}

			// 	tolIneq = new double[nIneq];
			// 	for (unsigned i = 0; i < nIneq; ++i)
			// 	{
			// 		tolIneq[i] = this->ineqTol;
			// 	}
			// }
			// else if (isThereConfl == COM)
			// {
			// 	//TODO
			// }
			// else if (isThereConfl == BOTH)
			// {
			// 	//TODO
			// }
			break;
		case true:
			//system("pause");
			objF = PathPlannerRecHor::objectFuncLS;
			eqF = PathPlannerRecHor::eqFuncLS;
			ieqF = PathPlannerRecHor::ineqFuncLS;
			nParam = this->noCtrlPts*aiv::FlatoutputMonocycle::flatDim + 1;
			nEq = (aiv::FlatoutputMonocycle::poseDim + aiv::FlatoutputMonocycle::velocityDim) * 2;
			nIneq = aiv::FlatoutputMonocycle::velocityDim * (this->noTimeSamples - 1) +
				aiv::FlatoutputMonocycle::accelerationDim * this->noTimeSamples +
				(this->noTimeSamples - 1) * detectedObstacles.size();

			optParam = new double[nParam];
			optParam[0] = this->finalPlanHorizon;
			for (unsigned i = 1; i < nParam; ++i)
			{
				optParam[i] = this->auxSpline.ctrls()((i - 1) % this->splDim, (i - 1) / this->splDim);
			}

			tolEq = new double[nEq];
			for (unsigned i = 0; i < nEq; ++i)
			{
				tolEq[i] = this->eqTol;
			}

			tolIneq = new double[nIneq];
			for (unsigned i = 0; i < nIneq; ++i)
			{
				tolIneq[i] = this->ineqTol;
			}
			break;
		}
		
		if (optimizerType == "NONE")
		{
			// GET optParam from XML
			std::string base_name = "AdeptLynx";
			//std::string agv = base_name + name[base_name.size()];
			////std::cout << "|___________ " << base_name << name[base_name.size()] << "______________|" << std::endl;
			////std::cout << "|___________ " << agv << "_______________|" << std::endl;
			////std::cout << "root." + base_name + name[base_name.size()] + ".plan" + std::to_string(_ULONGLONG(ongoingPlanIdx + 1)) << std::endl ;
			std::string optParamString;
			try
			{
				////std::cout << "GET" << std::endl;
				optParamString = property_tree.get<std::string>("root." + base_name + name[base_name.size()] + ".plan" + std::to_string(_ULONGLONG(ongoingPlanIdx)));
				////std::cout << "AFTER GET" << std::endl;
			}
			catch (std::exception& e)
			{
				//std::cout << "property tree execption: " << e.what() << std::endl;
				return;
			}
			//std::string optParamString = property_tree.get<std::string>("root.AdeptLynx0.plan0");
			////std::cout << "|____________\n" << base_name + name[base_name.size()] + ".plan" + std::to_string(_ULONGLONG(ongoingPlanIdx + 1)) << "\n___________________|" << std::endl;

			std::stringstream ss(optParamString);
			std::string token;
			int i = 0;
			while (getline(ss, token, ','))
			{
				optParam[i++] = std::strtod(token.c_str(), NULL);
			}

			objF(nParam, optParam, NULL, this);
			double *constrEq = new double[nEq];
			eqFunc(nEq, constrEq, nParam, optParam, NULL, this);
			double *constrIneq = new double[nIneq];
			//ineqFunc(nIneq, constrIneq, nParam, optParam, NULL, this);
			delete[] constrEq;
			delete[] constrIneq;

		}
		else if (optimizerType != "IPOPT")
		{

			//if (!lastPlanComputed)
			////if(true)
			//{
			//	myNLP = new RecHorNLP(nParam, nEq, nIneq, noCtrlPts, noTimeSamples, auxSpline, finalPlanHorizon,
			//		targetedPose, latestPose, latestVelocity, maxVelocity, maxAcceleration, NULL);
			//}
			//else
			//{
			//	////std::cout << "Build Term NLP" << std::endl;
			//	////opt_log << "Build Term NLP" << std::endl;
			//	myNLP = new TermNLP(nParam, nEq, nIneq, noCtrlPts, noTimeSamples, auxSpline,
			//		targetedPose, latestPose, targetedVelocity, latestVelocity, maxVelocity, maxAcceleration, NULL);
			//	////std::cout << "Done building NLP" << std::endl;
			//}

			//int gni_n, gni_m, gni_nn_jac_g, gni_nnz_h_lag;
			//Ipopt::TNLP::IndexStyleEnum gni_index_stype;
			//////std::cout << "call nlp info" << std::endl;
			//myNLP->get_nlp_info(gni_n, gni_m, gni_nn_jac_g, gni_nnz_h_lag, gni_index_stype);
			////std::cout << "done calling nlp info" << std::endl;

			nlopt_opt opt;

			if (optimizerType == "COBYLA")
			{
				opt = nlopt_create(NLOPT_LN_COBYLA, nParam);
			}
			else if (optimizerType == "SLSQP")
			{
				opt = nlopt_create(NLOPT_LD_SLSQP, nParam);
			}
			else
			{
				std::cout << "ERROR, unknown opt method\n";
			}

			nlopt_set_xtol_rel(opt, this->xTol);

			nlopt_set_min_objective(opt, objF, this);
			nlopt_add_equality_mconstraint(opt, nEq, eqF, this, tolEq);
			nlopt_add_inequality_mconstraint(opt, nIneq, ieqF, this, tolIneq);

			double minf;
			////std::cout << "Call optimizer" << std::endl;
			int status;
			//if (!lastPlanComputed)
			//{
			status = nlopt_optimize(opt, optParam, &minf);
			//status = 1;
			//}
			//else
			//{
			//	status = 1;
			//}
			////std::cout << "Optimizer return" << std::endl;


			//NLOPT_SUCCESS = 1
			//Generic success return value.
			//NLOPT_STOPVAL_REACHED = 2
			//Optimization stopped because stopval (above) was reached.
			//NLOPT_FTOL_REACHED = 3
			//Optimization stopped because ftol_rel or ftol_abs (above) was reached.
			//NLOPT_XTOL_REACHED = 4
			//Optimization stopped because xtol_rel or xtol_abs (above) was reached.
			//NLOPT_MAXEVAL_REACHED = 5
			//Optimization stopped because maxeval (above) was reached.
			//NLOPT_MAXTIME_REACHED = 6
			//Optimization stopped because maxtime (above) was reached.
			//[edit]
			//Error codes (negative return values)
			//NLOPT_FAILURE = -1
			//Generic failure code.
			//NLOPT_INVALID_ARGS = -2
			//Invalid arguments (e.g. lower bounds are bigger than upper bounds, an unknown algorithm was specified, etcetera).
			//NLOPT_OUT_OF_MEMORY = -3
			//Ran out of memory.
			//NLOPT_ROUNDOFF_LIMITED = -4
			//Halted because roundoff errors limited progress. (In this case, the optimization still typically returns a useful result.)
			//NLOPT_FORCED_STOP = -5
			//Halted because of a forced termination: the user called nlopt_force_stop(opt) on the optimization’s nlopt_opt object opt from the user’s objective function or constraints.


			//SolverReturn status2;

			//myNLP->finalize_solution(status2,
			//	nParam, optParam, NULL, NULL,
			//	nEq + nIneq, NULL, NULL,
			//	minf,
			//	NULL,
			//	NULL);
			//if (status < 0)
			//{
			//	//std::cout << "solve: _____________________________________________nlopt failed!\n";
			//	//opt_log << "solve: _____________________________________________nlopt failed!\n";
			//	//opt_log << "solve: with error: " << status << std::endl;
			//	//std::cout << "solve: with error: " << status << std::endl;
			//	////opt_log << "objectif func value: " << minf << std::endl;
			//}
			//////opt_log << "objectif func value: " << minf << std::endl;

			//delete myNLP;
		}

		std::ofstream arquivo;
		arquivo.open("optlog/xiter.csv", std::fstream::app);
		arquivo << "___________________________________________________" << std::endl;
		arquivo.close();
		arquivo.open("optlog/geqiter.csv", std::fstream::app);
		arquivo << "___________________________________________________" << std::endl;
		arquivo.close();
		arquivo.open("optlog/gieqiter.csv", std::fstream::app);
		arquivo << "___________________________________________________" << std::endl;
		arquivo.close();
		arquivo.open("optlog/citer.csv", std::fstream::app);
		arquivo << "___________________________________________________" << std::endl;
		arquivo.close();


		MySpline::ControlPointVectorType ctrlPts(this->splDim, this->noCtrlPts);

		MySpline::KnotVectorType knots;

		//update spline
		switch (lastPlanComputed)
		{
		case false:

			for (unsigned i = 0; i < nParam; ++i)
			{
				// get ctrl pts matrix => p1x, p1y, p2x, p2y...
				ctrlPts(i % this->splDim, i / this->splDim) = optParam[i];
			}

			knots = this->auxSpline.knots();
			////std::cout << knots << std::endl << std::endl;
			this->auxSpline.~MySpline();
			new (&this->auxSpline) MySpline(knots, ctrlPts);

			break;
		case true:
			knots = genKnots(0.0, optParam[0], false); // has to be false, otherwise jacobien will be wrong
			/*//std::cout << "\nKnots:";
			//opt_log << "\nKnots:";
			//std::cout << "\n__________________________\n" << knots << "\n__________________________\n" << std::endl;
			//opt_log << "\n__________________________\n" << knots << "\n__________________________\n" << std::endl;*/

			this->finalPlanHorizon = optParam[0];

			// get the rest of the ctrl pts
			for (unsigned i = 1; i < nParam; ++i)
			{
				// get ctrl pts matrix => p1x, p1y, p2x, p2y...
				ctrlPts((i - 1) % this->splDim, (i - 1) / this->splDim) = optParam[i];
			}

			this->auxSpline.~MySpline();
			new (&this->auxSpline) MySpline(knots, ctrlPts);

			break;
		}
		//std::cout << "END OF PLAN " << ongoingPlanIdx << " CALL" << std::endl;
		delete[] optParam;
		delete[] tolEq;
		delete[] tolIneq;
		////std::cout << "optParam\n" << ctrlPts << std::endl;
		////std::cout << "Knots\n" << this->auxSpline.knots() << std::endl;
	}

	/*

	Inequations should be in the form:
	myconstraint(x) <= 0

	*/

	double  PathPlannerRecHor::objectFunc(unsigned n, const double *x, double *grad, void *my_func_data)
	{

		// get this path planner pointer
		PathPlannerRecHor *thisPP = static_cast< PathPlannerRecHor *>(my_func_data);

		// Create a CtrlPoints vector
		MySpline::ControlPointVectorType ctrlPts(splDim, thisPP->noCtrlPts);

		// Feed ctrlPts rows with values from the primal variables x
		for (int i = 0; i < n; ++i)
		{
			ctrlPts(i % splDim, i / splDim) = x[i];
		}

		// Create a spline based on the new ctrlpts (using the same knots as before)
		MySpline optSpline(thisPP->auxSpline.knots(), ctrlPts);

		// Create a Matrix consisting of the flat output and its needed derivatives for the instant Tp (1.0)
		NDerivativesMatrix derivFlat =
			optSpline.derivatives(thisPP->finalPlanHorizon, aiv::FlatoutputMonocycle::flatDerivDeg);

		// Get pose at Tp from flatoutput
		PoseVector poseAtTp = aiv::FlatoutputMonocycle::flatToPose(derivFlat);

		// Compute euclidian distance to square from position at Tp and goal position which will be the cost
		double fx = pow((poseAtTp.block(aiv::FlatoutputMonocycle::posIdx, 0, aiv::FlatoutputMonocycle::posDim, 1) -
			(thisPP->targetedPose.block(aiv::FlatoutputMonocycle::posIdx, 0, aiv::FlatoutputMonocycle::posDim, 1) -
				thisPP->latestPose.block(aiv::FlatoutputMonocycle::posIdx, 0, aiv::FlatoutputMonocycle::posDim, 1))).norm(), 2);

		// KEEPING LOG
		std::ofstream arquivo;
		arquivo.open("optlog/xiter.csv", std::fstream::app);
		for (int i = 0; i < int(n); ++i)
		{
			//x[i] = startSpline.ctrls()(i%(splDim-1)+1, i/(splDim-1));
			arquivo << x[i] << ", ";

		}
		arquivo << std::endl;
		arquivo.close();
		arquivo.open("optlog/citer.csv", std::fstream::app);
		arquivo << fx << std::endl;
		arquivo.close();

		// If grad not NULL the Jacobian matrix must be given
		if (grad)
		{
			for (unsigned i = n - 1, j = 0; i > n - thisPP->splDim - 1; --i, ++j)
			{
				grad[i] = 2 * (x[i] -
					(thisPP->targetedPose(i % thisPP->splDim + aiv::FlatoutputMonocycle::posIdx, 0) -
						thisPP->latestPose(i % thisPP->splDim + aiv::FlatoutputMonocycle::posIdx, 0)));
			}
			for (unsigned i = 0; i < n - thisPP->splDim; ++i)
			{
				grad[i] = 0.0;
			}
		}
		return fx;
	}

	void PathPlannerRecHor::eval_eq(double *result, unsigned n, const double* x, void* data)
	{
		// get this path planner pointer
		PathPlannerRecHor *thisPP = static_cast< PathPlannerRecHor *>(data);

		// Create a CtrlPoints vector
		MySpline::ControlPointVectorType ctrlPts(splDim, thisPP->noCtrlPts);

		// Feed remaining rows with values from the primal variables x
		for (int i = 0; i < n; ++i)
		{
			ctrlPts(i % splDim, i / splDim) = x[i];
		}


		// Create a spline based on the new ctrlpts (using the same knots as before)
		MySpline optSpline(thisPP->auxSpline.knots(), ctrlPts);


		// EQUATIONS


		// Create a Matrix consisting of the flat output and its needed derivatives for the instant T0 (0.0)
		NDerivativesMatrix derivFlatEq =
			optSpline.derivatives(0.0, aiv::FlatoutputMonocycle::flatDerivDeg);

		// Get error from pose at T0 and pose at the end of the previous plan (or initial position if this is the very first plan)
		//PoseVector poseAtT0 = aiv::FlatoutputMonocycle::flatToPose(derivFlatEq);
		PoseVector diffPoseAtT0 = aiv::FlatoutputMonocycle::flatToPose(derivFlatEq);
		diffPoseAtT0.tail(1) -= thisPP->latestPose.tail(1);
		/*diffPoseAtT0.block<aiv::FlatoutputMonocycle::posDim, 1>(aiv::FlatoutputMonocycle::posIdx, 0) =
			poseAtT0.block<aiv::FlatoutputMonocycle::posDim, 1>(aiv::FlatoutputMonocycle::posIdx, 0);*/

		VelVector diffVelocityAtT0 = aiv::FlatoutputMonocycle::flatToVelocity(derivFlatEq) - thisPP->latestVelocity;

		int i = 0;
		for (; i < aiv::FlatoutputMonocycle::poseDim; ++i)
		{
			result[i] = diffPoseAtT0[i];
			//std::cout << "G[" << i << "] " << g[i] << std::endl;
		}
		for (int j = i; j - i < aiv::FlatoutputMonocycle::velocityDim; ++j)
		{
			result[j] = diffVelocityAtT0[j - i];
			//std::cout << "G[" << j << "] " << g[j] << std::endl;
		}
	}

	void  PathPlannerRecHor::eqFunc(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data)
	{
		// get this path planner pointer
		PathPlannerRecHor *thisPP = static_cast< PathPlannerRecHor *>(data);

		thisPP->eval_eq(result, n, x, data);



		if (grad)
		{
			/* ALGEBRIC JACOBIAN */

			int span = thisPP->findspan(0.0);
			span -= thisPP->noIntervNonNull-1;

			qJacMatrix qJ = qJac(0.0, thisPP->finalPlanHorizon, span, x, n, thisPP->splDegree);
			dqJacMatrix dqJ = dqJac(0.0, thisPP->finalPlanHorizon, span, x, n, thisPP->splDegree);

			Eigen::MatrixXd J(qJ.rows()+dqJ.rows(), qJ.cols());
			J << qJ, dqJ;

			Eigen::Map<Eigen::MatrixXd>(grad, J.cols(), J.rows()) =
					J.transpose();
		}
	}

	void PathPlannerRecHor::eval_ineq(double *result, unsigned n, const double* x, void* data)
	{
		// get this path planner pointer
		PathPlannerRecHor *thisPP = static_cast< PathPlannerRecHor *>(data);

		// Create a CtrlPoints vector
		MySpline::ControlPointVectorType ctrlPts(splDim, thisPP->noCtrlPts);

		// Feed remaining rows with values from the primal variables x
		for (int i = 0; i < n; ++i)
		{
			ctrlPts(i % splDim, i / splDim) = x[i];
		}

		// Create a spline based on the new ctrlpts (using the same knots as before)
		MySpline optSpline(thisPP->auxSpline.knots(), ctrlPts);


		// INEQUATIONS


		Np1DerivativesMatrix derivFlat;
		NDerivativesMatrix derivFlatSmall;

		// Create Matrices for storing velocity and acceleration
		VelVector velocity;
		AccVector acceleration;

		PoseVector pose;

		derivFlat =
			optSpline.derivatives(0.0, aiv::FlatoutputMonocycle::flatDerivDeg + 1);

		////std::cout << "got acc" << std::endl;
		acceleration = aiv::FlatoutputMonocycle::flatToAcceleration(derivFlat);

		int j = 0;
		for (; j < aiv::FlatoutputMonocycle::accelerationDim; ++j)
		{
			const double absAcc = (acceleration(j, 0) > -acceleration(j, 0)) ? acceleration(j, 0) : -acceleration(j, 0);
			result[j] = absAcc - thisPP->maxAcceleration(j, 0);
			////std::cout << "G[" << nEq+j+(i-1)*ieqPerSample << "] " << g[nEq+j+(i-1)*ieqPerSample] << std::endl;
		}

		int nAcc = j;
		int ieqPerSample = (aiv::FlatoutputMonocycle::velocityDim + aiv::FlatoutputMonocycle::accelerationDim + thisPP->detectedObstacles.size());

		//#pragma omp parallel for
		for (int i = 1; i <= int(thisPP->noTimeSamples); ++i)
		{
			////std::cout << "IEQ: SAMPLE IDX: " << i << std::endl;
			derivFlat =
				optSpline.derivatives(double(i) / thisPP->noTimeSamples * thisPP->finalPlanHorizon, aiv::FlatoutputMonocycle::flatDerivDeg + 1);

			////std::cout << "got derive meatrix" << std::endl;
			// reusing derivFlatEq variable, ugly but let's move on
			derivFlatSmall = derivFlat.block<aiv::FlatoutputMonocycle::flatDim, aiv::FlatoutputMonocycle::flatDerivDeg + 1>(0, 0);

			////std::cout << "got vel" << std::endl;
			velocity = aiv::FlatoutputMonocycle::flatToVelocity(derivFlatSmall);

			pose = aiv::FlatoutputMonocycle::flatToPose(derivFlatSmall);

			////std::cout << "got acc" << std::endl;
			acceleration = aiv::FlatoutputMonocycle::flatToAcceleration(derivFlat);

			//int ieqPerSample = (aiv::FlatoutputMonocycle::velocityDim);

			int j;
			for (j = 0; j < aiv::FlatoutputMonocycle::accelerationDim; ++j)
			{
				const double absAcc = (acceleration(j, 0) > -acceleration(j, 0)) ? acceleration(j, 0) : -acceleration(j, 0);
				result[nAcc + j + (i - 1)*ieqPerSample] = absAcc - thisPP->maxAcceleration(j, 0);
				////std::cout << "G[" << nEq+j+(i-1)*ieqPerSample << "] " << g[nEq+j+(i-1)*ieqPerSample] << std::endl;
			}

			int k;
			for (k = j; k - j < aiv::FlatoutputMonocycle::velocityDim; ++k)
			{
				const double absVel = (velocity(k - j, 0) > -velocity(k - j, 0)) ? velocity(k - j, 0) : -velocity(k - j, 0);
				result[nAcc + k + (i - 1)*ieqPerSample] = absVel - thisPP->maxVelocity(k - j, 0);
				////std::cout << "G[" << nEq+k+(i-1)*ieqPerSample << "] " << g[nEq+k+(i-1)*ieqPerSample] << std::endl;
			}

			// Obstacles
			std::map<std::string, Obstacle *>::iterator it;
			for (it = thisPP->detectedObstacles.begin(), j = k; j - k < thisPP->detectedObstacles.size(); ++j, ++it)
			{
				result[nAcc + (i - 1)*ieqPerSample + j] = -1. * it->second->distToAIV(pose.head(2) + thisPP->latestPose.head(2), thisPP->secRho);
			}
		}
	}

	void PathPlannerRecHor::eval_ineq_coll(double *result, unsigned n, const double* x, void* data)
	{
		// get this path planner pointer
		PathPlannerRecHor *thisPP = static_cast< PathPlannerRecHor *>(data);

		// Create a CtrlPoints vector
		MySpline::ControlPointVectorType ctrlPts(splDim, thisPP->noCtrlPts);

		// Feed remaining rows with values from the primal variables x
		for (int i = 0; i < n; ++i)
		{
			ctrlPts(i % splDim, i / splDim) = x[i];
		}

		// Create a spline based on the new ctrlpts (using the same knots as before)
		MySpline optSpline(thisPP->auxSpline.knots(), ctrlPts);


		// INEQUATIONS


		Np1DerivativesMatrix derivFlat;
		NDerivativesMatrix derivFlatSmall;

		// Create Matrices for storing velocity and acceleration
		VelVector velocity;
		AccVector acceleration;

		PoseVector pose;

		derivFlat =
			optSpline.derivatives(0.0, aiv::FlatoutputMonocycle::flatDerivDeg + 1);

		////std::cout << "got acc" << std::endl;
		acceleration = aiv::FlatoutputMonocycle::flatToAcceleration(derivFlat);

		int j = 0;
		for (; j < aiv::FlatoutputMonocycle::accelerationDim; ++j)
		{
			const double absAcc = (acceleration(j, 0) > -acceleration(j, 0)) ? acceleration(j, 0) : -acceleration(j, 0);
			result[j] = absAcc - thisPP->maxAcceleration(j, 0);
			//std::cout << "G[" << j << "] " << result[j] << std::endl;
		}

		int nAcc = j;
		int ieqPerSample = (aiv::FlatoutputMonocycle::velocityDim + aiv::FlatoutputMonocycle::accelerationDim + thisPP->detectedObstacles.size());

		//#pragma omp parallel for
		for (int i = 1; i <= int(thisPP->noTimeSamples); ++i)
		{
			//////std::cout << "IEQ: SAMPLE IDX: " << i << std::endl;
			derivFlat =
				optSpline.derivatives(double(i) / thisPP->noTimeSamples * thisPP->finalPlanHorizon, aiv::FlatoutputMonocycle::flatDerivDeg + 1);

			// reusing derivFlatEq variable, ugly but let's move on
			derivFlatSmall = derivFlat.block<aiv::FlatoutputMonocycle::flatDim, aiv::FlatoutputMonocycle::flatDerivDeg + 1>(0, 0);

			velocity = aiv::FlatoutputMonocycle::flatToVelocity(derivFlatSmall);

			pose = aiv::FlatoutputMonocycle::flatToPose(derivFlatSmall);

			acceleration = aiv::FlatoutputMonocycle::flatToAcceleration(derivFlat);

			//int ieqPerSample = (aiv::FlatoutputMonocycle::velocityDim);

			int j;
			for (j = 0; j < aiv::FlatoutputMonocycle::accelerationDim; ++j)
			{
				const double absAcc = (acceleration(j, 0) > -acceleration(j, 0)) ? acceleration(j, 0) : -acceleration(j, 0);
				result[nAcc + j + (i - 1)*ieqPerSample] = absAcc - thisPP->maxAcceleration(j, 0);
				//std::cout << "G[" << nAcc + j + (i - 1)*ieqPerSample << "] " << result[nAcc + j + (i - 1)*ieqPerSample] << std::endl;
			}

			int k;
			for (k = j; k - j < aiv::FlatoutputMonocycle::velocityDim; ++k)
			{
				const double absVel = (velocity(k - j, 0) > -velocity(k - j, 0)) ? velocity(k - j, 0) : -velocity(k - j, 0);
				result[nAcc + k + (i - 1)*ieqPerSample] = absVel - thisPP->maxVelocity(k - j, 0);
				//std::cout << "G[" << nAcc + k + (i - 1)*ieqPerSample << "] " << result[nAcc + k + (i - 1)*ieqPerSample] << std::endl;
			}

			// Obstacles
			std::map<std::string, Obstacle *>::iterator it;
			for (it = thisPP->detectedObstacles.begin(), j = k; j - k < thisPP->detectedObstacles.size(); ++j, ++it)
			{
				result[nAcc + (i - 1)*ieqPerSample + j] = -1. * it->second->distToAIV(pose.head(2) + thisPP->latestPose.head(2), thisPP->secRho);
				//std::cout << "G[" << nAcc + (i - 1)*ieqPerSample + j << "] " << result[nAcc + (i - 1)*ieqPerSample + j] << std::endl;

			}

		}

		NDerivativesMatrix derivFlatSmallOther;
		PoseVector myPose;
		PoseVector otherPose;


		for (std::map<std::string, AIV* >::const_iterator
				it = thisPP->collisionAIVs.begin();
				it != thisPP->collisionAIVs.end();
				++it)
		{
			int i, j, k;
			for (i = thisPP->conflictInfo[it->first][0]+1, j = thisPP->conflictInfo[it->first][1]+1, k=0; i <= int(thisPP->noTimeSamples) && j <= int(thisPP->noTimeSamples); ++i, ++j, ++k)
			{
				derivFlatSmall =
					optSpline.derivatives(double(i) / thisPP->noTimeSamples * thisPP->finalPlanHorizon, aiv::FlatoutputMonocycle::flatDerivDeg);

				derivFlatSmallOther = 
					it->second->getPathPlanner()->getSpline().derivatives(double(j) / thisPP->noTimeSamples * thisPP->finalPlanHorizon, aiv::FlatoutputMonocycle::flatDerivDeg);

				myPose = aiv::FlatoutputMonocycle::flatToPose(derivFlatSmall);
				otherPose = aiv::FlatoutputMonocycle::flatToPose(derivFlatSmallOther);

				result[nAcc + int(thisPP->noTimeSamples)*ieqPerSample + k] =
					thisPP->secRho + it->second->getPathPlanner()->getSecRho() - (myPose.head(2) - otherPose.head(2)).norm();
				//std::cout << "dist: " << thisPP->secRho + it->second->getPathPlanner()->getSecRho() << "myPos: " << myPose.head(2) << "othPos: " << otherPose.head(2) << std::endl;
				// result[nAcc + int(thisPP->noTimeSamples)*ieqPerSample + k] =
					// -1;
				// std::cout << "G[" << nAcc + int(thisPP->noTimeSamples)*ieqPerSample + k << "] " << result[nAcc + int(thisPP->noTimeSamples)*ieqPerSample + k] << std::endl;

			}
		}
	}

	void PathPlannerRecHor::eval_dobst_ineq(double *result, unsigned n, const double* x, void* data)
	{
		// get this path planner pointer
		PathPlannerRecHor *thisPP = static_cast< PathPlannerRecHor *>(data);

		// Create a CtrlPoints vector
		MySpline::ControlPointVectorType ctrlPts(splDim, thisPP->noCtrlPts);

		// Feed remaining rows with values from the primal variables x
		for (int i = 0; i < n; ++i)
		{
			ctrlPts(i % splDim, i / splDim) = x[i];
		}

		// Create a spline based on the new ctrlpts (using the same knots as before)
		MySpline optSpline(thisPP->auxSpline.knots(), ctrlPts);


		// Dist 2 obst INEQUATIONS

		NDerivativesMatrix derivFlatSmall;

		PoseVector pose;

		//#pragma omp parallel for
		for (int i = 1; i <= int(thisPP->noTimeSamples); ++i)
		{
			derivFlatSmall = optSpline.derivatives(double(i) / thisPP->noTimeSamples * thisPP->finalPlanHorizon, aiv::FlatoutputMonocycle::flatDerivDeg);

			pose = aiv::FlatoutputMonocycle::flatToPose(derivFlatSmall);

			// Obstacles
			std::map<std::string, Obstacle *>::iterator it;
			int j;
			for (it = thisPP->detectedObstacles.begin(), j = 0; j < thisPP->detectedObstacles.size(); ++j, ++it)
			{
				result[(i - 1)*thisPP->detectedObstacles.size() + j] = -1. * it->second->distToAIV(pose.head(2) + thisPP->latestPose.head(2), 0.55);
			}
		}
	}


	void  PathPlannerRecHor::ineqFunc(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data)
	{
		// get this path planner pointer
		PathPlannerRecHor *thisPP = static_cast< PathPlannerRecHor *>(data);

		thisPP->eval_ineq(result, n, x, data);

		std::ofstream arquivo;
		arquivo.open("optlog/gieqiter.csv", std::fstream::app);
		for (int i = 0; i < int(m); ++i)
		{
			//x[i] = startSpline.ctrls()(i%(splDim-1)+1, i/(splDim-1));
			arquivo << result[i] << ", ";

		}
		arquivo << std::endl;
		arquivo.close();

		const int accDim = aiv::FlatoutputMonocycle::accelerationDim;
		const int velDim = aiv::FlatoutputMonocycle::velocityDim;
		//int Xdim = thisPP->noCtrlPts * aiv::FlatoutputMonocycle::flatDim;
		int nobst = thisPP->detectedObstacles.size();

		if (grad)
		{
			int span = thisPP->findspan(0.0);
			span -= thisPP->noIntervNonNull - 1;

			////std::cout << "ineq grad\n";
			Eigen::MatrixXd J(m, n);

			J.block(0, 0, accDim, n) = absddqJac(0.0, thisPP->finalPlanHorizon, span, x, n, thisPP->splDegree);

			for (int i = 1; i <= int(thisPP->noTimeSamples); ++i)
			{

				span = thisPP->findspan(double(i) / thisPP->noTimeSamples * thisPP->finalPlanHorizon) - (thisPP->noIntervNonNull - 1);

				J.block(accDim + (i - 1)*(accDim + velDim + nobst), 0, accDim, n) =
					absddqJac(double(i) / thisPP->noTimeSamples * thisPP->finalPlanHorizon, thisPP->finalPlanHorizon, span, x, n, thisPP->splDegree);

				J.block(accDim * 2 + (i - 1)*(accDim + velDim + nobst), 0, velDim, n) =
					absdqJac(double(i) / thisPP->noTimeSamples * thisPP->finalPlanHorizon, thisPP->finalPlanHorizon, span, x, n, thisPP->splDegree);

				int j;
				std::map<std::string, Obstacle *>::iterator it;
				for (it = thisPP->detectedObstacles.begin(), j = 0; it != thisPP->detectedObstacles.end(); ++it, ++j)
				{

					/*J.block(j + accDim * 2 + velDim + (i - 1)*(accDim + velDim + nobst), 0, 1, n) =
						obstDistJac(double(i) / thisPP->noTimeSamples * thisPP->finalPlanHorizon, thisPP->finalPlanHorizon, span, x, n, thisPP->splDegree, it->second->getPosition().block(0, 0, 2, 1));*/
				}
			}

			Eigen::Map<Eigen::MatrixXd>(grad, J.cols(), J.rows()) = J.transpose();

			const double eps = sqrt(std::numeric_limits< double >::epsilon());

			double *constrPre = new double[thisPP->noTimeSamples*thisPP->detectedObstacles.size()];
			double *constrPos = new double[thisPP->noTimeSamples*thisPP->detectedObstacles.size()];
			double *x1 = new double[n];
			double h, dx;

			const int velaccDim = aiv::FlatoutputMonocycle::accelerationDim + aiv::FlatoutputMonocycle::velocityDim;
			const int vel2accDim = 2 * aiv::FlatoutputMonocycle::accelerationDim + aiv::FlatoutputMonocycle::velocityDim;

			for (int i = 0; i < int(n); ++i)
			{
				x1[i] = x[i];
			}

			for (int i = 0; i < int(n); ++i)
			{
				h = x[i] < eps*thisPP->h ? eps*thisPP->h : eps*x[i];
				//h = x[i] < eps ? eps*eps : eps*x[i];
				//h = eps;
				x1[i] = x[i] + h;
				thisPP->eval_dobst_ineq(constrPos, n, x1, data);
				dx = x1[i] - x[i];
				x1[i] = x[i] - h;
				thisPP->eval_dobst_ineq(constrPre, n, x1, data);
				dx = dx + x[i] - x1[i];
				//dx = 2 * h;
				x1[i] = x[i];
				////std::cout << "INEQ: " << x[i] << ", " << eps << ", " << h << ", " << dx << std::endl;
				for (int j = 0; j < thisPP->detectedObstacles.size(); ++j)
				{
					for (int k = 0; k < thisPP->noTimeSamples; ++k)
					{
						////std::cout << j * (velaccDim + thisPP->detectedObstacles.size()) + vel2accDim << ", " << i << std::endl;
						grad[(k * (velaccDim + thisPP->detectedObstacles.size()) + vel2accDim + j) * n + i] = 
								(constrPos[k*thisPP->detectedObstacles.size()+j] - constrPre[k*thisPP->detectedObstacles.size()+j]) / dx;
						////std::cout << grad[j*n + i] << std::endl;
					}
				}
				
			}
			delete[] constrPre;
			delete[] constrPos;
			delete[] x1;

			//Eigen::MatrixXd e = Eigen::Map<Eigen::MatrixXd>(grad, m, n);

			/*//std::cout << n << ", " << m << std::endl;

			for (int i = 0; i < m; ++i)
			{
				for (int j = 0; j < n; ++j)
				{
					//std::cout << grad[i*n + j] << ", ";
				}
				//std::cout << std::endl;
			}

			//std::cout << J << std::endl;*/

			//for (int i = 0; i < thisPP->noTimeSamples; ++i)
			//{
			//	for (int j = 0; j < n; ++j)
			//	{
			//		////std::cout << i * 6 + 6 << ", " << j << std::endl;
			//		J(i*6 + 6, j) = grad[(i * 6 + 6)*n + j];
			//		J(i*6 + 6 + 1, j) = grad[(i * 6 + 6 + 1)*n + j];
			//	}
			//}
			/*for (int i = 0; i < m; ++i)
			{
				for (int j = 0; j < n; ++j)
				{
					J(i, j) = grad[(i)*n + j];
				}
			}*/

			////std::cout << "\nGrad\n" << Eigen::Map<Eigen::MatrixXd>(grad, 1, m*n) << std::endl;
			//Eigen::IOFormat CleanFmt(3, 0, " ", "\n", "", "");
			////std::cout << "\nJ\n" << J.block<70, 8>(2, 0).format(CleanFmt) << std::endl;

			/*Eigen::MatrixXd e(m, n);

			for (int i = 0; i < m; ++i)
			{
				for (int j = 0; j < n; ++j)
				{
					e(i, j) = grad[(i)*n + j];
				}
			}*/
			//Eigen::MatrixXd e = Eigen::Map<Eigen::MatrixXd>(grad, n, m);
			////std::cout << "\nJnum\n" << (J.block<86, 14>(0, 0) - e.block<86,14>(0, 0)).format(CleanFmt) << std::endl

			//boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
		}
	}
	void  PathPlannerRecHor::ineqFuncColl(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data)
	{
		// get this path planner pointer
		PathPlannerRecHor *thisPP = static_cast< PathPlannerRecHor *>(data);

		thisPP->eval_ineq_coll(result, n, x, data);

		if (grad)
		{
			const double eps = sqrt(std::numeric_limits< double >::epsilon());

			double *constrPre = new double[m];
			double *constrPos = new double[m];
			double *x1 = new double[n];
			double h, dx;

			for (int i = 0; i < int(n); ++i)
			{
				x1[i] = x[i];
			}

			for (int i = 0; i < int(n); ++i)
			{
				h = x[i] < eps*thisPP->h ? eps*thisPP->h : eps*x[i];
				x1[i] = x[i] + h;
				thisPP->eval_ineq_coll(constrPos, n, x1, data);
				dx = x1[i] - x[i];
				x1[i] = x[i] - h;
				thisPP->eval_ineq_coll(constrPre, n, x1, data);
				dx = dx + x[i] - x1[i];
				x1[i] = x[i];

				for (int j = 0; j < m; ++j)
				{
					grad[j*n + i] = (constrPos[j] - constrPre[j]) / dx;
				}
				
			}
			delete[] constrPre;
			delete[] constrPos;
			delete[] x1;
		}
	}

	// Stand alone constraints, final step

	double  PathPlannerRecHor::objectFuncLS(unsigned n, const double *x, double *grad, void *my_func_data)
	{
		double fx = x[0] * x[0];

		////std::cout << "objectFunLS" << std::endl;

		if (grad)
		{
			//myNLP->eval_grad_f ( n, x, true, grad );
			grad[0] = 2 * x[0];
			for (unsigned i = 1; i < n; ++i)
			{
				grad[i] = 0.0;
			}
		}

		return fx;
	}

	void PathPlannerRecHor::eval_eqLS(double *result, unsigned n, const double* x, void* data)
	{
		double finalPlanHorizon = x[0];

		// get this path planner pointer
		PathPlannerRecHor *thisPP = static_cast< PathPlannerRecHor *>(data);

		// Create a CtrlPoints vector
		MySpline::ControlPointVectorType ctrlPts(splDim, thisPP->noCtrlPts);

		// Feed remaining rows with values from the primal variables x
		for (int i = 1; i < n; ++i)
		{
			ctrlPts((i - 1) % splDim, (i - 1) / splDim) = x[i];
		}

		// Create a spline based on the new ctrlpts (using the same knots as before)
		MySpline optSpline(thisPP->genKnots(0.0, finalPlanHorizon, false), ctrlPts); // has to be false, otherwise jacobien will be wrong


		// EQUATIONS


		// @t=0

		// Create a Matrix consisting of the flat output and its needed derivatives for the instant T0 (0.0)
		NDerivativesMatrix derivFlatEq =
			optSpline.derivatives(0.0, aiv::FlatoutputMonocycle::flatDerivDeg);

		// Get error from pose at T0 and pose at the end of the previous plan (or initial position if this is the very first plan)
		PoseVector diffPose = aiv::FlatoutputMonocycle::flatToPose(derivFlatEq);
		diffPose.tail(1) -= thisPP->latestPose.tail(1);

		VelVector diffVelocity = aiv::FlatoutputMonocycle::flatToVelocity(derivFlatEq) - thisPP->latestVelocity;

		int i = 0;
		for (; i < aiv::FlatoutputMonocycle::poseDim; ++i)
		{
			result[i] = diffPose[i];
		}
		int j = i;
		for (; j - i < aiv::FlatoutputMonocycle::velocityDim; ++j)
		{
			result[j] = diffVelocity[j - i];
		}

		// @t=Tp

		derivFlatEq =
			optSpline.derivatives(finalPlanHorizon, aiv::FlatoutputMonocycle::flatDerivDeg);

		// Get error from pose at Tp and targeted pose
		diffPose = aiv::FlatoutputMonocycle::flatToPose(derivFlatEq);
		diffPose.block<aiv::FlatoutputMonocycle::posDim, 1>(aiv::FlatoutputMonocycle::posIdx, 0) +=
			thisPP->latestPose.block<aiv::FlatoutputMonocycle::posDim, 1>(aiv::FlatoutputMonocycle::posIdx, 0);
		diffPose -= thisPP->targetedPose;

		diffVelocity = aiv::FlatoutputMonocycle::flatToVelocity(derivFlatEq) - thisPP->targetedVelocity;

		for (i = j; i - j < aiv::FlatoutputMonocycle::poseDim; ++i)
		{
			result[i] = diffPose[i - j];
		}
		for (j = i; j - i < aiv::FlatoutputMonocycle::velocityDim; ++j)
		{
			result[j] = diffVelocity[j - i];
		}
	}

	void  PathPlannerRecHor::eqFuncLS(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data)
	{
		////std::cout << "eqFuncLS" << std::endl;
		// get this path planner pointer
		PathPlannerRecHor *thisPP = static_cast< PathPlannerRecHor *>(data);

		thisPP->eval_eqLS(result, n, x, data);

		if (grad)
		{
			const double eps = sqrt(std::numeric_limits< double >::epsilon());
			double *constrPre = new double[m];
			double *constrPos = new double[m];
			double *x1 = new double[n];
			double h, dx;

			for (int i = 0; i < int(n); ++i)
			{
				x1[i] = x[i];
			}

			for (int i = 0; i < int(n); ++i)
			{
				//h = x[i] < eps*1e2 ? eps*eps*1e2 : eps*x[i];
				h = x[i] < eps ? eps*eps : eps*x[i];
				//h = eps;
				x1[i] = x[i] + h;
				thisPP->eval_eqLS(constrPos, n, x1, data);
				dx = x1[i] - x[i];
				x1[i] = x[i] - h;
				thisPP->eval_eqLS(constrPre, n, x1, data);
				dx = dx + x[i] - x1[i];
				//dx = 2 * h;
				x1[i] = x[i];
				////std::cout << "INEQ: " << x[i] << ", " << eps << ", " << h << ", " << dx << std::endl;
				for (int j = 0; j < m; ++j)
				{
					grad[j*n + i] = (constrPos[j] - constrPre[j]) / dx;
					////std::cout << grad[j*n + i] << std::endl;
				}
			}
			delete[] constrPre;
			delete[] constrPos;
			delete[] x1;
		}
	}

	void PathPlannerRecHor::eval_ineqLS(double *result, unsigned n, const double* x, void* data)
	{
		////std::cout << "eval_ineqLS" << std::endl;
		double finalPlanHorizon = x[0];

		// get this path planner pointer
		PathPlannerRecHor *thisPP = static_cast< PathPlannerRecHor *>(data);

		// Create a CtrlPoints vector
		MySpline::ControlPointVectorType ctrlPts(splDim, thisPP->noCtrlPts);

		// Feed remaining rows with values from the primal variables x
		for (int i = 1; i < n; ++i)
		{
			ctrlPts((i - 1) % splDim, (i - 1) / splDim) = x[i];
		}

		// Create a spline based on the new ctrlpts (using the same knots as before)
		MySpline optSpline(thisPP->genKnots(0.0, finalPlanHorizon, false), ctrlPts); // has to be false, otherwise jacobien will be wrong

		////std::cout << "eval_ineqLS 2" << std::endl;
		// INEQUATIONS


		typedef Eigen::Matrix< double, aiv::FlatoutputMonocycle::flatDim, aiv::FlatoutputMonocycle::flatDerivDeg + 2 > Np1DerivativesMatrix;

		Np1DerivativesMatrix derivFlat;
		NDerivativesMatrix derivFlatSmall;

		// inequations t = 0.0 and t = Tp

		// Create Matrices for storing velocity and acceleration
		VelVector velocity;
		AccVector acceleration;

		derivFlat =
			optSpline.derivatives(0.0, aiv::FlatoutputMonocycle::flatDerivDeg + 1);

		acceleration = aiv::FlatoutputMonocycle::flatToAcceleration(derivFlat);

		int j = 0;
		for (; j < aiv::FlatoutputMonocycle::accelerationDim; ++j)
		{
			const double absAcc = (acceleration(j, 0) > -acceleration(j, 0)) ? acceleration(j, 0) : -acceleration(j, 0);
			result[j] = absAcc - thisPP->maxAcceleration(j, 0);
			////std::cout << "G[" << j << "] " << result[j] << std::endl;
		}

		derivFlat =
			optSpline.derivatives(finalPlanHorizon, aiv::FlatoutputMonocycle::flatDerivDeg + 1);

		acceleration = aiv::FlatoutputMonocycle::flatToAcceleration(derivFlat);

		int nAcc = j;

		for (j = nAcc; j - nAcc < aiv::FlatoutputMonocycle::accelerationDim; ++j)
		{
			const double absAcc = (acceleration(j - nAcc, 0) > -acceleration(j - nAcc, 0)) ? acceleration(j - nAcc, 0) : -acceleration(j - nAcc, 0);
			result[j] = absAcc - thisPP->maxAcceleration(j - nAcc, 0);
			////std::cout << "G[" << j << "] " << result[j] << std::endl;
		}
		nAcc = 2*aiv::FlatoutputMonocycle::accelerationDim;

		// inequations in ]0.0, Tp[

		//#pragma omp parallel for
		for (int i = 1; i < int(thisPP->noTimeSamples); ++i)
		{
			////std::cout << "IEQ: SAMPLE IDX: " << i << std::endl;
			derivFlat =
				optSpline.derivatives(double(i) / thisPP->noTimeSamples * finalPlanHorizon, aiv::FlatoutputMonocycle::flatDerivDeg + 1);

			////std::cout << "got derive meatrix" << std::endl;
			// reusing derivFlatEq variable, ugly but let's move on
			derivFlatSmall = derivFlat.block<aiv::FlatoutputMonocycle::flatDim, aiv::FlatoutputMonocycle::flatDerivDeg + 1>(0, 0);

			////std::cout << "got vel" << std::endl;
			velocity = aiv::FlatoutputMonocycle::flatToVelocity(derivFlatSmall);

			////std::cout << "got acc" << std::endl;
			acceleration = aiv::FlatoutputMonocycle::flatToAcceleration(derivFlat);

			int ieqPerSample = (aiv::FlatoutputMonocycle::velocityDim + aiv::FlatoutputMonocycle::accelerationDim);
			//int ieqPerSample = (aiv::FlatoutputMonocycle::velocityDim);

			int j;
			for (j = 0; j < aiv::FlatoutputMonocycle::accelerationDim; ++j)
			{
				const double absAcc = (acceleration(j, 0) > -acceleration(j, 0)) ? acceleration(j, 0) : -acceleration(j, 0);
				result[nAcc + j + (i - 1)*ieqPerSample] = absAcc - thisPP->maxAcceleration(j, 0);
				////std::cout << "G[" << nAcc + j + (i - 1)*ieqPerSample << "] " << result[nAcc + j + (i - 1)*ieqPerSample] << std::endl;
			}

			for (int k = j; k - j < aiv::FlatoutputMonocycle::velocityDim; ++k)
			{
				const double absVel = (velocity(k - j, 0) > -velocity(k - j, 0)) ? velocity(k - j, 0) : -velocity(k - j, 0);
				result[nAcc + k + (i - 1)*ieqPerSample] = absVel - thisPP->maxVelocity(k - j, 0);
				////std::cout << "G[" << nAcc + k + (i - 1)*ieqPerSample << "] " << result[nAcc + k + (i - 1)*ieqPerSample] << std::endl;
			}
		}
		//system("pause");
	}

	void  PathPlannerRecHor::ineqFuncLS(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data)
	{
		////std::cout << "ineqFuncLS" << std::endl;
		// get this path planner pointer
		PathPlannerRecHor *thisPP = static_cast< PathPlannerRecHor *>(data);

		thisPP->eval_ineqLS(result, n, x, data);

		////std::cout << "ineqFuncLS 2" << std::endl;

		if (grad)
		{
			const double eps = sqrt(std::numeric_limits< double >::epsilon());
			double *constrPre = new double[m];
			double *constrPos = new double[m];
			double *x1 = new double[n];
			double h, dx;

			for (int i = 0; i < int(n); ++i)
			{
				x1[i] = x[i];
			}

			for (int i = 0; i < int(n); ++i)
			{
				//h = x[i] < eps*1e2 ? eps*eps*1e2 : eps*x[i];
				h = x[i] < eps ? eps*eps : eps*x[i];
				//h = eps;
				x1[i] = x[i] + h;
				thisPP->eval_ineqLS(constrPos, n, x1, data);
				dx = x1[i] - x[i];
				x1[i] = x[i] - h;
				thisPP->eval_ineqLS(constrPre, n, x1, data);
				dx = dx + x[i] - x1[i];
				//dx = 2 * h;
				x1[i] = x[i];
				////std::cout << "INEQ: " << x[i] << ", " << eps << ", " << h << ", " << dx << std::endl;
				for (int j = 0; j < m; ++j)
				{
					grad[j*n + i] = (constrPos[j] - constrPre[j]) / dx;
					////std::cout << grad[j*n + i] << std::endl;
				}
			}
			delete[] constrPre;
			delete[] constrPos;
			delete[] x1;
		}
	}

	Eigen::Array< double, 1, Eigen::Dynamic >  PathPlannerRecHor::genKnots(const double initT, const double finalT, bool nonUniform)
	{
		// TODO throw error
		if (this->noIntervNonNull < 2)
		{
			//std::cout << "ERROR\n";
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

	int PathPlannerRecHor::findspan(const double t)
	{
		int ret = 0;
		/*//std::cout << "\n all knots :\n" << this->auxSpline.knots() << std::endl;
		//std::cout << "\n knots@0 :\n" << this->auxSpline.knots()(0, 0) << std::endl;
		//std::cout << "\n knots@4 :\n" << this->auxSpline.knots()(0, 4) << std::endl;
		//std::cout << "\n knots@5 :\n" << this->auxSpline.knots()(0, 5) << std::endl;
		//std::cout << "\nt :\n"  << t << std::endl;*/
		while (ret <= this->noCtrlPts - 1 &&  t >= this->auxSpline.knots()(0,ret))
			ret++;
		////std::cout << "\nspan:\n" << ret - 1 << std::endl;
		return ret - 1;
	}
}
// cmake:sourcegroup=PathPlanner