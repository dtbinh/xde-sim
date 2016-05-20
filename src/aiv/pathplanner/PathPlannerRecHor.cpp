#include "aiv/pathplanner/PathPlannerRecHor.hpp"
#include "aiv/obstacle/Obstacle.hpp"
#include "aiv/pathplanner/ImaginaryForbiddenSpace.hpp"
#include "aiv/robot/AIV.hpp"

//#include "aiv/pathplanner/constrJac.hpp"
//#include "aiv/pathplanner/eq_cons_jac.hpp"
#include <omp.h>
#include <boost/foreach.hpp>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <string>

#include <vector>
#include <list>
#include <utility> // for pair


#include <boost/math/special_functions/fpclassify.hpp>

// #define foreach_ BOOST_FOREACH

// #ifndef HAVE_CONFIG_H
// #define HAVE_CONFIG_H
// #endif

using boost::property_tree::ptree;
using namespace Eigen;

namespace aiv
{

	PathPlannerRecHor::PathPlannerRecHor(std::string name, double updateTimeStep)
		: PathPlanner(name)
		, _updateTimeStep(updateTimeStep)
		, _currentPlanningTime(0.0)
		, _lastStepMinDist(0.5)
		, _xTol(0.001)
		, _fTol(0.001)
		, _eqTol(0.001)
		, _ineqTol(0.001)
		, _lastMaxIteration(50)
		, _firstMaxIteration(50)
		, _interMaxIteration(50)

		, _conflictFreePathDeviation(4.0)
		, _interRobotSafetyDist(0.1)
		, _robotObstacleSafetyDist(0.1)

		, _poseOutput(PoseVector::Zero())
		, _velocityOutput(VeloVector::Zero())
		, _accelOutput(AccelVector::Zero())

		, _maxVelocity(VeloVector::Constant(std::numeric_limits<double>::infinity()))
		, _maxAcceleration(AccelVector::Constant(std::numeric_limits<double>::infinity()))

		, _planStage(INIT)
		, _intendedPlanStage(INIT)
		//, _ongoingPlanIdx(-1)
		, _executingPlanIdx(-1)
		//, _isThereConfl(NONE)
		, _planLastPlan(false)
		, _optimizerType("SLSQP")
		// , _h(1e-15)
		, _comRange(15)
		, _firstPlanTimespan(1.0)
		, _numDerivativeFactor(1e3)
		, _radius(0.0)
		, _optIterCnter(0)
		, _timeOffset(Common::getRealTime<double>())
	{
		_planOngoingMutex.initialize();
		//this->opt_log.open("optlog///opt_log.txt");
		//_eq_log.open("optlog/eq_log.txt");
		read_xml("../../../../src/aiv/output.xml", _property_tree);
	}

	PathPlannerRecHor::~PathPlannerRecHor()
	{
		//this->opt_log.close();
		//_eq_log.close();
	}

	void PathPlannerRecHor::init(
		const PoseVector &initPose, // x, y, theta
		const VeloVector &initVelocity, // v, w
		const PoseVector &targetedPose, // x, y, theta
		const VeloVector &targetedVelocity, // v, w
		const VeloVector &maxVelocity, // v, w
		double compHorizon,
		double planHorizon,
		unsigned nTimeSamples,
		unsigned nIntervNonNull)
	{
		_initPose = initPose;
		_initFlat = FlatoutputMonocycle::poseToFlat(_initPose);
		_initVelocity = initVelocity;

		_targetedPose = targetedPose;
		_targetedFlat = FlatoutputMonocycle::poseToFlat(_targetedPose);
		_targetedVelocity = targetedVelocity;
		
		_poseOutput = initPose;
		_velocityOutput = initVelocity;

		_maxVelocity = maxVelocity;

		_compHorizon = compHorizon; // computation horizon
		_optPlanHorizon = planHorizon; // planning horizon
		_planHorizon = planHorizon; // planning horizon
		_refPlanHorizon = planHorizon;
		_nTimeSamples = nTimeSamples; // number of  time samples taken within a planning horizon
		
		unsigned nCtrlPts = nIntervNonNull + splDegree;
		_trajectory.setOption("nCtrlPts", nCtrlPts);
		_optTrajectory.setOption("nCtrlPts", nCtrlPts);
		_trajectory.setOption("nIntervNonNull", nIntervNonNull);
		_optTrajectory.setOption("nIntervNonNull", nIntervNonNull);
		
		_maxStraightDist = planHorizon * maxVelocity(FlatoutputMonocycle::linSpeedIdx);

		_latestFlat = _initFlat;
		_latestPose = _initPose;
		_initPoseForCurrPlan = initPose;
		_initPoseForFuturePlan = initPose;
		_latestVelocity = initVelocity;
	}

	void PathPlannerRecHor::init(
		const PoseVector &initPose, // x, y, theta
		const VeloVector &initVelocity, // v, w
		const PoseVector &targetedPose, // x, y, theta
		const VeloVector &targetedVelocity, // v, w
		const VeloVector &maxVelocity, // v, w
		const VeloVector &maxAcceleration, // dv, dw
		double compHorizon,
		double planHorizon,
		unsigned nTimeSamples,
		unsigned nIntervNonNull)
	{
		init(initPose, initVelocity, targetedPose, targetedVelocity, maxVelocity, compHorizon, planHorizon, nTimeSamples, nIntervNonNull);
		_maxAcceleration = maxAcceleration;
	}

	void PathPlannerRecHor::setOption(const std::string& optionName, const double optionValue)
	{
		if (optionName == "lastStepMinDist")
		{
			_lastStepMinDist = optionValue;
		}
		else if (optionName == "xTol")
		{
			_xTol = optionValue;
		}
		else if (optionName == "fTol")
		{
			_fTol = optionValue;
		}
		else if (optionName == "eqTol")
		{
			_eqTol = optionValue;
		}
		else if (optionName == "ineqTol")
		{
			_ineqTol = optionValue;
		}
		else if (optionName == "conflictFreePathDeviation")
		{
			_conflictFreePathDeviation = optionValue;
		}
		else if (optionName == "interRobotSafetyDist")
		{
			_interRobotSafetyDist = optionValue;
		}
		else if (optionName == "robotObstacleSafetyDist")
		{
			_robotObstacleSafetyDist = optionValue;
		}
		else if (optionName == "offsetTime")
		{
			_firstPlanTimespan = optionValue;
		}
		else if (optionName == "numDerivativeFactor")
		{
			_numDerivativeFactor = optionValue;
		}
	}
	void PathPlannerRecHor::setOption(const std::string& optionName, const bool optionValue)
	{
		if (optionName == "waitForThread")
		{
			_waitForThread = optionValue;
		}
	}
	void PathPlannerRecHor::setOption(const std::string& optionName, const unsigned optionValue)
	{
		if (optionName == "lastMaxIteration")
		{
			_lastMaxIteration = optionValue;
		}
		else if (optionName == "firstMaxIteration")
		{
			_firstMaxIteration = optionValue;
		}
		else if (optionName == "interMaxIteration")
		{
			_interMaxIteration = optionValue;
		}
	}
	void PathPlannerRecHor::setOption(const std::string& optionName, const std::string& optionValue)
	{
		if (optionName == "optimizerType")
		{
			_optimizerType = optionValue;
		}
	}

	void PathPlannerRecHor::_conflictEval()
	{
		// Clear conflicts robots maps
		_collisionAIVs.clear();
		_comOutAIVs.clear();
		_pathsFromConflictualAIVs.clear();

		//MapIntendedPath

		// Vector2d myPosition;
		// myPosition << _currentPose.x(), _currentPose.y();

		// Vector2d othPosition;

		for (MapAIV::iterator it = _knownRobots.begin(); it != _knownRobots.end(); ++it)
		{
			
			
			it->second->getPathPlanner()->lockIntendedSolMutex();
			
			
			double othBaseTime = it->second->getPathPlanner()->getIntendedBaseTime();
			double othPlanHor = it->second->getPathPlanner()->getIntendedPlanHor();
			FlatVector othPosition(it->second->getPathPlanner()->getIntendedBasePos());
			Trajectory othTraj = it->second->getPathPlanner()->getIntendedTrajectory();
			PlanStage othPlanStage = it->second->getPathPlanner()->getIntendedPlanStage();

			
			it->second->getPathPlanner()->unlockIntendedSolMutex();

			FlatVector myPosition(_intendedBasePos);

			double dSecutiry = std::max(_interRobotSafetyDist, it->second->getPathPlanner()->getInterRobSafDist());

			double minComRange = std::min(_comRange, it->second->getPathPlanner()->getComRange());
			
			if (othPlanStage == DONE) // assuming targeted velocity zero TODO
			{
				double dInterRobot = (myPosition - othPosition).norm();

				// VeloVector othTargetedLinVelo =  it->second->getPathPlanner()->getTargetedLinVelocity();

				double maxAbsRelVel = _maxVelocity[FlatoutputMonocycle::linAccelIdx];
					//+ othTargetedLinVelo;

				bool conflictual = false;

				if (dInterRobot <= dSecutiry + _intendedPlanHor*maxAbsRelVel)
				{
					_collisionAIVs[it->first] = it->second;
					std::cout << FG_B_L_CYAN << "=========== " << it->first << " was added" <<  "===========" << RESET << std::endl;

					conflictual = true;
				}
				
				if (_comAIVsSet.find(it->first) != _comAIVsSet.end() &&
					dInterRobot + maxAbsRelVel*_planHorizon >= minComRange)
				{
					_comOutAIVs[it->first] = it->second;

					conflictual = true;
				}

				if (conflictual)
				{
					FlatVector othTargetedPos = it->second->getPathPlanner()->getTargetedPose().block<FlatoutputMonocycle::posDim, 1>(FlatoutputMonocycle::posIdx, 0);

					_pathsFromConflictualAIVs[it->first] = othTargetedPos.replicate(1, _nTimeSamples);
				}

			}
			else
			{
				bool mIEarly = _intendedBaseTime >= othBaseTime && _intendedBaseTime <= othBaseTime + othPlanHor;
				bool mILate = _intendedBaseTime <= othBaseTime && _intendedBaseTime + _intendedPlanHor >= othBaseTime;

				if (mILate || mIEarly)
				{
					othPosition = mIEarly ? othTraj(_intendedBaseTime - othBaseTime) + othPosition : othPosition;
					myPosition = mILate ? _intendedTraj(othBaseTime - _intendedBaseTime) + myPosition : myPosition;
					
					std::cout << FG_B_L_CYAN << "=========== D_TIME " << _intendedBaseTime - othBaseTime << RESET << std::endl;
					std::cout << FG_B_L_CYAN << "=========== POS " << myPosition.transpose() << " ||| " << othPosition.transpose() << RESET << std::endl;

					double dInterRobot = (myPosition - othPosition).norm();

					double maxAbsRelVel = _maxVelocity[FlatoutputMonocycle::linAccelIdx] + it->second->getPathPlanner()->getMaxLinVelocity();

					std::cout << FG_B_L_CYAN << "=========== Dist|planHor|maxabsvel|sec " << dInterRobot << " ||| " << _intendedPlanHor << " ||| " << maxAbsRelVel <<  " ||| " << dSecutiry <<RESET << std::endl;

					bool conflictual = false;
					
					if (dInterRobot <= dSecutiry + _intendedPlanHor*maxAbsRelVel)
					{
						conflictual = true;
						_collisionAIVs[it->first] = it->second;
						std::cout << FG_B_L_CYAN << "=========== " << it->first << " was added" <<  "===========" << RESET << std::endl;
					}

					std::cout << FG_B_L_MAGENTA << "My pose: " << RESET << myPosition.transpose() << std::endl;
					std::cout << FG_B_L_MAGENTA << "Other pose: " << RESET << othPosition.transpose() << std::endl;

					std::cout << FG_B_L_MAGENTA << "Dist inter robots: " << RESET << dInterRobot << std::endl;
					std::cout << FG_B_L_MAGENTA << "Secuirty dist: " << RESET << dSecutiry << std::endl;
					std::cout << FG_B_L_MAGENTA << "MaxLinXplanHor: " << RESET << maxAbsRelVel*_planHorizon << std::endl;

					if (_comAIVsSet.find(it->first) != _comAIVsSet.end() &&
						dInterRobot + maxAbsRelVel*_planHorizon >= minComRange)
					{
						conflictual = true;
						_comOutAIVs[it->first] = it->second;
					}

					if (conflictual)
					{
						// othPosition = mIEarly ? othTraj(_intendedBaseTime - othBaseTime) + othPosition : othPosition;
						// myPosition = mILate ? _intendedTraj(othBaseTime - _intendedBaseTime) + myPosition : myPosition;

						// PoseVector othTargetedPose = it->second->getPathPlanner()->getTargetedPose();

						// FlatVector othTargetedPos = othTargetedPose.block<FlatoutputMonocycle::posDim, 1>(FlatoutputMonocycle::posIdx, 0);

						// FlatVector othTargetedOri;
						// othTargetedOri << cos(othTargetedPose(FlatoutputMonocycle::oriIdx, 0)),
						// 				  sin(othTargetedPose(FlatoutputMonocycle::oriIdx, 0));
						
						double diffTime = _intendedBaseTime - othBaseTime;

						// Initialization
						_pathsFromConflictualAIVs[it->first] = Matrix< double, FlatoutputMonocycle::posDim, Dynamic >::Zero(FlatoutputMonocycle::posDim, _nTimeSamples);

						// loop for creating path
						for (int i=1; i <= int(_nTimeSamples); ++i)
						{
							double othMoment = (double(i)/_nTimeSamples*_optPlanHorizon) + diffTime;

							if (othMoment > othPlanHor || othMoment < 0.0)
							{
								FlatVector othBasePosition;
								double othProjTime;
								double othLinVelo;
								FlatVector othOri;

								if (othMoment > othPlanHor)
								{
									NDerivativesMatrix derivFlat = othTraj(othPlanHor, FlatoutputMonocycle::flatDerivDeg);

									othBasePosition = derivFlat.col(0) + othPosition;

									double angle = FlatoutputMonocycle::flatToPose(derivFlat).tail<1>()(0,0);

									othOri << cos(angle),
											  sin(angle);

									othLinVelo = FlatoutputMonocycle::flatToVelocity(derivFlat)[FlatoutputMonocycle::linSpeedIdx];

									othProjTime = othMoment - othPlanHor;

								}
								else
								{
									NDerivativesMatrix derivFlat = othTraj(0.0, FlatoutputMonocycle::flatDerivDeg);

									othBasePosition = othPosition;

									double angle = FlatoutputMonocycle::flatToPose(derivFlat).tail<1>()(0,0);

									othOri << cos(angle),
											  sin(angle);

									othLinVelo = FlatoutputMonocycle::flatToVelocity(derivFlat)[FlatoutputMonocycle::linSpeedIdx];

									othProjTime = othMoment;
								}

								_pathsFromConflictualAIVs[it->first].col(i) = othBasePosition + othProjTime * othLinVelo * othOri;
							}
							else
							{
								_pathsFromConflictualAIVs[it->first].col(i) = othTraj(othMoment);
							}

						}
					}
				}
				else
				{
					std::cout << FG_B_L_MAGENTA << "TEMPORALY APART: " << RESET << _intendedBaseTime << ", " << othBaseTime << std::endl;
				}
			}
		}
	}

	void PathPlannerRecHor::update(
			MapObst detectedObst,
			MapAIV otherVehicles,
			const Displacementd & currentPose,
			const Twistd & currentVelo,
			const double myRadius) //const Displacementd & realPose, const Twistd &realVelocity)
	{
		// double tic = Common::getRealTime();
		
		_detectedObstacles = detectedObst;

		_knownRobotsMutex.lock();
		_knownRobots = otherVehicles;
		_knownRobotsMutex.unlock();
		
		_currentPose = currentPose;
		_currentVelo = currentVelo;
		_radius = myRadius;

		// Getting robot's orientation projected in the ground plane (unless the aiv is flying this is the correct orientation)
		// 1. get the quaternion part of the displacement
		Quaternion<double> q = Quaternion<double>(_currentPose.qw(), _currentPose.qx(), _currentPose.qy(), _currentPose.qz());
		// 2. rotation of UnityX by the quaternion q (i.e. x of the robot frame wrt the absolute frame, i.e. robot's direction)
		_currentDirec = q._transformVector(Vector3d::UnitX());

		// TODO Instead of communicating with everybody a comunication structure should be done
		_comAIVsSetMutex.lock();
		Common::KeysOnMapToVec(otherVehicles, _comAIVsSet);
		_comAIVsSetMutex.unlock();

		double evalTime = _currentPlanningTime - (std::max(_executingPlanIdx, 0)*_compHorizon);

		// --- PLANNING THREAD MANAGEMENT

		if (_planStage != DONE && _planStage != FINAL)
		{

			// First update call
			if (_currentPlanningTime == 0.0)
			{
				// _timeOffset = ;
				// std::cout << "_______ " << std::string(name).substr(0,10) <<" First update call ________ " << _executingPlanIdx << std::endl;
				// std::cout << "_______ PlanStage { INIT, INTER, FINAL, DONE }: " << _planStage << std::endl; 
				// std::cout << "_______ " << std::string(name).substr(0,10) <<" Spawn the first plan thread ________ " << _executingPlanIdx << std::endl;

				//_conflictEval();

				_planOngoingMutex.lock();
				
				_baseTime = 0.0 + _timeOffset;
				
				_planThread = boost::thread(&PathPlannerRecHor::_plan, this); // Do P0
				//std::cout << "_______ " << std::string(name).substr(0,10) <<" (C1) Spawn the first plan thread!!! ________ " << _executingPlanIdx << std::endl;
			}

			// If no plan is been executed yet but the fistPlanTimespan is over
			// Using  _currentPlanningTime - _firstPlanTimespan > 1e-14 instead of _currentPlanningTime > _firstPlanTimespan beacause num error 3.33067e-016
			else if (_executingPlanIdx == -1 && _currentPlanningTime - _firstPlanTimespan > 1e-14) // Time so robot touch the floor
			{
				// P0 will begin to be executed next
				++_executingPlanIdx;
				evalTime = _currentPlanningTime - _firstPlanTimespan;

				// Reset update call counter so evaluation time be corret
				//_updateCallCntr = 1; // the update for evalTime zero will be done next, during this same "update" call. Thus counter should be 1 for the next call.
				// std::cout << "evalTime: " << evalTime << std::endl;
				// std::cout << "updatetimestep: " << _updateTimeStep << std::endl;
				// std::cout << "_currentPlanningTime: " << _currentPlanningTime << std::endl;
				// std::cout << "_firstPlanTimespan: " << _firstPlanTimespan << std::endl;

				////std::cout << "firstPlanTimespan " <<  firstPlanTimespan << std::endl;
				// std::cout << "_______ " << std::string(name).substr(0,10) <<" Check if fist thread finished ________ " << _executingPlanIdx << std::endl;
				
				if (!_planOngoingMutex.try_lock()) // We are supposed to get this lock
				{
					if (_waitForThread)
						_planThread.join();
					else
						_planThread.interrupt();
					_planOngoingMutex.lock();
				}
				// else
					// std::cout << "_______ " << std::string(name).substr(0,10) <<" Got the mutex ________ " << _executingPlanIdx << std::endl;
				// Now we are sure that there is no ongoing planning!

				// update solution spline with the auxliar spline find in planning 0;
				// no need to lock a mutex associated to the auxiliar spline because we are sure there is no ongoing planning
				
				_currentPlanningTime = evalTime;
				
				_trajectory = _optTrajectory;
				_initPoseForFuturePlan = _latestPose;
				
				// after the end of a planning thread we are to not be in the INIT stage
				// if at the end of a planning thread _planLastPlan is true the next plan to be executed shall be the last
				_planStage = _planLastPlan ? FINAL : INTER;

				if (_planStage == FINAL) // No more planning threads
				{
					_planHorizon = _optPlanHorizon;
					_planOngoingMutex.unlock();
					//opt_log << "update: !!!!! Now goint to execute last plan !!!!!" << std::endl;
				}
				else
				{
					// plan next section!
					//this->initTimeOfCurrPlan = currentPlanningTime;
					// std::cout << "conflictEval 1" << std::endl;
					// _conflictEval();
					// std::cout << "conflictEval 2" << std::endl;
					_baseTime = _currentPlanningTime + _compHorizon + _timeOffset;;
					_planThread = boost::thread(&PathPlannerRecHor::_plan, this); // Do PX with X in (1, 2, ..., indefined_finit_value)
					//std::cout << "_______ " << std::string(name).substr(0,10) <<" (C2) Spawn the second plan thread!!! ________ " << _executingPlanIdx << std::endl;
					//opt_log << "update: _______ (C2) Spawn the second plan thread!!! ________ " << this->ongoingPlanIdx << std::endl;
				}

				// std::cout << "_______ PlanStage { INIT, INTER, FINAL, DONE }: " << _planStage << std::endl; 
				// std::cout << "_______ " << std::string(name).substr(0,10) <<" [S2] ________ " << _executingPlanIdx << std::endl;

			}

			// If the robot started to execute the motion
			else if (_executingPlanIdx >= 0 && evalTime >= _compHorizon)
			{
				std::cout << std::string(name).substr(0,10) << " pose: \033[1;44m";
				std::cout << _latestPose.transpose() << "\033[0m\n";
				++_executingPlanIdx;

				evalTime -= _compHorizon; // "Fix" evalTime

				// std::cout << "_______ " << std::string(name).substr(0,10) <<" Check if thread finished ________ " << _executingPlanIdx << std::endl;
				if (!_planOngoingMutex.try_lock()) // We are supposed to get this lock
				{
					if (_waitForThread == true)
						_planThread.join();
					else
						_planThread.interrupt();
					_planOngoingMutex.lock();
				}
				// else
					// std::cout << "_______ " << std::string(name).substr(0,10) <<" Got the mutex ________ " << _executingPlanIdx << std::endl;
				// Now we are sure that there is no ongoing planning!

				// update solution spline with the auxliar spline;
				// no need to lock a mutex associated to the auxiliar spline because we are sure there is no ongoing planning

				// std::cout << "_______ " << std::string(name).substr(0,10) <<" [S1] ________ " << _executingPlanIdx << std::endl;

				_trajectory = _optTrajectory;
				_initPoseForCurrPlan = _initPoseForFuturePlan; // update base pose
				_initPoseForFuturePlan = _latestPose; // get latest position found by the plan that just ended (will be the next base pos)

				// if at the end of a planning thread _planLastPlan is true the next plan to be executed shall be the last
				_planStage = _planLastPlan ? FINAL : _planStage;


				if (_planStage == FINAL)
				{
					_planHorizon = _optPlanHorizon;

					_intendedSolMutex.lock();
					_intendedBasePos = _targetedFlat;
					_intendedPlanStage = DONE;
					_intendedSolMutex.unlock();

					_planOngoingMutex.unlock();
					//opt_log << "update: !!!!! Now goint to execute last plan !!!!!" << std::endl;
				}
				else
				{
					// _conflictEval();
					_baseTime = _currentPlanningTime + _compHorizon + _timeOffset;;
					_planThread = boost::thread(&PathPlannerRecHor::_plan, this); // Do PX with X in (1, 2, ..., indefined_finit_value)
					//std::cout << "_______ " << std::string(name).substr(0,10) <<" (C3) Spawn new plan thread!!! ________ " << _executingPlanIdx << std::endl;
				}

				// std::cout << "_______ PlanStage { INIT, INTER, FINAL, DONE }: " << _planStage << std::endl; 
				// std::cout << "_______ " << std::string(name).substr(0,10) <<" [S2] ________ " << _executingPlanIdx << std::endl;

			}
		}
		else if (_planStage == FINAL && evalTime > _planHorizon)
		{
			////std::cout << "gone to DONE" << std::endl;
			_planStage = DONE;
			_poseOutput = _targetedPose;
			_velocityOutput = _targetedVelocity;
			_accelOutput = AccelVector::Zero();
		}

		// --- APPARENT UPDATE

		if (_planStage == INIT)
		{
			// do nothing
			// double ctrlpts[20];
			// for (auto i = 0; i < 20; ++i)
			// 	ctrlpts[i] = -1;
			// _trajectory.getParameters(ctrlpts);
			// std::cout << "____________" << std::endl;
			// for (auto i = 0; i < 20; ++i)
			// 	std::cout << ctrlpts[i] << ", " << std::endl;
			// std::cout << "____________" << std::endl;
			//std::cout << _trajectory(evalTime*_planHorizon, 3) << std::endl;
		}
		else if (_planStage == DONE)
		{
			std::cout << "Planning is over!" << std::endl;
			std::cout << "Last step planning horizon: " << _planHorizon << std::endl;
			boost::this_thread::sleep(boost::posix_time::milliseconds(20));
		}
		else //INTER or FINAL
		{
			// just use solSpline to get the nextReferences values
			//std::cout << "evalTime: " << evalTime << std::endl;

			NDerivativesMatrix derivFlat = _trajectory(evalTime, FlatoutputMonocycle::flatDerivDeg);

			Np1DerivativesMatrix derivFlat2 = _trajectory(evalTime, FlatoutputMonocycle::flatDerivDeg+1);

			_poseOutput = FlatoutputMonocycle::flatToPose(derivFlat);

			_poseOutput.block<FlatoutputMonocycle::posDim, 1>(FlatoutputMonocycle::posIdx, 0) =
				// _rotMat2WRef * _poseOutput.block<FlatoutputMonocycle::posDim, 1>(FlatoutputMonocycle::posIdx, 0) +
				_poseOutput.block<FlatoutputMonocycle::posDim, 1>(FlatoutputMonocycle::posIdx, 0) +
				_initPoseForCurrPlan.block<FlatoutputMonocycle::posDim, 1>(FlatoutputMonocycle::posIdx, 0);

			// _poseOutput(FlatoutputMonocycle::oriIdx, 0) =
				// Common::wrapToPi(_poseOutput(FlatoutputMonocycle::oriIdx, 0) + _initPoseForCurrPlan(FlatoutputMonocycle::oriIdx, 0));

			_velocityOutput = FlatoutputMonocycle::flatToVelocity(derivFlat);

			_accelOutput = FlatoutputMonocycle::flatToAcceleration(derivFlat2);
		}

		_currentPlanningTime += _updateTimeStep;
		// std::cout << "planner elapsed time: " <<  Common::getRealTime() - tic << std::endl; 
		return;
	}

	bool PathPlannerRecHor::_isAnyForbSpacInRobotsWayToTarget(MapObst forbiddenSpaces)
	{
		FlatVector robotToTarget = _targetedFlat - _latestFlat;
		double robotToTargetDist = robotToTarget.norm();
		FlatVector targetDirec = robotToTarget/robotToTargetDist;
		double targetTheta = atan2(targetDirec(1,0), targetDirec(0,0));

		for(std::map<std::string, Obstacle* >::iterator it=forbiddenSpaces.begin();it != forbiddenSpaces.end(); ++it )
		{
			FlatVector forbSpacToRobot;
			FlatVector forbSpacToWayPt;
			double segmentNormTarg;
			double discriminantTarg;
			double determinantP1P2Targ;
			double rad;

			rad = it->second->getRad() + _radius + _robotObstacleSafetyDist;

			forbSpacToRobot = _latestFlat - it->second->getCurrentPosition().getTranslation().block<2,1>(0,0);
			
			forbSpacToWayPt = _wayPt - it->second->getCurrentPosition().getTranslation().block<2,1>(0,0);

			segmentNormTarg = (forbSpacToWayPt - forbSpacToRobot).norm();

			determinantP1P2Targ = forbSpacToRobot(0,0)*forbSpacToWayPt(1,0) - forbSpacToWayPt(0,0)*forbSpacToRobot(1,0);

			discriminantTarg = rad*rad * segmentNormTarg*segmentNormTarg - determinantP1P2Targ*determinantP1P2Targ;

			if (discriminantTarg >= 0.0) // itersection
			{
				FlatVector robotToForbSpac = -1.*forbSpacToRobot;
				double robotToForbSpacDist = robotToForbSpac.norm();

				double signed_r2o_proj_on_tdir = robotToForbSpacDist * cos(atan2(robotToForbSpac(1,0), robotToForbSpac(0,0)) - targetTheta);

				if (signed_r2o_proj_on_tdir <= _maxVelocity.block<FlatoutputMonocycle::veloDim, 1>(FlatoutputMonocycle::linSpeedIdx, 0)(0,0)*_compHorizon)
				{
					continue;
				}
				else return true;
			}
		}
		return false;
	}

	// bool PathPlannerRecHor::_intersectionDiscriminant(const std::string& fsName, FlatVector )
	// {
	// 	double rad = _knownObstacles[fsName]->getRad() + _radius + _robotObstacleSafetyDist;

	// 	FlatVector robotToForbSpac = _knownObstacles[fsName]->getCurrentPosition().getTranslation().block<2,1>(0,0) - _latestFlat;

	// 	FlatVector forbSpacToRobot = -1.*robotToForbSpac;

	// 	// FlatVector robotToWayPt = _wayPt - _latestFlat;
	// 	// double robotToWayPtDist = robotToWayPt.norm();
	// 	double robotToForbSpacDist = robotToForbSpac.norm();
		
	// 	FlatVector wayPtDirec = robotToWayPt/robotToWayPtDist;
	// 	double wayPtTheta = atan2(wayPtDirec(1,0), wayPtDirec(0,0));

	// 	forbSpacToWayPt = _wayPt - _knownObstacles[fsName]->getCurrentPosition().getTranslation().block<2,1>(0,0);

	// 	forbSpacToTarget = _targetedFlat - _knownObstacles[fsName]->getCurrentPosition().getTranslation().block<2,1>(0,0);

	// 	// double rad = _knownObstacles[fsName]->getRad() + _radius;

	// 	double segmentNorm = (forbSpacToWayPt - forbSpacToRobot).norm();

	// 	determinantP1P2 = forbSpacToRobot(0,0)*forbSpacToWayPt(1,0) - forbSpacToWayPt(0,0)*forbSpacToRobot(1,0);

	// 	discriminantWay = rad*rad * segmentNorm*segmentNorm - determinantP1P2*determinantP1P2;

	// 	if (discriminantWay < 0)
	// 	{
	// 		return false;
	// 	}
	// 	else
	// 	{
	// 		//robotToForbSpacDist
	// 		signed_r2o_proj_on_wdir = robotToForbSpacDist * cos(atan2(robotToForbSpac(1,0), robotToForbSpac(0,0))-wayPtTheta)

	// 		if (signed_r2o_proj_on_wdir <= _maxVelocity.block<double FlatoutputMonocycle::veloDim, 1>(FlatoutputMonocycle::linSpeedIdx, 0)*_compHorizon)
	// 		{
	// 			return false;
	// 		}
	// 	}

	// 	if (!_isAnyForbSpacInRobotsWayToTarget())
	// 	{
	// 		return false;
	// 	}

	// 	return true;
	// }

	Common::CArray3d PathPlannerRecHor::_estimateCollisionCenterAndRadius(const std::string& othRobName)
	{

		// POSE is [x, y, z, q], where q = [q1, q2, q3, q4] a quarternion representation of the orientation
		// Get it
		// std::cout << FG_B_L_GREEN << "_estimateCollisionCenterAndRadius [1]" << RESET << std::endl;
		Displacementd otherPose = _knownRobots[othRobName]->getCurrentPosition();
		// std::cout << FG_B_L_GREEN << "_estimateCollisionCenterAndRadius [2]" << RESET << std::endl;

		// Getting robot's orientation projected in the ground plane (unless the aiv is flying this is the correct orientation)
		// 1. get the quaternion part of the displacement
		Quaternion<double> q = Quaternion<double>(otherPose.qw(), otherPose.qx(), otherPose.qy(), otherPose.qz());
		// 2. rotation of UnityX by the quaternion q (i.e. x of the robot frame wrt the absolute frame, i.e. robot's direction)
		QuaternionBase<Quaternion<double> >::Vector3 otherDirec = q._transformVector(Vector3d::UnitX());
		//atan2(coordXB.y(), coordXB.x())

		// std::cout << FG_B_L_GREEN << "_estimateCollisionCenterAndRadius [3]" << RESET << std::endl;

		FlatVector othPos;
		othPos << otherPose.x(), otherPose.y();

		// REAL VELOCITY is [wx wy wz vx vy vz] wrt the robot frame
		Twistd otherVelocity = _knownRobots[othRobName]->getCurrentVelocity();

		double otherLinVel = otherVelocity.block<2, 1>(3, 0).norm(); // sqrt(vx^2 + vy^2) linear speed in the robot plane xy
		// double otherAngVel = otherVelocity.topRows(3).z(); // rotation around z of the robot frame

		double myLinVel = _currentVelo.block<2, 1>(3, 0).norm();
		// double myAngVel = _currentVelo.topRows(3).z();

		double deltaVx = myLinVel*_currentDirec.x() - otherLinVel*otherDirec.x();
		double deltaVy = myLinVel*_currentDirec.y() - otherLinVel*otherDirec.y();

		double deltaP0x = _currentPose.x() - otherPose.x();
		double deltaP0y = _currentPose.y() - otherPose.y();

		double a = deltaVx*deltaVx + deltaVy*deltaVy;

		double b = 2*deltaP0x*deltaVx + 2*deltaP0y*deltaVy;

		double c = deltaP0x*deltaP0x + deltaP0y*deltaP0y - pow(_radius + _knownRobots[othRobName]->getRad() + _interRobotSafetyDist, 2);

		double discriminant = b*b - 4*a*c;

		// std::cout << FG_B_L_GREEN << "_estimateCollisionCenterAndRadius [4]" << RESET << std::endl;


		if (discriminant > 0)
		{
			double r1 = (-b + sqrt(discriminant))/(2*a);
			double r2 = (-b - sqrt(discriminant))/(2*a);

			if (r1 < 0.0 && r2 < 0.0)
			{
				Common::CArray3d ret;
				ret[2] = -1.0;
				ret[0] = 0.0;
				ret[1] = 0.0;
				return ret;
			}

			if (r1 < r2 && r1 < 0.0)
			{
				r1 = 0.0;
			}

			if (r2 < r1 && r2 < 0.0)
			{
				r2 = 0.0;
			}
			
			// std::cout << FG_B_L_GREEN << "_estimateCollisionCenterAndRadius [5]" << RESET << std::endl;

			FlatVector forbiddenSpaceCentroid = (
				(FlatVector() << _currentPose.x(), _currentPose.y()).finished() +
				myLinVel*(FlatVector() << _currentDirec.x(), _currentDirec.y()).finished()*r1 +
				(FlatVector() << _currentPose.x(), _currentPose.y()).finished() +
				myLinVel*(FlatVector() << _currentDirec.x(), _currentDirec.y()).finished()*r2)/2.0;
				//(FlatVector() << otherDirec.x(), otherDirec.y()).finished()*_knownRobots[othRobName]->getRad();

			Common::CArray3d ret;
			ret[2] = _knownRobots[othRobName]->getRad()/4.;
			// ret[2] = 0.01;
			ret[0] = forbiddenSpaceCentroid(0,0);
			ret[1] = forbiddenSpaceCentroid(1,0);

			return ret;
		}
		Common::CArray3d ret;
		ret[2] = -1.0;
		ret[0] = 0.0;
		ret[1] = 0.0;
		// std::cout << FG_B_L_GREEN << "_estimateCollisionCenterAndRadius [6]" << RESET << std::endl;
		return ret;
	}


	bool PathPlannerRecHor::_isForbSpaceInRobotsWay(const std::string& fsName, MapObst forbiddenSpaces)
	{
		// std::cout << "_isForbSpaceInRobotsWay:: " << fsName << std::endl;	
		double rad = forbiddenSpaces[fsName]->getRad() + _radius + _robotObstacleSafetyDist;

		FlatVector robotToForbSpac = forbiddenSpaces[fsName]->getCurrentPosition().getTranslation().block<2,1>(0,0) - _latestFlat;

		FlatVector forbSpacToRobot = -1.*robotToForbSpac;
		double robotToForbSpacDist = robotToForbSpac.norm();

		FlatVector robotToWayPt = _wayPt - _latestFlat;
		double robotToWayPtDist = robotToWayPt.norm();
		
		FlatVector wayPtDirec = robotToWayPt/robotToWayPtDist;
		double wayPtTheta = atan2(wayPtDirec(1,0), wayPtDirec(0,0));

		FlatVector forbSpacToWayPt = _wayPt - forbiddenSpaces[fsName]->getCurrentPosition().getTranslation().block<2,1>(0,0);

		FlatVector forbSpacToTarget = _targetedFlat - forbiddenSpaces[fsName]->getCurrentPosition().getTranslation().block<2,1>(0,0);

		// double rad = forbiddenSpaces[fsName]->getRad() + _radius;

		double segmentNorm = (forbSpacToWayPt - forbSpacToRobot).norm();

		double determinantP1P2 = forbSpacToRobot(0,0)*forbSpacToWayPt(1,0) - forbSpacToWayPt(0,0)*forbSpacToRobot(1,0);

		double discriminantWay = rad*rad * segmentNorm*segmentNorm - determinantP1P2*determinantP1P2;

		if (discriminantWay < 0)
		{
			return false;
		}
		else
		{
			//robotToForbSpacDist
			double signed_r2o_proj_on_wdir = robotToForbSpacDist * cos(atan2(robotToForbSpac(1,0), robotToForbSpac(0,0))-wayPtTheta);

			if (signed_r2o_proj_on_wdir <= _maxVelocity.block<FlatoutputMonocycle::veloDim, 1>(FlatoutputMonocycle::linSpeedIdx, 0)(0,0)*_compHorizon)
			{
				return false;
			}
		}

		if (!_isAnyForbSpacInRobotsWayToTarget(forbiddenSpaces))
		{
			return false;
		}

		return true;
	}

	Common::CArray3d PathPlannerRecHor::_getAngularVariationAndDistForAvoidance(const std::string& fsName, MapObst forbiddenSpaces)
	{
		double theta1, theta2;

		// FlatVector forbSpacToRobot = _latestFlat - forbiddenSpaces[fsName]->getCurrentPosition().getTranslation().block<2,1>(0,0);

		FlatVector robotToForbSpac = forbiddenSpaces[fsName]->getCurrentPosition().getTranslation().block<2,1>(0,0) - _latestFlat;

		FlatVector robotToWayPt = _wayPt - _latestFlat;
		double robotToWayPtDist = robotToWayPt.norm();
		FlatVector wayPtDirec = robotToWayPt/robotToWayPtDist;
		double wayPtTheta = atan2(wayPtDirec(1,0), wayPtDirec(0,0));

		double rad = forbiddenSpaces[fsName]->getRad() + _radius + _robotObstacleSafetyDist;

		// sovling second degree eq for finding angular variations for passing by the "left" and "right" of this forbidden space

		double a = robotToForbSpac(0,0)*robotToForbSpac(0,0) - rad*rad;
		double b = -2.*robotToForbSpac(0,0)*robotToForbSpac(1,0);
		double c = robotToForbSpac(1,0)*robotToForbSpac(1,0) - rad*rad;

		double discriminant = b*b - 4*a*c;

		if (discriminant < 0.0) // inside an forbidden zone
		{
			std::cout << "WARNING: inside forbidden space [ " << fsName << " ]\n";
			Common::CArray3d ret;
			ret[0] = 0.0;
			ret[1] = 0.0;
			ret[2] = robotToForbSpac.norm() - rad;
			return ret;
		}
		else if (discriminant == 0.0)
		{
			theta1 = atan(-b/2/a); // [-pi, pi)
			theta2 = theta1 - M_PI; // [-2pi, 0.0)
			theta2 = Common::wrapToPi(theta2);
		}
		else
		{
			theta1 = atan((-b + sqrt(discriminant))/2/a);
			theta2 = atan((-b - sqrt(discriminant))/2/a);
		}

		double refTheta = atan2(robotToForbSpac(1,0), robotToForbSpac(0,0));

		double quad14RefTheta = (refTheta < M_PI_2 && refTheta >= -M_PI_2/2.) ? refTheta : Common::wrapToPi(refTheta - M_PI);

		if ((quad14RefTheta < theta1 && quad14RefTheta < theta2) || (quad14RefTheta > theta1 && quad14RefTheta > theta2))
		{
			// angle to forbidden space is not between thetas
			if (abs(quad14RefTheta - theta1) > abs(quad14RefTheta - theta2))
			{
				theta1 = Common::wrapToPi(theta1 - M_PI);
			}
			else
			{
				theta2 = Common::wrapToPi(theta2 - M_PI);
			}
		}

		double dTheta1 = Common::wrapToPi(quad14RefTheta - theta1);
		double dTheta2 = Common::wrapToPi(quad14RefTheta - theta2);

		dTheta1 = Common::wrapToPi(wayPtTheta + dTheta1 - refTheta);
		dTheta2 = Common::wrapToPi(wayPtTheta + dTheta2 - refTheta);

		Common::CArray3d array;
		array[0] = dTheta1;
		array[1] = dTheta2;
		array[2] = robotToForbSpac.norm() - rad;
		return array;
	}

	void PathPlannerRecHor::_findNextWayPt()
	{
		// std::cout << FG_B_L_RED << "void PathPlannerRecHor::_findNextWayPt()" << RESET << std::endl;

		// Initiate the list of forbiddenSpaces (regions to be avoided). The waypoint must guide the robot towards a possible region among all the forbidden ones) even if knownObstaclesList is empty, of course
		// typedef std::map< std::string, Obstacle* > MapObst;
		typedef std::vector< Obstacle* > VecObst;
		typedef std::map< std::string, Common::CArray3d > MapArray3d;
		typedef std::vector< std::string > VecStr;
		typedef std::vector< VecStr > VecVecStr;
		typedef std::list< std::string > ListStr;

		// update known obstacles
		//self._known_obst_idxs += [i[0] for i in idx_list if i[0] not in self._known_obst_idxs]
		for(MapObst::iterator it=_detectedObstacles.begin(); it != _detectedObstacles.end(); ++it)
		{
			if (std::find(_knownObstacles.begin(), _knownObstacles.end(), *it) == _knownObstacles.end())
			{
				_knownObstacles[it->first] = it->second;
			}
		}

		MapObst forbiddenSpaces(_knownObstacles);
		// VecStr aux;
		// Common::KeysOnMapToVec(_knownRobots, aux);
		// Common::printNestedContainer(aux);

		// std::cout << FG_B_L_BLUE << "_knownRobots.empty() = " << RESET << _knownRobots.empty() << std::endl;

		// if (!_knownRobots.empty())
		// {
		// 	// std::cout << FG_B_L_BLUE << "[2] _knownRobots.empty() = " << RESET << _knownRobots.empty() << std::endl;
			
		// 	for (MapAIV::iterator otherRobot = _knownRobots.begin(); otherRobot != _knownRobots.end(); ++otherRobot)
		// 	{
		// 		// Priority to the right!!!

		// 		// line equation representing robot's current intended path (general form)
		// 		double a = _currentDirec.y();
		// 		double b = -1.*_currentDirec.x();
		// 		double c = _currentDirec.x()*_currentPose.y() - _currentDirec.y()*_currentPose.x();

		// 		Eigen::Displacementd otherPose = otherRobot->second->getCurrentPosition();

		// 		bool isAtRightSide = a*otherPose.x() + b*otherPose.y() + c > 0 ? true : false;

		// 		if (!isAtRightSide)
		// 		{
		// 			continue;
		// 		}
		// 		//

		// 		// std::cout << FG_B_L_BLUE << "[2.1] _knownRobots.empty() = " << RESET << _knownRobots.empty() << std::endl;
		// 		Common::CArray3d collInfo = _estimateCollisionCenterAndRadius(otherRobot->first);
		// 		// std::cout << FG_B_L_BLUE << "[2.2] _knownRobots.empty() = " << RESET << _knownRobots.empty() << std::endl;
		// 		if (collInfo[2] > 0) // collInfo[2] is the collision radius
		// 		{
		// 			// Getting robot's orientation projected in the ground plane (unless the aiv is flying this is the correct orientation)
		// 			// 1. get the quaternion part of the displacement
		// 			Quaternion<double> q = Quaternion<double>(_knownRobots[otherRobot->first]->getCurrentPosition().qw(), _knownRobots[otherRobot->first]->getCurrentPosition().qx(), _knownRobots[otherRobot->first]->getCurrentPosition().qy(), _knownRobots[otherRobot->first]->getCurrentPosition().qz());
		// 			// 2. rotation of UnityX by the quaternion q (i.e. x of the robot frame wrt the absolute frame, i.e. robot's direction)
		// 			QuaternionBase<Quaternion<double> >::Vector3 otherDirec = q._transformVector(Vector3d::UnitX());

		// 			// ImaginaryForbiddenSpace bla(otherRobot->first, collInfo[2], (FlatVector() << collInfo[0], collInfo[1]).finished());
		// 			// std::cout << "dummy cosntructed\n";
		// 			forbiddenSpaces[otherRobot->first] = new ImaginaryForbiddenSpace(otherRobot->first, collInfo[2], (FlatVector() << collInfo[0], collInfo[1]).finished());
		// 			forbiddenSpaces[otherRobot->first+"_2"] = new ImaginaryForbiddenSpace(otherRobot->first+"_2", collInfo[2], (FlatVector() << collInfo[0], collInfo[1]).finished()+
		// 				(FlatVector() << otherDirec.x(), otherDirec.y()).finished()*2*_knownRobots[otherRobot->first]->getRad());
		// 			forbiddenSpaces[otherRobot->first+"_3"] = new ImaginaryForbiddenSpace(otherRobot->first+"_3", collInfo[2], (FlatVector() << collInfo[0], collInfo[1]).finished()+
		// 				(FlatVector() << otherDirec.x(), otherDirec.y()).finished()*4.*_knownRobots[otherRobot->first]->getRad());
		// 			forbiddenSpaces[otherRobot->first+"_4"] = new ImaginaryForbiddenSpace(otherRobot->first+"_4", collInfo[2], (FlatVector() << collInfo[0], collInfo[1]).finished()+
		// 				(FlatVector() << otherDirec.x(), otherDirec.y()).finished()*6.*_knownRobots[otherRobot->first]->getRad());
		// 			forbiddenSpaces[otherRobot->first+"_5"] = new ImaginaryForbiddenSpace(otherRobot->first+"_5", collInfo[2], (FlatVector() << collInfo[0], collInfo[1]).finished()+
		// 				(FlatVector() << otherDirec.x(), otherDirec.y()).finished()*8.*_knownRobots[otherRobot->first]->getRad());
		// 			// std::cout << "*************AFTER fon[] = new\n";
		// 		}
		// 	}
		// 	// std::cout << FG_B_L_BLUE << "[3] _knownRobots.empty() = " << RESET << _knownRobots.empty() << std::endl;

		// }

		// Check if there is somthing to be avoided
		if (forbiddenSpaces.empty())
		{
			_wayPt = _targetedFlat;
			return;
		}
		// std::cout << "*************AFTER [2]\n";


		// In order to avoid trying to pass among forbiddenSpaces that are to close together we have to identify cluster of forbidden spaces

		// Initiate list of clusters of forbiddenSpaces
		VecVecStr forbiddenSpacesClusters;

		// Build the list of clusters
		std::cout << "BUILD CLUSTER\n";

		// Iterate over forbidden spaces vector
		for (MapObst::iterator fsIt = forbiddenSpaces.begin(); fsIt != forbiddenSpaces.end(); ++fsIt)
		{
			std::cout << fsIt->first << std::endl;
			Common::printNestedContainer(forbiddenSpacesClusters);
			std::cout << std::endl;
			// is fsIt in any cluster in forbiddenSpacesClusters?
			//
			bool foundIt = false;
			for (VecVecStr::iterator clIt = forbiddenSpacesClusters.begin(); clIt != forbiddenSpacesClusters.end(); ++clIt)
			{
				std::cout << "searching for " << fsIt->first << " in ";
				Common::printNestedContainer(*clIt);
				std::cout << std::endl;
				// if (clIt->find(fsIt->first) != clIt->end())
				if (std::find(clIt->begin(), clIt->end(), fsIt->first) != clIt->end())
				{
					std::cout << "found!\n";
					foundIt = true;
					break;
				}
			}
			//
			
			if (foundIt) continue;

			std::cout << "Add new list " << fsIt->first << std::endl;

			// fsIt is not in any cluster in forbiddenSpacesClusters
			// let's do a tree search in forbiddenSpaces looking for other forbiddenSpaces forming a cluster with this forbiddenSpace. If none, this forbiddenSpace is a cluster by it self

			// forbiddenSpacesClusters.push_back(VecStr(fsIt->first));
			// if the above doesn't work:
			VecStr auxList;
			auxList.push_back(fsIt->first);
			forbiddenSpacesClusters.push_back(auxList);

			// search using FIFO queue

			ListStr queue;
			queue.push_back(fsIt->first);
			
			while (queue.empty() == false)
			{
				std::string forbSpacName = queue.front();
				queue.pop_front();

				for (MapObst::iterator candidIt = forbiddenSpaces.begin(); candidIt != forbiddenSpaces.end(); ++candidIt)
				{
					// std::cout << "______ " << this->name << "::Pair: " << candidIt->first << ", " << forbSpacName << std::endl;
					if (candidIt->first == forbSpacName)
					{
						continue;
					}
					// is candidIt in any cluster in forbiddenSpacesClusters?
					//
					bool foundIt = false;
					for (VecVecStr::iterator clIt = forbiddenSpacesClusters.begin(); clIt != forbiddenSpacesClusters.end(); ++clIt)
					{
						// if (clIt->find(fsIt->first) != clIt->end())
						if (std::find(clIt->begin(), clIt->end(), candidIt->first) != clIt->end())
						{
							foundIt = true;
							break;
						}
					}
					//
					if (foundIt)
					{
						continue;
					}

					// std::cout << candidIt->first << " *************AFTER [3]\n";
					// Common::printNestedContainer(forbiddenSpacesClusters);
					// VecStr aux;
					// Common::KeysOnMapToVec(forbiddenSpaces, aux);
					// std::cout << std::endl;
					// Common::printNestedContainer(aux);
					// std::cout << std::endl;
					// std::cout << candidIt->first << std::endl;
					 // << candidIt->second->getRad() << std::endl;
					// std::cout << forbSpacName << std::endl;
					// std::cout << forbiddenSpaces[forbSpacName]->getRad() << std::endl;
					// std::cout << forbiddenSpaces[forbSpacName]->getCurrentPosition().getTranslation().block<2,1>(0,0)<< std::endl;

					// " fname: " << forbSpacName << " " << forbiddenSpaces[forbSpacName]->getRad() << ", " << forbiddenSpaces[forbSpacName]->getCurrentPosition().getTranslation().block<2,1>(0,0) << " *************AFTER [4]\n";
					
					double dist =  candidIt->second->distToAIV(forbiddenSpaces[forbSpacName]->getCurrentPosition().getTranslation().block<2,1>(0,0), forbiddenSpaces[forbSpacName]->getRad());
					// std::cout << 
					// std::cout << candidIt->first << " " << candidIt->second->getRad() << " " << dist << std::endl;
					bool tooClose = dist < 2.*_robotObstacleSafetyDist + _radius*2;
					// std::cout << candidIt->first << " *************AFTER [5]\n";
					std::cout << "Is " << candidIt->first << " too close to " << forbSpacName << "? [dist=" << dist << "]" << std::endl;
					if (tooClose)
					{
						std::cout << "YES\n";
						// std::cout << "\n";
						forbiddenSpacesClusters.back().push_back(candidIt->first);
						// std::cout << "*************AFTER [5.2]\n";
						queue.push_back(candidIt->first);
					}
					else
						std::cout << "NO\n";
					// {
					// 	VecStr auxList;
					// 	auxList.push_back(candidIt->first);
					// 	forbiddenSpacesClusters.push_back(auxList);
					// 	// queue.push_back(candidIt->first);
					// }
				}
				// std::cout << "=== bye for\n";
			}
		}

		std::cout << this->name << "::forbiddenSpacesClusters ------- \033[1;36m";
		Common::printNestedContainer(forbiddenSpacesClusters);
		std::cout << "\033[0m" << std::endl;
		// std::cout << "*************AFTER [6]\n";

		// Some clusters can be ignored, let's remove them. For the remaining clusters, let's find the information about avoiding each of their forbidden spaces

		MapArray3d _avoidanceInfo;

		// std::cout << "*************AFTER [6.1]\n";
		VecVecStr::iterator clIt = forbiddenSpacesClusters.begin();
		// std::cout << "*************AFTER [6.2]\n";
		while (clIt !=  forbiddenSpacesClusters.end())
		{
			// is any fsIt in clIt cluster is the robot way?
			//
			// std::cout << "*************AFTER [6.3]\n";
			bool inTheWay = false;
			// std::cout << "*************AFTER [6.4]\n";
			for (VecStr::iterator fsIt = clIt->begin(); fsIt != clIt->end(); ++fsIt)
			{
				// std::cout << "*************AFTER [7]\n";
				std::cout << "Is " << *fsIt << "[rad=" << forbiddenSpaces[*fsIt]->getRad() << "], [pos=" << forbiddenSpaces[*fsIt]->getCurrentPosition().getTranslation().block<2,1>(0,0) << "] in robot's way?\n";
				bool test = _isForbSpaceInRobotsWay(*fsIt, forbiddenSpaces);
				if (test) std::cout << "YES\n";
				else std::cout << "NO\n";
				// std::cout << "*************AFTER [7.1]\n";
				if (test)
				{
					// std::cout << "*************AFTER [8]\n";
					inTheWay = true;
					break;
				}
			}
			//
			// std::cout << "*************AFTER [9]\n";

			if (inTheWay)
			{
				// std::cout << "*************AFTER [10]\n";
				for (VecStr::iterator fsIt = clIt->begin(); fsIt != clIt->end(); ++fsIt)
				{
					// get two angular variations (positive if clockwise, negative otherwise) and distance for avoiding this forbiddenSpace
					_avoidanceInfo[*fsIt] = _getAngularVariationAndDistForAvoidance(*fsIt, forbiddenSpaces);
					std::cout << *fsIt << ": " << _avoidanceInfo[*fsIt][0] << ", " << _avoidanceInfo[*fsIt][1] << ", " << _avoidanceInfo[*fsIt][2] << std::endl;
					// [theta left, theta right, distance]
				}
				++clIt;
			}
			else // this cluster can be ignored
			{
				forbiddenSpacesClusters.erase(clIt);
			}
		}

		// Did all clusters were removed? In this case return goal
		if (forbiddenSpacesClusters.empty())
		{
			_wayPt = _targetedFlat;
			return;
		}

		CompObj comp(_avoidanceInfo);


		for (VecVecStr::iterator clIt = forbiddenSpacesClusters.begin(); clIt != forbiddenSpacesClusters.end(); ++clIt)
		{
			if (clIt->empty())
			{
				continue;
			}
			// Am I inside a forbiddenSpace? In this case return previous waypoint
			for (VecStr::iterator fsIt = clIt->begin(); fsIt != clIt->end(); ++fsIt)
			{
				if (_avoidanceInfo[*fsIt][2] < 0.0) //inside the obstacle
					//return previousWayPoint
					return;
			}

			// Sort forbidden spaces according to angular variation
			std::sort (clIt->begin(), clIt->end(), comp);//, *this);

		}
			// Save minimum absolute angular variontion
			// cluster.minAbsAngVar = minAbsAngVar(cluster)

		// Sort clusters according to abs ang variation
		std::sort (forbiddenSpacesClusters.begin(), forbiddenSpacesClusters.end(), comp);//, *this);


		// The first cluster in the list is a first guess for the cluster which will give the right direction to the new waypoint (based on the angular variation that gave the minimum absolute angular variation). But it has to respect a condition: do not suggest a direction that goes towards another clusters that is closer to the robot than the first cluster. In case it fails, check the following clusters
		clIt = forbiddenSpacesClusters.begin();
		//VecVecStr::iterator clIt;
		// for (clIt = forbiddenSpacesClusters.begin(); clIt != forbiddenSpacesClusters.end(); ++clIt)
		// {
		// 	VecStr possibleBlockingClusters;

		// 	VecVecStr subForbiddenSpacesClusters(forbiddenSpacesClusters.begin(), clIt-1);
		// 	subForbiddenSpacesClusters.insert(subForbiddenSpacesClusters.end(), VecVecStr(clIt+1, forbiddenSpacesClusters.end()));

		// 	for (VecVecStr::iterator othClIt = subForbiddenSpacesClusters.begin(); othClIt != subforbiddenSpacesClusters.end(); ++othClIt)
		// 	{
		// 		double cf0 = _avoidanceInfo[clIt->front()][0];
		// 		double cf1 = _avoidanceInfo[clIt->front()][1];
		// 		double cb0 = _avoidanceInfo[clIt->back()][0];
		// 		double cb1 = _avoidanceInfo[clIt->back()][1];
		// 		double of0 = _avoidanceInfo[othClIt->front()][0];
		// 		double of1 = _avoidanceInfo[othClIt->front()][1];
		// 		double ob0 = _avoidanceInfo[othClIt->back()][0];
		// 		double ob1 = _avoidanceInfo[othClIt->back()][1];

		// 		// cluster's angular variation that gave the min abs ang var is between otherCluster's min and max angular variation
		// 		double dTheta = abs(min(cf0, cf1)) < abs(max(cb0, cb1)) ? min(cf0, cf1) else max(cb0, cb1);
		// 		if (dTheta < max(ob0, ob1) && dTheta < min(of0, of1))
		// 		{
		// 			possibleBlockingClusters.push_back(*othClIt);
		// 		}
		// 	}

		// 	if (!possibleBlockingClusters.empty())
		// 	{
		// 		//
		// 		double myMinAbsAvg = abs((_avoidanceInfo[clIt->front()][0]+_avoidanceInfo[clIt->front()][1])/2.);
		// 		std::string myFSName = clIt->front();

		// 		for (VecStr::iterator fsIt = clIt->begin()+1; fsIt != clIt->end(); ++fsIt)
		// 		{
		// 			newMinAbsAvg = abs((_avoidanceInfo[clIt->front()][0]+_avoidanceInfo[clIt->front()][1])/2.);
		// 			if (myMinAbsAvg > newMinAbsAvg)
		// 			{
		// 				myMinAbsAvg = newMinAbsAvg;
		// 				myFSName = *fsIt;
		// 			}
		// 		}

		// 		bool foundIt = false;
		// 		for (VecVecStr::iterator othClIt = possibleBlockingClusters.begin(); othClIt != possibleBlockingClusters.end(); ++othClIt)
		// 		{
		// 			//
		// 			double othMinAbsAvg = abs((_avoidanceInfo[othClIt->front()][0]+_avoidanceInfo[othClIt->front()][1])/2.);
		// 			std::string othFSName = othClIt->front();

		// 			for (VecStr::iterator fsIt = othClIt->begin()+1; fsIt != othClIt->end(); ++fsIt)
		// 			{
		// 				newMinAbsAvg = abs((_avoidanceInfo[othClIt->front()][0]+_avoidanceInfo[othClIt->front()][1])/2.);
		// 				if (othMinAbsAvg > newMinAbsAvg)
		// 				{
		// 					othMinAbsAvg = newMinAbsAvg;
		// 					othFSName = *fsIt;
		// 				}
		// 			}
		// 			if (_avoidanceInfo[othFSName][2] < _avoidanceInfo[myFSName][2])
		// 			{
		// 				foundIt = true;
		// 				break;
		// 			}
		// 		}
		// 		//
		// 		if (foundIt) continue;
		// 	}	
		// 	break;
		// }

		FlatVector robotToWayPt = _wayPt - _latestFlat;
		double robotToWayPtDist = robotToWayPt.norm();
		FlatVector wayPtDirec = robotToWayPt/robotToWayPtDist;
		double wayPtTheta = atan2(wayPtDirec(1,0), wayPtDirec(0,0));

		double f0 = _avoidanceInfo[clIt->front()][0];
		double f1 = _avoidanceInfo[clIt->front()][1];
		double b0 = _avoidanceInfo[clIt->back()][0];
		double b1 = _avoidanceInfo[clIt->back()][1];

		std::cout << "wayTheta: " << wayPtTheta << std::endl;
		std::cout << "all ang var: " << f0 << ", " << f1 << ", " << b0 << ", " << b1 << std::endl;

		double dTheta = abs(std::min(f0, f1)) < abs(std::max(b0, b1)) ? std::min(f0, f1) : std::max(b0, b1);
		std::cout << "delta Theta: " << dTheta << std::endl;

		double eps = dTheta < 0.0 ? -0.03 : +0.03;

		double theta = Common::wrapToPi(wayPtTheta - dTheta + eps);

		std::cout << "final theta: " << theta  << std::endl;

		FlatVector newWayPtDirec;
		newWayPtDirec << cos(theta),
						 sin(theta);

		_wayPt = newWayPtDirec*robotToWayPtDist + _latestFlat;

		return;
	}

	void PathPlannerRecHor::_plan()
	{
		// std::cout << FG_B_L_RED << "void PathPlannerRecHor::_plan()" << RESET << std::endl;
		// double tic = Common::getRealTime<double>();
		// GET ESTIMATE POSITION OF LAST POINT

		FlatVector remainingDistVectorUni = _targetedFlat - _latestFlat;
		double remainingDist = remainingDistVectorUni.norm();
		remainingDistVectorUni /= remainingDist;
		FlatVector lastPt = _maxStraightDist*remainingDistVectorUni;

		// std::cout << FG_B_L_RED << "void PathPlannerRecHor::_plan()" << RESET << std::endl;


		// RECIDING HORIZON STOP CONDITION 


		double breakDist = (pow(_targetedVelocity(FlatoutputMonocycle::linSpeedIdx,0), 2) - pow(_latestVelocity(FlatoutputMonocycle::linSpeedIdx,0), 2))/(-2*_maxAcceleration(FlatoutputMonocycle::linAccelIdx,0));
		double breakDuration = 2*breakDist/(_targetedVelocity(FlatoutputMonocycle::linSpeedIdx,0) + _latestVelocity(FlatoutputMonocycle::linSpeedIdx,0));
		
		if (remainingDist < _compHorizon*_maxVelocity[FlatoutputMonocycle::linSpeedIdx] + breakDist + _lastStepMinDist)
		// if (remainingDist < _lastStepMinDist + _compHorizon*_maxVelocity[FlatoutputMonocycle::linSpeedIdx])
		{
			std::cout << BG_L_RED << "CONDITION FOR TERMINATION PLAN REACHED!" << RESET << std::endl;
			_planLastPlan = true;
			
			//estimate last planning horizon

			// std::cout << "breakDist\n" << breakDist << std::endl;


			// std::cout << "breakDuration\n" << breakDuration << std::endl;

			_optPlanHorizon = breakDuration + std::max(2*(remainingDist - breakDist)/(_latestVelocity(FlatoutputMonocycle::linSpeedIdx,0) + _latestVelocity(FlatoutputMonocycle::linSpeedIdx,0)), 0.0);

			// std::cout << "_optPlanHorizon before opt " << _optPlanHorizon << std::endl;
			// TODO recompute n_ctrlpts n_knots
			// call setoptions for optTrajectory and update maybe, think about it although there will be updates in the optimization process
			// using _optTrajectory
		}
		else
		{
			_optPlanHorizon = _refPlanHorizon;
		}


		// GET WAYPOINT AND NEW DIRECTION (NOT GENERALIZED FOR)
		// TODO: GENERALIZE FOR ANY DIMENSION, ONLY 2D SUPPORTED


		// std::cout <<  "current orientation: " << _latestPose(FlatoutputMonocycle::oriIdx, 0) << std::endl;
		FlatVector currDirec;
		currDirec << cos(_latestPose(FlatoutputMonocycle::oriIdx, 0)),
					 sin(_latestPose(FlatoutputMonocycle::oriIdx, 0));
		// std::cout <<  "current direction: " << currDirec << std::endl;

		FlatVector newDirec;
		//FlatVector wayPoint;

		// std::cout << FG_B_L_RED << "void PathPlannerRecHor::_plan()" << RESET << std::endl;

		_findNextWayPt();
		//
		//_wayPt = _targetedFlat;
		newDirec = _wayPt - _latestFlat;
		newDirec /= newDirec.norm();
		//std::cout <<  "current orientation: " << _latestPose(FlatoutputMonocycle::oriIdx, 0) << std::endl;
		// std::cout <<  "waypt orientation: " << atan2(_wayPt(1,0), _wayPt(0,0)) << std::endl;
		// std::cout <<  "waypt direction: " << newDirec << std::endl;
		//

		_rotMat2WRef << currDirec(0,0), -1.*currDirec(1,0), currDirec(1,0), currDirec(0,0);
		_rotMat2RRef << currDirec(0,0), currDirec(1,0), -1.*currDirec(1,0), currDirec(0,0);
		// std::cout << _rotMat2WRef << std::endl;
		// std::cout << _rotMat2RRef << std::endl;
		// self._latest_rot2ref_mat = np.hstack((init_direc, np.multiply(np.flipud(init_direc), np.vstack((-1,1)))))
		// self._latest_rot2rob_mat = np.hstack((np.multiply(init_direc, np.vstack((1,-1))), np.flipud(init_direc)))
		
		FlatVector rotatedNewDirec = _rotMat2RRef*newDirec;
		//std::cout <<  "rotated waypt direction: " << rotatedNewDirec << std::endl;


		// COMBINING CURVES FOR INTERPOLATION


		double accel = _maxAcceleration(FlatoutputMonocycle::linAccelIdx,0);
		double maxDisplVariation;
		double prevDispl;
		Matrix<double, FlatoutputMonocycle::flatDim, Dynamic> curve(FlatoutputMonocycle::flatDim, _optTrajectory.nParam());

		// NOT LAST STEP
		// if (_planLastPlan == false)
		// {
		maxDisplVariation = (_optPlanHorizon/(_optTrajectory.nParam() - 1))*_maxVelocity(FlatoutputMonocycle::linSpeedIdx,0);

		prevDispl = 0.0;

		// Create a sampled trajectory for a "bounded uniformed accelerated motion" in x axis
		Matrix<double, FlatoutputMonocycle::flatDim, Dynamic> curveCurrDirec(FlatoutputMonocycle::flatDim, _optTrajectory.nParam());
		curveCurrDirec = Matrix<double, FlatoutputMonocycle::flatDim, Dynamic>::Zero(FlatoutputMonocycle::flatDim, _optTrajectory.nParam());
	
		// Create a sampled trajectory for a "bounded uniformed accelerated motion" in (direc-init_direc) direction in the xy plane
		Matrix<double, FlatoutputMonocycle::flatDim, Dynamic> curveNewDirec(FlatoutputMonocycle::flatDim, _optTrajectory.nParam());
		// curveNewDirec = Matrix<double, FlatoutputMonocycle::flatDim, Dynamic>::Zero(FlatoutputMonocycle::flatDim, _optTrajectory.nParam());

		// double previousVelocity = _latestVelocity(FlatoutputMonocycle::linSpeedIdx,0);
		// double deltaT = _optPlanHorizon/(_optTrajectory.nParam()-1);
		// double displVar;

		// for (auto i=1; i<_optTrajectory.nParam(); ++i)
		// {
		// 	displVar = previousVelocity*deltaT + accel/2.*deltaT*deltaT;
		// 	displVar = displVar < maxDisplVariation ? std::max(displVar, 0.0) : maxDisplVariation;
		// 	curveCurrDirec(0,i) = curveCurrDirec(0,i-1)+displVar;
		// 	curveNewDirec.col(i) = curveNewDirec.col(i-1)+displVar*rotatedNewDirec;

		// 	previousVelocity = displVar/deltaT;
		// }

		double beforeBreakDispl;
		double beforeBreakDeltaT;
		bool breakOn = false;

		for (auto i=1; i<_optTrajectory.nParam(); ++i)
		{
			double deltaT = i*_optPlanHorizon/(_optTrajectory.nParam()-1);

			double displ;

			if (_planLastPlan && prevDispl >= remainingDist - breakDist)
			{
				if (breakOn == false)
				{
					beforeBreakDispl = prevDispl;
					beforeBreakDeltaT = (i-1)*_optPlanHorizon/(_optTrajectory.nParam()-1);
					accel = -1.2*_maxAcceleration(FlatoutputMonocycle::linAccelIdx,0);
					breakOn = true;
				}
				displ = beforeBreakDispl + _maxVelocity(FlatoutputMonocycle::linSpeedIdx,0)*(deltaT-beforeBreakDeltaT) + accel/2.*(deltaT-beforeBreakDeltaT)*(deltaT-beforeBreakDeltaT);
			}
			else
			{
				displ = _latestVelocity(FlatoutputMonocycle::linSpeedIdx,0)*deltaT + accel/2.*deltaT*deltaT;
			}
			displ = displ - prevDispl < maxDisplVariation ? std::max(displ, prevDispl) : prevDispl + maxDisplVariation;
			prevDispl = displ;
			curveCurrDirec(0,i) = displ;
			curveNewDirec.col(i) = displ*rotatedNewDirec;
		}

		double magicNumber = 1.5; //FIXME no magic numbers, at least no hard coded
		double p;

		for (auto i=0; i < _optTrajectory.nParam(); ++i)
		{
			p = pow(double(i)/_optTrajectory.nParam(), magicNumber);
			curve.col(i) = curveCurrDirec.col(i)*(1.0-p) + curveNewDirec.col(i)*p;
		}

		// IF LAST STEP LET US COMBINE CURVE WITH ANOTHER ONE THAT SMOOTHS THE ARRIVAL
		//if (_planLastPlan)
		if (false)
		{
			FlatVector fromGoalDirec;
			fromGoalDirec<< cos(_targetedPose.tail<1>()(0,0) - M_PI),
							sin(_targetedPose.tail<1>()(0,0) - M_PI);
			std::cout << "fromGoalDirec bef rot\n" << fromGoalDirec << std::endl;

			fromGoalDirec = _rotMat2RRef * fromGoalDirec;

			// FlatVector oposed2GoalDirec;
			// oposed2GoalDirec = _latestFlat - _targetedFlat;
			// oposed2GoalDirec /= oposed2GoalDirec.norm();

			// oposed2GoalDirec = _rotMat2RRef*oposed2GoalDirec;
			// std::cout << "oposed2GoalDirec\n" << oposed2GoalDirec << std::endl;
		
			maxDisplVariation = (_optPlanHorizon/(_optTrajectory.nParam() - 1))*_maxVelocity(FlatoutputMonocycle::linSpeedIdx,0);

			accel = _maxAcceleration(FlatoutputMonocycle::linAccelIdx,0);
	
			prevDispl = 0.0;

			// Create a sampled trajectory for a "bounded uniformed accelerated motion" in x axis
			Matrix<double, FlatoutputMonocycle::flatDim, Dynamic> fromGoalCurve(FlatoutputMonocycle::flatDim, _optTrajectory.nParam());

			for (auto i=1; i<_optTrajectory.nParam(); ++i)
			{
				double deltaT = i*_optPlanHorizon/(_optTrajectory.nParam()-1);
				double displ = _targetedVelocity(FlatoutputMonocycle::linSpeedIdx,0)*deltaT + accel/2.*deltaT*deltaT;
				displ = displ - prevDispl < maxDisplVariation ? std::max(displ, prevDispl) : prevDispl + maxDisplVariation;
				prevDispl = displ;
				fromGoalCurve.col(i) = displ*fromGoalDirec;
				// toGoalCurve.col(i) = displ*oposed2GoalDirec;
			}
			// Matrix<double, FlatoutputMonocycle::flatDim, FlatoutputMonocycle::flatDim> rot180;
			// rot180 << -1, 0, 0, -1;
			Matrix<double, FlatoutputMonocycle::flatDim, Dynamic> finalFromGoalCurve;
			std::cout << "fromGoalCurve bef changes\n" << fromGoalCurve << std::endl;

			finalFromGoalCurve = fromGoalCurve.rowwise().reverse() + (_rotMat2RRef*(_targetedFlat - _latestFlat)).replicate(1, _optTrajectory.nParam());


			
			// fromGoalCurve = rot180*fromGoalCurve +
			// 	fromGoalCurve.rightCols<1>().replicate(1, _optTrajectory.nParam()) +
			// 	(_rotMat2RRef*(_targetedFlat - _latestFlat)).replicate(1, _optTrajectory.nParam());

			// toGoalCurve = rot180*toGoalCurve;
			// toGoalCurve = toGoalCurve - toGoalCurve.rightCols<1>().replicate(1, _optTrajectory.nParam());	
			std::cout << "fromGoalCurve\n" << finalFromGoalCurve << std::endl;

			double magicNumber = 1.5; //FIXME no magic numbers, at least no hard coded
			double p;

			for (auto i=0; i < _optTrajectory.nParam(); ++i)
			//for (auto i=0; i < FlatoutputMonocycle::flatDim; ++i)
			{
				p = pow(double(i)/_optTrajectory.nParam(), magicNumber);
				curve.col(i) = curve.col(i)*(1.0-p) + finalFromGoalCurve.col(i)*p;
			}
			// curve = finalFromGoalCurve;
		}
		
		// INTERPOLATION OF INITIAL GUESS


		// Feed aux trajectory with desired points and time horizon
		_optTrajectory.interpolate(curve, _optPlanHorizon);
		std::cout << FG_B_L_YELLOW << "BEFORE OPT, WRT ROBOT:"<< RESET << std::endl;
		std::cout << _optTrajectory.getCtrlPts() << std::endl;


		// CALL OPT SOLVER

		bool gotInterrupted;

		try
		{
			gotInterrupted = _solveOptPbl(); // after this call _optTrajectory has the optimized solution
		}
		catch (std::exception& e)
		{
			// std::cout << "\033[1;31m!!!!!!!!!!!!!!!!!!!!!!!!!\033[0m\n";
			std::stringstream ss;
			ss << "PathPlannerRecHor::_plan: call to solve optimization problem failed. " << e.what();
			std::cout << ss.str() << std::endl;

			// return;
			throw(Common::MyException(ss.str()));
		}

		
		//---------------------------------------------------------------------------
		_intendedSolMutex.lock();
		//---------------------------------------------------------------------------

		_intendedTraj = _optTrajectory;
		_intendedPlanHor = _optPlanHorizon;
		_intendedBasePos = _latestFlat;
		_intendedBaseTime = _baseTime;

		//---------------------------------------------------------------------------
		_intendedSolMutex.unlock();
		//---------------------------------------------------------------------------

		if (!gotInterrupted)
		{
			_conflictEval();

			if (!_collisionAIVs.empty() || !_comOutAIVs.empty())
			{
				_optTrajectory.update(_rotMat2RRef * _optTrajectory.getCtrlPts());

				try
				{
					_solveOptPbl(); // after this call _optTrajectory has the optimized solution
				}
				catch (std::exception& e)
				{
					std::stringstream ss;
					ss << "PathPlannerRecHor::_plan: call to solve optimization problem failed. " << e.what();
					std::cout << ss.str() << std::endl;
					throw(Common::MyException(ss.str()));
				}
			}

			//---------------------------------------------------------------------------
			_intendedSolMutex.lock();
			//---------------------------------------------------------------------------

			_intendedTraj = _optTrajectory;
			_intendedPlanHor = _optPlanHorizon;

			//---------------------------------------------------------------------------
			_intendedSolMutex.unlock();
			//---------------------------------------------------------------------------
		
		}

		std::cout << "AFTER OPT, WRT DISPLACED WORLD:\n";
		std::cout << _optTrajectory.getCtrlPts() << std::endl;
		std::cout << FG_B_L_GREEN << "AFTER OPT, WRT ROBOT:" << RESET << std::endl;
		std::cout << _rotMat2RRef * _optTrajectory.getCtrlPts() << std::endl;
		//std::cout << _optTrajectory.getCtrlPts() << std::endl;

		// CONFLICT COMPUTATION

		// UPDATES

		NDerivativesMatrix derivFlat = _optTrajectory(_compHorizon, FlatoutputMonocycle::flatDerivDeg);
		
		// _latestFlat += _rotMat2WRef * derivFlat.col(0);
		_latestFlat +=  derivFlat.col(0);

		PoseVector auxLatestPose = _latestPose;
		_latestPose = FlatoutputMonocycle::flatToPose(derivFlat);
		_latestPose.block<FlatoutputMonocycle::posDim, 1>(FlatoutputMonocycle::posIdx, 0) =
			// _rotMat2WRef * _latestPose.block<FlatoutputMonocycle::posDim, 1>(FlatoutputMonocycle::posIdx, 0) +
			_latestPose.block<FlatoutputMonocycle::posDim, 1>(FlatoutputMonocycle::posIdx, 0) +
			auxLatestPose.block<FlatoutputMonocycle::posDim, 1>(FlatoutputMonocycle::posIdx, 0);

		// _latestPose(FlatoutputMonocycle::oriIdx,0) =
			// Common::wrapToPi(_latestPose(FlatoutputMonocycle::oriIdx,0) + auxLatestPose(FlatoutputMonocycle::oriIdx,0));

		_latestVelocity = FlatoutputMonocycle::flatToVelocity(derivFlat);

		//boost::this_thread::sleep(boost::posix_time::milliseconds(500));
		//std::cout << "planning thread elapsed time: " <<  Common::getRealTime() - tic << std::endl; 
		_planOngoingMutex.unlock();
		// std::cout << "END OF PLAN\n";
	}

	bool PathPlannerRecHor::_solveOptPbl()
	{
		unsigned nParam;
		unsigned nEq;
		unsigned nIneq;
		double(*objF) (unsigned, const double *, double *, void *);
		void(*eqF) (unsigned, double *, unsigned, const double*, double*, void*);
		void(*ieqF) (unsigned, double *, unsigned, const double*, double*, void*);

		double *optParam;
		double *backupOptParam;
		double *tolEq;
		double *tolIneq;

		int status;

		bool gotInterrupted = false;

		if (_planLastPlan == false)
		{

			objF = PathPlannerRecHor::objectFunc;
			eqF = PathPlannerRecHor::eqFunc;
			ieqF = PathPlannerRecHor::ineqFunc;
			nParam = _optTrajectory.nParam()*FlatoutputMonocycle::flatDim;
			nEq = FlatoutputMonocycle::poseDim + FlatoutputMonocycle::veloDim;
			nIneq = FlatoutputMonocycle::veloDim * _nTimeSamples +
				FlatoutputMonocycle::accelDim * (_nTimeSamples + 1) +
				_detectedObstacles.size() * _nTimeSamples +
				_collisionAIVs.size() * _nTimeSamples +
				_comOutAIVs.size() * _nTimeSamples;

			//std::cout <<  "======  Number of detected obstacles: " << _detectedObstacles.size() << std::endl;

			optParam = new double[nParam];
			backupOptParam = new double[nParam];
			_optTrajectory.getParameters(optParam);
			_optTrajectory.getParameters(backupOptParam);
			
			tolEq = new double[nEq];
			for (unsigned i = 0; i < nEq; ++i)
			{
				tolEq[i] = _eqTol;
			}

			tolIneq = new double[nIneq];
			for (unsigned i = 0; i < nIneq; ++i)
			{
				tolIneq[i] = _ineqTol;
			}
		}
		else
		{
			objF = PathPlannerRecHor::objectFuncLS;
			eqF = PathPlannerRecHor::eqFuncLS;
			ieqF = PathPlannerRecHor::ineqFuncLS;
			nParam = _optTrajectory.nParam()*FlatoutputMonocycle::flatDim + 1;
			nEq = (FlatoutputMonocycle::poseDim + FlatoutputMonocycle::veloDim) * 2;
			nIneq = FlatoutputMonocycle::veloDim * (_nTimeSamples - 1) +
				FlatoutputMonocycle::accelDim * (_nTimeSamples + 1) +
				_detectedObstacles.size() * (_nTimeSamples - 1) +
				_collisionAIVs.size() * (_nTimeSamples-1) +
				_comOutAIVs.size() * (_nTimeSamples-1) +
				1; // time > 0

			optParam = new double[nParam];
			backupOptParam = new double[nParam];
			optParam[0] = _optPlanHorizon;
			backupOptParam[0] = _optPlanHorizon;
			_optTrajectory.getParameters(&(optParam[1]));
			_optTrajectory.getParameters(&(backupOptParam[1]));

			tolEq = new double[nEq];
			for (unsigned i = 0; i < nEq; ++i)
			{
				tolEq[i] = _eqTol;
			}

			tolIneq = new double[nIneq];
			for (unsigned i = 0; i < nIneq; ++i)
			{
				tolIneq[i] = _ineqTol;
			}
		}
		
		if (_optimizerType == "NONE")
		{
			// GET optParam from XML
			std::string base_name = "AdeptLynx";

			std::stringstream ss;
			ss << "root." << base_name << name[base_name.size()] << ".plan" << std::to_string(_ULONGLONG(_executingPlanIdx+1));

			std::string optParamString;

			try
			{
				optParamString = _property_tree.get<std::string>(ss.str());
			}
			catch (std::exception& e)
			{
				std::stringstream strs;
				strs << "PathPlannerRecHor::_solveOptPbl: property tree execption. Verify output.xml file and/or python plan generator. " << e.what();
				delete[] optParam;
				delete[] backupOptParam;
				delete[] tolEq;
				delete[] tolIneq;
				throw(Common::MyException(strs.str()));
			}

			//std::stringstream ss(optParamString);
			ss << optParamString;	
			std::string token;
			int i = 0;
			while (getline(ss, token, ','))
			{
				optParam[i++] = std::strtod(token.c_str(), NULL);
			}
			status = 1;
		}
		else if (_optimizerType == "SLSQP" || _optimizerType == "COBYLA")
		{
			// nlopt_opt opt;

			if (_optimizerType == "COBYLA")
			{
				_opt = nlopt_create(NLOPT_LN_COBYLA, nParam);
			}
			else if (_optimizerType == "SLSQP")
			{
				_opt = nlopt_create(NLOPT_LD_SLSQP, nParam);
			}
			else
			{
				std::stringstream ss;
				ss << "PathPlannerRecHor::_solveOptPbl: unknown optimization method [" << _optimizerType << "]. Verify config.xml.";
				delete[] optParam;
				delete[] backupOptParam;
				delete[] tolEq;
				delete[] tolIneq;
				throw(Common::MyException(ss.str()));
			}

			nlopt_set_xtol_rel(_opt, _xTol);

			nlopt_set_min_objective(_opt, objF, this);
			nlopt_add_equality_mconstraint(_opt, nEq, eqF, this, tolEq);
			nlopt_add_inequality_mconstraint(_opt, nIneq, ieqF, this, tolIneq);

			double minf;
			
			_optIterCnter=0;

			std::cout << "Optimizing..." << std::endl;
			
			status = nlopt_optimize(_opt, optParam, &minf);
			
			// std::cout << _executingPlanIdx << " Done optimizing" << std::endl;

			//int status = -1;

			//int msg = status;
			std::stringstream msg;

			switch(status)
			{
				case 1:
					msg << FG_B_L_GREEN << "Generic success return value" << RESET;
					break;
				case 2:
					msg << FG_B_L_GREEN << "Optimization stopped because stopval was reached" << RESET;
					break;
				case 3:
					msg << FG_B_L_GREEN << "Optimization stopped because ftol_rel or ftol_abs was reached" << RESET;
					break;
				case 4:
					msg << FG_B_L_GREEN << "Optimization stopped because xtol_rel or xtol_abs was reached" << RESET;
					break;
				case 5:
					msg << FG_B_L_GREEN << "Optimization stopped because maxeval was reached" << RESET;
					break;
				case 6:
					msg << FG_B_L_GREEN << "Optimization stopped because maxtime was reached" << RESET;
					break;
				case -1:
					msg << FG_B_L_RED << "Generic failure" << RESET << " code";
					break;
				case -2:
					msg << "Invalid arguments (e.g. lower bounds are bigger than upper bounds, an unknown algorithm was specified, etcetera)";
					break;
				case -3:
					msg << "Ran " << FG_B_L_RED << "out of memory" << RESET;
					break;
				case -4:
					msg << "Halted because " << FG_B_L_RED << "roundoff errors" << RESET << " limited progress (in this case, the optimization still typically returns a useful result)";
					break;
				case -5:
					msg << "Halted because of a " << FG_B_L_RED << "forced termination" << RESET << ": the user called nlopt_force_stop(opt) on the optimization’s nlopt_opt object opt from the user’s objective function or constraints";
			}
			std::cout << "Optimization ended with status: \"" << msg.str() << "\" after " << _optIterCnter << " iterations" << std::endl;
			gotInterrupted = status == -5;
		}
		else if (_optimizerType == "TESTINIT")
		{
			//do nothing
			//std::cout << "TESTINIT" << std::endl;
			status = 1;
		}
		else
		{
			status = -1;
			std::stringstream ss;
			ss << "PathPlannerRecHor::_solveOptPbl: unknown optimization method [" << _optimizerType << "]. Verify config.xml.";
			delete[] optParam;
			delete[] backupOptParam;
			delete[] tolEq;
			delete[] tolIneq;
			throw(Common::MyException(ss.str()));
		}
		
		// boost::this_thread::interruption_point(); // Say bye bye if optimization took too long


		// if (_planLastPlan == false)
		// {
			// double *gradObjF = new double[nParam];
			// double cost = objF(nParam, optParam, gradObjF, this);
			
			// double *constrEq = new double[nEq];
			// double *gradEq = new double[nParam*nEq];
			// eqFunc(nEq, constrEq, nParam, optParam, gradEq, this);
			
			// double *constrIneq = new double[nIneq];
			// double *gradIneq = new double[nParam*nIneq];
			// ineqFunc(nIneq, constrIneq, nParam, optParam, gradIneq, this);

			// std::cout << "cost: " << cost << std::endl;
			// std::cout << "gradObjF: " << std::endl;
			// for (auto i=0; i<nParam; ++i)
			// {
			// 	std::cout << gradObjF[i] << ", ";
			// }
			// std::cout << std::endl;
			// std::cout << "eq: " << std::endl;
			// for (auto i=0; i<nEq; ++i)
			// {
			// 	std::cout << constrEq[i] << ", ";
			// }
			// std::cout << std::endl;
			// std::cout << "gradEq: " << std::endl;
			// for (auto i=0; i<nParam*nEq; ++i)
			// {
			// 	std::cout << gradEq[i] << ", ";
			// }
			// std::cout << std::endl;
			// std::cout << "ineq: " << std::endl;
			// for (auto i=0; i<nIneq; ++i)
			// {
			// 	std::cout << constrIneq[i] << ", ";
			// }
			// std::cout << std::endl;
			// std::cout << "gradIneq: " << std::endl;
			// for (auto i=0; i<nParam*nIneq; ++i)
			// {
			// 	std::cout << gradIneq[i] << ", ";
			// }
			// std::cout << std::endl;
			// delete[] constrEq;
			// delete[] constrIneq;
			// delete[] gradObjF;
		// }
		// else
		// {
			// double cost = objF(nParam-1, &(optParam[1]), NULL, this);
			// double *constrEq = new double[nEq];
			// eqFunc(nEq, constrEq, nParam-1,  &(optParam[1]), NULL, this);
			// double *constrIneq = new double[nIneq];
			// ineqFunc(nIneq, constrIneq, nParam-1,  &(optParam[1]), NULL, this);

			// std::cout << "cost: " << cost << std::endl;
			// std::cout << "eq: " << std::endl;
			// for (auto i=0; i<nEq; ++i)
			// {
			// 	std::cout << constrEq[i] << ", ";
			// }
			// std::cout << std::endl;
			// std::cout << "ineq: " << std::endl;
			// for (auto i=0; i<nIneq; ++i)
			// {
			// 	std::cout << constrIneq[i] << ", ";
			// }
			// std::cout << std::endl;
			// delete[] constrEq;
			// delete[] constrIneq;
		// }

		//boost::this_thread::sleep(boost::posix_time::milliseconds(200));

		// std::ofstream arquivo;
		// arquivo.open("optlog/xiter.csv", std::fstream::app);
		// arquivo << "___________________________________________________" << std::endl;
		// arquivo.close();
		// arquivo.open("optlog/geqiter.csv", std::fstream::app);
		// arquivo << "___________________________________________________" << std::endl;
		// arquivo.close();
		// arquivo.open("optlog/gieqiter.csv", std::fstream::app);
		// arquivo << "___________________________________________________" << std::endl;
		// arquivo.close();
		// arquivo.open("optlog/citer.csv", std::fstream::app);
		// arquivo << "___________________________________________________" << std::endl;
		// arquivo.close();


		// Update trajectory with optimized parameters
		// std::cout << "status: " << status << std::endl;;
		// std::cout << _optTrajectory.getCtrlPts() << std::endl;

		// Recovering from error status
		if (status == -4 || status == -5)
		{
			bool isSolutionUponErrorOk = true;

			// Check if results are usable
			for (unsigned i=0; i < nParam; ++i)
			{
				if(optParam[i] != optParam[i]) //Only checking for NAN (and IND)
				{
					isSolutionUponErrorOk = false;
					break;
				}
			}
			if (isSolutionUponErrorOk)
			{
				status = 1;
			}
		}

		if (status > 0)
		{

			if (_planLastPlan == false)
			{
				if (_optimizerType == "NONE")
					_optTrajectory.updateFromUniform(optParam);
					//_optTrajectory.updateFromUniform(_rotMat2WRef*_optTrajectory.cArray2CtrlPtsMat(optParam));
				else
				{
					_optTrajectory.update(_rotMat2WRef * _optTrajectory.cArray2CtrlPtsMat(optParam));
					// _optTrajectory.update((const double*)optParam);
				}

			}
			else
			{
				if (_optimizerType == "NONE")
					_optTrajectory.updateFromUniform(&(optParam[1]), optParam[0]);
					//_optTrajectory.updateFromUniform(_rotMat2WRef*_optTrajectory.cArray2CtrlPtsMat(&(optParam[1])), optParam[0]);
				else
				{
					_optTrajectory.update(_rotMat2WRef * _optTrajectory.cArray2CtrlPtsMat(&(optParam[1])), optParam[0]);
					// _optTrajectory.update((const double*)&(optParam[1]), optParam[0]);
				}
				_optPlanHorizon = optParam[0];
			}
		}
		else
		{
			if (_planLastPlan == false)
			{
				if (_optimizerType == "NONE")
					_optTrajectory.updateFromUniform(backupOptParam);
					//_optTrajectory.updateFromUniform(_rotMat2WRef*_optTrajectory.cArray2CtrlPtsMat(backupOptParam));
				else
				{
					std::cout << "\033[1;34mREJECTED, WRT ROBOT:\033[0m\n";
					std::cout << _optTrajectory.cArray2CtrlPtsMat(optParam) << std::endl;
					_optTrajectory.update(_rotMat2WRef * _optTrajectory.cArray2CtrlPtsMat(backupOptParam));
					// _optTrajectory.update((const double*)backupOptParam);
				}

			}
			else
			{
				if (_optimizerType == "NONE")
					_optTrajectory.updateFromUniform(&(backupOptParam[1]), backupOptParam[0]);
					//_optTrajectory.updateFromUniform(_rotMat2WRef*_optTrajectory.cArray2CtrlPtsMat(&(backupOptParam[1])), backupOptParam[0]);
				else
				{
					std::cout << "\033[1;34mREJECTED, WRT ROBOT:\033[0m\n";
					std::cout << _optTrajectory.cArray2CtrlPtsMat(&(optParam[1])) << std::endl;
					_optTrajectory.update(_rotMat2WRef * _optTrajectory.cArray2CtrlPtsMat(&(backupOptParam[1])), backupOptParam[0]);
					// _optTrajectory.update((const double*)&(backupOptParam[1]), backupOptParam[0]);
				}
				_optPlanHorizon = backupOptParam[0];
			}
		}
		
		// boost::this_thread::interruption_point(); // Say bye bye if optimization took too long

		delete[] optParam;
		delete[] backupOptParam;
		delete[] tolEq;
		delete[] tolIneq;

		return gotInterrupted;
		// std::cout << "END OF SOLVE OPT PROB\n";
		// boost::this_thread::interruption_point(); // Say bye bye if optimization took too long

	}


	/*

	Inequations should be in the form:
	myconstraint(x) <= 0

	*/

	void PathPlannerRecHor::computeNumGrad(unsigned m, unsigned n, const double* x, double* grad, void (*eval)(double*, unsigned, volatile double*, PathPlannerRecHor*))
	{
		const double eps = sqrt(std::numeric_limits< double >::epsilon()*_numDerivativeFactor);

		double *constrPre = new double[m];
		double *constrPos = new double[m];
		volatile double *x1 = new double[n];
		double h, dx;

		for (int i = 0; i < int(n); ++i)
		{
			x1[i] = x[i];
		}

		(*eval)(constrPre, n, x1, this);

		for (int i = 0; i < int(n); ++i)
		{
			h = x[i] < eps ? eps*eps : eps*x[i];
			x1[i] = x[i] + h;
			dx = x1[i] - x[i];
			(*eval)(constrPos, n, x1, this);

			// x1[i] = x[i] - h;
			// dx = dx + (x[i] - x1[i]);
			// (*eval)(constrPre, n, x1, this);
			x1[i] = x[i];

			for (int j = 0; j < int(m)	; ++j)
			{
				grad[j*n + i] = (constrPos[j] - constrPre[j]) / dx;
				if (boost::math::isnan(grad[j*n + i]))
				{
					grad[j*n + i] = 0.0;
				}
			}
			
		}
		delete[] constrPre;
		delete[] constrPos;
		delete[] x1;
	}

	double  PathPlannerRecHor::objectFunc(unsigned n, const double *x, double *grad, void *data)
	{
		// get this path planner pointer
		PathPlannerRecHor *context = static_cast< PathPlannerRecHor *>(data);

		double result;
		
		evalObj(&result, n, x, context);

		// If grad not NULL the Jacobian matrix must be given
		if (grad)
		{
			context->computeNumGrad(1, n, x, grad, &evalObj);
		}
		// std::cout << "ObjFResults: " << result << std::endl;
		// std::cout << "ObjFGrad: " << std::endl;
		// for (auto i=0; i<n; ++i)
		// {
		// 	std::cout << grad[i] << ", ";
		// }
		// std::cout << std::endl;
		return result;
	}

	void PathPlannerRecHor::eqFunc(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data)
	{
		// get this path planner pointer
		PathPlannerRecHor *context = static_cast< PathPlannerRecHor *>(data);
		
		evalEq(result, n, x, context);

		if (grad)
		{
			context->computeNumGrad(m, n, x, grad, &evalEq);
		}
		// std::cout << "EqResults: " << std::endl;
		// for (auto i=0; i < m; ++i)
		// {
		// 	std::cout << result[i] << ", ";
		// }
		// std::cout << std::endl << std::endl;
		// std::cout << "EqGrad: " << std::endl;
		// for (auto i=0; i<n*m; ++i)
		// {
		// 	std::cout << grad[i] << ", ";
		// }
		// std::cout << std::endl;
	}

	void PathPlannerRecHor::ineqFunc(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data)
	{
		// get this path planner pointer
		PathPlannerRecHor *context = static_cast< PathPlannerRecHor *>(data);
		
		evalIneq(result, n, x, context);

		if (grad)
		{
			context->computeNumGrad(m, n, x, grad, &evalIneq);
		}
		// std::cout << "IneqResults: " << std::endl;
		// for (auto i=0; i < m; ++i)
		// {
		// 	std::cout << result[i] << ", ";
		// }
		// std::cout << std::endl << std::endl;
		// std::cout << "IneqGrad: " << std::endl;
		// for (auto i=0; i<n*m; ++i)
		// {
		// 	std::cout << grad[i] << ", ";
		// }
		// std::cout << std::endl;
	}


	// LAST STEP COST EQ INEQ
	double  PathPlannerRecHor::objectFuncLS(unsigned n, const double *x, double *grad, void *data)
	{
		// get this path planner pointer
		PathPlannerRecHor *context = static_cast< PathPlannerRecHor *>(data);

		double result;
		
		evalObjLS(&result, n, x, context);

		// If grad not NULL the Jacobian matrix must be given
		if (grad)
		{
			context->computeNumGrad(1, n, x, grad, &evalObjLS);
		}
		// std::cout << "ObjFResults: " << result << std::endl;
		// std::cout << "ObjFGrad (" << n << "): " << std::endl;
		// for (auto i=0; i<n; ++i)
		// {
		// 	std::cout << grad[i] << ", ";
		// }
		// std::cout << std::endl;
		return result;
	}

	void PathPlannerRecHor::eqFuncLS(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data)
	{
		// get this path planner pointer
		PathPlannerRecHor *context = static_cast< PathPlannerRecHor *>(data);
		
		evalEqLS(result, n, x, context);

		// std::cout << "EqResults (" << m << "): " << std::endl;
		// for (auto i=0; i < m; ++i)
		// {
		// 	std::cout << "R[" << i << "]=" << result[i] << ", ";
		// }
		// std::cout << std::endl << std::endl;

		
		if (grad)
		{
			context->computeNumGrad(m, n, x, grad, &evalEqLS);
		}
		// std::cout << "EqGrad (" << n << ", " << m << "): " << std::endl;
		// for (auto i=0; i<n*m; ++i)
		// {
		// 	std::cout << "G[" << i << "]=" << grad[i] << ", ";
		// }
		// std::cout << std::endl;
	}

	void PathPlannerRecHor::ineqFuncLS(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data)
	{
		// get this path planner pointer
		PathPlannerRecHor *context = static_cast< PathPlannerRecHor *>(data);
		
		evalIneqLS(result, n, x, context);

		if (grad)
		{
			context->computeNumGrad(m, n, x, grad, &evalIneqLS);
		}
		// std::cout << "IneqResults: " << std::endl;
		// for (auto i=0; i < m; ++i)
		// {
		// 	std::cout << result[i] << ", ";
		// }
		// std::cout << std::endl << std::endl;
		// std::cout << "IneqGrad (" << n << ", " << m << "): " << std::endl;
		// for (auto i=0; i<n; ++i)
		// {
		// 	std::cout << grad[i] << ", ";
		// }
		// std::cout << std::endl;
	}

	

	// int PathPlannerRecHor::findspan(const double t)
	// {
	// 	int ret = 0;
	// 	//std::cout << "\n all knots :\n" << this->auxSpline.knots() << std::endl;
	// 	//std::cout << "\n knots@0 :\n" << this->auxSpline.knots()(0, 0) << std::endl;
	// 	//std::cout << "\n knots@4 :\n" << this->auxSpline.knots()(0, 4) << std::endl;
	// 	//std::cout << "\n knots@5 :\n" << this->auxSpline.knots()(0, 5) << std::endl;
	// 	//std::cout << "\nt :\n"  << t << std::endl;
	// 	while (ret <= _nCtrlPts - 1 &&  t >= this->auxSpline.knots()(0,ret))
	// 		ret++;
	// 	////std::cout << "\nspan:\n" << ret - 1 << std::endl;
	// 	return ret - 1;
	// }
}
// cmake:sourcegroup=PathPlanner_currentDirec.x()*_currentPose.y()