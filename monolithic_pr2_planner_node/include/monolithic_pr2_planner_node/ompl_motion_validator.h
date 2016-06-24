#pragma once
#include <ompl/base/MotionValidator.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/State.h>
#include <ompl/base/StateValidityChecker.h>
#include "ompl/base/goals/GoalState.h"
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/Cost.h>

using namespace monolithic_pr2_planner;
using namespace std;

typedef ompl::base::RealVectorStateSpace::StateType VectorState;
typedef ompl::base::SE2StateSpace::StateType SE2State;
typedef ompl::base::ScopedState<ompl::base::CompoundStateSpace> FullState;

class omplFullBodyMotionValidator : public ompl::base::MotionValidator {
  public:
    omplFullBodyMotionValidator(const ompl::base::SpaceInformationPtr &si) : 
        ompl::base::MotionValidator(si){
            m_si = si;
        }
    monolithic_pr2_planner::CSpaceMgrPtr m_cspace;

    void initialize(monolithic_pr2_planner::CSpaceMgrPtr cspace, std::vector<double> l_arm);
    virtual bool checkMotion(const ompl::base::State* nstate, const ompl::base::State* dstate) const{
        std::pair<ompl::base::State*, double> last_valid;
        return checkMotion(nstate, dstate, last_valid);
    }
    virtual bool checkMotion(const ompl::base::State* nstate, const ompl::base::State* dstate, std::pair<ompl::base::State*, double>& last_valid) const;
    bool convertFullState(const ompl::base::State* state, RobotState& robot_state, ContBaseState& base) const;
    std::vector<double> l_arm_init;
    ompl::base::SpaceInformationPtr m_si;
};
