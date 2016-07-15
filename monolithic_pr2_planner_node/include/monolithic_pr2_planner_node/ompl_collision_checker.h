#pragma once
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/StateReps/RealVectorWeightedStateSpace.h>

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

typedef ompl::base::RealVectorWeightedStateSpace::StateType VectorState;
typedef ompl::base::SE2StateSpace::StateType SE2State;
typedef ompl::base::ScopedState<ompl::base::CompoundStateSpace> FullState;

class omplFullBodyCollisionChecker : public ompl::base::StateValidityChecker {
  public:
    omplFullBodyCollisionChecker(const ompl::base::SpaceInformationPtr &si) : 
        ompl::base::StateValidityChecker(si){}
    monolithic_pr2_planner::CSpaceMgrPtr m_cspace;

    void initialize(monolithic_pr2_planner::CSpaceMgrPtr cspace, std::vector<double> l_arm);
    //void readFile(char filename[], std::vector<std::pair<State, State> >& pairs);
    void initializeRegions(std::string file);
    //bool generateRandomValidState(State& s, vector<double>& arm_right, vector<double>& arm_left, int idx, int region_id=0);
    //void generateRandomState(State& s, int region_id);
    inline double randomDouble(double min, double max);
    virtual bool isValid(const ompl::base::State *state) const;
    bool convertFullState(const ompl::base::State* state, RobotState& robot_state, ContBaseState& base) const;
    std::vector<double> l_arm_init;

};
