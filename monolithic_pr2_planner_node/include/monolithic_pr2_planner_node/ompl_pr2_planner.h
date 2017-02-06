#pragma once
#include <ros/ros.h>
#include <ompl/base/Cost.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/State.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/base/goals/GoalState.h>
#include <monolithic_pr2_planner_node/ompl_collision_checker.h>
#include <monolithic_pr2_planner_node/ompl_motion_validator.h>
#include <memory>
#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/RealVectorWeightedStateSpace.h>
#include <monolithic_pr2_planner_node/GetMobileArmPlan.h>
#include <monolithic_pr2_planner/SearchRequest.h>
#include <monolithic_pr2_planner/StatsWriter.h>

typedef ompl::base::RealVectorWeightedStateSpace::StateType VectorState;
typedef ompl::base::ScopedState<ompl::base::RealVectorWeightedStateSpace> ScVectorState;
typedef ompl::base::SE2StateSpace::StateType SE2State;
typedef ompl::base::ScopedState<ompl::base::CompoundStateSpace> FullState;
typedef monolithic_pr2_planner_node::GetMobileArmPlan::Request NodeRequest;

#define RRT_NUM 0
#define PRM_P_NUM 1
#define RRTCONNECT_NUM 2 
#define RRTSTAR_NUM 3
#define RRTSTARFIRSTSOL_NUM 4
#define BITSTAR_NUM 5 
#define BITSTARFIRSTSOL_NUM 6


class OMPLPR2Planner{
    public:
        OMPLPR2Planner(const monolithic_pr2_planner::CSpaceMgrPtr& cspace, int planner_id){}
        OMPLPR2Planner(const monolithic_pr2_planner::CSpaceMgrPtr& cspace, int planner_id,
                       SearchRequestParams& search_request);
        bool planPathCallback(monolithic_pr2_planner::SearchRequestParams& search_request, int trial_id,
            StatsWriter& m_stats_writer);
        bool checkRequest(monolithic_pr2_planner::SearchRequestParams& search_request);
        bool createStartGoal(ScVectorState& start, ScVectorState& goal, monolithic_pr2_planner::SearchRequestParams& req);
        void setPlanningTime(double t){m_allocated_planning_time = t;};
        ompl::base::SpaceInformationPtr GetSpaceInformationPtr() { return m_si;}
        ompl::base::StateSpacePtr GetStateSpacePtr() { return fullBodySpace;}
        ompl::base::ProblemDefinition* GetProblemDefinition() { return pdef;}
    private:
        bool convertFullState(const ompl::base::State* state,
                              monolithic_pr2_planner::RobotState& robot_state,
                              monolithic_pr2_planner::ContBaseState& base);
        ompl::base::SpaceInformationPtr m_si;
        ompl::base::StateSpacePtr fullBodySpace;
        ompl::base::ProblemDefinition* pdef;
        ompl::base::Planner* planner;
        ompl::geometric::PathSimplifier* pathSimplifier;
        omplFullBodyCollisionChecker* m_collision_checker;
        omplFullBodyMotionValidator* m_motion_validator;
        // StatsWriter m_stats_writer;
        int m_planner_id;
        double m_allocated_planning_time;
        ros::NodeHandle ph_;
        double base_lin_weight_;
        double base_ang_weight_;
        double obj_lin_weight_;
        double obj_ang_weight_;
};

