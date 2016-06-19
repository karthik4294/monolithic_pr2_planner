#pragma once
#include <ros/ros.h>
#include <sbpl/headers.h>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
#include <monolithic_pr2_planner/HashManager.h>
#include <monolithic_pr2_planner/SearchRequest.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitivesMgr.h>
#include <monolithic_pr2_planner/Heuristics/HeuristicMgr.h>
#include <monolithic_pr2_planner/PathPostProcessor.h>
#include <stdexcept>
#include <vector>
#include <memory>

//OMPL headers
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/State.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include "ompl/base/goals/GoalState.h"
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/Cost.h>

typedef ompl::base::RealVectorStateSpace::StateType VectorState;
typedef ompl::base::SE2StateSpace::StateType SE2State;
typedef ompl::base::ScopedState<ompl::base::CompoundStateSpace> FullState;

#define NUM_SMHA_HEUR 4 // Used in EnvInterfaces to initialize the planner.
#define NUM_IMHA_HEUR 4 // Used in EnvInterfaces to initialize the planner.
// This should include the Anchor search -> Total number of searches.

namespace monolithic_pr2_planner {
    /*! \brief Implements a complete environment used by the SBPL planner.
     * Contains everything from managing state IDs to collision space
     * information.
     */
    typedef std::pair<int, int> Edge;
    class EnvironmentMonolithic : public EnvironmentPPMA {
        public:
            EnvironmentMonolithic(ros::NodeHandle nh);
            CSpaceMgrPtr getCollisionSpace(){ return m_cspace_mgr; };
            HeuristicMgrPtr getHeuristicMgr(){ return m_heur_mgr; };
            bool configureRequest(SearchRequestParamsPtr search_request_params,
                                  int& start_id, int& goal_id);
            void GetSuccs(int sourceStateID, vector<int>* succIDs, 
                vector<int>* costs);
            void GetSuccs(int q_id, int sourceStateID, vector<int>* succIDs, 
                vector<int>* costs);

            void GetLazySuccs(int sourceStateID, vector<int>* succIDs, 
                vector<int>* costs, std::vector<bool>* isTrueCost);
            void GetLazySuccs(int q_id, int sourceStateID, vector<int>* succIDs, 
                vector<int>* costs, std::vector<bool>* isTrueCost);

            int GetTrueCost(int parentID, int childID);
            std::vector<FullBodyState> reconstructPath(std::vector<int> 
                state_ids);
            void reset(const ompl::base::SpaceInformationPtr);
            void setPlannerType(int planner_type);
            void setUseNewHeuristics(bool use_new_heuristics){m_use_new_heuristics = use_new_heuristics;};

        protected:
            bool setStartGoal(SearchRequestPtr search_request, 
                              int& start_id, int& goal_id);
            int saveFakeGoalState(const GraphStatePtr& graph_state);
            void configurePlanningDomain();
            void configureQuerySpecificParams(SearchRequestPtr search_request);
            void generateStartState(SearchRequestPtr search_request);

            //PPMA
            void GetNearestLatticeState(const ompl::base::State *continuous_state, ompl::base::State* nearest_lattice_state, int *nearest_lattice_state_id);
            void GetContState(int state_id, ompl::base::State *state);
            int GetContStateID(const ompl::base::State* state); 
            int GetContEdgeCost(const ompl::base::State *parent, const ompl::base::State *child);
            void printContState(std::vector<double> val){
                return;
            }
            bool convertFullState(const ompl::base::State* state, RobotState& robot_state, ContBaseState& base);
            int getAdditionalCostMult();

            ParameterCatalog m_param_catalog;
            CSpaceMgrPtr m_cspace_mgr;
            HashManagerPtr m_hash_mgr;
            ros::NodeHandle m_nodehandle;
            GoalStatePtr m_goal;
            bool m_using_lazy;
            MotionPrimitivesMgr m_mprims;
            HeuristicMgrPtr m_heur_mgr;

            int m_planner_type;
            bool m_use_new_heuristics;

            ompl::base::SpaceInformationPtr si_;

        // SBPL interface stuff
        public:
            bool InitializeEnv(const char* sEnvFile){return false;};
            bool InitializeMDPCfg(MDPConfig *MDPCfg){ return true; };
            int  GetFromToHeuristic(int FromStateID, int ToStateID){ throw std::runtime_error("unimplement");  };
            int  GetGoalHeuristic(int stateID);
            int  GetGoalHeuristic(int heuristic_id, int stateID);
            int  GetStartHeuristic(int stateID) { throw std::runtime_error("unimplement"); };
            void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV){};
            void SetAllActionsandAllOutcomes(CMDPSTATE* state){};
            void SetAllPreds(CMDPSTATE* state){};
            int  SizeofCreatedEnv(){ return m_hash_mgr->size(); };
            void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL){};
            void PrintEnv_Config(FILE* fOut){};
            std::map<Edge, MotionPrimitivePtr> m_edges;
    };
}