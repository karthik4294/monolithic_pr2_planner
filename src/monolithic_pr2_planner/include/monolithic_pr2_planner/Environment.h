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
#include <numeric>
#include <random>
#include <math.h>
#include <unordered_map>

#include <eigen3/Eigen/Geometry> 

#define NUM_SMHA_HEUR 4 // Used in EnvInterfaces to initialize the planner.
#define NUM_IMHA_HEUR 4 // Used in EnvInterfaces to initialize the planner.
// This should include the Anchor search -> Total number of searches.

#define INFINITECOST 1000000000

namespace monolithic_pr2_planner {
    /*! \brief Implements a complete environment used by the SBPL planner.
     * Contains everything from managing state IDs to collision space
     * information.
     */

    struct Trajectory{
        std::vector<int> traj_ids;
        std::vector<int> action_ids;
        std::vector<double> cum_rewards;
        bool contain;
    };

    typedef std::pair<int, int> Edge;
    class Environment : public EnvironmentMHA {
        public:
            Environment(ros::NodeHandle nh, bool learn_phase = false);
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
            void reset();
            void setPlannerType(int planner_type);
            void setUseNewHeuristics(bool use_new_heuristics){m_use_new_heuristics = use_new_heuristics;};

            void set_theta(Eigen::MatrixXd theta);
            Trajectory GenerateTraj(int sourceStateID);
            std::discrete_distribution<> GetDistribution(std::vector<double> p);
            Eigen::MatrixXd GetFeatureVector(int lm_state_id_1, int lm_state_id_2);
            std::vector<double> GetSoftmaxProbs(int sourceStateID, std::vector<int> succ_ids);
            Eigen::MatrixXd GetGradient(int state_id, int action_id, double cum_reward);
            void UpdateTheta(Eigen::MatrixXd &theta);

        protected:
            bool setStartGoal(SearchRequestPtr search_request, 
                              int& start_id, int& goal_id);
            int saveFakeGoalState(const GraphStatePtr& graph_state);
            void configurePlanningDomain();
            void configureQuerySpecificParams(SearchRequestPtr search_request);
            void generateStartState(SearchRequestPtr search_request);

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

            int m_min_heur;
            bool m_learn_phase;
            int m_num_trajs;
            int m_traj_ts;
            double m_alpha;
            double m_explr;
            Eigen::MatrixXd m_theta;
            
            std::vector<Trajectory> m_trajectories;
            std::unordered_map<int, std::vector<int>> m_succ_map;
            std::unordered_map<int, std::vector<double>> m_prob_map;

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
