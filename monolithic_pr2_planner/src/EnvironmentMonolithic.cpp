#include <monolithic_pr2_planner/EnvironmentMonolithic.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/DiscBaseState.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/MotionPrimitives/ArmAdaptiveMotionPrimitive.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <monolithic_pr2_planner/Constants.h>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <algorithm>
#include <cassert>

#define GOAL_STATE 1
using namespace monolithic_pr2_planner;
using namespace boost;

// stateid2mapping pointer inherited from sbpl interface. needed for planner.
EnvironmentMonolithic::EnvironmentMonolithic(ros::NodeHandle nh)
    :   m_hash_mgr(new HashManager(&StateID2IndexMapping)),
        m_nodehandle(nh), m_mprims(m_goal),
        m_heur_mgr(new HeuristicMgr()),
        m_using_lazy(false),
        m_planner_type(T_SMHA) {
        m_param_catalog.fetch(nh);
        configurePlanningDomain();
}

/**
 * @brief Resets the environment.
 * @details Intended to be used between calls to subsequent planning
 * requests.
 */
void EnvironmentMonolithic::reset() {
    m_heur_mgr->reset();
    // m_heur_mgr->setCollisionSpaceMgr(m_cspace_mgr);
    m_hash_mgr.reset(new HashManager(&StateID2IndexMapping));
    m_edges.clear();

    //si_ = si;

    // Fetch params again, in case they're being modified between calls.
    // m_param_catalog.fetch(m_nodehandle);
}

/**
 * @brief sets the planner type - mainly for experiments for the MHA paper
 * @details change the internal planner type to any of the different planners
 */
void EnvironmentMonolithic::setPlannerType(int planner_type) {
    m_planner_type = planner_type;
    m_heur_mgr->setPlannerType(planner_type);
    ROS_INFO_NAMED(SEARCH_LOG, "Setting planner type: %d", m_planner_type);
}

bool EnvironmentMonolithic::configureRequest(SearchRequestParamsPtr search_request_params,
                                   int& start_id, int& goal_id) {
    SearchRequestPtr search_request = SearchRequestPtr(new SearchRequest(
        search_request_params));
    configureQuerySpecificParams(search_request);
    if(search_request->m_params->underspecified_start) {
        ROS_DEBUG_NAMED(CONFIG_LOG, "underspecified_start. Will generate start state.");
        generateStartState(search_request);
    }
    if (!setStartGoal(search_request, start_id, goal_id)) {
        return false;
    }

    return true;
}

int EnvironmentMonolithic::GetGoalHeuristic(int stateID) {
    // For now, return the max of all the heuristics
    return GetGoalHeuristic(0, stateID);
}

int EnvironmentMonolithic::GetGoalHeuristic(int heuristic_id, int stateID) {
    GraphStatePtr successor = m_hash_mgr->getGraphState(stateID);
    if(m_goal->isSatisfiedBy(successor) || stateID == GOAL_STATE){
        return 0;
    }
    std::unique_ptr<stringintmap> values;
    m_heur_mgr->getGoalHeuristic(successor, values);
    
    for (auto& heur : (*values)) {
        ROS_DEBUG_NAMED(HEUR_LOG, "%s : %d", heur.first.c_str(), heur.second);
    }

    if(!m_use_new_heuristics){
      switch (heuristic_id) {
        case 0:  // Anchor
          return std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
        case 1:  // ARA Heur 
          return std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
        case 2:  // Base1, Base2 heur
          return static_cast<int>(0.5f*(*values).at("base_with_rot_0") + 0.5f*(*values).at("endeff_rot_goal"));
        case 3:
          return static_cast<int>(0.5f*(*values).at("base_with_rot_door") + 0.5f*(*values).at("endeff_rot_vert"));
      }
    }
    else{
      double w_bfsRot = 0.2;
      double w_armFold = 0.2;
      int ad_base = (*values).at("admissible_base");
      int ad_endeff = (*values).at("admissible_endeff");
      int anchor_h = std::max(ad_base, ad_endeff);
      int endeff_rot_goal = (*values).at("endeff_rot_goal");

      int inad_arm_heur = static_cast<int>(0.1*(*values).at("endeff_rot_goal") + 0.1*ad_endeff);
      if (ad_base > 1000) //TODO: check multiplier
      { 
        inad_arm_heur = (*values).at("arm_angles_folded");
      }

      switch (heuristic_id) {
        case 0:  // Anchor
          return anchor_h;
        case 1:  // Anchor
          return int(0.1*ad_base) + int(0.1*ad_endeff) + int(0.2*endeff_rot_goal);
          //return anchor_h;
        case 2:  // Base1, Base2 heur
          return static_cast<int>(0.1*(*values).at("base_with_rot_0") + 0.1*(*values).at("endeff_rot_goal"));
        case 3:  // Base1, Base2 heur
          //return static_cast<int>(1.0*(*values).at("base_with_rot_0") + 0.0*(*values).at("endeff_rot_goal"));
          return static_cast<int>(0.1*(*values).at("base_with_rot_0") + 0.1*(*values).at("arm_angles_folded"));
        case 4:
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot0") + w_armFold*inad_arm_heur);
        case 5:
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot1") + w_armFold*inad_arm_heur);
        case 6:
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot2") + w_armFold*inad_arm_heur);
        case 7:
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot3") + w_armFold*inad_arm_heur);
        case 8:
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot4") + w_armFold*inad_arm_heur);
        case 9:
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot5") + w_armFold*inad_arm_heur);
        case 10:
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot6") + w_armFold*inad_arm_heur);
        case 11:
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot7") + w_armFold*inad_arm_heur);
        case 12:
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot8") + w_armFold*inad_arm_heur);
        case 13:
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot9") + w_armFold*inad_arm_heur);
        case 14:
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot10") + w_armFold*inad_arm_heur);
        case 15:
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot11") + w_armFold*inad_arm_heur);
        case 16:
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot12") + w_armFold*inad_arm_heur);
        case 17:
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot13") + w_armFold*inad_arm_heur);
        case 18:
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot14") + w_armFold*inad_arm_heur);
        case 19:
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot15") + w_armFold*inad_arm_heur);
      }

      // switch (heuristic_id) {
      //   case 0:  // Anchor
      //     return std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
      //   case 1:  // Anchor
      //     return std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
      //   //case 1:  // ARA Heur 
      //     //return std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
      //   case 2:  // Base1, Base2 heur
      //     return static_cast<int>(0.5*(*values).at("base_with_rot_0") + 0.5*(*values).at("endeff_rot_goal"));
      //   case 3:  // Base1, Base2 heur
      //     //return static_cast<int>(1.0*(*values).at("base_with_rot_0") + 0.0*(*values).at("endeff_rot_goal"));
      //     return static_cast<int>(0.5*(*values).at("base_with_rot_0") + 0.5*(*values).at("arm_angles_folded"));
      //   case 4:
      //     if((*values).at("bfsRotFoot0")==0)
      //       return 0;
      //     return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot0") + w_armFold*(*values).at("arm_angles_folded"));
      //   case 5:
      //     if((*values).at("bfsRotFoot1")==0)
      //       return 0;
      //     return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot1") + w_armFold*(*values).at("arm_angles_folded"));
      //   case 6:
      //     if((*values).at("bfsRotFoot2")==0)
      //       return 0;
      //     return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot2") + w_armFold*(*values).at("arm_angles_folded"));
      //   case 7:
      //     if((*values).at("bfsRotFoot3")==0)
      //       return 0;
      //     return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot3") + w_armFold*(*values).at("arm_angles_folded"));
      //   case 8:
      //     if((*values).at("bfsRotFoot4")==0)
      //       return 0;
      //     return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot4") + w_armFold*(*values).at("arm_angles_folded"));
      //   case 9:
      //     if((*values).at("bfsRotFoot5")==0)
      //       return 0;
      //     return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot5") + w_armFold*(*values).at("arm_angles_folded"));
      //   case 10:
      //     if((*values).at("bfsRotFoot6")==0)
      //       return 0;
      //     return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot6") + w_armFold*(*values).at("arm_angles_folded"));
      //   case 11:
      //     if((*values).at("bfsRotFoot7")==0)
      //       return 0;
      //     return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot7") + w_armFold*(*values).at("arm_angles_folded"));
      //   case 12:
      //     if((*values).at("bfsRotFoot8")==0)
      //       return 0;
      //     return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot8") + w_armFold*(*values).at("arm_angles_folded"));
      //   case 13:
      //     if((*values).at("bfsRotFoot9")==0)
      //       return 0;
      //     return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot9") + w_armFold*(*values).at("arm_angles_folded"));
      //   case 14:
      //     if((*values).at("bfsRotFoot10")==0)
      //       return 0;
      //     return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot10") + w_armFold*(*values).at("arm_angles_folded"));
      //   case 15:
      //     if((*values).at("bfsRotFoot11")==0)
      //       return 0;
      //     return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot11") + w_armFold*(*values).at("arm_angles_folded"));
      //   case 16:
      //     if((*values).at("bfsRotFoot12")==0)
      //       return 0;
      //     return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot12") + w_armFold*(*values).at("arm_angles_folded"));
      //   case 17:
      //     if((*values).at("bfsRotFoot13")==0)
      //       return 0;
      //     return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot13") + w_armFold*(*values).at("arm_angles_folded"));
      //   case 18:
      //     if((*values).at("bfsRotFoot14")==0)
      //       return 0;
      //     return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot14") + w_armFold*(*values).at("arm_angles_folded"));
      //   case 19:
      //     if((*values).at("bfsRotFoot15")==0)
      //       return 0;
      //     return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot15") + w_armFold*(*values).at("arm_angles_folded"));
      // }
    }

    /*
    switch (heuristic_id) {
      case 0:  // Anchor
        return std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
      case 1:  // ARA Heur 
        return std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
      case 2:  // Base1, Base2 heur
        return static_cast<int>(0.5f*(*values).at("base_with_rot_0") + 0.5f*(*values).at("endeff_rot_goal"));
      case 3:
        return static_cast<int>(0.5f*(*values).at("base_with_rot_door") + 0.5f*(*values).at("endeff_rot_vert"));
    }
    */

    /*
    switch (m_planner_type) {
        case T_SMHA:
        case T_MHG_REEX:
        case T_MHG_NO_REEX:
        case T_ARA:
            switch (heuristic_id) {
                case 0:  // Anchor
                    return std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
                case 1:  // ARA Heur
                    return EPS2*std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
                // case 2:
                //     return EPS2*(*values).at("admissible_endeff");
                case 2:  // Base1, Base2 heur
                    return static_cast<int>(0.5f*(*values).at("base_with_rot_0") +
                        0.5f*(*values).at("endeff_rot_goal"));
                case 3:
                    return static_cast<int>(0.5f*(*values).at("base_with_rot_door") +
                        0.5f*(*values).at("endeff_rot_goal"));
            }
            break;
        case T_IMHA:
            switch (heuristic_id) {
                case 0:  // Anchor
                    return std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
                case 1:  // ARA Heur
                    return EPS2*std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
                case 2:  // Base1, Base2 heur
                    return static_cast<int>(0.5f*(*values).at("base_with_rot_0") + 0.5f*(*values).at("endeff_rot_goal"));
                case 3:
                    return static_cast<int>(0.5f*(*values).at("base_with_rot_door") + 0.5f*(*values).at("endeff_rot_goal"));
            }
            break;
        case T_MPWA:
            return (heuristic_id+1)*(EPS1*EPS2/NUM_SMHA_HEUR)*std::max(
                (*values).at("admissible_endeff"), (*values).at("admissible_base"));
            break;
        case T_EES:
            switch (heuristic_id) {
                case 0:  // Anchor
                    return std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
                case 1:  // Inadmissible
                    return (*values).at("base_with_rot_0") + (*values).at("admissible_endeff");
                    // return static_cast<int>(0.5f*(*values)[4] + 0.5f*(*values).at("admissible_endeff"));
                case 2:  // Distance function
                    // return (*values)[2];
                    // ROS_DEBUG_NAMED(HEUR_LOG, "Arm : %d, Base : %d", (*values)[2],
                    //     (*values)[3]);
                    return (*values)["uniform_2d"] + (*values)["uniform_3d"];
            }
            break;
    }
    */

    // Post-paper
    // switch(heuristic_id){
    //     case 0: //Anchor
    //         return std::max(values.at("admissible_endeff"), values.at("admissible_base"));
    //     case 1: // base
    //         return values[2];
    //     case 2: // Base + arm
    //         return static_cast<int>(0.5f*values[2] +
    //             0.5f*values[0]);
    // }


    // ROS_DEBUG_NAMED(HEUR_LOG, "2: %d,\t 3: %d", values[2],
    //     values[3]);
    // EES

    return std::max((*values).at("admissible_base"), (*values).at("admissible_endeff"));
}

void EnvironmentMonolithic::GetSuccs(int sourceStateID, vector<int>* succIDs, 
                           vector<int>* costs){
    GetSuccs(0, sourceStateID, succIDs, costs);
}

void EnvironmentMonolithic::GetSuccs(int q_id, int sourceStateID, vector<int>* succIDs, 
                           vector<int>* costs){
    assert(sourceStateID != GOAL_STATE);

    ROS_DEBUG_NAMED(SEARCH_LOG, 
            "==================Expanding state %d==================", 
                    sourceStateID);
    succIDs->clear();
    succIDs->reserve(m_mprims.getMotionPrims().size());
    costs->clear();
    costs->reserve(m_mprims.getMotionPrims().size());

    GraphStatePtr source_state = m_hash_mgr->getGraphState(sourceStateID);
    ROS_DEBUG_NAMED(SEARCH_LOG, "Source state is:");
    source_state->robot_pose().printToDebug(SEARCH_LOG);
    if (m_param_catalog.m_visualization_params.expansions) {
        RobotState expansion_pose = source_state->robot_pose();
        
        //************************DEBUG*********************//        
        // ROS_INFO("From GetSuccs");
        // ContBaseState base_state = expansion_pose.getContBaseState();
        // RightContArmState right_arm_state = expansion_pose.right_arm();
        // LeftContArmState left_arm_state = expansion_pose.left_arm();
        // ContObjectState obj_state = expansion_pose.getObjectStateRelMap();
        // std::vector<double> r_angles(7,0);
        // right_arm_state.getAngles(&r_angles);
        // ROS_INFO("Object : %f %f %f %f %f %f", obj_state.x(), obj_state.y(), obj_state.z(), obj_state.roll(), obj_state.pitch(), obj_state.yaw() );
        // ROS_INFO("Upper arm Roll : %f %f", right_arm_state.getUpperArmRollAngle(), left_arm_state.getUpperArmRollAngle());
        // ROS_INFO("Right arm angles : %f %f %f %f %f %f %f", r_angles[0],r_angles[1],r_angles[2],r_angles[3],r_angles[4],r_angles[5],r_angles[6]) ;        
        // ROS_INFO("Torso : %f", base_state.z());
        // ROS_INFO("Base : %f %f %f", base_state.x(), base_state.y(), base_state.theta());
        //************************DEBUG*********************//

        //Karthik
        //expansion_pose.visualize(250/NUM_SMHA_HEUR*q_id);
        
        // source_state->robot_pose().visualize(250/NUM_SMHA_HEUR*q_id);
        m_cspace_mgr->visualizeAttachedObject(expansion_pose, 250/NUM_SMHA_HEUR*q_id);
        // m_cspace_mgr->visualizeCollisionModel(expansion_pose);
        usleep(5000);
    }
    for (auto mprim : m_mprims.getMotionPrims()) {
        ROS_DEBUG_NAMED(SEARCH_LOG, "Applying motion:");
        // mprim->printEndCoord();
        GraphStatePtr successor;
        TransitionData t_data;
        if (!mprim->apply(*source_state, successor, t_data)) {
            ROS_DEBUG_NAMED(MPRIM_LOG, "couldn't apply mprim");
            continue;
        }

        if (m_cspace_mgr->isValidSuccessor(*successor,t_data) &&
            m_cspace_mgr->isValidTransitionStates(t_data)){
            ROS_DEBUG_NAMED(SEARCH_LOG, "Source state is:");
            source_state->printToDebug(SEARCH_LOG);
            m_hash_mgr->save(successor);
            ROS_DEBUG_NAMED(MPRIM_LOG, "successor state with id %d is:", 
                            successor->id());
            successor->printToDebug(MPRIM_LOG);

            RobotState succ_pose = successor->robot_pose();

            //************************DEBUG*********************//        
            // ROS_INFO("Succ:");
            // ContBaseState base_state = succ_pose.getContBaseState();
            // RightContArmState right_arm_state = succ_pose.right_arm();
            // LeftContArmState left_arm_state = succ_pose.left_arm();
            // ContObjectState obj_state = succ_pose.getObjectStateRelMap();
            // std::vector<double> r_angles(7,0);
            // right_arm_state.getAngles(&r_angles);
            // ROS_INFO("Object : %f %f %f %f %f %f", obj_state.x(), obj_state.y(), obj_state.z(), obj_state.roll(), obj_state.pitch(), obj_state.yaw() );
            // ROS_INFO("Upper arm Roll : %f %f", right_arm_state.getUpperArmRollAngle(), left_arm_state.getUpperArmRollAngle());
            // ROS_INFO("Right arm angles : %f %f %f %f %f %f %f", r_angles[0],r_angles[1],r_angles[2],r_angles[3],r_angles[4],r_angles[5],r_angles[6]) ;        
            // ROS_INFO("Torso : %f", base_state.z());
            // ROS_INFO("Base : %f %f %f", base_state.x(), base_state.y(), base_state.theta());
            //************************DEBUG*********************//


            if (m_goal->isSatisfiedBy(successor)){
                m_goal->storeAsSolnState(successor);
                ROS_DEBUG_NAMED(SEARCH_LOG, "Found potential goal at state %d %d", successor->id(),
                    mprim->cost());
                succIDs->push_back(GOAL_STATE);
            } else {
                succIDs->push_back(successor->id());
            }

            costs->push_back(mprim->cost());
            
            ROS_DEBUG_NAMED(SEARCH_LOG, "motion succeeded with cost %d", mprim->cost());
        } else {
            //successor->robot_pose().visualize();
            ROS_DEBUG_NAMED(SEARCH_LOG, "successor failed collision checking");
        }
    }
}

void EnvironmentMonolithic::GetLazySuccs(int sourceStateID, vector<int>* succIDs, 
                           vector<int>* costs, std::vector<bool>* isTrueCost)
{
  if (!m_using_lazy)
  {
    GetSuccs(0, sourceStateID, succIDs, costs);
    isTrueCost->clear();
    isTrueCost->resize(succIDs->size(), 1);
    return;
  }
  GetLazySuccs(0, sourceStateID, succIDs, costs, isTrueCost);
}

void EnvironmentMonolithic::GetLazySuccs(int q_id, int sourceStateID, vector<int>* succIDs, 
                           vector<int>* costs, std::vector<bool>* isTrueCost){
    
  if (!m_using_lazy)
  {
    GetSuccs(q_id, sourceStateID, succIDs, costs);
    isTrueCost->clear();
    isTrueCost->resize(succIDs->size(), 1);
    return;
  }

  double expansion_color = 250/NUM_SMHA_HEUR*q_id;
  vector<MotionPrimitivePtr> all_mprims = m_mprims.getMotionPrims();
    ROS_DEBUG_NAMED(SEARCH_LOG, "==================Expanding state %d==================", 
                    sourceStateID);
    succIDs->clear();
    succIDs->reserve(all_mprims.size());
    costs->clear();
    costs->reserve(all_mprims.size());

    GraphStatePtr source_state = m_hash_mgr->getGraphState(sourceStateID);
    ROS_DEBUG_NAMED(SEARCH_LOG, "Source state is:");
    source_state->robot_pose().printToDebug(SEARCH_LOG);
    if(m_param_catalog.m_visualization_params.expansions){
        source_state->robot_pose().visualize(expansion_color);
        usleep(10000);
    }
    for (auto mprim : all_mprims){
        //ROS_DEBUG_NAMED(SEARCH_LOG, "Applying motion:");
        //mprim->printEndCoord();
        GraphStatePtr successor;
        TransitionData t_data;

        if (mprim->motion_type() == MPrim_Types::ARM){
            successor.reset(new GraphState(*source_state));
            successor->lazyApplyMPrim(mprim->getEndCoord());
            ROS_DEBUG_NAMED(SEARCH_LOG, "arm mprim/source/successor");
            mprim->printEndCoord();
            source_state->printToDebug(MPRIM_LOG);
            successor->printToDebug(MPRIM_LOG);
            ROS_DEBUG_NAMED(SEARCH_LOG, "done");
        } else {
            if (!mprim->apply(*source_state, successor, t_data)){
                //ROS_DEBUG_NAMED(MPRIM_LOG, "couldn't apply mprim");
                continue;
            }
            ROS_DEBUG_NAMED(SEARCH_LOG, "non-arm mprim/source/successor");
            mprim->printEndCoord();
            source_state->printToDebug(MPRIM_LOG);
            successor->printToDebug(MPRIM_LOG);
            ROS_DEBUG_NAMED(SEARCH_LOG, "done");
        }
        m_hash_mgr->save(successor);
        Edge key; 

        if (m_goal->isSatisfiedBy(successor)){
          m_goal->storeAsSolnState(successor);
          //ROS_DEBUG_NAMED(SEARCH_LOG, "Found potential goal at state %d %d", successor->id(),
            //  mprim->cost());
          ROS_INFO("Found potential goal at: source->id %d, successor->id %d,"
            "cost: %d, mprim type: %d ", source_state->id(), successor->id(),
              mprim->cost(), mprim->motion_type());
          succIDs->push_back(GOAL_STATE);
          key = Edge(sourceStateID, GOAL_STATE);
        } else {
          succIDs->push_back(successor->id());
          key = Edge(sourceStateID, successor->id());
        }

//        succIDs->push_back(successor->id());
//        key = Edge(sourceStateID, successor->id());
        m_edges.insert(map<Edge, MotionPrimitivePtr>::value_type(key, mprim));
        costs->push_back(mprim->cost());
        isTrueCost->push_back(false);
    }
}

/*
 * Evaluates the edge. Assumes that the edge has already been generated and we
 * know the motion primitive used
 */
int EnvironmentMonolithic::GetTrueCost(int parentID, int childID){
    TransitionData t_data;

    vector<MotionPrimitivePtr> small_mprims;
    if (m_edges.find(Edge(parentID, childID)) == m_edges.end()){
      ROS_ERROR("transition hasn't been found between %d and %d??", parentID, childID);
        assert(false);
    }
    small_mprims.push_back(m_edges[Edge(parentID, childID)]);
    PathPostProcessor postprocessor(m_hash_mgr, m_cspace_mgr);

    ROS_DEBUG_NAMED(SEARCH_LOG, "evaluating edge (%d %d)", parentID, childID);
    GraphStatePtr source_state = m_hash_mgr->getGraphState(parentID);
    GraphStatePtr real_next_successor = m_hash_mgr->getGraphState(childID);
    GraphStatePtr successor;
    MotionPrimitivePtr mprim = m_edges.at(Edge(parentID, childID));
    if (!mprim->apply(*source_state, successor, t_data)){
        return -1;
    }
    mprim->printEndCoord();
    mprim->print();
    //source_state->printToInfo(SEARCH_LOG);
    //successor->printToInfo(SEARCH_LOG);
    successor->id(m_hash_mgr->getStateID(successor));

    // right before this point, the successor's graph state does not match the
    // stored robot state (because we modified the graph state without calling
    // ik and all that). this call updates the stored robot pose.
    real_next_successor->robot_pose(successor->robot_pose());

    bool matchesEndID = (successor->id() == childID) || (childID == GOAL_STATE);
    assert(matchesEndID);

    bool valid_successor = (m_cspace_mgr->isValidSuccessor(*successor, t_data) && 
                            m_cspace_mgr->isValidTransitionStates(t_data));
    if (!valid_successor){
        return -1;
    }
    return t_data.cost();
}

bool EnvironmentMonolithic::setStartGoal(SearchRequestPtr search_request,
                               int& start_id, int& goal_id){
    RobotState start_pose(search_request->m_params->base_start, 
                         search_request->m_params->right_arm_start,
                         search_request->m_params->left_arm_start);
    ContObjectState obj_state = start_pose.getObjectStateRelMap();
    obj_state.printToInfo(SEARCH_LOG);

    m_edges.clear();

    if (!search_request->isValid(m_cspace_mgr)){
        obj_state.printToInfo(SEARCH_LOG);
        start_pose.visualize();
        return false;
    }

    start_pose.visualize();
    m_cspace_mgr->visualizeAttachedObject(start_pose);
    //m_cspace_mgr->visualizeCollisionModel(start_pose);
    //std::cin.get();

    GraphStatePtr start_graph_state = make_shared<GraphState>(start_pose);
    m_hash_mgr->save(start_graph_state);
    start_id = start_graph_state->id();
    assert(m_hash_mgr->getGraphState(start_graph_state->id()) == start_graph_state);

    ROS_INFO_NAMED(SEARCH_LOG, "Start state set to:");
    start_pose.printToInfo(SEARCH_LOG);
    obj_state.printToInfo(SEARCH_LOG);
    // start_pose.visualize();


    m_goal = search_request->createGoalState();

    if (m_hash_mgr->size() < 2){
        goal_id = saveFakeGoalState(start_graph_state);
    } else {
        goal_id = 1;
    }

    ROS_INFO_NAMED(SEARCH_LOG, "Goal state created:");
    ContObjectState c_goal = m_goal->getObjectState();
    c_goal.printToInfo(SEARCH_LOG);
    m_goal->visualize();

    // This informs the adaptive motions about the goal.
    ArmAdaptiveMotionPrimitive::goal(*m_goal);
    BaseAdaptiveMotionPrimitive::goal(*m_goal);

    // informs the heuristic about the goal
    m_heur_mgr->setGoal(*m_goal);

    return true;
}

// a hack to reserve a goal id in the hash so that no real graph state is ever
// saved as the goal state id
int EnvironmentMonolithic::saveFakeGoalState(const GraphStatePtr& start_graph_state){
    GraphStatePtr fake_goal = make_shared<GraphState>(*start_graph_state);
    RobotState fake_robot_state = fake_goal->robot_pose();
    DiscBaseState fake_base = fake_robot_state.base_state();
    fake_base.x(0); fake_base.y(0); fake_base.z(0);
    fake_robot_state.base_state(fake_base);
    fake_goal->robot_pose(fake_robot_state);
    m_hash_mgr->save(fake_goal);
    int goal_id = fake_goal->id();
    assert(goal_id == GOAL_STATE);
    return goal_id;
}

// this sets up the environment for things that are query independent.
void EnvironmentMonolithic::configurePlanningDomain(){
    // used for collision space and discretizing plain xyz into grid world 
    OccupancyGridUser::init(m_param_catalog.m_occupancy_grid_params,
                        m_param_catalog.m_robot_resolution_params);
    

    // used for discretization of robot movements
    ContArmState::setRobotResolutionParams(m_param_catalog.m_robot_resolution_params);

#ifdef USE_IKFAST_SOLVER
    ROS_DEBUG_NAMED(CONFIG_LOG, "Using IKFast");
#endif
#ifdef USE_KDL_SOLVER
    ROS_DEBUG_NAMED(CONFIG_LOG, "Using KDL");
#endif

    // Initialize the heuristics. The (optional) parameter defines the cost multiplier.

    m_heur_mgr->initializeHeuristics();

    // used for arm kinematics
    LeftContArmState::initArmModel(m_param_catalog.m_left_arm_params);
    RightContArmState::initArmModel(m_param_catalog.m_right_arm_params);

    // collision space mgr needs arm models in order to do collision checking
    // have to do this funny thing  of initializing an object because of static
    // variable + inheritance (see ContArmState for details)
    LeftContArmState l_arm;
    RightContArmState r_arm;
    m_cspace_mgr = make_shared<CollisionSpaceMgr>(r_arm.getArmModel(),
                                                  l_arm.getArmModel());
    m_heur_mgr->setCollisionSpaceMgr(m_cspace_mgr);
    // load up motion primitives
    m_mprims.loadMPrims(m_param_catalog.m_motion_primitive_params);

    // load up static pviz instance for visualizations. 
    Visualizer::createPVizInstance();
    Visualizer::setReferenceFrame(std::string("/map"));
}

// sets parameters for query specific things
void EnvironmentMonolithic::configureQuerySpecificParams(SearchRequestPtr search_request){
    // sets the location of the object in the frame of the wrist
    // have to do this funny thing  of initializing an object because of static
    // variable + inheritance (see ContArmState for details)
    LeftContArmState l_arm;
    RightContArmState r_arm;
    l_arm.setObjectOffset(search_request->m_params->left_arm_object);
    r_arm.setObjectOffset(search_request->m_params->right_arm_object);
    ROS_DEBUG_NAMED(SEARCH_LOG, "Setting planning mode to : %d",
        search_request->m_params->planning_mode);
    RobotState::setPlanningMode(search_request->m_params->planning_mode);
}

/*! \brief Given the solution path containing state IDs, reconstruct the
 * actual corresponding robot states. This also makes the path smooth in between
 * each state id because we add in the intermediate states given by the
 * transition data.
 */
vector<FullBodyState> EnvironmentMonolithic::reconstructPath(vector<int> soln_path){
    PathPostProcessor postprocessor(m_hash_mgr, m_cspace_mgr);
    std::vector<FullBodyState> final_path = postprocessor.reconstructPath(soln_path, *m_goal, m_mprims.getMotionPrims());
    if(m_param_catalog.m_visualization_params.final_path){
        postprocessor.visualizeFinalPath(final_path);
    }
    return final_path;
}

void EnvironmentMonolithic::generateStartState(SearchRequestPtr search_request) {
    ContObjectState start_obj_state(search_request->m_params->obj_start);
    ContBaseState base_start(search_request->m_params->base_start);
    RobotState start_robot_state(base_start, start_obj_state);
    start_robot_state.visualize();
    m_cspace_mgr->visualizeAttachedObject(start_robot_state);
    ROS_DEBUG_NAMED(CONFIG_LOG, "Generate start state : Keyboard");
    // std::cin.get();
    search_request->m_params->base_start = start_robot_state.getContBaseState();
    search_request->m_params->right_arm_start = start_robot_state.right_arm();
    search_request->m_params->left_arm_start = start_robot_state.left_arm();
}

void EnvironmentMonolithic::GetNearestLatticeState(const ompl::base::State *continuous_state, ompl::base::State* nearest_lattice_state, int *nearest_lattice_state_id){
  
  RobotState continous_robot_state, graph_robot_state;
  ContBaseState continous_base_state;
  
  if (!convertFullState(continuous_state, continous_robot_state, continous_base_state)) {
    //ROS_ERROR("[GetNearestLatticeState]ik failed for visualization!");
  }

  GraphStatePtr continous_graph_state = make_shared<GraphState>(continous_robot_state);
  m_hash_mgr->save(continous_graph_state);
  *nearest_lattice_state_id = continous_graph_state->id();

  GetContState(*nearest_lattice_state_id, nearest_lattice_state);

}

void EnvironmentMonolithic::GetContState(int state_id, ompl::base::State *state){
  
  GraphStatePtr graph_state = m_hash_mgr->getGraphState(state_id);
  RobotState robot_state = graph_state->robot_pose();

  ContBaseState base_state = robot_state.getContBaseState();
  RightContArmState right_arm_state = robot_state.right_arm();
  LeftContArmState left_arm_state = robot_state.left_arm();
  ContObjectState obj_state = robot_state.getObjectStateRelBody();

  ompl::base::CompoundState* s = dynamic_cast<ompl::base::CompoundState*> (state);

  s->as<VectorState>(0)->values[0] = obj_state.x();
  s->as<VectorState>(0)->values[1] = obj_state.y();
  s->as<VectorState>(0)->values[2] = obj_state.z();
  s->as<VectorState>(0)->values[3] = obj_state.roll();
  s->as<VectorState>(0)->values[4] = obj_state.pitch();
  s->as<VectorState>(0)->values[5] = obj_state.yaw();
  s->as<VectorState>(0)->values[6] = right_arm_state.getUpperArmRollAngle();
  s->as<VectorState>(0)->values[7] = left_arm_state.getUpperArmRollAngle();
  s->as<VectorState>(0)->values[8] = base_state.z();

  s->as<SE2State>(1)->setX(base_state.x());
  s->as<SE2State>(1)->setY(base_state.y());
  s->as<SE2State>(1)->setYaw(angles::normalize_angle(base_state.theta() ) );

}

int EnvironmentMonolithic::GetContStateID(const ompl::base::State* state){
  RobotState robot_state;
  ContBaseState base;
  if (!convertFullState(state, robot_state, base)) {
    //ROS_ERROR("[GetContStateid]ik failed for visualization!");
  }
  GraphStatePtr graph_state = make_shared<GraphState>(robot_state);
  m_hash_mgr->save(graph_state);
  return graph_state->id();
} 

int EnvironmentMonolithic::GetContEdgeCost(const ompl::base::State *parent, const ompl::base::State *child){
  
    double METER_TO_MM_MULT = 1000;

    RobotState parent_robot_state, child_robot_state;
    ContBaseState parent_base, child_base;
    
    if (!convertFullState(parent, parent_robot_state, parent_base)) {
      //ROS_ERROR("[GetContedgecost]ik failed for parent!");
    }

    if (!convertFullState(child, child_robot_state, child_base)) {
      //ROS_ERROR("[GetContedgecost]ik failed for child!");
    }

    double dx = parent_base.x() - child_base.x();
    double dy = parent_base.y()-child_base.y();
    double linear_distance = sqrt(dx*dx + dy*dy);
    double linear_time = linear_distance/static_cast<double>(m_param_catalog.m_motion_primitive_params.nominal_vel);
    double first_angle = parent_base.theta();
    double final_angle = child_base.theta();
    double angular_distance = fabs(shortest_angular_distance(first_angle, 
                                                             final_angle));
    double angular_time = angular_distance/static_cast<double>(m_param_catalog.m_motion_primitive_params.angular_vel);

    //make the cost the max of the two times
    int cost = ceil(static_cast<double>(METER_TO_MM_MULT)*(max(linear_time, angular_time)));
    //use any additional cost multiplier
    cost *= getAdditionalCostMult();
    
    return cost;
}

void EnvironmentMonolithic::VisualizeContState(const ompl::base::State *child, const ompl::base::State *parent, bool is_discrete, bool is_path)
{
    if(is_path)
      return;

    RobotState parent_robot_state, child_robot_state;
    ContBaseState parent_base, child_base;
    
    if(is_discrete)
    {
      if (!convertFullState(child, child_robot_state, child_base)) {
        //ROS_ERROR("[VisualizeContState] Discrete child ik failed for visualization!");
      }
    }

    if(!is_discrete)
    {
      if (!convertFullState(child, child_robot_state, child_base)) {
       //ROS_ERROR("[VisualizeContState] Conitnous child ik failed for visualization!");
       //printContState(child);
      }
    }

    vector<double> l_arm, r_arm;
    //robot_states.push_back(child_robot_state);
    //base_states.push_back(child_base);

    child_robot_state.right_arm().getAngles(&r_arm);
    child_robot_state.left_arm().getAngles(&l_arm);
    BodyPose bp = child_base.body_pose();  

    if(is_discrete)
      Visualizer::pviz->visualizeRobot(r_arm, l_arm, bp, 60, "discrete_state", 0);
    else
      Visualizer::pviz->visualizeRobot(r_arm, l_arm, bp, 240, "continous_state", 0);
    
    usleep(5000);
}

void EnvironmentMonolithic::printContState(const ompl::base::State* state)
{
    const ompl::base::CompoundState* s = dynamic_cast<const ompl::base::CompoundState*> (state);

    ROS_INFO("OBJECT STATE: %f %f %f %f %f %f", s->as<VectorState>(0)->values[0], s->as<VectorState>(0)->values[1],
                                             s->as<VectorState>(0)->values[2], s->as<VectorState>(0)->values[3], 
                                             s->as<VectorState>(0)->values[4], s->as<VectorState>(0)->values[5]);
    ROS_INFO("UPPER ARM ROLL STATE: %f %f", s->as<VectorState>(0)->values[6], s->as<VectorState>(0)->values[7]);

    ROS_INFO("TORSO STATE: %f", s->as<VectorState>(0)->values[8]);

    ROS_INFO("BASE STATE: %f %f %f", s->as<SE2State>(1)->getX(), s->as<SE2State>(1)->getY(), s->as<SE2State>(1)->getYaw());
}


bool EnvironmentMonolithic::convertFullState(const ompl::base::State* state, RobotState& robot_state, ContBaseState& base)
{
    ContObjectState obj_state;

    // fix the l_arm angles
    vector<double> init_l_arm(7,0);
    init_l_arm[0] = 0.200000;       
    init_l_arm[1] = 1.400000;
    init_l_arm[2] = 1.900000;
    init_l_arm[3] = -0.400000;
    init_l_arm[4] = -0.100000;
    init_l_arm[5] = -1.000000;
    init_l_arm[6] = 0.000000;

    vector<double> init_r_arm(7,0);

    const ompl::base::CompoundState* s = dynamic_cast<const ompl::base::CompoundState*> (state);

    init_r_arm[2] = (*(s->as<VectorState>(0)))[6];

    LeftContArmState l_arm(init_l_arm);
    RightContArmState r_arm(init_r_arm);

    obj_state.x((*(s->as<VectorState>(0)))[0]);
    obj_state.y((*(s->as<VectorState>(0)))[1]);
    obj_state.z((*(s->as<VectorState>(0)))[2]);
    obj_state.roll((*(s->as<VectorState>(0)))[3]);
    obj_state.pitch((*(s->as<VectorState>(0)))[4]);
    obj_state.yaw((*(s->as<VectorState>(0)))[5]);
    //r_arm.setUpperArmRoll((*(s->as<VectorState>(0)))[6]);
    //l_arm.setUpperArmRoll((*(s->as<VectorState>(0)))[7]);
    base.z((*(s->as<VectorState>(0)))[8]);
    base.x(s->as<SE2State>(1)->getX());
    base.y(s->as<SE2State>(1)->getY());
    base.theta(s->as<SE2State>(1)->getYaw());

    RobotState seed_state(base, r_arm, l_arm);
    RobotPosePtr final_state;

    if (!RobotState::computeRobotPose(DiscObjectState(obj_state), seed_state, final_state))
        return false;

    robot_state = *final_state;
    return true;

}

int EnvironmentMonolithic::getAdditionalCostMult(){
  return 1;
}