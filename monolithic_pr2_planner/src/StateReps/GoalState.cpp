#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/Visualizer.h>

#define GOAL_STATE_ID 1

using namespace monolithic_pr2_planner;

//GoalState::GoalState(SearchRequestPtr search_request):
//    m_goal_state(search_request->m_params->obj_goal), m_tolerances(4,0){
//
//    m_tolerances[Tolerances::XYZ] = search_request->m_params->xyz_tolerance;
//    m_tolerances[Tolerances::ROLL] = search_request->m_params->roll_tolerance;
//    m_tolerances[Tolerances::PITCH] = search_request->m_params->pitch_tolerance;
//    m_tolerances[Tolerances::YAW] = search_request->m_params->yaw_tolerance;
//}

GoalState::GoalState(RobotState goal_state, double xyz_tol, 
                     double roll_tol, double pitch_tol, double yaw_tol):
    m_goal_state(goal_state), m_tolerances(4,0){

    m_tolerances[Tolerances::XYZ] = xyz_tol;
    m_tolerances[Tolerances::ROLL] = roll_tol;
    m_tolerances[Tolerances::PITCH] = pitch_tol;
    m_tolerances[Tolerances::YAW] = yaw_tol;
}

bool GoalState::withinXYZTol(const GraphStatePtr& graph_state){
    // not sure why there's a .005 here. ask ben
    ContObjectState c_tol(m_tolerances[Tolerances::XYZ]-.005, 
                          m_tolerances[Tolerances::XYZ]-.005, 
                          m_tolerances[Tolerances::XYZ]-.005,
                          m_tolerances[Tolerances::ROLL],
                          m_tolerances[Tolerances::PITCH],
                          m_tolerances[Tolerances::YAW]);
    DiscObjectState d_tol = c_tol.getDiscObjectState();
    
    RobotState robot_pose = graph_state->robot_pose();
    DiscBaseState base = robot_pose.base_state();

    DiscObjectState obj = graph_state->getObjectStateRelMap();


    bool within_xyz_tol = (abs(m_goal_state.getObjectStateRelMap().getDiscObjectState().x()-obj.x()) < d_tol.x() &&
                           abs(m_goal_state.getObjectStateRelMap().getDiscObjectState().y()-obj.y()) < d_tol.y() &&
                           abs(m_goal_state.getObjectStateRelMap().getDiscObjectState().z()-obj.z()) < d_tol.z());
    return within_xyz_tol;
}

bool GoalState::isSatisfiedBy(const GraphStatePtr& graph_state){
    // not sure why there's a .005 here. ask ben
    ContObjectState c_tol(m_tolerances[Tolerances::XYZ]-.005, 
                          m_tolerances[Tolerances::XYZ]-.005, 
                          m_tolerances[Tolerances::XYZ]-.005,
                          m_tolerances[Tolerances::ROLL],
                          m_tolerances[Tolerances::PITCH],
                          m_tolerances[Tolerances::YAW]);
    DiscObjectState d_tol = c_tol.getDiscObjectState();
    
    RobotState robot_pose = graph_state->robot_pose();
    DiscBaseState base = robot_pose.base_state();

    DiscObjectState obj = graph_state->getObjectStateRelMap();


    bool within_xyz_tol = (abs(m_goal_state.getObjectStateRelMap().getDiscObjectState().x()-obj.x()) < d_tol.x() &&
                           abs(m_goal_state.getObjectStateRelMap().getDiscObjectState().y()-obj.y()) < d_tol.y() &&
                           abs(m_goal_state.getObjectStateRelMap().getDiscObjectState().z()-obj.z()) < d_tol.z());
    // bool within_rpy_tol = (abs(m_goal_state.roll()-obj.roll()) < d_tol.roll() &&
    //                        abs(m_goal_state.pitch()-obj.pitch()) < d_tol.pitch() &&
    //                        abs(m_goal_state.yaw()-obj.yaw()) < d_tol.yaw());


    bool within_quat_tol;
    tf::Quaternion quat_goal;
    quat_goal.setRPY(m_goal_state.getObjectStateRelMap().getDiscObjectState().roll(),m_goal_state.getObjectStateRelMap().getDiscObjectState().pitch(),m_goal_state.getObjectStateRelMap().getDiscObjectState().yaw());
    tf::Quaternion quat_state;
    quat_state.setRPY(obj.roll(),obj.pitch(),obj.yaw());
    double diff = quat_state.angleShortestPath(quat_goal);

    // within_quat_tol = diff < d_tol.roll();      //should be another parameter d_tol.quat()
    within_quat_tol = diff < 0.1*d_tol.roll();      //should be another parameter d_tol.quat()


    bool within_basexyz_tol = (abs(m_goal_state.base_state().x()-base.x()) < d_tol.x() &&
                               abs(m_goal_state.base_state().y()-base.y()) < d_tol.y() &&
                               abs(m_goal_state.base_state().z()-base.z()) < d_tol.z());

    bool within_baseyaw_tol = (abs(m_goal_state.base_state().theta()-base.theta()) < d_tol.yaw());


    if (within_xyz_tol && within_quat_tol &&  within_basexyz_tol && within_baseyaw_tol){
        return true;
    } else {
        return false;
    }

    // if (within_xyz_tol && within_rpy_tol){
    //     return true;
    // } else {
    //     return false;
    // }
}

bool GoalState::isSatisfiedBy(const GraphStatePtr& graph_state, bool & search_near_goal){
      
    if(graph_state->id() == GOAL_STATE_ID)
      return true;

    // not sure why there's a .005 here. ask ben
    ContObjectState c_tol(m_tolerances[Tolerances::XYZ]-.005, 
                          m_tolerances[Tolerances::XYZ]-.005, 
                          m_tolerances[Tolerances::XYZ]-.005,
                          m_tolerances[Tolerances::ROLL],
                          m_tolerances[Tolerances::PITCH],
                          m_tolerances[Tolerances::YAW]);
    DiscObjectState d_tol = c_tol.getDiscObjectState();
    
    RobotState robot_pose = graph_state->robot_pose();
    DiscBaseState base = robot_pose.base_state();

    DiscObjectState obj = graph_state->getObjectStateRelMap();


    bool within_xyz_tol = (abs(m_goal_state.getObjectStateRelMap().getDiscObjectState().x()-obj.x()) < d_tol.x() &&
                           abs(m_goal_state.getObjectStateRelMap().getDiscObjectState().y()-obj.y()) < d_tol.y() &&
                           abs(m_goal_state.getObjectStateRelMap().getDiscObjectState().z()-obj.z()) < d_tol.z());
    // bool within_rpy_tol = (abs(m_goal_state.roll()-obj.roll()) < d_tol.roll() &&
    //                        abs(m_goal_state.pitch()-obj.pitch()) < d_tol.pitch() &&
    //                        abs(m_goal_state.yaw()-obj.yaw()) < d_tol.yaw());

    // ROS_INFO("Goal Endeff x: %d State Endeff x: %d tol : %d", m_goal_state.getObjectStateRelMap().getDiscObjectState().x(), obj.x(), d_tol.x());
    // ROS_INFO("Goal Endeff y: %d State Endeff y: %d tol : %d", m_goal_state.getObjectStateRelMap().getDiscObjectState().y(), obj.y(), d_tol.x());
    // ROS_INFO("Goal Endeff z: %d State Endeff z: %d tol : %d", m_goal_state.getObjectStateRelMap().getDiscObjectState().z(), obj.z(), d_tol.x());

    bool within_quat_tol;
    tf::Quaternion quat_goal;
    quat_goal.setRPY(m_goal_state.getObjectStateRelMap().getDiscObjectState().roll(),m_goal_state.getObjectStateRelMap().getDiscObjectState().pitch(),m_goal_state.getObjectStateRelMap().getDiscObjectState().yaw());
    tf::Quaternion quat_state;
    quat_state.setRPY(obj.roll(),obj.pitch(),obj.yaw());

    double diff = quat_state.angleShortestPath(quat_goal);

    within_quat_tol = diff < 0.1*d_tol.roll();      //should be another parameter d_tol.quat()

    // ROS_INFO("Goal Endeff quat: %f tol : %d", diff, d_tol.roll());

    bool within_basexy_tol = (abs(m_goal_state.base_state().x()-base.x()) < 1.5*d_tol.x() &&
                               abs(m_goal_state.base_state().y()-base.y()) < 1.5*d_tol.y());
    
    // ROS_INFO("Goal Base x: %d State Base x: %d tol : %d", m_goal_state.base_state().x(), base.x(), d_tol.x());
    // ROS_INFO("Goal Base y: %d State Base y: %d tol : %d", m_goal_state.base_state().y(), base.y(), d_tol.y());

    bool within_basez_tol = abs(m_goal_state.base_state().z()-base.z()) < d_tol.z();

    // ROS_INFO("Goal Base z: %d State Base z: %d tol : %d", m_goal_state.base_state().z(), base.z(), d_tol.z());

    bool within_baseyaw_tol = (abs(m_goal_state.base_state().theta()-base.theta()) < d_tol.yaw());

    // ROS_INFO("Goal Base yaw: %d State Base yaw: %d tol : %d", m_goal_state.base_state().theta(), base.theta(), d_tol.yaw());

    //ROS_INFO("within_xyz_tol %d within_quat_tol %d within_basexy_tol %d", within_xyz_tol, within_quat_tol, within_basexy_tol);

    if(within_xyz_tol && within_quat_tol && within_basexy_tol)
    { 
      ROS_INFO("[GoalState] Search near goal");
      search_near_goal = true;
    }
    // else{
    //   ROS_INFO("within_xyz_tol : %f  within_quat_tol : %f within_basexy_tol : %f", )
    // }

    if (within_xyz_tol && within_quat_tol &&  within_basexy_tol && within_basez_tol && within_baseyaw_tol){
        return true;
    } else {
        return false;
    }

    // if (within_xyz_tol && within_rpy_tol){
    //     return true;
    // } else {
    //     return false;
    // }
}

bool GoalState::isSolnStateID(int state_id){
    for (auto& goal : m_possible_goals){
        if (goal == state_id){
            return true;
        }
    }
    return false;
}
void GoalState::addPotentialSolnState(const GraphStatePtr& graph_state) { 
    m_possible_goals.push_back(graph_state->id());
}

void GoalState::visualize(){
    ContObjectState cont_goal = ContObjectState(m_goal_state.getObjectStateRelMap());
    std::vector<double> pose;
    pose.push_back(cont_goal.x());
    pose.push_back(cont_goal.y());
    pose.push_back(cont_goal.z());
    pose.push_back(cont_goal.roll());
    pose.push_back(cont_goal.pitch());
    pose.push_back(cont_goal.yaw());
    Visualizer::pviz->visualizePose(pose, "goal");
}
