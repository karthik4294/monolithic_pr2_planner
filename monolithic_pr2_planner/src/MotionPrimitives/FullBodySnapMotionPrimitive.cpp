#include <monolithic_pr2_planner/MotionPrimitives/FullBodySnapMotionPrimitive.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <boost/shared_ptr.hpp>

using namespace monolithic_pr2_planner;

bool FullBodySnapMotionPrimitive::apply(const GraphState& source_state, 
                           GraphStatePtr& successor,
                           TransitionData& t_data){

    // not sure why there's a .005 here. ask ben
    ContObjectState c_tol(m_tolerances[Tolerances::XYZ]-.005, 
                          m_tolerances[Tolerances::XYZ]-.005, 
                          m_tolerances[Tolerances::XYZ]-.005,
                          m_tolerances[Tolerances::ROLL],
                          m_tolerances[Tolerances::PITCH],
                          m_tolerances[Tolerances::YAW]);
    DiscObjectState d_tol = c_tol.getDiscObjectState();
    
    RobotState robot_pose = source_state.robot_pose();
    DiscObjectState obj = source_state.getObjectStateRelMap();
    DiscBaseState base = robot_pose.base_state();
    unsigned int r_free_angle = robot_pose.right_free_angle();

    bool within_xyz_tol = (abs(m_goal->getObjectState().x()-obj.x()) < d_tol.x() &&
                           abs(m_goal->getObjectState().y()-obj.y()) < d_tol.y() &&
                           abs(m_goal->getObjectState().z()-obj.z()) < d_tol.z());

    // ROS_INFO("Goal Endeff x: %d State Endeff x: %d tol : %d", m_goal->getObjectState().x(), obj.x(), d_tol.x());
    // ROS_INFO("Goal Endeff y: %d State Endeff y: %d tol : %d", m_goal->getObjectState().y(), obj.y(), d_tol.x());
    // ROS_INFO("Goal Endeff z: %d State Endeff z: %d tol : %d", m_goal->getObjectState().z(), obj.z(), d_tol.x());


    bool within_basexy_tol = (abs(m_goal->getRobotState().base_state().x()-base.x()) < 1.5*d_tol.x() &&
                               abs(m_goal->getRobotState().base_state().y()-base.y()) < 1.5*d_tol.y());
    
    // ROS_INFO("Goal Base x: %d State Base x: %d tol : %d", m_goal->getRobotState().base_state().x(), base.x(), d_tol.x());
    // ROS_INFO("Goal Base y: %d State Base y: %d tol : %d", m_goal->getRobotState().base_state().y(), base.y(), d_tol.y());

    if(within_xyz_tol && within_basexy_tol)
    { 
      ROS_INFO("[FBS] Successor near goal");      

      RobotState rs = m_goal->getRobotState();
      successor.reset(new GraphState(rs));

      t_data.motion_type(motion_type());
      t_data.cost(cost());
    
     return computeIntermSteps(source_state, *successor, t_data);
    }
    else{
        return false;
    } 
}


bool FullBodySnapMotionPrimitive::computeIntermSteps(const GraphState& source_state, 
                        const GraphState& successor, 
                        TransitionData& t_data){
    std::vector<RobotState> interp_steps;
    bool interpolate = RobotState::workspaceInterpolate(source_state.robot_pose(), 
                                     successor.robot_pose(),
                                     &interp_steps);
    bool j_interpolate;

    if (!interpolate) {
        interp_steps.clear();
        j_interpolate = RobotState::jointSpaceInterpolate(source_state.robot_pose(),
                                    successor.robot_pose(), &interp_steps);
    }

    ROS_DEBUG_NAMED(MPRIM_LOG, "interpolation for full body snap primitive");
    for (auto robot_state: interp_steps){
        robot_state.printToDebug(MPRIM_LOG);
    }
    t_data.interm_robot_steps(interp_steps);

    // fill in the cont base steps to be the same throughout; this is an arm
    // only motion
    ContBaseState c_base = source_state.robot_pose().base_state();
    std::vector<ContBaseState> cont_base_states(interp_steps.size(), c_base);
    t_data.cont_base_interm_steps(cont_base_states);

    if(!interpolate && !j_interpolate)
    {
        ROS_WARN("No valid interpolation found to snap to full body goal pose");
        return false;
    }

    return true;
}


void FullBodySnapMotionPrimitive::print() const {
    ROS_DEBUG_NAMED(MPRIM_LOG, 
                    "FullBodySnapMotionPrimitive cost %d", cost());
}

void FullBodySnapMotionPrimitive::computeCost(const MotionPrimitiveParams& params){
    m_cost = 1;
}
