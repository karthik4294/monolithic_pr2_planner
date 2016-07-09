#include <monolithic_pr2_planner/MotionPrimitives/ArmSnapMotionPrimitive.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <boost/shared_ptr.hpp>

using namespace monolithic_pr2_planner;

bool ArmSnapMotionPrimitive::apply(const GraphState& source_state, 
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


    bool within_basexy_tol = (abs(m_goal->getRobotState().base_state().x()-base.x()) < 25*d_tol.x() &&
                              abs(m_goal->getRobotState().base_state().y()-base.y()) < 25*d_tol.y());
    
    if(within_basexy_tol)
    { 
      //ROS_INFO("[FBS] Search near goal");      

      RobotState rs(source_state.robot_pose().getContBaseState(), m_goal->getRobotState().right_arm(), m_goal->getRobotState().left_arm());
      successor.reset(new GraphState(rs));

      t_data.motion_type(motion_type());
      t_data.cost(cost());
    
     return computeIntermSteps(source_state, *successor, t_data);
    }
    else{
        return false;
    } 
}

bool ArmSnapMotionPrimitive::computeIntermSteps(const GraphState& source_state, 
                        const GraphState& successor, 
                        TransitionData& t_data){

    ROS_DEBUG_NAMED(MPRIM_LOG, "interpolation for arm snap primitive");
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

    for (auto robot_state: interp_steps){
        robot_state.printToDebug(MPRIM_LOG);
    }
    t_data.interm_robot_steps(interp_steps);

    if(!interpolate && !j_interpolate)
    {
        ROS_WARN("No valid arm interpolation found to snap arm to goal pose");
        return false;
    }

    ContBaseState c_base = source_state.robot_pose().getContBaseState();
    std::vector<ContBaseState> cont_base_states(interp_steps.size(), c_base);
    t_data.cont_base_interm_steps(cont_base_states);

    return true;
}

void ArmSnapMotionPrimitive::print() const {
    ROS_DEBUG_NAMED(MPRIM_LOG, 
                    "ArmSnapMotionPrimitive cost %d", cost());
}

void ArmSnapMotionPrimitive::computeCost(const MotionPrimitiveParams& params){
    //TODO: Calculate actual cost 
    m_cost = 1;
}
