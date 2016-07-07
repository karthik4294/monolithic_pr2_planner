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


    bool within_basexy_tol = (abs(m_goal->getRobotState().base_state().x()-base.x()) < 50*d_tol.x() &&
                              abs(m_goal->getRobotState().base_state().y()-base.y()) < 50*d_tol.y());
    
    if(within_basexy_tol)
    { 
      ROS_INFO("[FBS] Search near goal");      

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

    ROS_DEBUG_NAMED(MPRIM_LOG, "interpolation for full body snap primitive");
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
        ROS_WARN("No valid arm interpolation found to snap to full body goal pose");
        return false;
    }
        
    ContBaseState start_base_state = source_state.robot_pose().base_state();
    ContBaseState end_base_state = successor.robot_pose().base_state();
    start_base_state.printToDebug(MPRIM_LOG);
    end_base_state.printToDebug(MPRIM_LOG);
    int num_interp_steps = static_cast<int>(interp_steps.size());

    double del_x = end_base_state.x() - start_base_state.x();
    double del_y = end_base_state.y() - start_base_state.y();
    double del_z = end_base_state.z() - start_base_state.z();
    double del_theta = shortest_angular_distance(start_base_state.theta(),
                                                 end_base_state.theta());
    
    vector<ContBaseState> interp_base_states;
    for (int i=0; i <= num_interp_steps; i++){
        ContBaseState interm_step = start_base_state;
        double rotate_by_angle = i*del_theta/num_interp_steps;
        double dx = i*del_x/num_interp_steps;
        double dy = i*del_y/num_interp_steps;
        double dz = i*del_z/num_interp_steps;
        interm_step.x(start_base_state.x() + dx);
        interm_step.y(start_base_state.y() + dy);
        interm_step.z(start_base_state.z() + dz);
        interm_step.theta(start_base_state.theta() + rotate_by_angle);
        interp_base_states.push_back(interm_step);
    }

    t_data.cont_base_interm_steps(interp_base_states);
    assert(interm_robot_steps.size() == cont_base_state_steps.size());

    return true;
}


void FullBodySnapMotionPrimitive::print() const {
    ROS_DEBUG_NAMED(MPRIM_LOG, 
                    "FullBodySnapMotionPrimitive cost %d", cost());
}

void FullBodySnapMotionPrimitive::computeCost(const MotionPrimitiveParams& params){
    //TODO: Calculate actual cost 
    m_cost = 1;
}
