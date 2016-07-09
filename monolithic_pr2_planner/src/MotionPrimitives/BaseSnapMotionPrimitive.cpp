#include <monolithic_pr2_planner/MotionPrimitives/BaseSnapMotionPrimitive.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <boost/shared_ptr.hpp>

using namespace monolithic_pr2_planner;

bool BaseSnapMotionPrimitive::apply(const GraphState& source_state, 
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

      RobotState rs(m_goal->getRobotState().getContBaseState(), source_state.robot_pose().right_arm(), source_state.robot_pose().left_arm());
      successor.reset(new GraphState(rs));

      t_data.motion_type(motion_type());
      t_data.cost(cost());
    
     return computeIntermSteps(source_state, *successor, t_data);
    }
    else{
        return false;
    } 
}

bool BaseSnapMotionPrimitive::computeIntermSteps(const GraphState& source_state, 
                        const GraphState& successor, 
                        TransitionData& t_data){

    ROS_DEBUG_NAMED(MPRIM_LOG, "interpolation for base snap primitive");

    ContBaseState start_base = source_state.robot_pose().getContBaseState();
    ContBaseState end_base = successor.robot_pose().getContBaseState();
    vector<ContBaseState> interp_base_steps;

    int num_interp_steps = numInterpSteps(source_state.robot_pose(), successor.robot_pose());

    interp_base_steps = ContBaseState::interpolate(start_base, end_base, 
                                                     num_interp_steps);

    t_data.cont_base_interm_steps(interp_base_steps);

    std::vector<RobotState> interp_steps(num_interp_steps, source_state.robot_pose());
    t_data.interm_robot_steps(interp_steps);

    return true;
}

void BaseSnapMotionPrimitive::print() const {
    ROS_DEBUG_NAMED(MPRIM_LOG, 
                    "BaseSnapMotionPrimitive cost %d", cost());
}

void BaseSnapMotionPrimitive::computeCost(const MotionPrimitiveParams& params){
    //TODO: Calculate actual cost 
    m_cost = 1;
}

int BaseSnapMotionPrimitive::numInterpSteps(const RobotState& start, const RobotState& end){

    ContBaseState start_base = start.base_state();
    ContBaseState end_base = end.base_state();
    
    double dbase_theta = shortest_angular_distance(start_base.theta(),
                                                   end_base.theta());

    double d_rot = fabs(dbase_theta);

    double d_base = ContBaseState::distance(ContBaseState(start.base_state()), 
                                         ContBaseState(end.base_state()));

    int rot_steps = static_cast<int>(d_rot/ContObjectState::getRPYResolution());
    int dist_steps = static_cast<int>(d_base/ContBaseState::getXYZResolution());

    int num_interp_steps = max(rot_steps, dist_steps);
    return num_interp_steps;
}