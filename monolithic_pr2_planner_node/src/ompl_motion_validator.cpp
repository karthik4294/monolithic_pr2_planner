#include <monolithic_pr2_planner_node/ompl_motion_validator.h>

using namespace monolithic_pr2_planner;
using namespace std;
void omplFullBodyMotionValidator::initialize(CSpaceMgrPtr cspace, vector<double> l_arm){
    m_cspace = cspace;
    l_arm_init = l_arm;
}


bool omplFullBodyMotionValidator::checkMotion(const ompl::base::State* nstate, const ompl::base::State* dstate, std::pair<ompl::base::State*, double>& last_valid) const {

    if( (!m_si->isValid(nstate)) || (!m_si->isValid(dstate)))
        return false;

    RobotState n_robot_state, d_robot_state;
    ContBaseState n_base_state, d_base_state;

    if (!convertFullState(nstate, n_robot_state, n_base_state)) {
        return false;
    }

    if (!convertFullState(dstate, d_robot_state, d_base_state)) {
        return false;
    }

    std::vector<RobotState> interp_steps;

    bool w_interpolate = RobotState::workspaceInterpolate(n_robot_state, d_robot_state, &interp_steps); 

    if(w_interpolate)
    {
        bool valid = true;

        for (size_t i = 0; i < interp_steps.size(); i++)
        {
          RightContArmState rarm = interp_steps[i].right_arm(); 
          LeftContArmState larm = interp_steps[i].left_arm();
          ContBaseState base = interp_steps[i].getContBaseState();

          if(!m_cspace->isValid(base, rarm, larm) )
            valid = false;    
        }

        if(valid)
            return true;
    }

    interp_steps.clear();

    bool j_interpolate = RobotState::jointSpaceInterpolate(n_robot_state, d_robot_state, &interp_steps);

    if(j_interpolate)
    {
        for (size_t i = 0; i < interp_steps.size(); i++)
        {
          RightContArmState rarm = interp_steps[i].right_arm(); 
          LeftContArmState larm = interp_steps[i].left_arm();
          ContBaseState base = interp_steps[i].getContBaseState();

          if(!m_cspace->isValid(base, rarm, larm) )
            return false;
        }

        return true;
    }
    else
        return j_interpolate;

}

bool omplFullBodyMotionValidator::convertFullState(const ompl::base::State* state, RobotState& robot_state, ContBaseState& base) const
{
    ContObjectState obj_state;

    vector<double> init_r_arm(7,0);

    init_r_arm[2] = state->as<VectorState>()->values[6];

    LeftContArmState l_arm(l_arm_init);
    RightContArmState r_arm(init_r_arm);

    obj_state.x(state->as<VectorState>()->values[0]);
    obj_state.y(state->as<VectorState>()->values[1]);
    obj_state.z(state->as<VectorState>()->values[2]);
    obj_state.roll(state->as<VectorState>()->values[3]);
    obj_state.pitch(state->as<VectorState>()->values[4]);
    obj_state.yaw(state->as<VectorState>()->values[5]);
    base.z(state->as<VectorState>()->values[8]);
    base.x(state->as<VectorState>()->values[9]);
    base.y(state->as<VectorState>()->values[10]);
    base.theta(state->as<VectorState>()->values[11]);

    RobotState seed_state(base, r_arm, l_arm);
    RobotPosePtr final_state;

    if (!RobotState::computeRobotPose(DiscObjectState(obj_state), seed_state, final_state))
        return false;

    robot_state = *final_state;
    return true;
}