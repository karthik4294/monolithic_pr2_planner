#include <monolithic_pr2_planner_node/ompl_collision_checker.h>

using namespace monolithic_pr2_planner;
using namespace std;
void omplFullBodyCollisionChecker::initialize(CSpaceMgrPtr cspace, vector<double> l_arm){
    m_cspace = cspace;
    l_arm_init = l_arm;
}

bool omplFullBodyCollisionChecker::isValid(const ompl::base::State *state) const {

    RobotState robot_state;
    ContBaseState base;

    if (!convertFullState(state, robot_state, base)) {
        return false;
    }

    RightContArmState r_arm = robot_state.right_arm();
    LeftContArmState l_arm = robot_state.left_arm();
    
    if ( m_cspace->isValid(base, r_arm, l_arm)){
        return true;
    } else {
        return false;
    }
}

bool omplFullBodyCollisionChecker::convertFullState(const ompl::base::State* state, RobotState& robot_state, ContBaseState& base) const
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