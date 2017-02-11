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

    const ompl::base::CompoundState* s = dynamic_cast<const ompl::base::CompoundState*> (state);

    init_r_arm[2] = (*(s->as<VectorState>(0)))[6];

    LeftContArmState l_arm(l_arm_init);
    RightContArmState r_arm(init_r_arm);

    obj_state.x((*(s->as<VectorState>(0)))[0]);
    obj_state.y((*(s->as<VectorState>(0)))[1]);
    obj_state.z((*(s->as<VectorState>(0)))[2]);
    obj_state.roll((*(s->as<VectorState>(0)))[3]);
    obj_state.pitch((*(s->as<VectorState>(0)))[4]);
    obj_state.yaw((*(s->as<VectorState>(0)))[5]);
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