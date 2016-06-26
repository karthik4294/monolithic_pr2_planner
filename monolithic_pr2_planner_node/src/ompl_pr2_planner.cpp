#include <monolithic_pr2_planner_node/ompl_pr2_planner.h>
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/SearchRequest.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <tf_conversions/tf_kdl.h>
#include <kdl/frames.hpp>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <angles/angles.h>

using namespace monolithic_pr2_planner;
using namespace monolithic_pr2_planner_node;
ompl::base::OptimizationObjectivePtr getThresholdPathLengthObj(const ompl::base::SpaceInformationPtr& si,
    int planner_id)
{
    if(planner_id == RRTSTARFIRSTSOL_NUM || planner_id == BITSTARFIRSTSOL_NUM){
        ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(si));
	obj->setCostThreshold(ompl::base::Cost(10000000.51));        
        return obj;

    }
    else{
        ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(si));
	   //obj->setCostThreshold(ompl::base::Cost(10.51));               
        return obj;
    }
}

OMPLPR2Planner::OMPLPR2Planner(const CSpaceMgrPtr& cspace, int planner_id):
    m_planner_id(planner_id){
    //create the StateSpace (defines the dimensions and their bounds)
    ROS_INFO("initializing OMPL");
    ompl::base::SE2StateSpace* se2 = new ompl::base::SE2StateSpace();
    ompl::base::RealVectorBounds base_bounds(2);
    base_bounds.setLow(0,0);
    base_bounds.setHigh(0,10);//3
    base_bounds.setLow(1,0);
    base_bounds.setHigh(1,6);//3
    se2->setBounds(base_bounds);
    ompl::base::RealVectorStateSpace* r7 = new ompl::base::RealVectorStateSpace(9);
    r7->setDimensionName(0,"arms_x");
    r7->setDimensionName(1,"arms_y");
    r7->setDimensionName(2,"arms_z");
    r7->setDimensionName(3,"arms_roll");
    r7->setDimensionName(4,"arms_pitch");
    r7->setDimensionName(5,"arms_yaw");
    r7->setDimensionName(6,"free_angle_right");
    r7->setDimensionName(7,"free_angle_left");
    r7->setDimensionName(8,"torso");
    ompl::base::RealVectorBounds bounds(9);

    bounds.setLow(0,0.35);//arms_x
    bounds.setHigh(0,1.2);//arms_x
    bounds.setLow(1,-0.6);//arms_y
    bounds.setHigh(1,0.6);//arms_y
    bounds.setLow(2,-0.6);//arms_z
    bounds.setHigh(2,0.6);//arms_z

    // TODO may need to fix this!
    bounds.setLow(3,0);//arms_roll
    bounds.setHigh(3,2*M_PI);//arms_roll
    bounds.setLow(4,0);//arms_pitch
    bounds.setHigh(4,2*M_PI);//arms_pitch
    bounds.setLow(5,0);//arms_yaw
    bounds.setHigh(5,2*M_PI);//arms_yaw
    bounds.setLow(6,-3.75);//fa_right
    bounds.setHigh(6,0.65);//fa_right
    bounds.setLow(7,-0.65);//fa_left
    bounds.setHigh(7,3.75);//fa_left
    bounds.setLow(8,0); //torso
    bounds.setHigh(8,0.30); //torso
    r7->setBounds(bounds);
    ompl::base::StateSpacePtr se2_p(se2);
    ompl::base::StateSpacePtr r7_p(r7);
    fullBodySpace = r7_p + se2_p;
    
    //Define our SpaceInformation (combines the state space and collision checker)
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(fullBodySpace));
    m_si = si;

    vector<double> init_l_arm(7,0);
    init_l_arm[0] = (0.038946287971107774);
    init_l_arm[1] = (1.2146697069025374);
    init_l_arm[2] = (1.3963556492780154);
    init_l_arm[3] = -1.1972269899800325;
    init_l_arm[4] = (-4.616317135720829);
    init_l_arm[5] = -0.9887266887318599;
    init_l_arm[6] = 1.1755681069775656;

    m_collision_checker = new omplFullBodyCollisionChecker(si);
    m_collision_checker->initialize(cspace, init_l_arm);

    m_motion_validator = new omplFullBodyMotionValidator(si);
    m_motion_validator->initialize(cspace, init_l_arm);

    ompl::base::StateValidityChecker* temp2 = m_collision_checker;
    si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(temp2));
    si->setStateValidityCheckingResolution(0.002/si->getMaximumExtent());
    
    ompl::base::MotionValidator* temp3 = m_motion_validator;
    si->setMotionValidator(ompl::base::MotionValidatorPtr(temp3));

    si->setup();

    //Define a ProblemDefinition (a start/goal pair)
    pdef = new ompl::base::ProblemDefinition(si);

    if (planner_id == RRT_NUM)
        planner = new ompl::geometric::RRT(si);
    else if (planner_id == RRTCONNECT_NUM)
        planner = new ompl::geometric::RRTConnect(si);
    else if (planner_id == PRM_P_NUM)
        planner = new ompl::geometric::PRM(si);
    else if (planner_id == RRTSTAR_NUM || planner_id == RRTSTARFIRSTSOL_NUM)
        planner = new ompl::geometric::RRTstar(si);
    else if (planner_id == BITSTAR_NUM || planner_id == BITSTARFIRSTSOL_NUM)
        planner = new ompl::geometric::BITstar(si);
    else
        ROS_ERROR("invalid planner id!");

    if (planner_id == RRTSTAR_NUM || planner_id == RRTSTARFIRSTSOL_NUM || 
        planner_id == BITSTAR_NUM || planner_id == BITSTARFIRSTSOL_NUM){
        pdef->setOptimizationObjective(getThresholdPathLengthObj(si, planner_id));
    }
    planner->setProblemDefinition(ompl::base::ProblemDefinitionPtr(pdef));
    planner->setup();
    pathSimplifier = new ompl::geometric::PathSimplifier(si);
    ROS_INFO("finished initializing OMPL planner");
}

// given the start and goal from the request, create a start and goal that
// conform to the ompl types
bool OMPLPR2Planner::createStartGoal(FullState& ompl_start, FullState& ompl_goal, 
                                     SearchRequestParams& req){
    // ROS_INFO("createStartGoal received a start of ");
    LeftContArmState left_arm_start = req.left_arm_start;
    RightContArmState right_arm_start = req.right_arm_start;
    ContBaseState base_start = req.base_start;
    ContObjectState obj_state = right_arm_start.getObjectStateRelBody();

    (*(ompl_start->as<VectorState>(0)))[0] = obj_state.x();
    (*(ompl_start->as<VectorState>(0)))[1] = obj_state.y();
    (*(ompl_start->as<VectorState>(0)))[2] = obj_state.z();
    (*(ompl_start->as<VectorState>(0)))[3] = obj_state.roll();
    (*(ompl_start->as<VectorState>(0)))[4] = obj_state.pitch();
    (*(ompl_start->as<VectorState>(0)))[5] = obj_state.yaw();
    (*(ompl_start->as<VectorState>(0)))[6] = right_arm_start.getUpperArmRollAngle();
    (*(ompl_start->as<VectorState>(0)))[7] = left_arm_start.getUpperArmRollAngle();
    (*(ompl_start->as<VectorState>(0)))[8] = base_start.z();
    ompl_start->as<SE2State>(1)->setXY(base_start.x(),
                                       base_start.y());
    // may need to normalize the theta?
    double normalized_theta = angles::normalize_angle(base_start.theta());
    ompl_start->as<SE2State>(1)->setYaw(normalized_theta);
    // ROS_INFO("Start : obj xyzrpy (%f %f %f %f %f %f) base xyztheta (%f %f %f %f) Upper arm roll (%f %f)",
    //           obj_state.x(), obj_state.y(), obj_state.z(), obj_state.roll(), obj_state.pitch(), obj_state.yaw(),
    //           base_start.x(), base_start.y(), base_start.z(), normalized_theta,
    //           right_arm_start.getUpperArmRollAngle(), left_arm_start.getUpperArmRollAngle());

    ContObjectState goal_obj_state = req.right_arm_goal.getObjectStateRelBody();
    (*(ompl_goal->as<VectorState>(0)))[0] = goal_obj_state.x();
    (*(ompl_goal->as<VectorState>(0)))[1] = goal_obj_state.y();
    (*(ompl_goal->as<VectorState>(0)))[2] = goal_obj_state.z();
    (*(ompl_goal->as<VectorState>(0)))[3] = goal_obj_state.roll();
    (*(ompl_goal->as<VectorState>(0)))[4] = goal_obj_state.pitch();
    (*(ompl_goal->as<VectorState>(0)))[5] = goal_obj_state.yaw();
    (*(ompl_goal->as<VectorState>(0)))[6] = req.right_arm_goal.getUpperArmRollAngle();//req.rarm_goal[2];
    (*(ompl_goal->as<VectorState>(0)))[7] = req.left_arm_goal.getUpperArmRollAngle();//req.larm_goal[2];
    (*(ompl_goal->as<VectorState>(0)))[8] = req.base_goal.z();
    ompl_goal->as<SE2State>(1)->setXY(req.base_goal.x(),req.base_goal.y());
    normalized_theta = angles::normalize_angle(req.base_goal.theta());
    ompl_goal->as<SE2State>(1)->setYaw(normalized_theta);
    
    ROS_INFO("Goal : obj xyzrpy (%f %f %f %f %f %f) base xyztheta (%f %f %f %f) Upper arm roll (%f %f)",
          goal_obj_state.x(), goal_obj_state.y(), goal_obj_state.z(), goal_obj_state.roll(), goal_obj_state.pitch(), goal_obj_state.yaw(),
          req.base_goal.x(), req.base_goal.y(), req.base_goal.z(), normalized_theta,
          req.right_arm_goal.getUpperArmRollAngle(), req.left_arm_goal.getUpperArmRollAngle());

    return (planner->getSpaceInformation()->isValid(ompl_goal.get()) && 
            planner->getSpaceInformation()->isValid(ompl_start.get()));
}

// takes in an ompl state and returns a proper robot state that represents the
// same state.
bool OMPLPR2Planner::convertFullState(const ompl::base::State* state, monolithic_pr2_planner::RobotState& robot_state, monolithic_pr2_planner::ContBaseState& base)
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

bool OMPLPR2Planner::checkRequest(SearchRequestParams& search_request){
    planner->clear();
    planner->getProblemDefinition()->clearSolutionPaths();
    if (m_planner_id == PRM_P_NUM)
        planner->as<ompl::geometric::PRM>()->clearQuery();
    search_request.left_arm_start.getAngles(&m_collision_checker->l_arm_init);
    FullState ompl_start(fullBodySpace);
    FullState ompl_goal(fullBodySpace);
    return createStartGoal(ompl_start, ompl_goal, search_request);
}

bool OMPLPR2Planner::planPathCallback(SearchRequestParams& search_request, int trial_id,
    StatsWriter& m_stats_writer){
    if (m_planner_id == PRM_P_NUM)
        ROS_INFO("running PRM planner!");
    if (m_planner_id == RRT_NUM)
        ROS_INFO("running RRT planner!");
    if (m_planner_id == RRTSTAR_NUM || m_planner_id == RRTSTARFIRSTSOL_NUM)
        ROS_INFO("running RRTStar planner!");
    if (m_planner_id == BITSTAR_NUM || m_planner_id == BITSTARFIRSTSOL_NUM)
        ROS_INFO("running BITStar planner!");
    if (m_planner_id == RRTCONNECT_NUM)
        ROS_INFO("running RRTConnect planner!");
    planner->clear();
    planner->getProblemDefinition()->clearSolutionPaths();
    planner->as<ompl::geometric::PRM>()->clearQuery();
    search_request.left_arm_start.getAngles(&m_collision_checker->l_arm_init);
    FullState ompl_start(fullBodySpace);
    FullState ompl_goal(fullBodySpace);
    if (!createStartGoal(ompl_start, ompl_goal, search_request))
        return false;
    pdef->clearGoal();
    pdef->clearStartStates();
    pdef->setStartAndGoalStates(ompl_start,ompl_goal);
    ompl::base::GoalState* temp_goal = new ompl::base::GoalState(planner->getSpaceInformation());
    temp_goal->setState(ompl_goal);
    ompl::base::GoalPtr temp_goal2(temp_goal);

    // something about different planner types here
    //if(planner_id_==2 || planner_id_==3){
        pdef->setGoal(temp_goal2);
    //}
    double t0 = ros::Time::now().toSec();
    ROS_INFO("Allocated planning time %f", m_allocated_planning_time);
    ompl::base::PlannerStatus ompl_res = planner->solve(m_allocated_planning_time);
    double t1 = ros::Time::now().toSec();
    double planning_time = t1-t0;
    RRTData data;
    ompl::base::PathPtr path = planner->getProblemDefinition()->getSolutionPath();
    if (ompl_res.asString().compare("Exact solution") == 0 && path){
        ROS_INFO("OMPL found exact solution! Plan time : %f", planning_time);
        data.planned = true;
        ompl::geometric::PathGeometric geo_path = static_cast<ompl::geometric::PathGeometric&>(*path);
        double t2 = ros::Time::now().toSec();
        bool b1 = pathSimplifier->reduceVertices(geo_path);
        bool b2 = pathSimplifier->collapseCloseVertices(geo_path);
        bool b3 = pathSimplifier->shortcutPath(geo_path);
        
        //geo_path.interpolate();
        //ROS_ERROR("shortcut:%d\n",b3);
        
        double t3 = ros::Time::now().toSec();
        double reduction_time = t3-t2;

        data.plan_time = planning_time;
        data.shortcut_time = reduction_time;
        vector<RobotState> robot_states;
        vector<ContBaseState> base_states;

        for(unsigned int i=0; i<geo_path.getStateCount()-1; i++){
            ompl::base::State* state = geo_path.getState(i);
            ompl::base::State* next_state = geo_path.getState(i+1);

            RobotState robot_state, next_robot_state;
            ContBaseState base, next_base;
            std::vector<RobotState> interp_steps;

            if (convertFullState(state, robot_state, base) && convertFullState(next_state, next_robot_state, next_base)){
                
                bool w_interpolate = RobotState::workspaceInterpolate(robot_state, next_robot_state, &interp_steps); 

                if (!w_interpolate) {
                    interp_steps.clear();
                    bool j_interpolate = RobotState::jointSpaceInterpolate(robot_state, next_robot_state, &interp_steps);
                }

            }
            else{
                interp_steps.clear();
                bool j_interpolate = RobotState::jointSpaceInterpolate(robot_state, next_robot_state, &interp_steps);
            }

            vector<double> l_arm, r_arm;
            for(size_t  j = 0; j < interp_steps.size(); j++)
            {
                robot_states.push_back(interp_steps[j]);
                base_states.push_back(interp_steps[j].getContBaseState());

                interp_steps[j].right_arm().getAngles(&r_arm);
                interp_steps[j].left_arm().getAngles(&l_arm);
                BodyPose bp = interp_steps[j].getContBaseState().body_pose();
            
            }
            // Visualizer::pviz->visualizeRobot(r_arm, l_arm, bp, 150, "robot", 0);
            // usleep(5000);
        }
        data.robot_state = robot_states;
        data.base = base_states;
        data.path_length = robot_states.size();//geo_path.getStateCount();
        m_stats_writer.setPlannerId(m_planner_id);
        m_stats_writer.write(trial_id, data);
    } else {
        data.planned = false;
        ROS_ERROR("OMPL failed to plan in allocated time");
        return false;
    }

    return true;
}
