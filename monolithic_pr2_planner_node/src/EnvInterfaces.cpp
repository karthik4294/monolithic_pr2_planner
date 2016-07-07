#include <monolithic_pr2_planner_node/EnvInterfaces.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>
#include <monolithic_pr2_planner/Constants.h>
#include <kdl/frames.hpp>
#include <boost/filesystem.hpp>
#include <geometry_msgs/Pose.h>
#include <leatherman/utils.h>
#include <tf2/LinearMath/btVector3.h>
#include <climits>

#define EPS 100.0

using namespace monolithic_pr2_planner_node;
using namespace monolithic_pr2_planner;
using namespace boost;
using namespace std;
using namespace KDL;

// constructor automatically launches the collision space interface, which only
// loads it up with a pointer to the collision space mgr. it doesn't bind to any
// topic.

EnvInterfaces::EnvInterfaces(
  boost::shared_ptr<monolithic_pr2_planner::Environment> env,
  ros::NodeHandle nh) :
  m_nodehandle(nh),
  m_env(env), m_collision_space_interface(new CollisionSpaceInterface(
                                            env->getCollisionSpace(), env->getHeuristicMgr())),
  m_generator(new StartGoalGenerator(env->getCollisionSpace()))
  // m_rrt(new OMPLPR2Planner(env->getCollisionSpace(), RRT)),
  // m_prm(new OMPLPR2Planner(env->getCollisionSpace(), PRM_P)),
  // m_rrtstar(new OMPLPR2Planner(env->getCollisionSpace(), RRTSTAR)),
  // m_rrtstar_first_sol(new OMPLPR2Planner(env->getCollisionSpace(),
  // RRTSTARFIRSTSOL))
{

  m_collision_space_interface->mutex = &mutex;

  getParams();
  bool forward_search = true;

  m_ara_planner.reset(new ARAPlanner(m_env.get(), forward_search));
  
  m_mha_planner.reset(new MHAPlanner(m_env.get(), NUM_SMHA_HEUR,
                                     forward_search));
  m_costmap_pub = m_nodehandle.advertise<nav_msgs::OccupancyGrid>("costmap_pub",
                                                                  1);
  m_costmap_publisher.reset(new
                            costmap_2d::Costmap2DPublisher(m_nodehandle, 1, "/map"));

  interrupt_sub_ = nh.subscribe("/sbpl_planning/interrupt", 1,
                                &EnvInterfaces::interruptPlannerCallback, this);
}

EnvInterfaces::EnvInterfaces(
  boost::shared_ptr<monolithic_pr2_planner::EnvironmentMonolithic> env,
  ros::NodeHandle nh) :
  m_nodehandle(nh),
  m_mon_env(env), m_collision_space_interface(new CollisionSpaceInterface(
                                            env->getCollisionSpace(), env->getHeuristicMgr())),
  m_generator(new StartGoalGenerator(env->getCollisionSpace()))
  // m_rrt(new OMPLPR2Planner(env->getCollisionSpace(), RRT)),
  // m_prm(new OMPLPR2Planner(env->getCollisionSpace(), PRM_P)),
  // m_rrtstar(new OMPLPR2Planner(env->getCollisionSpace(), RRTSTAR)),
  // m_rrtstar_first_sol(new OMPLPR2Planner(env->getCollisionSpace(),
  // RRTSTARFIRSTSOL))
{

  m_collision_space_interface->mutex = &mutex;

  getParams();
  
  m_costmap_pub = m_nodehandle.advertise<nav_msgs::OccupancyGrid>("costmap_pub",
                                                                  1);
  m_costmap_publisher.reset(new
                            costmap_2d::Costmap2DPublisher(m_nodehandle, 1, "/map"));

  interrupt_sub_ = nh.subscribe("/sbpl_planning/interrupt", 1,
                                &EnvInterfaces::interruptPlannerCallback, this);
}

void EnvInterfaces::ompl_spi_init(const monolithic_pr2_planner::CSpaceMgrPtr& cspace) {
    //ROS_INFO("initializing OMPL");
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
    si_.reset(new ompl::base::SpaceInformation(fullBodySpace));

    vector<double> init_l_arm(7,0);
    init_l_arm[0] = (0.038946287971107774);
    init_l_arm[1] = (1.2146697069025374);
    init_l_arm[2] = (1.3963556492780154);
    init_l_arm[3] = -1.1972269899800325;
    init_l_arm[4] = (-4.616317135720829);
    init_l_arm[5] = -0.9887266887318599;
    init_l_arm[6] = 1.1755681069775656;

    m_collision_checker = new omplFullBodyCollisionChecker(si_);
    m_collision_checker->initialize(cspace, init_l_arm);

    ompl::base::StateValidityChecker* temp2 = m_collision_checker;
    si_->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(temp2));
    si_->setStateValidityCheckingResolution(0.002/si_->getMaximumExtent());
    si_->setup();

    pdef_.reset(new ompl::base::ProblemDefinition(si_));

}

void EnvInterfaces::interruptPlannerCallback(std_msgs::EmptyConstPtr) {
  ROS_INFO("Planner interrupt received!");
  m_ppma_planner->interrupt();
}

void EnvInterfaces::getParams() {
  m_nodehandle.param<string>("reference_frame", m_params.ref_frame,
                             string("map"));
  m_nodehandle.param<bool>("run_trajectory", m_params.run_trajectory, false);
  m_nodehandle.param<string>("controller_service",
                             m_params.controller_service, "/monolithic_controller/execute_path");
}

void EnvInterfaces::bindPlanPathToEnv(string service_name) {
  m_plan_service = m_nodehandle.advertiseService(service_name,
                                                 &EnvInterfaces::planPathCallback,
                                                 this);
}

void EnvInterfaces::bindExperimentToEnv(string service_name) {
  m_experiment_service = m_nodehandle.advertiseService(service_name,
                                                       &EnvInterfaces::experimentCallback,
                                                       this);
}

void EnvInterfaces::bindWriteExperimentToEnv(string service_name) {
  m_write_experiments_service = m_nodehandle.advertiseService(service_name,
                                                              &EnvInterfaces::GenerateExperimentFile,
                                                              this);
}

void EnvInterfaces::bindDemoToEnv(string service_name) {
  m_demo_service = m_nodehandle.advertiseService(service_name,
                                                 &EnvInterfaces::demoCallback,
                                                 this);
}

//making experiments:
//launch file brings up stlToOctomap with rosparams that tell it to randomize environment
//addTableObstacles
//randomizeTableObstacles - if this is false we have to provide the next one (when true we have to write configuration to file)
//pathToTableObstacleParamFile (this file has num_surfaces, num_obs, and seed)
//
//service call is made to this function with the number of trials to make
//this guy makes random trials with checks for rrt-connect feasibility
//it writes the start/goal pairs to file (in the yaml format)
//
//////////
//
//running experiments:
//stlToOctomap is launched with rosparams (and table config file) to re-create the env
//runTests is run with the yaml file
bool EnvInterfaces::GenerateExperimentFile(std_srvs::Empty::Request &req,
                                           std_srvs::Empty::Response &res) {
  ROS_INFO("generating trials!");
  vector<pair<RobotState, RobotState>> start_goal_pairs;
  RobotState::setPlanningMode(PlanningModes::RIGHT_ARM_MOBILE);
  int number_of_trials = 10;
  m_generator->initializeRegions();//reads from ros params set by stlToOctomap
  m_generator->generateUniformPairs(number_of_trials, start_goal_pairs);

  int test_num = 0;
  FILE *fout = fopen("fbp_tests.yaml", "w");
  fprintf(fout, "experiments:\n\n");

  for (auto &start_goal : start_goal_pairs) {
    geometry_msgs::Quaternion start_obj_q;
    leatherman::rpyToQuatMsg(start_goal.first.getObjectStateRelMap().roll(),
                             start_goal.first.getObjectStateRelMap().pitch(),
                             start_goal.first.getObjectStateRelMap().yaw(),
                             start_obj_q);
    geometry_msgs::Quaternion goal_obj_q;
    leatherman::rpyToQuatMsg(start_goal.second.getObjectStateRelMap().roll(),
                             start_goal.second.getObjectStateRelMap().pitch(),
                             start_goal.second.getObjectStateRelMap().yaw(),
                             goal_obj_q);

    fprintf(fout, "  - test: test_%d\n", test_num);
    fprintf(fout, "    start:\n");
    fprintf(fout, "      object_xyz_wxyz: %f %f %f %f %f %f %f\n",
            start_goal.first.getObjectStateRelMap().x(),
            start_goal.first.getObjectStateRelMap().y(),
            start_goal.first.getObjectStateRelMap().z(),
            start_obj_q.w, start_obj_q.x, start_obj_q.y, start_obj_q.z);
    fprintf(fout, "      base_xyzyaw: %f %f %f %f\n",
            start_goal.first.getContBaseState().x(),
            start_goal.first.getContBaseState().y(),
            start_goal.first.getContBaseState().z(),
            start_goal.first.getContBaseState().theta());
    fprintf(fout, "      rarm: %f %f %f %f %f %f %f\n",
            start_goal.first.right_arm().getShoulderPanAngle(),
            start_goal.first.right_arm().getShoulderLiftAngle(),
            start_goal.first.right_arm().getUpperArmRollAngle(),
            start_goal.first.right_arm().getElbowFlexAngle(),
            start_goal.first.right_arm().getForearmRollAngle(),
            start_goal.first.right_arm().getWristFlexAngle(),
            start_goal.first.right_arm().getWristRollAngle());
    fprintf(fout, "      larm: %f %f %f %f %f %f %f\n",
            start_goal.first.left_arm().getShoulderPanAngle(),
            start_goal.first.left_arm().getShoulderLiftAngle(),
            start_goal.first.left_arm().getUpperArmRollAngle(),
            start_goal.first.left_arm().getElbowFlexAngle(),
            start_goal.first.left_arm().getForearmRollAngle(),
            start_goal.first.left_arm().getWristFlexAngle(),
            start_goal.first.left_arm().getWristRollAngle());
    fprintf(fout, "    goal:\n");
    fprintf(fout, "      object_xyz_wxyz: %f %f %f %f %f %f %f\n",
            start_goal.second.getObjectStateRelMap().x(),
            start_goal.second.getObjectStateRelMap().y(),
            start_goal.second.getObjectStateRelMap().z(),
            start_obj_q.w, start_obj_q.x, start_obj_q.y, start_obj_q.z);
    fprintf(fout, "      base_xyzyaw: %f %f %f %f\n",
            start_goal.second.getContBaseState().x(),
            start_goal.second.getContBaseState().y(),
            start_goal.second.getContBaseState().z(),
            start_goal.second.getContBaseState().theta());
    fprintf(fout, "      rarm: %f %f %f %f %f %f %f\n",
            start_goal.second.right_arm().getShoulderPanAngle(),
            start_goal.second.right_arm().getShoulderLiftAngle(),
            start_goal.second.right_arm().getUpperArmRollAngle(),
            start_goal.second.right_arm().getElbowFlexAngle(),
            start_goal.second.right_arm().getForearmRollAngle(),
            start_goal.second.right_arm().getWristFlexAngle(),
            start_goal.second.right_arm().getWristRollAngle());
    fprintf(fout, "      larm: %f %f %f %f %f %f %f\n",
            start_goal.second.left_arm().getShoulderPanAngle(),
            start_goal.second.left_arm().getShoulderLiftAngle(),
            start_goal.second.left_arm().getUpperArmRollAngle(),
            start_goal.second.left_arm().getElbowFlexAngle(),
            start_goal.second.left_arm().getForearmRollAngle(),
            start_goal.second.left_arm().getWristFlexAngle(),
            start_goal.second.left_arm().getWristRollAngle());
    fprintf(fout, "\n");
    test_num++;
  }

  fclose(fout);
  return true;
}

/*! \brief this is callback is purely for simulation purposes
 */
bool EnvInterfaces::experimentCallback(GetMobileArmPlan::Request &req,
                                       GetMobileArmPlan::Response &res) {
  ROS_INFO("running simulations!");
  vector<pair<RobotState, RobotState>> start_goal_pairs;
  RobotState::setPlanningMode(PlanningModes::RIGHT_ARM_MOBILE);
  int number_of_trials = 10;
  int counter = 0;

  while (counter < number_of_trials) {
    m_generator->initializeRegions();
    m_generator->generateUniformPairs(number_of_trials, start_goal_pairs);

    for (auto &start_goal : start_goal_pairs) {
      ROS_ERROR("running trial %d", counter);
      // start_goal.first.visualize();
      SearchRequestParamsPtr search_request =
        make_shared<SearchRequestParams>();
      search_request->initial_epsilon = req.initial_eps;
      search_request->final_epsilon = req.final_eps;
      search_request->decrement_epsilon = req.dec_eps;
      search_request->obj_goal = start_goal.second.getObjectStateRelMap();
      search_request->base_start = start_goal.first.base_state();
      search_request->base_goal = start_goal.second.base_state();
      search_request->left_arm_start = start_goal.first.left_arm();
      search_request->right_arm_start = start_goal.first.right_arm();
      search_request->left_arm_goal = start_goal.second.left_arm();
      search_request->right_arm_goal = start_goal.second.right_arm();
      search_request->obj_goal = start_goal.second.getObjectStateRelMap();

      KDL::Frame rarm_offset, larm_offset;
      rarm_offset.p.x(req.rarm_object.pose.position.x);
      rarm_offset.p.y(req.rarm_object.pose.position.y);
      rarm_offset.p.z(req.rarm_object.pose.position.z);
      larm_offset.p.x(req.larm_object.pose.position.x);
      larm_offset.p.y(req.larm_object.pose.position.y);
      larm_offset.p.z(req.larm_object.pose.position.z);

      rarm_offset.M = Rotation::Quaternion(
                        req.rarm_object.pose.orientation.x,
                        req.rarm_object.pose.orientation.y,
                        req.rarm_object.pose.orientation.z,
                        req.rarm_object.pose.orientation.w);
      larm_offset.M = Rotation::Quaternion(
                        req.larm_object.pose.orientation.x,
                        req.larm_object.pose.orientation.y,
                        req.larm_object.pose.orientation.z,
                        req.larm_object.pose.orientation.w);
      search_request->left_arm_object = larm_offset;
      search_request->right_arm_object = rarm_offset;
      search_request->xyz_tolerance = req.xyz_tolerance;
      search_request->roll_tolerance = req.roll_tolerance;
      search_request->pitch_tolerance = req.pitch_tolerance;
      search_request->yaw_tolerance = req.yaw_tolerance;
      search_request->planning_mode = req.planning_mode;

      res.stats_field_names.resize(18);
      res.stats.resize(18);
      int start_id, goal_id;
      bool return_first_soln = true;
      bool forward_search = true;
      clock_t total_planning_time;
      bool isPlanFound;
      vector<double> stats;
      vector<string> stat_names;
      vector<FullBodyState> states;
      vector<int> soln;
      int soln_cost;

      int environment_seed;

      m_nodehandle.getParam("/monolithic_pr2_planner_node/experiments/seed",
                            environment_seed);

      if (!m_ompl_planner->checkRequest(*search_request)) {
        ROS_WARN("bad start goal for ompl");
      } else {
        // Here starts the actual planning requests
        start_goal.first.visualize();
        if( (req.planner_type == mha_planner::PlannerType::SMHA || req.planner_type == mha_planner::PlannerType::IMHA)  && (!req.use_ompl) ){
          runMHAPlanner(monolithic_pr2_planner::T_SMHA, "smha_", req, res,
                      search_request, counter);
          runMHAPlanner(monolithic_pr2_planner::T_IMHA, "imha_", req, res,
                      search_request, counter);
        }else{

          runPPMAPlanner(monolithic_pr2_planner::T_IMHA, "ppma", req, res,
                      search_request, counter);
        }
        // Write env if the whole thing didn't crash.
        m_stats_writer.writeStartGoal(counter, start_goal, environment_seed);
        counter++;
      }
    }
  }

  return true;
}

bool EnvInterfaces::runMHAPlanner(int planner_type,
                                  std::string planner_prefix,
                                  GetMobileArmPlan::Request &req,
                                  GetMobileArmPlan::Response &res,
                                  SearchRequestParamsPtr search_request,
                                  int counter) {
  // std::cin.get();
  int start_id, goal_id;
  bool return_first_soln = true;
  bool forward_search = true;
  clock_t total_planning_time;
  bool isPlanFound;
  vector<double> stats;
  vector<string> stat_names;
  vector<FullBodyState> states;

  /*
  int planner_queues = NUM_SMHA_HEUR;
  if (planner_type == monolithic_pr2_planner::T_EES)
      planner_queues = 3;
  else if (planner_type == monolithic_pr2_planner::T_IMHA)
      planner_queues = NUM_IMHA_HEUR;
      */


  ros::NodeHandle ph("~");
  bool use_new_heuristics;
  ph.param("use_new_heuristics", use_new_heuristics, false);
  int planner_queues;

  if (!use_new_heuristics) {
    planner_queues = 4;
  } else {
    planner_queues = 20;
  }

  printf("\n");
  ROS_INFO("Initialize environment");
  m_env->reset();
  m_env->setPlannerType(planner_type);
  m_env->setUseNewHeuristics(use_new_heuristics);
  m_mha_planner.reset(new MHAPlanner(m_env.get(), planner_queues,
                                     forward_search));
  total_planning_time = clock();
  ROS_INFO("configuring request");

  if (!m_env->configureRequest(search_request, start_id, goal_id)) {
    ROS_ERROR("Unable to configure request for %s! Trial ID: %d",
              planner_prefix.c_str(), counter);

    total_planning_time = -1;
    int soln_cost = -1;
    packageMHAStats(stat_names, stats, soln_cost, 0, total_planning_time);

    for (unsigned int i = 0; i < stats.size(); i++) {
      stats[i] = -1;
    }

    m_stats_writer.writeSBPL(stats, states, counter, planner_prefix);
    res.stats_field_names = stat_names;
    res.stats = stats;
    return true;
  }

  if (req.use_ompl) {
    ROS_INFO("rrt init");
    RobotState::setPlanningMode(PlanningModes::RIGHT_ARM_MOBILE);
    m_ompl_planner.reset(new OMPLPR2Planner(m_env->getCollisionSpace(), req.planner_type));
    ROS_INFO("rrt check request");

    if (!m_ompl_planner->checkRequest(*search_request)) {
      ROS_WARN("bad start goal for ompl");
    }

    ROS_INFO("rrt plan");
    m_ompl_planner->setPlanningTime(req.allocated_planning_time);
    double t0 = ros::Time::now().toSec();
    bool found_path = m_ompl_planner->planPathCallback(*search_request, counter,
                                              m_stats_writer);
    double t1 = ros::Time::now().toSec();
    ROS_INFO("rrt done planning");

    res.stats_field_names.clear();
    res.stats_field_names.push_back("total_plan_time");
    res.stats.clear();

    if (found_path) {
      res.stats.push_back(t1 - t0);
    } else {
      res.stats.push_back(-1.0);
    }

    sleep(5);
    return true;
  } else {
    m_mha_planner->set_start(start_id);
    ROS_INFO("setting %s goal id to %d", planner_prefix.c_str(), goal_id);
    m_mha_planner->set_goal(goal_id);
    m_mha_planner->force_planning_from_scratch();
    vector<int> soln;
    int soln_cost;
    ROS_INFO("allocated time is %f", req.allocated_planning_time);
    MHAReplanParams replan_params(req.allocated_planning_time);

    replan_params.meta_search_type = static_cast<mha_planner::MetaSearchType>
                                     (req.meta_search_type);
    replan_params.planner_type = static_cast<mha_planner::PlannerType>
                                 (req.planner_type);
    replan_params.mha_type = static_cast<mha_planner::MHAType>(req.mha_type);

    if (replan_params.mha_type == mha_planner::MHAType::ORIGINAL) {
      if (EPS >= 2.0) {
        replan_params.inflation_eps = EPS / 2.0;
        replan_params.anchor_eps = 2.0;
      } else {
        replan_params.inflation_eps = sqrt(EPS);
        replan_params.anchor_eps = sqrt(EPS);
      }
    } else {
      replan_params.inflation_eps = EPS;
      replan_params.anchor_eps = 1.0;
    }

    replan_params.use_anchor = true;
    replan_params.return_first_solution = false;
    replan_params.final_eps = EPS;

    //isPlanFound = m_mha_planner->replan(req.allocated_planning_time,
    //                                     &soln, &soln_cost);
    isPlanFound = m_mha_planner->replan(&soln, replan_params, &soln_cost);

    if (isPlanFound) {
      ROS_INFO("Plan found in %s Planner. Moving on to reconstruction.",
               planner_prefix.c_str());
      states =  m_env->reconstructPath(soln);
      total_planning_time = clock() - total_planning_time;
      packageMHAStats(stat_names, stats, soln_cost, states.size(),
                      total_planning_time);
      m_stats_writer.writeSBPL(stats, states, counter, planner_prefix);
      res.stats_field_names = stat_names;
      res.stats = stats;
      ROS_INFO("Reconstruction Done!!!");
    } else {
      packageMHAStats(stat_names, stats, soln_cost, states.size(),
                      total_planning_time);
      res.stats_field_names = stat_names;
      res.stats = stats;
      ROS_INFO("No plan found in %s!", planner_prefix.c_str());
    }

    if (m_params.run_trajectory) {
      ROS_INFO("Running trajectory!");
      //runTrajectory(states); Commented out to compile without driver.
    }

    return true;
  }
}

bool EnvInterfaces::runPPMAPlanner(int planner_type,
                                  std::string planner_prefix,
                                  GetMobileArmPlan::Request &req,
                                  GetMobileArmPlan::Response &res,
                                  SearchRequestParamsPtr search_request,
                                  int counter) {
  int start_id, goal_id;
  bool return_first_soln = false;
  bool forward_search = true;
  clock_t total_planning_time;
  bool isPlanFound;
  vector<double> stats;
  vector<string> stat_names;
  vector<FullBodyState> states;

  ros::NodeHandle ph("~");
 
  printf("\n");
  ROS_INFO("Initialize environment");
  m_mon_env->reset();
  m_mon_env->setPlannerType(planner_type);
  //m_mon_env->setUseNewHeuristics(use_new_heuristics);

  RobotState::setPlanningMode(PlanningModes::RIGHT_ARM_MOBILE);


  m_rrt.reset(new OMPLPR2Planner(m_mon_env->getCollisionSpace(), RRT_NUM));
  // ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(*m_rrt->GetSpaceInformationPtr()));
  ompl::base::SpaceInformationPtr si = m_rrt->GetSpaceInformationPtr();

  // LeftContArmState left_arm_goal = search_request->left_arm_goal;
  // RightContArmState right_arm_goal = search_request->right_arm_goal;
  // ContBaseState base_goal = search_request->base_goal;

  // std::vector<double> l_arm, r_arm;
  // right_arm_goal.getAngles(&r_arm);
  // left_arm_goal.getAngles(&l_arm);
  // BodyPose bp = base_goal.body_pose();

  // Visualizer::pviz->visualizeRobot(r_arm, l_arm, bp, 150, "robot", 0);
  // getchar();

  ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
  pathSimplifier = new ompl::geometric::PathSimplifier(si);
  m_full_body_space = m_rrt->GetStateSpacePtr();

  FullState ompl_start(m_full_body_space);
  FullState ompl_goal(m_full_body_space);
  if (!m_rrt->createStartGoal(ompl_start, ompl_goal, *search_request))                                                            
    return false;
  
  pdef->clearGoal();
  pdef->clearStartStates();
  pdef->setStartAndGoalStates(ompl_start,ompl_goal); 
  pdef->setOptimizationObjective(ompl::base::OptimizationObjectivePtr(new ompl::base::PathLengthOptimizationObjective(si)));

  PPMAReplanParams ppma_replan_params(req.allocated_planning_time);
  ppma_replan_params.initial_eps = EPS;
  ppma_replan_params.final_eps = EPS;
  ppma_replan_params.return_first_solution = return_first_soln;
  ppma_replan_params.planner_mode = static_cast<ppma_planner::PlannerMode>(req.planner_type);

  m_ppma_planner.reset(new PPMAPlanner(si, m_mon_env.get(), forward_search, req.allocated_planning_time, &ppma_replan_params));
  m_ppma_planner->setup();
  m_ppma_planner->setProblemDefinition(ompl::base::ProblemDefinitionPtr(pdef));

  ompl::base::GoalState* temp_goal = new ompl::base::GoalState(m_ppma_planner->getSpaceInformation());
  temp_goal->setState(ompl_goal);
  ompl::base::GoalPtr temp_goal2(temp_goal);  

  total_planning_time = clock();
  ROS_INFO("configuring request");

  if (!m_mon_env->configureRequest(search_request, start_id, goal_id)) {
    ROS_ERROR("Unable to configure request for %s! Trial ID: %d",
              planner_prefix.c_str(), counter);

    total_planning_time = -1;
    int soln_cost = -1;
    packageMHAStats(stat_names, stats, soln_cost, 0, total_planning_time);

    for (unsigned int i = 0; i < stats.size(); i++) {
      stats[i] = -1;
    }

    m_stats_writer.writeSBPL(stats, states, counter, planner_prefix);
    res.stats_field_names = stat_names;
    res.stats = stats;
    return true;
  }

  if (req.use_ompl) {
    ROS_INFO("OMPL planner init : %d", req.planner_type);
    RobotState::setPlanningMode(PlanningModes::RIGHT_ARM_MOBILE);
    m_ompl_planner.reset(new OMPLPR2Planner(m_mon_env->getCollisionSpace(), static_cast<int>(req.planner_type) ));
    ROS_INFO("ompl check request");

    if (!m_ompl_planner->checkRequest(*search_request)) {
      ROS_WARN("bad start goal for ompl");
    }

    m_ompl_planner->setPlanningTime(req.allocated_planning_time);
    double t0 = ros::Time::now().toSec();
    bool found_path = m_ompl_planner->planPathCallback(*search_request, counter,
                                              m_stats_writer);
    double t1 = ros::Time::now().toSec();

    res.stats_field_names.clear();
    res.stats_field_names.push_back("total_plan_time");
    res.stats.clear();

    if (found_path) {
      res.stats.push_back(t1 - t0);
    } else {
      res.stats.push_back(-1.0);
    }

    sleep(5);
    return true;
  } else {

    //m_ppma_planner->setGoalBias(0.5);
    //m_ppma_planner->setRange();

    m_ppma_planner->set_start(start_id);
    ROS_INFO("setting %s goal id to %d", planner_prefix.c_str(), goal_id);
    m_ppma_planner->set_goal(goal_id);
    m_ppma_planner->force_planning_from_scratch();
    vector<int> soln;
    int soln_cost;
    RRTData data;

    ROS_INFO("allocated time is %f", req.allocated_planning_time);

    double totalTime;
    isPlanFound = m_ppma_planner->replan(&soln, ppma_replan_params, &soln_cost, totalTime);

    if (isPlanFound) {
        
        ROS_INFO("Plan found in %s Planner. Moving on to reconstruction.", planner_prefix.c_str());

        ompl::base::PathPtr path = m_ppma_planner->getProblemDefinition()->getSolutionPath();
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

        // data.plan_time = totalTime;
        // data.shortcut_time = reduction_time;

        // vector<RobotState> robot_states;
        // vector<ContBaseState> base_states;

        // for(unsigned int i=0; i<geo_path.getStateCount(); i++){
        //     ompl::base::State* state = geo_path.getState(i);
            
        //     RobotState robot_state;
        //     ContBaseState base;
            
        //     if (!convertFullState(state, robot_state, base)){
        //         ROS_ERROR("ik failed on path reconstruction!");
        //     }
        //     vector<double> l_arm, r_arm;
        //     robot_states.push_back(robot_state);
        //     base_states.push_back(base);

        //     robot_state.right_arm().getAngles(&r_arm);
        //     robot_state.left_arm().getAngles(&l_arm);
        //     BodyPose bp = base.body_pose();
            
        //     //Visualizer::pviz->visualizeRobot(r_arm, l_arm, bp, 150, "robot", 0);
        //     //usleep(5000);
        // }
        // data.robot_state = robot_states;
        // data.base = base_states;
        // data.path_length = geo_path.getStateCount();
        // m_stats_writer.setPlannerId(req.planner_type);
        // m_stats_writer.write(counter, data);

        data.plan_time = totalTime;
        data.shortcut_time = reduction_time;
        vector<RobotState> robot_states;
        vector<ContBaseState> base_states;
        for(unsigned int i = 0; i < geo_path.getStateCount()-1; i++){

            ompl::base::State* state = geo_path.getState(i);
            ompl::base::State* next_state = geo_path.getState(i+1);

            RobotState robot_state, next_robot_state;
            ContBaseState base, next_base;
            std::vector<RobotState> interp_steps;

            bool w_interpolate, j_interpolate;

            bool c1 = convertFullState(state, robot_state, base);
            bool c2 = convertFullState(next_state, next_robot_state, next_base); 

            if ( c1 && c2){
                
                w_interpolate = RobotState::workspaceInterpolate(robot_state, next_robot_state, &interp_steps); 

                if (!w_interpolate) {
                    interp_steps.clear();
                    j_interpolate = RobotState::jointSpaceInterpolate(robot_state, next_robot_state, &interp_steps);
                }

            }
            else{
                RightContArmState temp_r_arm({-0.2, 1.1072800, -1.5566882, -2.124408, 0.0, -1.57, 0.0});
                LeftContArmState temp_l_arm({0.038946, 1.214670, 1.396356, -1.197227, -4.616317, -0.988727, 1.175568});
                if(!c1)
                {
                  RobotState temp_state(base, temp_r_arm, temp_l_arm);
                  robot_state = temp_state;
                }
                if(!c2)
                {
                  RobotState next_temp_state(next_base, temp_r_arm, temp_l_arm);
                  next_robot_state = next_temp_state;
                }

                interp_steps.clear();
                  
                // interp_steps.push_back(robot_state);
                // interp_steps.push_back(next_robot_state);
                w_interpolate = RobotState::workspaceInterpolate(robot_state, next_robot_state, &interp_steps); 

                if (!w_interpolate) 
                {
                    interp_steps.clear();
                    j_interpolate = RobotState::jointSpaceInterpolate(robot_state, next_robot_state, &interp_steps);
                }
            }

            vector<double> l_arm, r_arm;
            BodyPose bp;

            if(i == 0)
            {
              robot_states.push_back(robot_state);
              base_states.push_back(base);

              robot_state.right_arm().getAngles(&r_arm);
              robot_state.left_arm().getAngles(&l_arm);
              bp = next_robot_state.getContBaseState().body_pose();
              Visualizer::pviz->visualizeRobot(r_arm, l_arm, bp, 150, "robot", 0);
              usleep(5000);
            }

            for(size_t  j = 0; j < interp_steps.size(); j++)
            {
                robot_states.push_back(interp_steps[j]);
                base_states.push_back(interp_steps[j].getContBaseState());

                interp_steps[j].right_arm().getAngles(&r_arm);
                interp_steps[j].left_arm().getAngles(&l_arm);
                bp = interp_steps[j].getContBaseState().body_pose();
                Visualizer::pviz->visualizeRobot(r_arm, l_arm, bp, 150, "robot", 0);
                usleep(5000);
              
            }

            robot_states.push_back(next_robot_state);
            base_states.push_back(next_base);

            next_robot_state.right_arm().getAngles(&r_arm);
            next_robot_state.left_arm().getAngles(&l_arm);
            bp = next_robot_state.getContBaseState().body_pose();
            Visualizer::pviz->visualizeRobot(r_arm, l_arm, bp, 150, "robot", 0);
            usleep(5000);
        }
        data.robot_state = robot_states;
        data.base = base_states;
        data.path_length = robot_states.size();//geo_path.getStateCount();
        m_stats_writer.setPlannerId(req.planner_type);
        m_stats_writer.write(counter, data);

    } else {
        data.planned = false;
        ROS_INFO("No plan found in %s!", planner_prefix.c_str());
    }

    if (m_params.run_trajectory) {
      ROS_INFO("Running trajectory!");
      //runTrajectory(states); Commented out to compile without driver.
    }

    return true;
  }
}

bool EnvInterfaces::planPathCallback(GetMobileArmPlan::Request &req,
                                     GetMobileArmPlan::Response &res) {
  boost::unique_lock<boost::mutex> lock(mutex);

  SearchRequestParamsPtr search_request = make_shared<SearchRequestParams>();
  search_request->initial_epsilon = req.initial_eps;
  search_request->final_epsilon = req.final_eps;
  search_request->decrement_epsilon = req.dec_eps;
  search_request->base_start = req.body_start;
  search_request->left_arm_start = LeftContArmState(req.larm_start);
  search_request->right_arm_start = RightContArmState(req.rarm_start);
  search_request->underspecified_start = req.underspecified_start;

  search_request->base_goal = req.body_goal;
  search_request->left_arm_goal = LeftContArmState(req.larm_goal);
  search_request->right_arm_goal = RightContArmState(req.rarm_goal);

  KDL::Frame rarm_offset, larm_offset;
  rarm_offset.p.x(req.rarm_object.pose.position.x);
  rarm_offset.p.y(req.rarm_object.pose.position.y);
  rarm_offset.p.z(req.rarm_object.pose.position.z);
  larm_offset.p.x(req.larm_object.pose.position.x);
  larm_offset.p.y(req.larm_object.pose.position.y);
  larm_offset.p.z(req.larm_object.pose.position.z);

  rarm_offset.M = Rotation::Quaternion(req.rarm_object.pose.orientation.x,
                                       req.rarm_object.pose.orientation.y,
                                       req.rarm_object.pose.orientation.z,
                                       req.rarm_object.pose.orientation.w);
  larm_offset.M = Rotation::Quaternion(req.larm_object.pose.orientation.x,
                                       req.larm_object.pose.orientation.y,
                                       req.larm_object.pose.orientation.z,
                                       req.larm_object.pose.orientation.w);
  search_request->left_arm_object = larm_offset;
  search_request->right_arm_object = rarm_offset;
  search_request->xyz_tolerance = req.xyz_tolerance;
  search_request->roll_tolerance = req.roll_tolerance;
  search_request->pitch_tolerance = req.pitch_tolerance;
  search_request->yaw_tolerance = req.yaw_tolerance;
  search_request->planning_mode = req.planning_mode;
  search_request->obj_goal = req.goal;
  search_request->obj_start = req.start;

  res.stats_field_names.resize(18);
  res.stats.resize(18);
  int start_id, goal_id;
  static int counter = 0;
  bool isPlanFound;

  double total_planning_time = clock();
  bool forward_search = true;
  if(req.planner_type == mha_planner::PlannerType::SMHA && (!req.use_ompl)){
    isPlanFound = runMHAPlanner(monolithic_pr2_planner::T_SMHA, "smha_", req, res,
                              search_request, counter);
  }else{

    isPlanFound = runPPMAPlanner(monolithic_pr2_planner::T_SMHA, "ppma", req, res,
                              search_request, counter);
  }
  counter++;
  return true;
}

bool EnvInterfaces::demoCallback(GetMobileArmPlan::Request &req,
                                 GetMobileArmPlan::Response &res) {

  SearchRequestParamsPtr search_request = make_shared<SearchRequestParams>();
  search_request->initial_epsilon = req.initial_eps;
  search_request->final_epsilon = req.final_eps;
  search_request->decrement_epsilon = req.dec_eps;

  // Get the current state of the robot from the real robot.
  BodyPose start_body_pos;
  std::vector<double> start_rangles;
  std::vector<double> start_langles;
  getRobotState(m_tf, start_body_pos, start_rangles, start_langles);
  search_request->left_arm_start = LeftContArmState(start_langles);
  search_request->right_arm_start = RightContArmState(start_rangles);
  ContBaseState start_base_pose(start_body_pos);
  search_request->base_start = start_base_pose;
  // Underspecified goal
  search_request->underspecified_start = req.underspecified_start;

  KDL::Frame rarm_offset, larm_offset;
  rarm_offset.p.x(req.rarm_object.pose.position.x);
  rarm_offset.p.y(req.rarm_object.pose.position.y);
  rarm_offset.p.z(req.rarm_object.pose.position.z);
  larm_offset.p.x(req.larm_object.pose.position.x);
  larm_offset.p.y(req.larm_object.pose.position.y);
  larm_offset.p.z(req.larm_object.pose.position.z);

  rarm_offset.M = Rotation::Quaternion(req.rarm_object.pose.orientation.x,
                                       req.rarm_object.pose.orientation.y,
                                       req.rarm_object.pose.orientation.z,
                                       req.rarm_object.pose.orientation.w);
  larm_offset.M = Rotation::Quaternion(req.larm_object.pose.orientation.x,
                                       req.larm_object.pose.orientation.y,
                                       req.larm_object.pose.orientation.z,
                                       req.larm_object.pose.orientation.w);
  search_request->left_arm_object = larm_offset;
  search_request->right_arm_object = rarm_offset;
  search_request->xyz_tolerance = req.xyz_tolerance;
  search_request->roll_tolerance = req.roll_tolerance;
  search_request->pitch_tolerance = req.pitch_tolerance;
  search_request->yaw_tolerance = req.yaw_tolerance;
  search_request->planning_mode = req.planning_mode;

  // relative goal for now.
  geometry_msgs::PoseStamped goal_pose = req.goal;
  goal_pose.pose.position.x += start_body_pos.x;
  goal_pose.pose.position.y += start_body_pos.y;

  search_request->obj_goal = goal_pose;
  search_request->obj_start = req.start;

  res.stats_field_names.resize(18);
  res.stats.resize(18);
  int start_id, goal_id;
  int counter = 42;
  bool isPlanFound;

  bool forward_search = true;
  if(req.planner_type == mha_planner::PlannerType::SMHA  && (!req.use_ompl)){
    isPlanFound = runMHAPlanner(monolithic_pr2_planner::T_SMHA, "smha_", req, res,
                              search_request, counter);
  }else{

    isPlanFound = runPPMAPlanner(monolithic_pr2_planner::T_SMHA, "ppma", req, res,
                              search_request, counter);
  }

  return isPlanFound;
}

void EnvInterfaces::packageStats(vector<string> &stat_names,
                                 vector<double> &stats,
                                 int solution_cost,
                                 size_t solution_size,
                                 double total_planning_time) {

  stat_names.resize(10);
  stats.resize(10);
  stat_names[0] = "total plan time";
  stat_names[1] = "initial solution planning time";
  stat_names[2] = "initial epsilon";
  stat_names[3] = "initial solution expansions";
  stat_names[4] = "final epsilon planning time";
  stat_names[5] = "final epsilon";
  stat_names[6] = "solution epsilon";
  stat_names[7] = "expansions";
  stat_names[8] = "solution cost";
  stat_names[9] = "path length";

  // TODO fix the total planning time
  //stats[0] = totalPlanTime;
  // TODO: Venkat. Handle the inital/final solution eps correctly when this becomes anytime someday.
  /*
  stats[0] = total_planning_time/static_cast<double>(CLOCKS_PER_SEC);
  stats[1] = m_ara_planner->get_initial_eps_planning_time();
  stats[2] = m_ara_planner->get_initial_eps();
  stats[3] = m_ara_planner->get_n_expands_init_solution();
  stats[4] = m_ara_planner->get_final_eps_planning_time();
  stats[5] = m_ara_planner->get_final_epsilon();
  stats[6] = m_ara_planner->get_solution_eps();
  stats[7] = m_ara_planner->get_n_expands();
  stats[8] = static_cast<double>(solution_cost);
  stats[9] = static_cast<double>(solution_size);
  */
  vector<PlannerStats> planner_stats;
  m_mha_planner->get_search_stats(&planner_stats);
  // Take stats only for the first solution, since this is not anytime currently
  stats[0] = planner_stats[0].time;
  stats[1] = stats[0];
  stats[2] = EPS;
  stats[3] = planner_stats[0].expands;
  stats[4] = stats[0];
  stats[5] = stats[2];
  stats[6] = stats[2];
  stats[7] = stats[3];
  stats[8] = static_cast<double>(planner_stats[0].cost);
  stats[9] = static_cast<double>(solution_size);
}

void EnvInterfaces::packageMHAStats(vector<string> &stat_names,
                                    vector<double> &stats,
                                    int solution_cost,
                                    size_t solution_size,
                                    double total_planning_time) {
  stat_names.resize(10);
  stats.resize(10);
  stat_names[0] = "total plan time";
  stat_names[1] = "initial solution planning time";
  stat_names[2] = "epsilon 1";
  stat_names[3] = "initial solution expansions";
  stat_names[4] = "final epsilon planning time";
  stat_names[5] = "epsilon 2";
  stat_names[6] = "solution epsilon";
  stat_names[7] = "expansions";
  stat_names[8] = "solution cost";
  stat_names[9] = "path length";

  vector<PlannerStats> planner_stats;
  m_mha_planner->get_search_stats(&planner_stats);

  if (planner_stats.empty()) {
    stats[0] = -1;
    stats[1] = -1;
    stats[2] = -1;
    stats[3] = -1;
    stats[4] = -1;
    stats[5] = -1;
    stats[6] = -1;
    stats[7] = -1;
    stats[8] = -1;
    stats[9] = -1;
  } else {
    // Take stats only for the first solution, since this is not anytime currently
    stats[0] = planner_stats[0].time;
    stats[1] = stats[0];
    stats[2] = EPS;
    stats[3] = planner_stats[0].expands;
    stats[4] = stats[0];
    stats[5] = stats[2];
    stats[6] = stats[2];
    stats[7] = stats[3];
    stats[8] = static_cast<double>(planner_stats[0].cost);
    stats[9] = static_cast<double>(solution_size);
  }
}

bool EnvInterfaces::bindCollisionSpaceToTopic(string topic_name) {
  m_collision_space_interface->bindCollisionSpaceToTopic(topic_name,
                                                         m_tf,
                                                         m_params.ref_frame);
  return true;
}

void EnvInterfaces::bindNavMapToTopic(string topic) {
  sleep(3.0);//TODO: ??!!*U8084u
  m_nav_map = m_nodehandle.subscribe(topic, 1, &EnvInterfaces::loadNavMap, this);
}

void EnvInterfaces::crop2DMap(const nav_msgs::MapMetaData &map_info, const
                              std::vector<unsigned char> &v, double new_origin_x, double new_origin_y,
                              double width, double height) {
  ROS_DEBUG_NAMED(CONFIG_LOG, "to be cropped to : %f (width), %f (height)",
                  width, height);
  vector<vector<unsigned char>> tmp_map(map_info.height);

  for (unsigned int i = 0; i < map_info.height; i++) {
    for (unsigned int j = 0; j < map_info.width; j++) {
      tmp_map[i].push_back(v[i * map_info.width + j]);
    }
  }

  double res = map_info.resolution;
  ROS_DEBUG_NAMED(CONFIG_LOG, "resolution : %f", res);
  int new_origin_x_idx = (new_origin_x - map_info.origin.position.x) / res;
  int new_origin_y_idx = (new_origin_y - map_info.origin.position.y) / res;
  int new_width = static_cast<int>((width / res) + 1 + 0.5);
  int new_height = static_cast<int>((height / res) + 1 + 0.5);
  ROS_DEBUG_NAMED(HEUR_LOG, "new origin: %d %d, new_width and new_height: %d %d",
                  new_origin_x_idx, new_origin_y_idx, new_width,
                  new_height);
  ROS_DEBUG_NAMED(HEUR_LOG, "size of map %lu %lu", tmp_map.size(),
                  tmp_map[0].size());

  vector<vector<unsigned char>> new_map(new_height);
  int row_count = 0;

  for (int i = new_origin_y_idx; i < new_origin_y_idx + new_height; i++) {
    for (int j = new_origin_x_idx; j < new_origin_x_idx + new_width; j++) {
      new_map[row_count].push_back(tmp_map[i][j]);
    }

    row_count++;
  }

  m_final_map.clear();
  m_cropped_map.clear();

  // m_final_map.resize(new_width * new_height);
  for (size_t i = 0; i < new_map.size(); i++) {
    for (size_t j = 0; j < new_map[i].size(); j++) {
      m_final_map.push_back(static_cast<signed char>(double(new_map[i][j]) / 255.0 *
                                                     100.0));
      m_cropped_map.push_back(new_map[i][j]);
    }
  }

  ROS_DEBUG_NAMED(HEUR_LOG, "size of final map: %lu", m_final_map.size());
}

void EnvInterfaces::loadNavMap(const nav_msgs::OccupancyGridPtr &map) {
  boost::unique_lock<boost::mutex> lock(mutex);
  ROS_DEBUG_NAMED(CONFIG_LOG, "received navmap of size %u %u, resolution %f",
                  map->info.width, map->info.height, map->info.resolution);
  ROS_DEBUG_NAMED(CONFIG_LOG, "origin is at %f %f", map->info.origin.position.x,
                  map->info.origin.position.y);

  // look up the values from the occup grid parameters
  // This stuff is in cells.
  int dimX, dimY, dimZ;
  m_collision_space_interface->getOccupancyGridSize(dimX, dimY,
                                                    dimZ);
  ROS_DEBUG_NAMED(CONFIG_LOG, "Size of OccupancyGrid : %d %d %d", dimX, dimY,
                  dimZ);
  // This costmap_ros object listens to the map topic as defined
  // in the costmap_2d.yaml file.
  m_costmap_ros.reset(new costmap_2d::Costmap2DROS("costmap_2d", m_tf));

  // Get the underlying costmap in the cost_map object.
  // Publish for visualization. Publishing is done for the entire (uncropped) costmap.
  costmap_2d::Costmap2D cost_map;
  m_costmap_ros->getCostmapCopy(cost_map);

  // Normalize and convert to array.
  for (unsigned int j = 0; j < cost_map.getSizeInCellsY(); ++j) {
    for (unsigned int i = 0; i < cost_map.getSizeInCellsX(); ++i) {
      // Row major. X is row wise, Y is column wise.
      int c = cost_map.getCost(i, j);

      // Set unknowns to free space (we're dealing with static maps for
      // now)
      if (c == costmap_2d::NO_INFORMATION) {
        c = costmap_2d::FREE_SPACE;
      }

      // c = (c == (costmap_2d::NO_INFORMATION)) ? (costmap_2d::FREE_SPACE) : (c);

      // Re-set the cost.
      cost_map.setCost(i, j, c);
    }
  }

  // Re-inflate because we modified the unknown cells to be free space.
  // API : center point of window x, center point of window y, size_x ,
  // size_y
  cost_map.reinflateWindow(dimX * map->info.resolution / 2,
                           dimY * map->info.resolution / 2, dimX * map->info.resolution,
                           dimY * map->info.resolution);

  std::vector<unsigned char> uncropped_map;

  for (unsigned int j = 0; j < cost_map.getSizeInCellsY(); ++j) {
    for (unsigned int i = 0; i < cost_map.getSizeInCellsX(); ++i) {
      /*
        // Normalize the values from 0 to 100.
        // makes life easier when dealing with the heuristic later.
        uncropped_map.push_back(
            static_cast<unsigned char>(
                static_cast<double>(cost_map.getCost(i,j))/UCHAR_MAX*100.0f)
            );
      */
      uncropped_map.push_back(cost_map.getCost(i, j));

    }
  }

  m_costmap_publisher->updateCostmapData(cost_map,
                                         m_costmap_ros->getRobotFootprint());

  // Publish the full costmap
  // topic : /monolithic_pr2_planner_node/inflated_obstacles (RViz: Grid
  // Cells)
  m_costmap_publisher->publishCostmap();
  // topic : /monolithic_pr2_planner_node/robot_footprint (RViz: polygon)
  m_costmap_publisher->publishFootprint();

  // TODO: Check if this is the right thing to do : Take the resolution from
  // the map for the occupancy grid's values.
  double width = dimX * map->info.resolution;
  double height = dimY * map->info.resolution;

  crop2DMap(map->info, uncropped_map, 0, 0, width, height);

  // Don't want to publish this.
  nav_msgs::OccupancyGrid costmap_pub;
  costmap_pub.header.frame_id = "/map";
  costmap_pub.header.stamp = ros::Time::now();
  costmap_pub.info.map_load_time = ros::Time::now();
  costmap_pub.info.resolution = map->info.resolution;
  // done in the crop function too.
  costmap_pub.info.width = (width / map->info.resolution + 1 + 0.5);
  costmap_pub.info.height = (height / map->info.resolution + 1 + 0.5);
  costmap_pub.info.origin.position.x = 0;
  costmap_pub.info.origin.position.y = 0;
  costmap_pub.data = m_final_map;

  // Publish the cropped version of the costmap; publishes
  // /monolithic_pr2_planner/costmap_pub
  ROS_INFO_NAMED(CONFIG_LOG, "Publishing the final map that's supposed to fit"
                 " within the occupancy grid.");
  m_costmap_pub.publish(costmap_pub);

  m_collision_space_interface->update2DHeuristicMaps(m_cropped_map);

}

void EnvInterfaces::getRobotState(tf::TransformListener &tf_,
                                  BodyPose &body_pos, std::vector<double> &rangles,
                                  std::vector<double> &langles) {
  tf::StampedTransform base_map_transform;
  sensor_msgs::JointStateConstPtr state = ros::topic::waitForMessage
                                          <sensor_msgs::JointState> ("joint_states");
  rangles.resize(7);
  langles.resize(7);

  rangles[0] = getJointAngle("r_shoulder_pan_joint", state);
  rangles[1] = getJointAngle("r_shoulder_lift_joint", state);
  rangles[2] = getJointAngle("r_upper_arm_roll_joint", state);
  rangles[3] = getJointAngle("r_elbow_flex_joint", state);
  rangles[4] = getJointAngle("r_forearm_roll_joint", state);
  rangles[5] = getJointAngle("r_wrist_flex_joint", state);
  rangles[6] = getJointAngle("r_wrist_roll_joint", state);

  langles[0] = getJointAngle("l_shoulder_pan_joint", state);
  langles[1] = getJointAngle("l_shoulder_lift_joint", state);
  langles[2] = getJointAngle("l_upper_arm_roll_joint", state);
  langles[3] = getJointAngle("l_elbow_flex_joint", state);
  langles[4] = getJointAngle("l_forearm_roll_joint", state);
  langles[5] = getJointAngle("l_wrist_flex_joint", state);
  langles[6] = getJointAngle("l_wrist_roll_joint", state);

  body_pos.z = getJointAngle("torso_lift_joint", state);

  bool done = false;

  while (!done) {
    try {
      tf_.lookupTransform("map", "base_footprint", ros::Time(0), base_map_transform);
      body_pos.x = base_map_transform.getOrigin().x();
      body_pos.y = base_map_transform.getOrigin().y();
      body_pos.theta = 2 * atan2(base_map_transform.getRotation().getZ(),
                                 base_map_transform.getRotation().getW());
      done = true;
    } catch (tf::TransformException &ex) {
      ROS_ERROR("[EnvInterfaces] Is there a map? The map-robot transform failed. (%s)",
                ex.what());
      sleep(1);
    }
  }
}

double EnvInterfaces::getJointAngle(std::string name,
                                    sensor_msgs::JointStateConstPtr msg) {
  for (unsigned int i = 0; i < msg->name.size(); i++) {
    if (msg->name[i] == name) {
      return msg->position[i];
    }
  }

  ROS_ERROR("joint doesn't exist! (exit)\n");
  exit(1);
}

bool EnvInterfaces::convertFullState(ompl::base::State* state, RobotState& robot_state,
                                      ContBaseState& base){
    ContObjectState obj_state;

    // fix the l_arm angles
    vector<double> init_l_arm(7,0);
    init_l_arm[0] = (0.038946287971107774);
    init_l_arm[1] = (1.2146697069025374);
    init_l_arm[2] = (1.3963556492780154);
    init_l_arm[3] = -1.1972269899800325;
    init_l_arm[4] = (-4.616317135720829);
    init_l_arm[5] = -0.9887266887318599;
    init_l_arm[6] = 1.1755681069775656;

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
    {
      RightContArmState temp_r_arm({-0.2, 1.1072800, -1.5566882, -2.124408, 0.0, -1.57, 0.0});
      RobotState temp_seed_state(base, temp_r_arm, l_arm);
      if (!RobotState::computeRobotPose(DiscObjectState(obj_state), temp_seed_state, final_state))
        return false;
    }

    robot_state = *final_state;

    return true;

}

/**
 * @brief calls the monolithic_trajectory controller
 * @details Converts the FullBodyState objects to a full_body_controller
 * message and subsequently calls the service given by the parameter.
 *
 * @param controller_service The name of the service to be called
 * @param states The final path
 * @return status of the call
 */
/* Commenting out to compile without the PR2 driver
void EnvInterfaces::runTrajectory(std::vector<FullBodyState> &states) {

  // Create the messages from the full body states
  trajectory_msgs::JointTrajectory arms_trajectory;
  trajectory_msgs::JointTrajectory body_trajectory;
  trajectory_msgs::JointTrajectory gripper_trajectory;

  // prepare arms trajectory
  arms_trajectory.joint_names.push_back("r_shoulder_pan_joint");
  arms_trajectory.joint_names.push_back("r_shoulder_lift_joint");
  arms_trajectory.joint_names.push_back("r_upper_arm_roll_joint");
  arms_trajectory.joint_names.push_back("r_elbow_flex_joint");
  arms_trajectory.joint_names.push_back("r_forearm_roll_joint");
  arms_trajectory.joint_names.push_back("r_wrist_flex_joint");
  arms_trajectory.joint_names.push_back("r_wrist_roll_joint");
  arms_trajectory.joint_names.push_back("l_shoulder_pan_joint");
  arms_trajectory.joint_names.push_back("l_shoulder_lift_joint");
  arms_trajectory.joint_names.push_back("l_upper_arm_roll_joint");
  arms_trajectory.joint_names.push_back("l_elbow_flex_joint");
  arms_trajectory.joint_names.push_back("l_forearm_roll_joint");
  arms_trajectory.joint_names.push_back("l_wrist_flex_joint");
  arms_trajectory.joint_names.push_back("l_wrist_roll_joint");

  arms_trajectory.points.resize(states.size());
  body_trajectory.points.resize(states.size());
  gripper_trajectory.points.resize(states.size());

  for (size_t i = 0; i < states.size(); ++i) {
    // // Insert the right arm joint angles
    // auto it = arms_trajectory.points[i].positions.begin();
    // arms_trajectory.points[i].positions.insert(it,
    //     states[i].right_arm.begin(), states[i].right_arm.end());
    // // insert the left arm joint angles
    // it = arms_trajectory.points[i].positions.end();
    // arms_trajectory.points[i].positions.insert(it,
    //     states[i].left_arm.begin(), states[i].left_arm.end());

    // Insert the right arm joint angles
    for (int r_arm = 0; r_arm < 7; ++r_arm) {
      arms_trajectory.points[i].positions.push_back(states[i].right_arm[r_arm]);
    }

    // insert the left arm joint angles
    for (int l_arm = 0; l_arm < 7; ++l_arm) {
      arms_trajectory.points[i].positions.push_back(states[i].left_arm[l_arm]);
    }

    // ROS_DEBUG_NAMED(POSTPROCESSOR_LOG, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f",
    //         arms_trajectory.points[i].positions[0],
    //         arms_trajectory.points[i].positions[1],
    //         arms_trajectory.points[i].positions[2],
    //         arms_trajectory.points[i].positions[3],
    //         arms_trajectory.points[i].positions[4],
    //         arms_trajectory.points[i].positions[5],
    //         arms_trajectory.points[i].positions[6],
    //         arms_trajectory.points[i].positions[7],
    //         arms_trajectory.points[i].positions[8],
    //         arms_trajectory.points[i].positions[9],
    //         arms_trajectory.points[i].positions[10],
    //         arms_trajectory.points[i].positions[11],
    //         arms_trajectory.points[i].positions[12],
    //         arms_trajectory.points[i].positions[13]
    //         );

    // insert the base X, Y, Z, Theta
    body_trajectory.points[i].positions.assign(states[i].base.begin(),
                                               states[i].base.end());

    // Set the gripper poses
    gripper_trajectory.points[i].positions.resize(2, 0);
  }

  // Package into full body message
  full_body_controller::ExecutePath::Request req;
  full_body_controller::ExecutePath::Response res;

  req.trajectory = arms_trajectory;
  req.body_trajectory = body_trajectory;
  req.gripper_trajectory = gripper_trajectory;

  // Check if service has been advertised
  ros::service::waitForService(m_params.controller_service);
  ros::ServiceClient client =
    m_nodehandle.serviceClient<full_body_controller::ExecutePath>
    (m_params.controller_service,
     true);

  if (client.call(req, res)) {
    ROS_INFO("It ran! Oh yeah!!");
  } else {
    ROS_INFO("Trajectory failed.");
  }
}
*/
