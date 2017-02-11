#include <ros/ros.h>
#include <monolithic_pr2_planner_node/GetMobileArmPlan.h>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner_node/fbp_stat_writer.h>
#include <sbpl/planners/mha_planner.h>
#include <angles/angles.h>
#include <monolithic_pr2_planner_node/ompl_pr2_planner.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/CollisionSpaceMgr.h>

using namespace monolithic_pr2_planner;
using namespace monolithic_pr2_planner_node;

int main(int argc, char** argv){

  ros::init(argc,argv,"run_sampled_tests");
  ros::NodeHandle nh;

  monolithic_pr2_planner_node::GetMobileArmPlan::Request req;
  monolithic_pr2_planner_node::GetMobileArmPlan::Response res;

  req.use_ompl = true;

  // Defaults
  req.mha_type = mha_planner::MHAType::ORIGINAL;
  req.planner_type = -1;
  req.meta_search_type = mha_planner::MetaSearchType::ROUND_ROBIN;

  //planner parameters
  req.initial_eps = 50.0;
  req.final_eps = 50.0;
  req.dec_eps = 0.2;

  req.xyz_tolerance = .04;
  req.roll_tolerance = .1;
  req.pitch_tolerance = .1;
  req.yaw_tolerance = .1;
  req.allocated_planning_time = 50;
  req.planning_mode = monolithic_pr2_planner::PlanningModes::RIGHT_ARM_MOBILE;

  //position of the wrist in the object's frame
  req.rarm_object.pose.position.x = 0;
  req.rarm_object.pose.position.y = 0;
  req.rarm_object.pose.position.z = 0;
  req.rarm_object.pose.orientation.x = 0;
  req.rarm_object.pose.orientation.y = 0;
  req.rarm_object.pose.orientation.z = 0;
  req.rarm_object.pose.orientation.w = 1;
  req.larm_object.pose.position.x = 0;
  req.larm_object.pose.position.y = 0;
  req.larm_object.pose.position.z = 0;
  req.larm_object.pose.orientation.x = 0;
  req.larm_object.pose.orientation.y = 0;
  req.larm_object.pose.orientation.z = 0;
  req.larm_object.pose.orientation.w = 1;

  ros::service::waitForService("/sbpl_planning/plan_path",10);
  ros::ServiceClient planner = nh.serviceClient<monolithic_pr2_planner_node::GetMobileArmPlan>("/sbpl_planning/plan_path", true);
  sleep(1);

  planner.call(req,res);

  return 0;
}

