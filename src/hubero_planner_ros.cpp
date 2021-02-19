#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <hubero_local_planner/hubero_planner_ros.h>
#include <nav_msgs/Path.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(hubero_local_planner::HuberoPlannerROS, nav_core::BaseLocalPlanner)

namespace hubero_local_planner {

void HuberoPlannerROS::reconfigureCB(HuberoPlannerConfig &config, uint32_t level) {
}

HuberoPlannerROS::HuberoPlannerROS() : initialized_(false),
	odom_helper_("odom"), setup_(false), dsrv_(nullptr) {
}

HuberoPlannerROS::~HuberoPlannerROS(){
	//make sure to clean things up
	delete dsrv_;
}

void HuberoPlannerROS::initialize(
	std::string name,
	tf::TransformListener* tf,
	costmap_2d::Costmap2DROS* costmap_ros) {
}

bool HuberoPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
	return true;
}

bool HuberoPlannerROS::isGoalReached() {
	return false;
}

void HuberoPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
	base_local_planner::publishPlan(path, l_plan_pub_);
}

void HuberoPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
	base_local_planner::publishPlan(path, g_plan_pub_);
}

bool HuberoPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
	cmd_vel.linear.x = 0.25;
	return true;
}

}; // namespace hubero_local_planner
