#include <hubero_local_planner/hubero_planner.h>
#include <math.h>

namespace hubero_local_planner {

HuberoPlanner::HuberoPlanner(std::string name, std::shared_ptr<base_local_planner::LocalPlannerUtil> planner_util):
	planner_util_(planner_util),
	sim_period_(0.2),
	obstacles_(nullptr)
{
	ros::NodeHandle private_nh("~/" + name);
	printf("[HuberoPlanner::HuberoPlanner] ctor, name: %s \r\n", name.c_str());
}

HuberoPlanner::~HuberoPlanner() {
	printf("[HuberoPlanner::HuberoPlanner] dtor \r\n");
}

void HuberoPlanner::initialize(HuberoConfigConstPtr cfg) {
	cfg_ = cfg;
}

bool HuberoPlanner::checkTrajectory(
		const Eigen::Vector3f pos,
		const Eigen::Vector3f vel,
		const Eigen::Vector3f vel_samples) {
	printf("[HuberoPlanner::checkTrajectory] \r\n");
	return false;
}

Vector3 HuberoPlanner::computeForce() {
	return Vector3();
}

base_local_planner::Trajectory HuberoPlanner::findBestPath(
        tf::Stamped<tf::Pose> global_pose,
        tf::Stamped<tf::Pose> global_vel,
        tf::Stamped<tf::Pose>& drive_velocities) {
	printf("[HuberoPlanner::findBestPath] \r\n");
	return base_local_planner::Trajectory();
}

void HuberoPlanner::updatePlanAndLocalCosts(tf::Stamped<tf::Pose> global_pose,
		const std::vector<geometry_msgs::PoseStamped>& new_plan,
		const std::vector<geometry_msgs::Point>& footprint_spec) {
	printf("[HuberoPlanner::updatePlanAndLocalCosts] \r\n");
}

bool HuberoPlanner::getCellCosts(int cx,
		int cy,
		float &path_cost,
		float &goal_cost,
		float &occ_cost,
		float &total_cost) {
	printf("[HuberoPlanner::getCellCosts] \r\n");
	return true;
}

bool HuberoPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
	printf("[HuberoPlanner::setPlan] \r\n");
	return planner_util_->setPlan(orig_global_plan);
}

}; // namespace hubero_local_planner
