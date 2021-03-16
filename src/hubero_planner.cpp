#include <hubero_local_planner/hubero_planner.h>
#include <hubero_common/converter.h>
#include <math.h>

// debugging macros
#include <hubero_local_planner/debug.h>
#define DEBUG_BASIC 1
#define debug_print_basic(fmt, ...) _template_debug_print_basic_(DEBUG_BASIC, fmt, ##__VA_ARGS__)

namespace hubero_local_planner {

HuberoPlanner::HuberoPlanner(std::string name, std::shared_ptr<base_local_planner::LocalPlannerUtil> planner_util):
	planner_util_(planner_util),
	// TODO: make it param
	sim_period_(0.2),
	goal_reached_(false),
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
	sfm_.init(cfg);
}

bool HuberoPlanner::checkTrajectory(
		const Eigen::Vector3f pos,
		const Eigen::Vector3f vel,
		const Eigen::Vector3f vel_samples) {
	printf("[HuberoPlanner::checkTrajectory] \r\n");
	return false;
}

bool HuberoPlanner::compute(
		const tf::Stamped<tf::Pose>& pose,
		const geometry_msgs::Twist& velocity,
		const tf::Stamped<tf::Pose>& goal,
		ObstContainerConstPtr obstacles,
		Eigen::Vector3f& force
) {
	// NOTE: typical operations are available in `base_local_planner` ns as free functions
	// base_local_planner::getGoalPose()
	// NOTE: stick to ROS interface with base_local_planner functions,
	// let Hubero Planner abstract from the actual tf conversions etc
	//
	Vector3 force_result;
	bool status = compute(
			converter::tfPoseToIgnPose(pose),
			converter::twistToIgnVector3(velocity), // TODO angular velocity!
			converter::tfPoseToIgnPose(goal),
			obstacles,
			force_result
	);
	force = converter::ignVectorToEigenV3f(force_result);

//	double dt = 0.2;
//	Vector3 result_vel = (force / cfg_->sfm.mass) * 0.2;
//	std::cout << "\t\t\t human_action_force: " << human_action_force << std::endl;
//	std::cout << "\t\t\t sfm comb: " << sfm_.getForceCombined() << std::endl;
//	std::cout << "\t\t\t force: " << force << std::endl;
//	std::cout << "\t\t\t result: " << result_vel << std::endl;
//
//	printf("[HuberoPlanner::compute()] 6 \r\n");
//
//	// transform force vector (expressed w.r.t. global frame) into odom frame
//	tf::Matrix3x3 odom_rot_mat(pose_global.getRotation());
//	double roll, pitch, yaw = 0.0;
//	odom_rot_mat.getRPY(roll, pitch, yaw);
//
//	// orientation of the force vector
//	Angle angle_force_v(std::atan2(force_total.Normalized().Y(), force_total.Normalized().X()));
//	angle_force_v.Normalize();
//
//	Angle rot(angle_force_v - Angle(yaw));

	// TODO: saturate

	return true;
}

bool HuberoPlanner::compute(
		const Pose3& pose,
		const Vector3& velocity,
		const Pose3& goal,
		ObstContainerConstPtr obstacles,
		Vector3& force
) {
	// calculate `social` force (i.e. `internal` and `interaction` components)
	sfm_.computeSocialForce(
			obstacles,
			pose,
			velocity, // TODO angular velocity!
			goal.Pos(), // TODO:
			cfg_->general.sim_period
	);

	// actual `social` vector
	Vector3 human_action_force;

	// evaluate whether more complex forces are supposed to be calculated
	// TODO: add param `disable fuzzy behaviours`
	if (!cfg_->sfm.disable_interaction_forces) {

//		printf("DYNAMIC: %lu,  %lu,  %lu\r\n",
//			sfm_.getDirectionBetaDynamic().size(),
//			sfm_.getRelativeLocationDynamic().size(),
//			sfm_.getDistanceAngleDynamic().size()
//		);

		// execute fuzzy operations block
		fuzzy_processor_.load(
			sfm_.getDirectionAlpha(),
			sfm_.getDirectionBetaDynamic(),
			sfm_.getRelativeLocationDynamic(),
			sfm_.getDistanceAngleDynamic()
		);
		fuzzy_processor_.process();

		// create a force vector according to the activated `social behaviour`
		social_conductor_.apply(
			sfm_.getForceCombined(),
			sfm_.getDirectionAlpha(),
			sfm_.getDistanceDynamic(),
			fuzzy_processor_.getOutput()
		);

		// assign `social` vector
		human_action_force = social_conductor_.getSocialVector();

	}

	force = sfm_.getForceCombined() + human_action_force;

	motion_data_.force_internal = sfm_.getForceInternal();
	motion_data_.force_interaction = sfm_.getForceInternal();
	motion_data_.force_social = social_conductor_.getSocialVector();
	motion_data_.force_combined = force;
	motion_data_.closest_points = sfm_.getClosestPointsVector();
	motion_data_.behaviour_active = social_conductor_.getBehaviourActive();

	return true;
}

bool HuberoPlanner::plan() {
	//sfm_.computeSocialForce(obstacles_, pose, vel, goal_pos, dt);
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

bool HuberoPlanner::checkGoalReached(const tf::Stamped<tf::Pose>& pose, const tf::Stamped<tf::Pose>& goal) {
	// this is slightly modified base_local_planner::isGoalReached() method
	double dist_xy = sqrt(
		pow(pose.getOrigin().getX() - goal.getOrigin().getX(), 2) +
		pow(pose.getOrigin().getY() - goal.getOrigin().getY(), 2)
	);
	debug_print_basic("\t dist_xy = %2.4f, xy_goal_tolerance = %2.4f  /  cond: %d \r\n",
			dist_xy,
			cfg_->limits.xy_goal_tolerance,
			!(dist_xy > cfg_->limits.xy_goal_tolerance)
	);

	if (dist_xy > cfg_->limits.xy_goal_tolerance) {
		goal_reached_ = false;
		return false;
	}

	double dist_ang = angles::shortest_angular_distance(
			tf::getYaw(goal.getRotation()), tf::getYaw(pose.getRotation())
	);
	debug_print_basic("\t dist_ang = %2.4f, yaw_tolerance = %2.4f  /  cond: %d \r\n",
			dist_ang,
			cfg_->limits.yaw_goal_tolerance,
			!(std::abs(dist_ang) > cfg_->limits.yaw_goal_tolerance)
	);

	if (std::abs(dist_ang) > cfg_->limits.yaw_goal_tolerance) {
		goal_reached_ = false;
		return false;
	}

	goal_reached_ = true;
	return true;

//	// TEB - check if global goal is reached
//	tf::Stamped<tf::Pose> global_goal;
//	tf::poseStampedMsgToTF(global_plan_.back(), global_goal);
//	global_goal.setData( tf_plan_to_global * global_goal );
//	double dx = global_goal.getOrigin().getX() - robot_pose_.x();
//	double dy = global_goal.getOrigin().getY() - robot_pose_.y();
//	double delta_orient = g2o::normalize_theta( tf::getYaw(global_goal.getRotation()) - robot_pose_.theta() );
//	if(fabs(std::sqrt(dx*dx+dy*dy)) < cfg_.goal_tolerance.xy_goal_tolerance
//	&& fabs(delta_orient) < cfg_.goal_tolerance.yaw_goal_tolerance
//	&& (!cfg_.goal_tolerance.complete_global_plan || via_points_.size() == 0))
//	{
//	goal_reached_ = true;
//	return true;
//	}
}

}; // namespace hubero_local_planner
