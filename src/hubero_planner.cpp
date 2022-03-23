#include <hubero_local_planner/hubero_planner.h>
#include <hubero_local_planner/utils/transformations.h>
#include <hubero_local_planner/sfm/social_force_model.h>

#include <math.h>

#include <hubero_local_planner/utils/debug.h>
// debugging macros
#define DEBUG_BASIC 1
#define debug_print_basic(fmt, ...) _template_debug_print_basic_(DEBUG_BASIC, fmt, ##__VA_ARGS__)

namespace hubero_local_planner {

HuberoPlanner::HuberoPlanner(
		const std::string& name,
		std::shared_ptr<base_local_planner::LocalPlannerUtil> planner_util,
		RobotFootprintModelPtr robot_model,
		std::vector<geometry_msgs::Point> footprint_spec
):
	planner_util_(planner_util),
	goal_reached_(false),
	obstacles_(nullptr),
	robot_model_(robot_model)
{
	ros::NodeHandle private_nh("~/" + name);
	printf("[HuberoPlanner::HuberoPlanner] ctor, name: %s \r\n", name.c_str());

	// prints Fuzzy Inference System configuration, FIS used for trajectory generation
	generator_.printFisConfiguration();
	// set up all the cost functions that will be applied in order
	// (any function returning negative values will abort scoring, so the order can improve performance)
	std::vector<base_local_planner::TrajectoryCostFunction*> critics;
	// trajectory generators
	std::vector<base_local_planner::TrajectorySampleGenerator*> generator_list;
	generator_list.push_back(&generator_);

	// score sampled trajectories
	scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(generator_list, critics);
}

HuberoPlanner::~HuberoPlanner() {
	printf("[HuberoPlanner::HuberoPlanner] dtor \r\n");
}

void HuberoPlanner::reconfigure(HuberoConfigConstPtr cfg) {
	// make sure that our configuration doesn't change mid-run
	std::lock_guard<std::mutex> l(configuration_mutex_);
	cfg_ = cfg;

	generator_.setParameters(
		cfg->getSfm(),
		cfg->getBehaviour(),
		cfg->getGeneral()->sim_time,
		cfg->getGeneral()->sim_granularity,
		cfg->getGeneral()->angular_sim_granularity,
		cfg->getGeneral()->sim_period
	);
}

bool HuberoPlanner::checkTrajectory(
		const Eigen::Vector3f pos,
		const Eigen::Vector3f vel,
		const Eigen::Vector3f vel_samples) {
	printf("[HuberoPlanner::checkTrajectory] \r\n");
	return false;
}

base_local_planner::Trajectory HuberoPlanner::findBestTrajectory(
	const Pose& pose,
	const Vector& velocity,
	const Pose& goal,
	const ObstContainerConstPtr obstacles,
	geometry_msgs::PoseStamped& drive_velocities
) {
	// make sure that our configuration doesn't change mid-run
	std::lock_guard<std::mutex> l(configuration_mutex_);

	// assign, will likely be useful for planning
	pose_ = pose;
	vel_ = velocity;
	goal_ = goal;
	obstacles_ = obstacles;
	goal_local_ = goal_;
	chooseGoalBasedOnGlobalPlan();

	// velocity transformation - from base coordinate system to planner's frame (global velocity vector)
	Vector robot_vel_glob;
	computeVelocityGlobal(velocity, pose, robot_vel_glob);

	world_model_ = World(pose, robot_vel_glob, goal_local_, goal_);
	createEnvironmentModel(pose);

	// prepare data for planning
	// TSP: Trajectory Sampling Parameters
	TrajectorySamplingParams tsp {};
	tsp.force_internal_amplifier_min = cfg_->getTrajectorySampling()->force_internal_amplifier_min;
	tsp.force_internal_amplifier_max = cfg_->getTrajectorySampling()->force_internal_amplifier_max;
	tsp.force_interaction_static_amplifier_min = cfg_->getTrajectorySampling()->force_interaction_static_amplifier_min;
	tsp.force_interaction_static_amplifier_max = cfg_->getTrajectorySampling()->force_interaction_static_amplifier_max;
	tsp.force_interaction_dynamic_amplifier_min = cfg_->getTrajectorySampling()->force_interaction_dynamic_amplifier_min;
	tsp.force_interaction_dynamic_amplifier_max = cfg_->getTrajectorySampling()->force_interaction_dynamic_amplifier_max;
	tsp.force_interaction_social_amplifier_min = cfg_->getTrajectorySampling()->force_interaction_social_amplifier_min;
	tsp.force_interaction_social_amplifier_max = cfg_->getTrajectorySampling()->force_interaction_social_amplifier_max;

	tsp.force_internal_amplifier_granularity = cfg_->getTrajectorySampling()->force_internal_amplifier_granularity;
	tsp.force_interaction_static_amplifier_granularity = cfg_->getTrajectorySampling()->force_interaction_static_amplifier_granularity;
	tsp.force_interaction_dynamic_amplifier_granularity = cfg_->getTrajectorySampling()->force_interaction_dynamic_amplifier_granularity;
	tsp.force_interaction_social_amplifier_granularity = cfg_->getTrajectorySampling()->force_interaction_social_amplifier_granularity;

	// initialize generator with updated parameters
	generator_.initialise(
		world_model_,
		vel_,
		tsp,
		cfg_->getLimits(),
		cfg_->getSfm()->mass,
		cfg_->getGeneral()->twist_rotation_compensation,
		true
	);

	// find best trajectory by sampling and scoring the samples, `scored_sampling_planner_` uses `generator_` internally
	result_traj_.cost_ = -7;
	result_traj_.resetPoints();
	std::vector<base_local_planner::Trajectory> all_explored;
	bool traj_valid = scored_sampling_planner_.findBestTrajectory(result_traj_, &all_explored);

	collectTrajectoryMotionData();

	// no legal trajectory, command zero
	if (!traj_valid || result_traj_.cost_ < 0) {
		drive_velocities.pose.position.x = 0;
		drive_velocities.pose.position.y = 0;
		drive_velocities.pose.position.z = 0;
		drive_velocities.pose.orientation.w = 1;
		drive_velocities.pose.orientation.x = 0;
		drive_velocities.pose.orientation.y = 0;
		drive_velocities.pose.orientation.z = 0;
	} else {
		drive_velocities.pose.position.x = result_traj_.xv_;
		drive_velocities.pose.position.y = result_traj_.yv_;
		drive_velocities.pose.position.z = 0;
		tf2::Quaternion q;
		q.setRPY(0, 0, result_traj_.thetav_);
		tf2::convert(q, drive_velocities.pose.orientation);
	}
	return result_traj_;
}

base_local_planner::Trajectory HuberoPlanner::findTrajectory(
	const Pose& pose,
	const Vector& velocity,
	const Pose& goal,
	const ObstContainerConstPtr obstacles,
	geometry_msgs::PoseStamped& drive_velocities
) {
	// assign, will likely be useful for planning
	pose_ = pose;
	vel_ = velocity;
	goal_ = goal;
	obstacles_ = obstacles;
	goal_local_ = goal_;
	chooseGoalBasedOnGlobalPlan();

	// velocity transformation - from base coordinate system to planner's frame (global velocity vector)
	Vector robot_vel_glob;
	computeVelocityGlobal(velocity, pose, robot_vel_glob);

	world_model_ = World(pose, robot_vel_glob, goal_local_, goal_);
	createEnvironmentModel(pose);

	// initialize generator with updated parameters
	generator_.initialise(
		world_model_,
		velocity,
		cfg_->getLimits(),
		cfg_->getSfm()->mass,
		cfg_->getGeneral()->twist_rotation_compensation
	);

	// trajectory that stores velocity commands, contains only seed vels
	base_local_planner::Trajectory traj;

	// perform SFM and fuzzy logic computations without planning
	generator_.generateTrajectoryWithoutPlanning(traj);

	drive_velocities.pose.position.x = traj.xv_;
	drive_velocities.pose.position.y = traj.yv_;
	drive_velocities.pose.position.z = 0;
	tf2::Quaternion q;
	q.setRPY(0, 0, traj.thetav_);
	tf2::convert(q, drive_velocities.pose.orientation);

	collectTrajectoryMotionData();

	return traj;
}

void HuberoPlanner::updatePlanAndLocalCosts(const geometry_msgs::PoseStamped& global_pose,
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
	printf("[HuberoPlanner::setPlan] length %lu \r\n", orig_global_plan.size());
	return planner_util_->setPlan(orig_global_plan);
}

bool HuberoPlanner::checkGoalReached(const tf::Stamped<tf::Pose>& pose, const tf::Stamped<tf::Pose>& goal) {
	printf("[HuberoPlanner::checkGoalReached] \r\n");

	// this is slightly modified base_local_planner::isGoalReached() method
	double dist_xy = sqrt(
		pow(pose.getOrigin().getX() - goal.getOrigin().getX(), 2) +
		pow(pose.getOrigin().getY() - goal.getOrigin().getY(), 2)
	);
	debug_print_basic("\t dist_xy = %2.4f, xy_goal_tolerance = %2.4f  /  cond: %d \r\n",
			dist_xy,
			cfg_->getLimits()->xy_goal_tolerance,
			!(dist_xy > cfg_->getLimits()->xy_goal_tolerance)
	);

	if (dist_xy > cfg_->getLimits()->xy_goal_tolerance) {
		goal_reached_ = false;
		return false;
	}

	double dist_ang = angles::shortest_angular_distance(
			tf::getYaw(goal.getRotation()), tf::getYaw(pose.getRotation())
	);
	debug_print_basic("\t dist_ang = %2.4f, yaw_tolerance = %2.4f  /  cond: %d \r\n",
			dist_ang,
			cfg_->getLimits()->yaw_goal_tolerance,
			!(std::abs(dist_ang) > cfg_->getLimits()->yaw_goal_tolerance)
	);

	if (std::abs(dist_ang) > cfg_->getLimits()->yaw_goal_tolerance) {
		goal_reached_ = false;
		return false;
	}

	goal_reached_ = true;
	return true;
}

void HuberoPlanner::createEnvironmentModel(const Pose& pose_ref) {
	teb_local_planner::PoseSE2 pose = pose_ref.getAsTebPose();

	for (const hubero_local_planner::ObstaclePtr obstacle: *obstacles_) {
		// TODO: consider actor personal space even when he is standing!

		BaseRobotFootprintModel::ClosestPoints pts = robot_model_->calculateClosestPoints(pose, obstacle.get());

		Pose robot_closest_to_obstacle_pose(pts.robot);
		Pose obstacle_closest_to_robot_pose(pts.obstacle);

		// FIXME
		auto distance_v_eig =
				Eigen::Vector2d(
					obstacle_closest_to_robot_pose.getX(),
					obstacle_closest_to_robot_pose.getY())
				- Eigen::Vector2d(
					pose.x(),
					pose.y()
		);
		Vector distance_v(distance_v_eig[0], distance_v_eig[1], 0.0);
		double distance = obstacle->getMinimumDistance(Eigen::Vector2d(pose.x(), pose.y()));
		Vector model_vel = Vector((obstacle->getCentroidVelocity())[0], (obstacle->getCentroidVelocity())[1], 0.0);

		bool force_dynamic_object_interpretation =
				cfg_->getSfm()->static_obj_interaction == sfm::StaticObjectInteraction::INTERACTION_REPULSIVE_EVASIVE;
		world_model_.addObstacle(
			robot_closest_to_obstacle_pose,
			obstacle_closest_to_robot_pose,
			model_vel,
			force_dynamic_object_interpretation
		);
	}
}

bool HuberoPlanner::chooseGoalBasedOnGlobalPlan() {
	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.pose = pose_.getAsMsgPose();
	pose_stamped.header.frame_id = planner_util_->getGlobalFrame();
	pose_stamped.header.stamp = ros::Time::now();

	std::vector<geometry_msgs::PoseStamped> plan_local_pruned;
	// this prunes plan if appropriate parameter is set
	planner_util_->getLocalPlan(pose_stamped, plan_local_pruned);

	// NOTE: global_plan_ is not stored currently!
	// base_local_planner::prunePlan(poseTf, plan_local_, global_plan_);

	if (plan_local_pruned.size() > 0) {
		// find a point far enough so the social force can drive the robot towards instead of produce oscillations
		for (const auto& pose_plan : plan_local_pruned) {
			double dist_x = pose_plan.pose.position.x - pose_.getX();
			double dist_y = pose_plan.pose.position.y - pose_.getY();
			double dist_sq = dist_x * dist_x + dist_y * dist_y;
			if (dist_sq > cfg_->getGeneral()->forward_point_distance) {
				goal_local_ = Pose(pose_plan.pose);
				return true;
			}
		}
	}
	return false;
}


void HuberoPlanner::collectTrajectoryMotionData() {
	motion_data_.behaviour_active = generator_.getActiveFuzzyBehaviour();
	motion_data_.closest_points = generator_.getRobotObstacleDistances();
	motion_data_.force_combined =
		generator_.getForceInternal()
		+ generator_.getForceInteraction()
		+ generator_.getForceSocial();
	motion_data_.force_interaction = generator_.getForceInteraction();
	motion_data_.force_internal = generator_.getForceInternal();
	motion_data_.force_social = generator_.getForceSocial();
}

}; // namespace hubero_local_planner
