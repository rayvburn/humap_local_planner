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
	robot_model_(robot_model),
	obstacle_costs_(planner_util_->getCostmap()),
	path_costs_(planner_util_->getCostmap()),
	goal_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
	alignment_costs_(planner_util_->getCostmap()),
	ttc_costs_(world_model_)
{
	ros::NodeHandle private_nh("~/" + name);
	printf("[HuberoPlanner::HuberoPlanner] ctor, name: %s \r\n", name.c_str());

	// prints Fuzzy Inference System configuration, FIS used for trajectory generation
	generator_.printFisConfiguration();

	// configure trajectory critics
	alignment_costs_.setStopOnFailure(false);
	// whether footprint cost is summed throughout each point or maximum cost is used, max by default
	obstacle_costs_.setSumScores(false);
	oscillation_costs_.resetOscillationFlags();

	// set up all the cost functions that will be applied in order
	// (any function returning negative values will abort scoring, so the order can improve performance)
	std::vector<base_local_planner::TrajectoryCostFunction*> critics;
	critics.push_back(&obstacle_costs_);
	critics.push_back(&path_costs_);
	critics.push_back(&goal_costs_);
	critics.push_back(&alignment_costs_);
	critics.push_back(&ttc_costs_);

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

	updateCostParameters();
}

bool HuberoPlanner::checkTrajectory(
		const Eigen::Vector3f pos,
		const Eigen::Vector3f vel,
		const Eigen::Vector3f vel_samples) {
	printf("[HuberoPlanner::checkTrajectory] \r\n");
	return false;
}

bool HuberoPlanner::updatePlan(const geometry_msgs::PoseStamped& global_pose) {
	// make sure that our configuration doesn't change mid-run
	std::lock_guard<std::mutex> l(configuration_mutex_);

	// save for later use
	pose_ = Pose(global_pose);

	std::vector<geometry_msgs::PoseStamped> plan_local_pruned;
	// this prunes plan if appropriate parameter is set
	planner_util_->getLocalPlan(global_pose, plan_local_pruned);

	if (plan_local_pruned.empty()) {
		ROS_ERROR_NAMED("HuberoPlanner", "Could not update global plan - pruned plan is empty");
		return false;
	}

	// copy contents of the new global plan
	global_plan_.resize(plan_local_pruned.size());
	std::copy(plan_local_pruned.cbegin(), plan_local_pruned.cend(), global_plan_.begin());

	// try to update goal pose
	goal_ = getGoalFromPlan();

	// update local goal pose
	if (!chooseLocalGoal()) {
		ROS_ERROR_NAMED("HuberoPlanner", "Could not choose local goal during plan update");
		return false;
	}
	return true;
}

void HuberoPlanner::updateLocalCosts(const std::vector<geometry_msgs::Point>& footprint_spec) {
	// update robot footprint so it won't crash with anything
	obstacle_costs_.setFootprint(footprint_spec);
	// costs for going away from path
	path_costs_.setTargetPoses(global_plan_);
	// costs for not going towards the local goal as much as possible
	goal_costs_.setTargetPoses(global_plan_);
	// costs for robot being aligned with path (nose on path)
	alignment_costs_.setTargetPoses(global_plan_);
	// reset TTC datasets collected during previous iteration
	ttc_costs_.reset();
}

base_local_planner::Trajectory HuberoPlanner::findBestTrajectory(
	const Vector& velocity,
	const ObstContainerConstPtr obstacles,
	geometry_msgs::PoseStamped& drive_velocities
) {
	// make sure that our configuration doesn't change mid-run
	std::lock_guard<std::mutex> l(configuration_mutex_);

	// assign, will likely be useful for planning
	vel_ = velocity;
	obstacles_ = obstacles;

	// velocity transformation - from base coordinate system to planner's frame (global velocity vector)
	Vector robot_vel_glob;
	computeVelocityGlobal(vel_, pose_, robot_vel_glob);

	world_model_ = World(pose_, robot_vel_glob, goal_local_, goal_);
	createEnvironmentModel(pose_);

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
	traj_explored_.clear();
	bool traj_valid = scored_sampling_planner_.findBestTrajectory(result_traj_, &traj_explored_);

	ROS_DEBUG_NAMED(
		"HuberoPlanner",
		"Explored %lu trajectories. Best trajectory consists of %u points and has cost of %2.2f. "
		"Commands base with x: %2.3f, y: %2.3f, th: %2.3f",
		traj_explored_.size(),
		result_traj_.getPointsSize(),
		result_traj_.cost_,
		result_traj_.xv_,
		result_traj_.yv_,
		result_traj_.thetav_
	);

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
		ROS_ERROR_NAMED(
			"HuberoPlanner",
			"Could not find a valid trajectory (cost %2.2f), applying zero velocity to base",
			result_traj_.cost_
		);
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
	const Vector& velocity,
	const ObstContainerConstPtr obstacles,
	geometry_msgs::PoseStamped& drive_velocities
) {
	// make sure that our configuration doesn't change mid-run
	std::lock_guard<std::mutex> l(configuration_mutex_);

	vel_ = velocity;
	obstacles_ = obstacles;

	// velocity transformation - from base coordinate system to planner's frame (global velocity vector)
	Vector robot_vel_glob;
	computeVelocityGlobal(vel_, pose_, robot_vel_glob);

	world_model_ = World(pose_, robot_vel_glob, goal_local_, goal_);
	createEnvironmentModel(pose_);

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

bool HuberoPlanner::computeCellCost(
	int cx,
	int cy,
	float& path_cost,
	float& goal_cost,
	float& occ_cost,
	float& total_cost
) {
	// initial checks if cell is traversible
	path_cost = path_costs_.getCellCosts(cx, cy);
	goal_cost = goal_costs_.getCellCosts(cx, cy);
	occ_cost = planner_util_->getCostmap()->getCost(cx, cy);

	// evaluate if any component's cost is critical
	bool cell_unreachable =
		path_cost == path_costs_.obstacleCosts()
		|| path_cost == path_costs_.unreachableCellCosts()
		|| occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

	if (cell_unreachable) {
		return false;
	}

	// scale and sum retrieved costs
	path_cost *= cfg_->getCost()->path_distance_scale;
	goal_cost *= cfg_->getCost()->goal_distance_scale;
	occ_cost *= cfg_->getCost()->occdist_scale;
	total_cost = path_cost + goal_cost + occ_cost;
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

void HuberoPlanner::updateCostParameters() {
	// update cost scales (adjust them with costmap resolution)
	double cm_resolution = planner_util_->getCostmap()->getResolution();
	double occdist_scale_adjusted = cfg_->getCost()->occdist_scale * cm_resolution;
	double path_distance_scale_adjusted = cfg_->getCost()->path_distance_scale * cm_resolution;
	double goal_distance_scale_adjusted = cfg_->getCost()->goal_distance_scale * cm_resolution;

	obstacle_costs_.setScale(occdist_scale_adjusted);
	path_costs_.setScale(path_distance_scale_adjusted);
	goal_costs_.setScale(goal_distance_scale_adjusted);
	alignment_costs_.setScale(path_distance_scale_adjusted);
	ttc_costs_.setScale(cfg_->getCost()->ttc_scale);

	// update other cost params
	oscillation_costs_.setOscillationResetDist(
		cfg_->getCost()->oscillation_reset_dist,
		cfg_->getCost()->oscillation_reset_angle
	);
	obstacle_costs_.setParams(
		cfg_->getLimits()->max_vel_trans,
		cfg_->getCost()->max_scaling_factor,
		cfg_->getCost()->scaling_speed
	);
	alignment_costs_.setXShift(cfg_->getGeneral()->forward_point_distance);
	ttc_costs_.setParameters(cfg_->getCost()->ttc_rollout_time, cfg_->getCost()->ttc_collision_distance);
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

bool HuberoPlanner::chooseLocalGoal() {
	if (global_plan_.empty()) {
		ROS_ERROR_NAMED("HuberoPlanner", "Cannot choose local goal because global plan is empty");
		return false;
	}

	// find a point far enough so the social force can drive the robot towards instead of produce oscillations
	for (const auto& pose_plan: global_plan_) {
		double dist_x = pose_plan.pose.position.x - pose_.getX();
		double dist_y = pose_plan.pose.position.y - pose_.getY();
		double dist_sq = dist_x * dist_x + dist_y * dist_y;
		if (dist_sq > cfg_->getGeneral()->forward_point_distance) {
			goal_local_ = Pose(pose_plan.pose);
			return true;
		}
	}

	// seems that we have reached the furthest point in the plan
	goal_local_ = getGoalFromPlan();
	return true;
}

Pose HuberoPlanner::getGoalFromPlan() const {
	// it's safer to use second to last element, last may consist some strange pose,
	// e.g. point backwards or unreachable goal pose
	if (global_plan_.size() > 2) {
		return Pose(global_plan_.end()[-2].pose);
	}

	ROS_DEBUG_NAMED(
		"HuberoPlanner",
		"Cannot retrieve goal from the plan since it is too short (%lu poses)",
		global_plan_.size()
	);
	return goal_;
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
