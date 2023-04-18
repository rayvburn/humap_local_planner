#include <hubero_local_planner/hubero_planner.h>
#include <hubero_local_planner/utils/transformations.h>
#include <hubero_local_planner/sfm/social_force_model.h>

#include <math.h>

namespace hubero_local_planner {

HuberoPlanner::HuberoPlanner(
		const std::string& name,
		std::shared_ptr<base_local_planner::LocalPlannerUtil> planner_util,
		RobotFootprintModelPtr robot_model,
		std::vector<geometry_msgs::Point> footprint_spec
):
	state_ptr_(nullptr),
	planner_util_(planner_util),
	goal_reached_(false),
	obstacles_(nullptr),
	// people_ must be initialized, otherwise heading_disturbance_costs_ cannot see a valid ptr
	people_(std::make_shared<const People>()),
	groups_(std::make_shared<const Groups>()),
	robot_model_(robot_model),
	obstacle_costs_(planner_util_->getCostmap()),
	path_costs_(planner_util_->getCostmap()),
	goal_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
	goal_front_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
	alignment_costs_(planner_util_->getCostmap()),
	backward_costs_(0.0),
	ttc_costs_(world_model_),
	speedy_goal_costs_(goal_),
	velocity_smoothness_costs_(vel_),
	contextualized_costs_(planner_util_->getCostmap())
{
	ros::NodeHandle private_nh("~/" + name);
	printf("[HuberoPlanner::HuberoPlanner] ctor, name: %s \r\n", name.c_str());

	// avoid initializing global goal with zeros as it may be a valid initial goal
	goal_global_frame_.pose.position.x = NAN;
	goal_global_frame_.pose.position.y = NAN;
	goal_global_frame_.pose.orientation.w = NAN;

	// prints Fuzzy Inference System configuration, FIS used for trajectory generation
	generator_social_.printFisConfiguration();

	// configure trajectory critics
	/*
	 * Disabling of `setStopOnFailure` dramatically helps reaching goals hidden by an obstacle (wavefront propagation
	 * cannot reach it so a valid trajectory may be rejected)
	 */
	path_costs_.setStopOnFailure(false);
	goal_costs_.setStopOnFailure(false);
	goal_front_costs_.setStopOnFailure(false);
	alignment_costs_.setStopOnFailure(false);
	// whether footprint cost is summed throughout each point or maximum cost is used, max by default
	obstacle_costs_.setSumScores(false);
	contextualized_costs_.setSumScores(false);
	oscillation_costs_.resetOscillationFlags();

	// set up all the cost functions that will be applied in order
	// (any function returning negative values will abort scoring, so the order can improve performance)
	std::vector<base_local_planner::TrajectoryCostFunction*> critics;
	critics.push_back(&obstacle_costs_);
	critics.push_back(&path_costs_);
	critics.push_back(&goal_costs_);
	critics.push_back(&alignment_costs_);
	critics.push_back(&goal_front_costs_);
	critics.push_back(&backward_costs_);
	critics.push_back(&ttc_costs_);
	critics.push_back(&heading_change_smoothness_costs_);
	critics.push_back(&speedy_goal_costs_);
	critics.push_back(&velocity_smoothness_costs_);
	critics.push_back(&contextualized_costs_);
	critics.push_back(&heading_disturbance_costs_);
	critics.push_back(&personal_space_costs_);
	critics.push_back(&fformation_space_costs_);
	critics.push_back(&passing_speed_costs_);

	// trajectory generators
	std::vector<base_local_planner::TrajectorySampleGenerator*> generator_list;
	generator_list.push_back(&generator_social_);
	generator_list.push_back(&generator_vel_space_);

	// score sampled trajectories
	scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(
		generator_list,
		critics,
		-1, // unlimited number of samples allowed
		true // always use all provided generators
	);

	// helper functions for PlannerState
	auto goal_reached_fun = std::function<bool()>([&]{
		geometry_msgs::PoseStamped velocity;
		velocity.pose = Pose(vel_.getX(), vel_.getY(), vel_.getZ()).getAsMsgPose();
		geometry_msgs::PoseStamped pose;
		pose.pose = pose_.getAsMsgPose();
		geometry_msgs::PoseStamped goal_pose;
		goal_pose.pose = goal_.getAsMsgPose();
		return stop_rotate_controller_.isGoalReached(goal_pose, velocity, pose, *cfg_->getLimits());
	});
	auto pos_reached_fun = std::function<bool()>([&]{
		geometry_msgs::PoseStamped pose;
		pose.pose = pose_.getAsMsgPose();
		geometry_msgs::PoseStamped goal_pose;
		goal_pose.pose = goal_.getAsMsgPose();
		return stop_rotate_controller_.isPositionReached(goal_pose, pose, cfg_->getLimits()->xy_goal_tolerance);
	});

	state_ptr_ = std::make_unique<PlannerState>(
		pose_,
		goal_,
		goal_local_,
		goal_initiation_,
		pos_reached_fun,
		goal_reached_fun
	);
}

HuberoPlanner::~HuberoPlanner() {
	printf("[HuberoPlanner::HuberoPlanner] dtor \r\n");
}

void HuberoPlanner::reconfigure(HuberoConfigConstPtr cfg) {
	// make sure that our configuration doesn't change mid-run
	std::lock_guard<std::mutex> l(configuration_mutex_);
	cfg_ = cfg;

	generator_social_.setParameters(
		cfg->getSfm(),
		cfg->getFis(),
		cfg->getGeneral()->sim_time,
		cfg->getGeneral()->sim_granularity,
		cfg->getGeneral()->angular_sim_granularity,
		cfg->getGeneral()->sim_period,
		cfg->getDiagnostics()->log_trajectory_generation_samples,
		cfg->getDiagnostics()->log_trajectory_generation_details,
		cfg->getDiagnostics()->log_trajectory_generation_fails
	);

	generator_vel_space_.setParameters(
		cfg->getGeneral()->sim_time,
		cfg->getGeneral()->sim_granularity,
		cfg->getGeneral()->angular_sim_granularity,
		true, // DO NOT limit the velocities to those that do not overshoot goal in sim_time
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

	// update movement initiation pose - located slightly further than xy_goal_tolerance
	goal_initiation_ = getPoseFromPlan(1.1 * cfg_->getLimits()->xy_goal_tolerance);

	// update local goal pose - located at parameterized distance from the current pose
	goal_local_ = getPoseFromPlan(cfg_->getGeneral()->local_goal_distance);
	return true;
}

void HuberoPlanner::updateLocalCosts(const std::vector<geometry_msgs::Point>& footprint_spec) {
	// update robot footprint so it won't crash with anything
	obstacle_costs_.setFootprint(footprint_spec);
	// costs for going away from path
	path_costs_.setTargetPoses(global_plan_);
	// costs for not going towards the local goal as much as possible
	goal_costs_.setTargetPoses(global_plan_);

	/*
	 * NOTE: dwa_local_planner source code mentions that alignment_costs_ should have variable scales assigned, as in:
	 * https://github.com/ros-planning/navigation/blob/noetic-devel/dwa_local_planner/src/dwa_planner.cpp#L282
	 */
	// costs for robot being aligned with path (nose on path)
	alignment_costs_.setTargetPoses(global_plan_);

	/*
	 * Based on DWAPlanner::updatePlanAndLocalCosts authored by Eitan Marder-Eppstein
	 * We want the robot nose to be drawn to its final position before robot turns towards goal orientation
	 * Also, `goal_front_costs_` uses `Last` as `CostAggregationType`
	 */
	static constexpr double FRONT_PLAN_POSES_NUM = 10;
	std::vector<geometry_msgs::PoseStamped> front_global_plan;
	for (
		double dist = 0.0;
		dist <= cfg_->getCost()->forward_point_distance;
		dist += cfg_->getCost()->forward_point_distance / FRONT_PLAN_POSES_NUM
	) {
		geometry_msgs::PoseStamped plan_pose;
		plan_pose.header.frame_id = planner_util_->getGlobalFrame();
		plan_pose.pose = getPoseFromPlan(dist).getAsMsgPose();
		front_global_plan.push_back(plan_pose);
	}
	goal_front_costs_.setTargetPoses(front_global_plan);

	// reset TTC datasets collected during previous iteration
	ttc_costs_.reset();

	// update cost function with the people detections dataset
	heading_disturbance_costs_.setPeopleDetections(*people_);
	personal_space_costs_.setPeopleDetections(*people_);
	fformation_space_costs_.setFformationsDetections(*groups_);
	passing_speed_costs_.setPeopleDetections(*people_);
}

base_local_planner::Trajectory HuberoPlanner::findBestTrajectory(
	const Vector& velocity,
	const ObstContainerConstPtr obstacles,
	std::shared_ptr<const People> people,
	std::shared_ptr<const Groups> groups,
	geometry_msgs::PoseStamped& drive_velocities
) {
	// make sure that our configuration doesn't change mid-run
	std::lock_guard<std::mutex> l(configuration_mutex_);

	// assign, will likely be useful for planning
	vel_ = velocity;
	obstacles_ = obstacles;
	people_ = people;
	groups_ = groups;

	// velocity transformation - from base coordinate system to planner's frame (global velocity vector)
	Vector robot_vel_glob;
	computeVelocityGlobal(vel_, pose_, robot_vel_glob);

	world_model_ = World(pose_, robot_vel_glob, goal_local_, goal_);
	createEnvironmentModel(pose_, world_model_);

	// update internal state to decide which behaviour should be selected for operation
	state_ptr_->update();

	// find trajectory and store results in `result_traj_`
	bool traj_valid = false;
	switch (state_ptr_->getState()) {
		case PlannerState::STOPPED:
			traj_valid = true;
			break;
		case PlannerState::INITIATE_EXECUTION:
			traj_valid = planOrientationAdjustment(goal_initiation_);
			break;
		case PlannerState::ADJUST_ORIENTATION:
			traj_valid = planOrientationAdjustment(goal_);
			break;
		case PlannerState::MOVE:
			traj_valid = planMovingRobot();
			break;
		default:
			break;
	}

	// debrief stateful scoring functions
	oscillation_costs_.updateOscillationFlags(
		pose_.getAsEigen2D(),
		&result_traj_,
		cfg_->getLimits()->min_vel_trans
	);

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
	std::shared_ptr<const People> people,
	std::shared_ptr<const Groups> groups,
	geometry_msgs::PoseStamped& drive_velocities
) {
	// make sure that our configuration doesn't change mid-run
	std::lock_guard<std::mutex> l(configuration_mutex_);

	vel_ = velocity;
	obstacles_ = obstacles;
	people_ = people;
	groups_ = groups;

	// velocity transformation - from base coordinate system to planner's frame (global velocity vector)
	Vector robot_vel_glob;
	computeVelocityGlobal(vel_, pose_, robot_vel_glob);

	world_model_ = World(pose_, robot_vel_glob, goal_local_, goal_);
	createEnvironmentModel(pose_, world_model_);

	// initialize generator with updated parameters
	generator_social_.initialise(
		world_model_,
		velocity,
		cfg_->getLimits(),
		cfg_->getSfm()->mass,
		true // discretize by time
	);

	// trajectory that stores velocity commands, contains only seed vels
	base_local_planner::Trajectory traj;

	// perform SFM and fuzzy logic computations without planning
	generator_social_.generateTrajectoryWithoutPlanning(traj);

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

	// when we get a new plan, we also want to clear any latch we may have on goal tolerances
	stop_rotate_controller_.resetLatching();

	bool plan_updated = planner_util_->setPlan(orig_global_plan);

	// get goal pose transformed to the local frame (usually 'odom')
	geometry_msgs::PoseStamped goal_new;
	if (!planner_util_->getGoal(goal_new)) {
		// can't update the goal stored locally
		return plan_updated;
	}

	// next step is the new goal detection (necessary for switching between FSM states)
	// first, make sure that the given global plan is not empty and frames are the same
	if (orig_global_plan.empty()
		|| (!goal_global_frame_.header.frame_id.empty()
		&& goal_global_frame_.header.frame_id != orig_global_plan.back().header.frame_id)
	) {
		return plan_updated;
	}

	// compare goals expressed in global frames, those are better references compared to poses in local frame
	// heuristics to detect a new global goal
	auto goal_global_prev = Pose(goal_global_frame_);
	auto goal_global_new = Pose(orig_global_plan.back());
	double goal_xy_diff = (goal_global_prev.getPosition() - goal_global_new.getPosition()).calculateLength();
	double goal_yaw_diff = angles::shortest_angular_distance(goal_global_prev.getYaw(), goal_global_new.getYaw());

	// update for further comparisons
	goal_global_frame_ = orig_global_plan.back();

	// use goal tolerances to detect goal pose relocation
	if (goal_xy_diff >= cfg_->getLimits()->xy_goal_tolerance
		|| std::isnan(goal_xy_diff)
		|| std::abs(goal_yaw_diff) >= cfg_->getLimits()->yaw_goal_tolerance
		|| std::isnan(goal_yaw_diff)
	) {
		// update both goals (global and local, see call below)
		goal_ = Pose(goal_new);
		// this is not perfectly accurate - we use the last pose instead of the first from the global plan (which must
		// have been transformed first); reason to do this is that a local goals must be updated before state update
		geometry_msgs::PoseStamped pose_msg;
		pose_msg.pose = pose_.getAsMsgPose();
		pose_msg.header = goal_new.header;
		updatePlan(pose_msg);
		// immediately notify planner state handler about this event
		state_ptr_->update(true);
		return plan_updated;
	}
	/*
	 * Frequently update local goal to avoid problem with goal relocation that is caused by global pose estimate correction.
	 * When the global pose estimate significantly moved the robot's 'odom' frame and the local goal will not be valid anymore
	 */
	goal_ = Pose(goal_new);
	return plan_updated;
}

bool HuberoPlanner::checkGoalReached(
	const geometry_msgs::PoseStamped& pose,
	const geometry_msgs::PoseStamped& velocity,
	const geometry_msgs::PoseStamped& goal
) {
	goal_reached_ = state_ptr_->isGoalReached();
	return goal_reached_;
}

geometry::Vector HuberoPlanner::computeForceAtPosition(const Vector& pos) {
	// velocity transformation - from base coordinate system to planner's frame (global velocity vector)
	Vector robot_vel_glob;
	computeVelocityGlobal(vel_, pose_, robot_vel_glob);

	// simulated position with current orientation
	Pose pose(pos, pose_.getOrientation());

	auto world_model = World(pose, robot_vel_glob, goal_local_, goal_);
	createEnvironmentModel(pose, world_model);

	generator_social_.initialise(
		world_model,
		vel_,
		cfg_->getLimits(),
		cfg_->getSfm()->mass,
		true // discretize by time
	);

	base_local_planner::Trajectory traj;
	generator_social_.generateTrajectoryWithoutPlanning(traj);

	return generator_social_.getForceInternal()
		+ generator_social_.getForceInteractionStatic()
		+ generator_social_.getForceInteractionDynamic()
		+ generator_social_.getForceSocial();
}

// static
bool HuberoPlanner::enlargeObstacle(
	const Pose& robot_closest_to_obstacle_pose,
	Pose& obstacle_closest_to_robot_pose,
	double extension_distance,
	double distance_collision_imminent
) {
	// first, check if any action will be valid (distance vector will really be extended)
	if (extension_distance <= 0.0) {
		return false;
	}

	Vector dist_init(obstacle_closest_to_robot_pose.getPosition() - robot_closest_to_obstacle_pose.getPosition());
	// check if initial distance is smaller than collision distance; if true, artificial movement of the obstacle will not do any better
	if (dist_init.calculateLength() <= distance_collision_imminent) {
		return false;
	}

	// create unit vector with direction equal to `dist_init`
	Angle dist_init_dir(dist_init);
	Vector obstacle_extension_v(dist_init_dir);
	// scale the vector length
	obstacle_extension_v *= extension_distance;

	// make sure that vector connecting robot and object does not change direction
	auto obstacle_pose_hypothesis = Pose(
		obstacle_closest_to_robot_pose.getPosition() - obstacle_extension_v,
		obstacle_closest_to_robot_pose.getOrientation()
	);
	Vector dist_modded(obstacle_pose_hypothesis.getPosition() - robot_closest_to_obstacle_pose.getPosition());
	Angle dist_modded_dir(dist_modded);
	double dist_angle_diff = std::abs(dist_modded_dir.getRadian() - dist_init_dir.getRadian());

	// check if vector is not inverted
	if (dist_angle_diff <= IGN_DTOR(1.0)) {
		obstacle_closest_to_robot_pose = obstacle_pose_hypothesis;
		return true;
	}

	// try to keep the obstacle point close to the robot (few cm) but do not allow it to be placed within footprint
	obstacle_extension_v = Vector(dist_init_dir);
	obstacle_pose_hypothesis = Pose(
		robot_closest_to_obstacle_pose.getPosition() + distance_collision_imminent * obstacle_extension_v,
		obstacle_closest_to_robot_pose.getOrientation()
	);
	dist_modded = Vector(
		obstacle_closest_to_robot_pose.getPosition() - robot_closest_to_obstacle_pose.getPosition()
	);
	dist_modded_dir = Angle(dist_modded);
	dist_angle_diff = std::abs(dist_modded_dir.getRadian() - dist_init_dir.getRadian());

	if (dist_angle_diff <= IGN_DTOR(1.0)) {
		obstacle_closest_to_robot_pose = obstacle_pose_hypothesis;
		return true;
	}
	return false;
}

void HuberoPlanner::updateCostParameters() {
	// update cost scales (adjust them with costmap resolution)
	double cm_resolution = planner_util_->getCostmap()->getResolution();
	double occdist_scale_adjusted = cfg_->getCost()->occdist_scale * cm_resolution;
	double path_distance_scale_adjusted = cfg_->getCost()->path_distance_scale * cm_resolution;
	double goal_distance_scale_adjusted = cfg_->getCost()->goal_distance_scale * cm_resolution;
	double alignment_scale_adjusted = cfg_->getCost()->alignment_scale * cm_resolution;
	double goal_front_scale_adjusted = cfg_->getCost()->goal_front_scale * cm_resolution;

	obstacle_costs_.setScale(occdist_scale_adjusted);
	path_costs_.setScale(path_distance_scale_adjusted);
	goal_costs_.setScale(goal_distance_scale_adjusted);
	goal_front_costs_.setScale(goal_front_scale_adjusted);
	alignment_costs_.setScale(alignment_scale_adjusted);
	backward_costs_.setScale(cfg_->getCost()->backward_scale);
	ttc_costs_.setScale(cfg_->getCost()->ttc_scale);
	heading_change_smoothness_costs_.setScale(cfg_->getCost()->heading_change_smoothness_scale);
	speedy_goal_costs_.setScale(cfg_->getCost()->speedy_goal_scale);
	velocity_smoothness_costs_.setScale(cfg_->getCost()->velocity_smoothness_scale);
	contextualized_costs_.setScale(cfg_->getCost()->contextualized_costs_scale);
	heading_disturbance_costs_.setScale(cfg_->getCost()->heading_dir_scale);
	personal_space_costs_.setScale(cfg_->getCost()->personal_space_scale);
	fformation_space_costs_.setScale(cfg_->getCost()->fformation_space_scale);
	passing_speed_costs_.setScale(cfg_->getCost()->passing_speed_scale);

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
	goal_front_costs_.setXShift(cfg_->getCost()->forward_point_distance);
	alignment_costs_.setXShift(cfg_->getCost()->forward_point_distance);
	backward_costs_.setPenalty(cfg_->getCost()->backward_penalty);
	ttc_costs_.setParameters(cfg_->getCost()->ttc_rollout_time, cfg_->getCost()->ttc_collision_distance);
	speedy_goal_costs_.setParameters(cfg_->getCost()->speedy_goal_distance, cfg_->getLimits()->min_vel_trans);
	heading_disturbance_costs_.setParameters(
		// creates value of the full FOV
		2.0 * cfg_->getGeneral()->person_fov,
		cfg_->getGeneral()->person_model_radius,
		// TODO: for non-circular robots this won't be a valid circumradius
		robot_model_->getInscribedRadius(),
		cfg_->getLimits()->max_vel_trans
	);
}

void HuberoPlanner::createEnvironmentModel(const Pose& pose_ref, World& world_model) {
	/*
	 * People are detected independently from obstacles so they must be erased from obstacles.
	 * Obstacle is treated as a person when most of its points are located within the given radius.
	 */
	static constexpr double PERSON_POLYGON_CONTAINMENT_RATE = 0.667;

	ObstContainer obstacles;
	for (const ObstaclePtr obstacle: *obstacles_) {
		// create a temporary polygon to find distances between person center and its boundary points
		geometry_msgs::Polygon polygon;
		obstacle->toPolygonMsg(polygon);

		// vector of containment rates (within people radiuses) of the obstacle
		std::vector<double> containment_rates;

		for (const auto& person: *people_) {
			// number of polygon points that are located within a radius of person model
			int polygon_pts_within_radius = 0;

			for (const auto& pt: polygon.points) {
				double dist = std::hypot(pt.x - person.getPositionX(), pt.y - person.getPositionY());
				if (dist <= cfg_->getGeneral()->person_model_radius) {
					polygon_pts_within_radius++;
				}
			}

			// if a majority of polygon pts is located within radius - abort this obstacle, otherwise add to container
			double containment_rate = static_cast<double>(polygon_pts_within_radius) / polygon.points.size();
			containment_rates.push_back(containment_rate);
		}

		if (
			!containment_rates.empty()
			&&
			*std::max_element(containment_rates.cbegin(), containment_rates.cend()) >= PERSON_POLYGON_CONTAINMENT_RATE
		) {
			continue;
		}
		obstacles.push_back(obstacle);
	}

	// fill sparse world model with obstacles
	teb_local_planner::PoseSE2 pose = pose_ref.getAsTebPose();
	for (const ObstaclePtr obstacle: obstacles) {
		BaseRobotFootprintModel::ClosestPoints pts = robot_model_->calculateClosestPoints(pose, obstacle.get());

		Pose robot_closest_to_obstacle_pose(pts.robot);
		Pose obstacle_closest_to_robot_pose(pts.obstacle);

		/*
		 * Robot size was already accounted in ClosestPoints calculation, however
		 * slight extension of obstacle size helps with avoidance of locks.
		 * In fact, these locks are not related to local minimae, but with directing robot to locations from which
		 * robot cannot produce collision-free trajectories.
		 * Extending an obstacle also compensates poor costmap resolution in some cases.
		 */
		HuberoPlanner::enlargeObstacle(
			robot_closest_to_obstacle_pose,
			obstacle_closest_to_robot_pose,
			// extension determined based on multiplied inscribed radius distance
			cfg_->getGeneral()->obstacle_extension_multiplier * robot_model_->getInscribedRadius(),
			// as threshold distance we use slightly extended TTC collision threshold
			1.05 * cfg_->getCost()->ttc_collision_distance
		);

		Vector model_vel = Vector(
			(obstacle->getCentroidVelocity())[0],
			(obstacle->getCentroidVelocity())[1],
			0.0
		);

		bool force_dynamic_object_interpretation =
				cfg_->getSfm()->static_obj_interaction == sfm::StaticObjectInteraction::INTERACTION_REPULSIVE_EVASIVE;

		world_model.addObstacle(
			robot_closest_to_obstacle_pose,
			obstacle_closest_to_robot_pose,
			model_vel,
			force_dynamic_object_interpretation
		);
	}

	// represent human with an obstacle
	for (const auto& person: *people_) {
		ObstaclePtr person_model_ptr = std::make_shared<CircularObstacle>(
			person.getPositionX(),
			person.getPositionY(),
			cfg_->getGeneral()->person_model_radius
		);

		auto pts = robot_model_->calculateClosestPoints(pose, person_model_ptr.get());

		Pose robot_closest_to_obstacle_pose(pts.robot);
		Pose obstacle_closest_to_robot_pose(pts.obstacle);

		bool force_dynamic_object_interpretation =
				cfg_->getSfm()->static_obj_interaction == sfm::StaticObjectInteraction::INTERACTION_REPULSIVE_EVASIVE;

		world_model.addObstacle(
			robot_closest_to_obstacle_pose,
			obstacle_closest_to_robot_pose,
			person.getVelocity(),
			force_dynamic_object_interpretation
		);
	}

	ROS_DEBUG_NAMED(
		"HuberoPlanner",
		"Created environment model with %2lu objects extracted from world and %2lu people (%2lu static and %2lu dynamic obstacles)",
		obstacles.size(),
		people_->size(),
		world_model.getStaticObjectsData().size(),
		world_model.getDynamicObjectsData().size()
	);
}

Pose HuberoPlanner::getPoseFromPlan(const double& dist_from_current_pose) const {
	if (global_plan_.empty()) {
		ROS_ERROR_NAMED("HuberoPlanner", "Cannot choose local goal because global plan is empty");
		return Pose();
	}

	// find a point far enough so the social force can drive the robot towards instead of produce oscillations
	for (
		std::vector<geometry_msgs::PoseStamped>::const_iterator it = global_plan_.begin();
		it != global_plan_.end();
		++it
	) {
		double dist_x = it->pose.position.x - pose_.getX();
		double dist_y = it->pose.position.y - pose_.getY();
		double dist_sq = dist_x * dist_x + dist_y * dist_y;
		if (dist_sq > dist_from_current_pose) {
			// global path does not account orientation - interpolate
			auto it_prev = std::prev(it);
			// make sure that iterator is valid (within vector range)
			if (it_prev >= global_plan_.begin()) {
				// orientation of the local goal is determined based on direction that allows to pass
				// through 2 consecutive path points (local goal and the previous one)
				Vector p0(it_prev->pose.position.x, it_prev->pose.position.y, it_prev->pose.position.z);
				Vector p1(it->pose.position.x, it->pose.position.y, it->pose.position.z);
				Angle dir_v = (p1 - p0).calculateDirection();
				return Pose(
					Vector(it->pose.position.x, it->pose.position.y, it->pose.position.z),
					Quaternion(dir_v.getRadian())
				);
			}
			// fallback - cannot determine orientation, global goal orientation is assumed
			return Pose(
				Vector(
					it->pose.position.x,
					it->pose.position.y,
					it->pose.position.z
				),
				Quaternion(
					global_plan_.back().pose.orientation.x,
					global_plan_.back().pose.orientation.y,
					global_plan_.back().pose.orientation.z,
					global_plan_.back().pose.orientation.w
				)
			);
		}
	}
	// cannot determine orientation, global goal orientation is assumed
	return Pose(
		Vector(
			global_plan_.back().pose.position.x,
			global_plan_.back().pose.position.y,
			global_plan_.back().pose.position.z
		),
		Quaternion(
			global_plan_.back().pose.orientation.x,
			global_plan_.back().pose.orientation.y,
			global_plan_.back().pose.orientation.z,
			global_plan_.back().pose.orientation.w
		)
	);
}

bool HuberoPlanner::planMovingRobot() {
	// initialize generator with updated parameters
	generator_social_.initialise(
		world_model_,
		vel_,
		*cfg_->getTrajectorySampling(),
		cfg_->getLimits(),
		cfg_->getSfm()->mass,
		true // discretize by time
	);

	if (cfg_->getTrajectoryGeneration()->use_equisampled_velocities_generator) {
		Eigen::Vector3f vsamples;
		vsamples[0] = cfg_->getTrajectoryGeneration()->equisampled_vx;
		vsamples[1] = cfg_->getTrajectoryGeneration()->equisampled_vy;
		vsamples[2] = cfg_->getTrajectoryGeneration()->equisampled_vth;

		// must copy limits since SimpleTrajectoryGenerator::initialise is not marked as const input;
		// limits must exist until at least end of this method
		base_local_planner::LocalPlannerLimits limits = *cfg_->getLimits();

		/*
		 * This generator shouldn't create in-place trajectories as they are often highest rated by cost functions
		 *
		 * Let `min_vel_x` comply with acceleration limits, try to keep it bigger than
		 * `equisampled_min_vel_x` and `limits.min_vel_x` if possible
		 *
		 * TODO: this is prepared only for non-holonomic mobile bases
		 */
		auto maximum_from_min_vel_x = std::max(
			// do not violate kinematic requirements/constraints
			std::max(
				cfg_->getTrajectoryGeneration()->equisampled_min_vel_x,
				limits.min_vel_x
			),
			// keep velocities as high as possible
			std::max(
				cfg_->getTrajectoryGeneration()->equisampled_min_vel_x,
				vel_.getX() - limits.acc_lim_x * cfg_->getGeneral()->sim_period
			)
		);
		// do not limit velocities to maximum (this is most likely not necessary)
		limits.min_vel_x = std::min(
			maximum_from_min_vel_x,
			limits.max_vel_x
		);

		generator_vel_space_.initialise(
			pose_.getAsEigen2D(),
			vel_.getAsEigen<Eigen::Vector3f>(),
			goal_.getAsEigen2D(),
			&limits,
			vsamples,
			true // use_acceleration_limits
		);
	}

	// find best trajectory by sampling and scoring the samples, `scored_sampling_planner_` uses `generator_social_` internally
	result_traj_.cost_ = -7;
	result_traj_.resetPoints();
	traj_explored_.clear();
	bool traj_valid = scored_sampling_planner_.findBestTrajectory(result_traj_, &traj_explored_);

	logTrajectoriesDetails();

	collectTrajectoryMotionData();

	return traj_valid;
}

bool HuberoPlanner::planOrientationAdjustment(const Pose& goal) {
	// NOTE: LatchedStopRotateController assumes that all received poses are expressed in the same frame,
	// headers of stamped types are not required then
	geometry_msgs::PoseStamped velocity;
	velocity.pose = Pose(vel_.getX(), vel_.getY(), vel_.getZ()).getAsMsgPose();

	geometry_msgs::PoseStamped pose;
	pose.pose = pose_.getAsMsgPose();

	geometry_msgs::PoseStamped goal_pose;
	goal_pose.pose = goal.getAsMsgPose();

	// base_local_planner::LocalPlannerLimits::getAccLimits is not marked as `const`...
	Eigen::Vector3f acc_limits;
	acc_limits[0] = cfg_->getLimits()->acc_lim_x;
	acc_limits[1] = cfg_->getLimits()->acc_lim_y;
	acc_limits[2] = cfg_->getLimits()->acc_lim_theta;

	// output
	geometry_msgs::Twist cmd_vel;

	bool traj_valid = stop_rotate_controller_.computeVelocityCommandsStopRotate(
		cmd_vel,
		acc_limits,
		cfg_->getGeneral()->sim_period,
		goal_pose,
		velocity,
		pose,
		*cfg_->getLimits(),
		std::bind(
			&HuberoPlanner::checkInPlaceTrajectory,
			this,
			std::placeholders::_1,
			std::placeholders::_2,
			std::placeholders::_3
		)
	);

	result_traj_.resetPoints();
	result_traj_.time_delta_ = cfg_->getGeneral()->sim_period;
	result_traj_.xv_ = cmd_vel.linear.x;
	result_traj_.yv_ = cmd_vel.linear.y;
	result_traj_.thetav_ = cmd_vel.angular.z;
	// negative cost when trajectory is invalid
	result_traj_.cost_ = traj_valid ? 0.0 : -1.0;

	// update explored trajectories (1 valid trajectory will be added to the list)
	traj_explored_.clear();
	traj_explored_.push_back(result_traj_);
	return traj_valid;
}

bool HuberoPlanner::checkInPlaceTrajectory(
	const Eigen::Vector3f pos,
	const Eigen::Vector3f vel,
	const Eigen::Vector3f vel_samples
) {
	oscillation_costs_.resetOscillationFlags();

	geometry_msgs::PoseStamped goal_pose = global_plan_.back();
	Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));

	// number of samples along each direction (x, y, theta), arbitrarily chosen
	Eigen::Vector3f v_sample_num;
	v_sample_num[0] = 1;
	v_sample_num[1] = 1;
	v_sample_num[2] = 10;

	// struct must be copied to comply with existing interface
	base_local_planner::LocalPlannerLimits limits = *cfg_->getLimits();

	// create new, simple trajectory generator
	base_local_planner::SimpleTrajectoryGenerator traj_gen;
	traj_gen.initialise(
		pos,
		vel,
		goal,
		&limits,
		v_sample_num
	);

	std::vector<base_local_planner::TrajectorySampleGenerator*> traj_gen_list;
	traj_gen_list.push_back(&traj_gen);

	// use class member critics
	std::vector<base_local_planner::TrajectoryCostFunction*> critics;
	critics.push_back(&obstacle_costs_);
	critics.push_back(&path_costs_);
	critics.push_back(&goal_costs_);
	critics.push_back(&alignment_costs_);
	critics.push_back(&goal_front_costs_);
	critics.push_back(&heading_change_smoothness_costs_);
	critics.push_back(&speedy_goal_costs_);

	base_local_planner::SimpleScoredSamplingPlanner traj_scorer(traj_gen_list, critics);

	base_local_planner::Trajectory traj;
	traj_gen.generateTrajectory(pos, vel, vel_samples, traj);
	double cost = traj_scorer.scoreTrajectory(traj, -1);

	//if the trajectory is a legal one... the check passes
	if (cost >= 0) {
		ROS_INFO_COND_NAMED(
			cfg_->getDiagnostics()->log_explored_trajectories,
			"HuberoPlanner",
			"Selected trajectory for stop & rotate mode - x: %2.3f, y: %2.3f, th: %2.3f (cost %2.2f)",
			vel_samples[0],
			vel_samples[1],
			vel_samples[2],
			cost
		);
		return true;
	}

	ROS_ERROR_COND_NAMED(
		cfg_->getDiagnostics()->log_explored_trajectories,
		"HuberoPlanner",
		"Invalid Trajectory for stop & rotate mode - x: %2.3f, y: %2.3f, th: %2.3f (cost: %2.2f)",
		vel_samples[0],
		vel_samples[1],
		vel_samples[2],
		cost
	);

	// otherwise the check fails
	return false;
}

void HuberoPlanner::collectTrajectoryMotionData() {
	motion_data_.behaviour_active_ = generator_social_.getActiveFuzzyBehaviour();
	motion_data_.closest_points_static_ = generator_social_.getRobotStaticObstacleDistances();
	motion_data_.closest_points_dynamic_ = generator_social_.getRobotDynamicObstacleDistances();
	motion_data_.force_combined_ =
		generator_social_.getForceInternal()
		+ generator_social_.getForceInteractionDynamic()
		+ generator_social_.getForceInteractionStatic()
		+ generator_social_.getForceSocial();
	motion_data_.force_interaction_ = generator_social_.getForceInteractionStatic() + generator_social_.getForceInteractionDynamic();
	motion_data_.force_internal_ = generator_social_.getForceInternal();
	motion_data_.force_social_ = generator_social_.getForceSocial();
}

void HuberoPlanner::logTrajectoriesDetails() {
	if (result_traj_.cost_ < 0) {
		ROS_ERROR_COND_NAMED(
			cfg_->getDiagnostics()->log_explored_trajectories,
			"HuberoPlanner",
			"Explored %lu trajectories, but couldn't find a valid one (cost %2.2f)",
			traj_explored_.size(),
			result_traj_.cost_
		);
	} else {
		ROS_INFO_COND_NAMED(
			cfg_->getDiagnostics()->log_explored_trajectories,
			"HuberoPlanner",
			"Explored %lu trajectories. Best trajectory consists of %u points and has cost of %2.5f. "
			"Commands base with x: %2.3f, y: %2.3f, th: %2.3f",
			traj_explored_.size(),
			result_traj_.getPointsSize(),
			result_traj_.cost_,
			result_traj_.xv_,
			result_traj_.yv_,
			result_traj_.thetav_
		);
	}

	if (!cfg_->getDiagnostics()->log_explored_trajectories && !cfg_->getDiagnostics()->log_pts_of_explored_trajectories) {
		return;
	}

	// investigate costs and velocities of all explored trajectories
	int traj_num = 0;
	for (const auto& traj: traj_explored_) {
		double traj_x, traj_y, traj_th = 0.0;
		traj.getEndpoint(traj_x, traj_y, traj_th);

		// print basic trajectory info
		ROS_INFO_COND_NAMED(
			cfg_->getDiagnostics()->log_explored_trajectories,
			"HuberoPlanner",
			"%sExplored trajectory %3d / %3lu: cost %2.5f, vel {x %2.3f, y %2.3f, th %2.3f}, "
			"points %u, end {x %2.2f, y %2.2f, th %2.2f}%s",
			result_traj_.cost_ == traj.cost_ ? "\033[32m" : "", // mark the best trajectory green
			++traj_num,
			traj_explored_.size(),
			traj.cost_,
			traj.xv_, traj.yv_, traj.thetav_,
			traj.getPointsSize(),
			traj_x, traj_y, traj_th,
			result_traj_.cost_ == traj.cost_ ? "\033[0m" : "" // reset the output colorized green
		);

		// base_local_planner::TrajectoryCostFunction::scoreTrajectory is not marked const
		auto traj_copy = traj;

		// evaluate components contribution in total cost of the trajectory
		ROS_INFO_COND_NAMED(
			cfg_->getDiagnostics()->log_trajectory_cost_details,
			"HuberoPlanner",
			"%sExplored trajectory %3d / %3lu cost details: "
			"obstacle %2.2f, oscillation %2.2f, path %2.2f, goal %2.2f, goal_front %2.2f, alignment %2.2f, "
			"backward %2.2f, TTC %2.2f, HSM %2.2f, speedy_goal %2.2f, vel_smoothness %2.2f, context %2.2f, "
			"head_dir %2.2f, PSI %2.2f, FSI %2.2f, PSPD %2.2f%s",
			result_traj_.cost_ == traj.cost_ ? "\033[32m" : "", // mark the best trajectory green
			traj_num,
			traj_explored_.size(),
			obstacle_costs_.getScale() * obstacle_costs_.scoreTrajectory(traj_copy),
			oscillation_costs_.getScale() * oscillation_costs_.scoreTrajectory(traj_copy),
			path_costs_.getScale() * path_costs_.scoreTrajectory(traj_copy),
			goal_costs_.getScale() * goal_costs_.scoreTrajectory(traj_copy),
			goal_front_costs_.getScale() * goal_front_costs_.scoreTrajectory(traj_copy),
			alignment_costs_.getScale() * alignment_costs_.scoreTrajectory(traj_copy),
			backward_costs_.getScale() * backward_costs_.scoreTrajectory(traj_copy),
			ttc_costs_.getScale() * ttc_costs_.scoreTrajectory(traj_copy),
			heading_change_smoothness_costs_.getScale() * heading_change_smoothness_costs_.scoreTrajectory(traj_copy),
			speedy_goal_costs_.getScale() * speedy_goal_costs_.scoreTrajectory(traj_copy),
			velocity_smoothness_costs_.getScale() * velocity_smoothness_costs_.scoreTrajectory(traj_copy),
			contextualized_costs_.getScale() * contextualized_costs_.scoreTrajectory(traj_copy),
			heading_disturbance_costs_.getScale() * heading_disturbance_costs_.scoreTrajectory(traj_copy),
			personal_space_costs_.getScale() * personal_space_costs_.scoreTrajectory(traj_copy),
			fformation_space_costs_.getScale() * fformation_space_costs_.scoreTrajectory(traj_copy),
			passing_speed_costs_.getScale() * passing_speed_costs_.scoreTrajectory(traj_copy),
			result_traj_.cost_ == traj.cost_ ? "\033[0m" : "" // reset the output colorized green
		);

		// skip if points of explored trajectories are not needed to be printed
		if (!cfg_->getDiagnostics()->log_pts_of_explored_trajectories) {
			continue;
		}

		// also print trajectory points
		for (unsigned int pt_num = 0; pt_num < traj.getPointsSize(); pt_num++) {
			double x, y, th = 0.0;
			traj.getPoint(pt_num, x, y, th);
			ROS_INFO_NAMED(
				"HuberoPlanner",
				"\tpoint %3d / %3u: x %4.7f, y %4.7f, th %4.7f",
				pt_num + 1,
				traj.getPointsSize(),
				x,
				y,
				th
			);
		}
	}
}

}; // namespace hubero_local_planner
