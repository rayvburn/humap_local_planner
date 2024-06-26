#include <humap_local_planner/humap_planner.h>
#include <humap_local_planner/utils/transformations.h>
#include <humap_local_planner/sfm/social_force_model.h>

#include <math.h>

namespace humap_local_planner {

HumapPlanner::HumapPlanner(
		const std::string& name,
		std::shared_ptr<base_local_planner::LocalPlannerUtil> planner_util,
		RobotFootprintModelPtr robot_model,
		std::vector<geometry_msgs::Point> footprint_spec
):
	state_ptr_(nullptr),
	planner_util_(planner_util),
	goal_reached_(false),
	obstacles_(nullptr),
	people_(std::make_shared<const People>()),
	groups_(std::make_shared<const Groups>()),
	robot_model_(robot_model),
	trajectories_num_total_(0),
	trajectories_num_social_gen_(0),
	scales_cm_costs_(
		cfg_ ? *cfg_->getCost() : CostParams{}, // in case of the uninitialized shared pointer, use the default values
		planner_util_->getCostmap()->getResolution()
	),
	obstacle_costs_(planner_util_->getCostmap()),
	path_costs_(planner_util_->getCostmap()),
	goal_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
	goal_front_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
	alignment_costs_(planner_util_->getCostmap()),
	backward_costs_(0.0),
	ttc_costs_(world_model_),
	heading_change_smoothness_costs_(vel_),
	velocity_smoothness_costs_(vel_),
	heading_disturbance_costs_(people_env_model_),
	personal_space_costs_(people_env_model_),
	fformation_space_costs_(groups_env_model_),
	passing_speed_costs_(people_env_model_)
{
	ros::NodeHandle private_nh("~/" + name);
	printf("[HumapPlanner::HumapPlanner] ctor, name: %s \r\n", name.c_str());

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
	oscillation_costs_.resetOscillationFlags();

	// set up all the cost functions that will be applied in order
	// (any function returning negative values will abort scoring, so the order can improve performance)
	std::vector<base_local_planner::TrajectoryCostFunction*> critics;
	critics.push_back(&obstacle_costs_);
	critics.push_back(&path_costs_);
	critics.push_back(&goal_costs_);
	critics.push_back(&alignment_costs_);
	critics.push_back(&goal_front_costs_);
	critics.push_back(&unsaturated_trans_costs_);
	critics.push_back(&backward_costs_);
	critics.push_back(&ttc_costs_);
	critics.push_back(&heading_change_smoothness_costs_);
	critics.push_back(&velocity_smoothness_costs_);
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
		double dist = base_local_planner::getGoalPositionDistance(
			pose,
			goal_.getX(),
			goal_.getY()
		);
		double dist_threshold = XY_GOAL_TOLERANCE_MULTIPLIER * cfg_->getLimits()->xy_goal_tolerance;
		return dist <= dist_threshold;
	});
	auto crossing_detected_fun = std::function<bool()>([&]{
		// also check if the closest dynamic person is far enough (to prevent switching states immediately)
		double dist_threshold = 2.0 * cfg_->getGeneral()->person_model_radius;
		return crossing_.isCrossingDetected() && crossing_.getGapToClosestPerson() <= dist_threshold;
	});
	auto yield_way_crossing_finished_fun = std::function<bool()>([&]{
		return !yield_way_crossing_.isActive();
	});
	auto is_oscillating_fun = std::function<bool()>([&]{
		return recovery_.isOscillating();
	});
	auto is_stuck_fun = std::function<bool()>([&]{
		return recovery_.isStuck();
	});
	auto is_deviating_from_ref_path_fun = std::function<bool()>([&]{
		return recovery_.isDeviatingFromGlobalPath();
	});
	auto near_collision_fun = std::function<bool()>([&]{
		return recovery_.isNearCollision();
	});
	auto can_recover_fun = std::function<bool()>([&]{
		return recovery_.canRecover();
	});
	auto is_global_plan_outdated_fun = std::function<bool()>([&]{
		return recovery_.isGlobalPathPlanOutdated();
	});
	auto is_look_around_finished_fun = std::function<bool()>([&]{
		return !recovery_.isLookAroundRecoveryActive();
	});

	state_ptr_ = std::make_unique<PlannerState>(
		pose_,
		goal_,
		goal_local_,
		goal_initiation_,
		pos_reached_fun,
		goal_reached_fun,
		crossing_detected_fun,
		yield_way_crossing_finished_fun,
		is_look_around_finished_fun,
		is_oscillating_fun,
		is_stuck_fun,
		is_deviating_from_ref_path_fun,
		near_collision_fun,
		can_recover_fun,
		is_global_plan_outdated_fun
	);
}

HumapPlanner::~HumapPlanner() {
	double stg_percentage = (100.0 * trajectories_num_social_gen_) / static_cast<double>(trajectories_num_total_);
	printf(
		"HumapPlanner destructor: %5.2f%% of selected trajectories were generated by the SocialTrajectoryGenerator\r\n",
		stg_percentage
	);
}

void HumapPlanner::reconfigure(HumapConfigConstPtr cfg) {
	// make sure that our configuration doesn't change mid-run
	std::lock_guard<std::mutex> l(cfg->getMutex());
	cfg_ = cfg;

	generator_social_.setParameters(
		cfg->getSfm(),
		cfg->getFis(),
		cfg->getGeneral()->sim_time,
		cfg->getGeneral()->sim_granularity,
		cfg->getGeneral()->angular_sim_granularity,
		cfg->getGeneral()->sim_period,
		cfg->getLimits()->maintain_vel_components_rate,
		cfg->getDiagnostics()->log_trajectory_generation_samples,
		cfg->getDiagnostics()->log_trajectory_generation_details,
		cfg->getDiagnostics()->log_trajectory_generation_fails,
		cfg->getDiagnostics()->log_trajectory_generation_forces
	);

	generator_vel_space_.setParameters(
		cfg->getGeneral()->sim_time,
		cfg->getGeneral()->sim_granularity,
		cfg->getGeneral()->angular_sim_granularity,
		// whether to limit the velocities to those that do not overshoot goal in sim_time; inverted logic here
		!cfg->getTrajectoryGeneration()->equisampled_continued_acceleration,
		cfg->getGeneral()->sim_period
	);

	updateCostParameters();

	crossing_.setParameters(
		cfg_->getGeneral()->person_model_radius,
		robot_model_->getInscribedRadius(),
		std::max(cfg_->getCost()->occdist_separation, 0.025),
		cfg_->getPlannerBehaviors()->path_crossing_confidence_threshold,
		cfg_->getPlannerBehaviors()->path_crossing_front_stddev,
		PathCrossingDetector::SAFE_POINT_DISTANCE_MULTIPLIER_DEFAULT,
		cfg_->getPlannerBehaviors()->path_crossing_person_speed_threshold
	);

	// keep N sec of history (buffer length not parameterized)
	recovery_.setOscillationBufferLength(
		static_cast<int>(std::ceil(
			cfg_->getPlannerBehaviors()->oscillation_buffer_length / cfg_->getGeneral()->sim_period
		))
	);
	recovery_.setGlobalPathPlanMaxAge(
		cfg_->getGeneral()->path_planning_period * cfg_->getPlannerBehaviors()->path_plan_update_violation_multiplier
	);
}

bool HumapPlanner::checkTrajectory(
	const Eigen::Vector3f pos,
	const Eigen::Vector3f vel,
	const Eigen::Vector3f vel_samples
) {
	geometry_msgs::PoseStamped goal_pose = global_plan_.back();
	Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));

	// no samples along each direction - zeroing at least 1 component disables collecting the sample params (desirable)
	Eigen::Vector3f v_sample_num;
	v_sample_num[0] = 0;
	v_sample_num[1] = 0;
	v_sample_num[2] = 0;

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

	base_local_planner::SimpleScoredSamplingPlanner traj_scorer(traj_gen_list, critics);

	base_local_planner::Trajectory traj;
	traj_gen.generateTrajectory(pos, vel, vel_samples, traj);
	double cost = traj_scorer.scoreTrajectory(traj, -1);

	//if the trajectory is a legal one... the check passes
	if (cost >= 0) {
		ROS_INFO_COND_NAMED(
			cfg_->getDiagnostics()->log_explored_trajectories,
			"HumapPlanner",
			"Valid trajectory was checked - x: %2.3f, y: %2.3f, th: %2.3f (cost %2.2f)",
			vel_samples[0],
			vel_samples[1],
			vel_samples[2],
			cost
		);
		return true;
	}

	ROS_ERROR_COND_NAMED(
		cfg_->getDiagnostics()->log_explored_trajectories,
		"HumapPlanner",
		"Invalid trajectory was checked - x: %2.3f, y: %2.3f, th: %2.3f (cost: %2.2f)",
		vel_samples[0],
		vel_samples[1],
		vel_samples[2],
		cost
	);

	// otherwise the check fails
	return false;
}

bool HumapPlanner::updatePlan(const geometry_msgs::PoseStamped& global_pose) {
	// make sure that our configuration doesn't change mid-run
	std::lock_guard<std::mutex> l(cfg_->getMutex());

	// save for later use
	pose_ = Pose(global_pose);

	std::vector<geometry_msgs::PoseStamped> plan_local_pruned;
	// this prunes plan if appropriate parameter is set
	planner_util_->getLocalPlan(global_pose, plan_local_pruned);

	if (plan_local_pruned.empty()) {
		ROS_ERROR_NAMED("HumapPlanner", "Could not update global plan - pruned plan is empty");
		return false;
	}

	// copy contents of the new global plan
	global_plan_.resize(plan_local_pruned.size());
	std::copy(plan_local_pruned.cbegin(), plan_local_pruned.cend(), global_plan_.begin());

	// update movement initiation pose - located slightly further than xy_goal_tolerance
	goal_initiation_ = getPoseFromPlan(1.1 * cfg_->getLimits()->xy_goal_tolerance, false, true);

	// update local goal pose - located at parameterized distance from the current pose;
	// compute local goal distance according to the distance achievable within the planning horizon;
	// the local goal is placed at the maximum distance the robot can travel within the planning horizon
	double dist_achievable = computeDistanceLimits(
		pose_,
		vel_,
		cfg_->getGeneral()->sim_time,
		cfg_->getGeneral()->sim_period,
		cfg_->getLimits()->acc_lim_x,
		cfg_->getLimits()->acc_lim_y,
		cfg_->getLimits()->acc_lim_theta,
		cfg_->getLimits()->min_vel_x,
		cfg_->getLimits()->min_vel_y,
		-cfg_->getLimits()->max_vel_theta,
		cfg_->getLimits()->max_vel_x,
		cfg_->getLimits()->max_vel_y,
		cfg_->getLimits()->max_vel_theta
	);
	// local goal should be in front of the robot (after rotating according to the motion initiation goal it will
	// always be), therefore last arg. is false
	goal_local_ = getPoseFromPlan(cfg_->getGeneral()->local_goal_distance_multiplier * dist_achievable, true, false);
	return true;
}

base_local_planner::Trajectory HumapPlanner::findBestTrajectory(
	const std::vector<geometry_msgs::Point>& footprint_spec,
	const Vector& velocity,
	const ObstContainerConstPtr obstacles,
	std::shared_ptr<const People> people,
	std::shared_ptr<const Groups> groups,
	geometry_msgs::PoseStamped& drive_velocities
) {
	// make sure that our configuration doesn't change mid-run
	std::lock_guard<std::mutex> l(cfg_->getMutex());

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

	updateLocalCosts(footprint_spec);

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
		case PlannerState::YIELD_WAY_CROSSING:
			traj_valid = planYieldWayCrossing();
			break;
		case PlannerState::RECOVERY_ROTATE_AND_RECEDE:
			traj_valid = planRecoveryRotateAndRecede();
			break;
		case PlannerState::RECOVERY_LOOK_AROUND:
			traj_valid = planRecoveryLookAround();
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
			"HumapPlanner",
			"Could not find a valid trajectory (cost %.2f) among %lu trajectories. Applying zero velocity to the mobile base.",
			result_traj_.cost_,
			traj_explored_.size()
		);
	} else {
		drive_velocities.pose.position.x = result_traj_.xv_;
		drive_velocities.pose.position.y = result_traj_.yv_;
		drive_velocities.pose.position.z = 0;
		tf2::Quaternion q;
		q.setRPY(0, 0, result_traj_.thetav_);
		tf2::convert(q, drive_velocities.pose.orientation);
	}

	// post-planning faults detection
	recovery_.checkOscillation(
		(!traj_valid || result_traj_.cost_ < 0) ? 0.0 : result_traj_.xv_,
		(!traj_valid || result_traj_.cost_ < 0) ? 0.0 : result_traj_.thetav_,
		cfg_->getLimits()->max_vel_x,
		cfg_->getLimits()->min_vel_x < 0.0 ? std::abs(cfg_->getLimits()->min_vel_x) : 0.0,
		cfg_->getLimits()->max_vel_theta,
		// translational velocity epsilon for oscillation detection
		std::abs(cfg_->getLimits()->min_vel_x) / 5.0,
		// rotational velocity epsilon for oscillation detection
		std::abs(cfg_->getLimits()->min_vel_theta) / 5.0
	);

	recovery_.checkDeviation(
		// select the best trajectory or the first explored (its first pose is crucial)
		(result_traj_.cost_ < 0 && !traj_explored_.empty()) ? traj_explored_.front() : result_traj_,
		goal_front_costs_,
		alignment_costs_
	);

	recovery_.checkCollision(
		// select the best trajectory or first explored (its first pose is crucial)
		(result_traj_.cost_ < 0 && !traj_explored_.empty()) ? traj_explored_.front() : result_traj_,
		obstacle_costs_
	);

	recovery_.computeGlobalPlanAge();

	auto traj_robot_g = Trajectory(result_traj_, true);
	auto traj_robot_poses = traj_robot_g.getPoses();
	std::vector<Trajectory> trajs_people;
	for (const auto& person: people_env_model_) {
		auto traj_person = person.getTrajectoryPrediction();
		trajs_people.push_back(traj_person);
	}
	if (crossing_.detect(traj_robot_g, global_plan_, trajs_people)) {
		ROS_WARN_NAMED(
			"HumapPlanner",
			"Detected the intersection of robot's and %lu people's trajectories. Gap to the closest person is %5.2f m.",
			crossing_.getCrossingPeopleData().size(),
			crossing_.getGapToClosestPerson()
		);
	}

	geometry_msgs::PoseStamped goal_pose;
	goal_pose.pose = goal_.getAsMsgPose();
	group_intrusion_.detect(groups_env_model_, global_plan_, goal_pose);
	return result_traj_;
}

base_local_planner::Trajectory HumapPlanner::findTrajectory(
	const Vector& velocity,
	const ObstContainerConstPtr obstacles,
	std::shared_ptr<const People> people,
	std::shared_ptr<const Groups> groups,
	geometry_msgs::PoseStamped& drive_velocities
) {
	// make sure that our configuration doesn't change mid-run
	std::lock_guard<std::mutex> l(cfg_->getMutex());

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

bool HumapPlanner::computeCellCost(int cx, int cy, std::map<std::string, float>& costs) {
	// initial checks if cell is traversible
	float path_cost = path_costs_.getCellCosts(cx, cy);
	float goal_cost = goal_costs_.getCellCosts(cx, cy);
	float occ_cost = obstacle_costs_.getFootprintCost(cx, cy);
	float align_cost = alignment_costs_.getCellCosts(cx, cy);
	float goal_front_cost = goal_front_costs_.getCellCosts(cx, cy);

	// evaluate if any component's cost is critical
	bool cell_unreachable =
		path_cost == path_costs_.obstacleCosts()
		|| path_cost == path_costs_.unreachableCellCosts()
		|| goal_cost == goal_costs_.obstacleCosts()
		|| goal_cost == goal_costs_.unreachableCellCosts()
		|| occ_cost >= costmap_2d::LETHAL_OBSTACLE
		|| occ_cost < costmap_2d::FREE_SPACE
		|| align_cost == alignment_costs_.obstacleCosts()
		|| align_cost == alignment_costs_.unreachableCellCosts()
		|| goal_front_cost == goal_front_costs_.obstacleCosts()
		|| goal_front_cost == goal_front_costs_.unreachableCellCosts();

	if (cell_unreachable) {
		return false;
	}

	// scale and sum retrieved costs
	path_cost *= path_costs_.getScale();
	goal_cost *= goal_costs_.getScale();
	occ_cost *= obstacle_costs_.getScale();
	align_cost *= alignment_costs_.getScale();
	goal_front_cost *= goal_front_costs_.getScale();
	float total_cost = path_cost + goal_cost + occ_cost + align_cost + goal_front_cost;

	// assign costs to the hashmap
	costs["path"] = path_cost;
	costs["goal"] = goal_cost;
	costs["layered"] = occ_cost;
	costs["alignment"] = align_cost;
	costs["goal_front"] = goal_front_cost;
	costs["total"] = total_cost;
	return true;
}

bool HumapPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
	// when we get a new plan, we also want to clear any latch we may have on goal tolerances
	stop_rotate_controller_.resetLatching();

	bool plan_updated = planner_util_->setPlan(orig_global_plan);
	recovery_.markGlobalPlanUpdate();

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

	// use goal tolerances to detect goal pose relocation
	if (goal_xy_diff >= cfg_->getLimits()->xy_goal_tolerance
		|| std::isnan(goal_xy_diff)
		|| std::abs(goal_yaw_diff) >= cfg_->getLimits()->yaw_goal_tolerance
		|| std::isnan(goal_yaw_diff)
	) {
		ROS_INFO_NAMED(
			"HumapPlanner",
			"Got a new goal at (x %6.3f, y %6.3f, th %6.3f)",
			goal_new.pose.position.x,
			goal_new.pose.position.y,
			tf2::getYaw(goal_new.pose.orientation)
		);
		// update for further comparisons
		goal_global_frame_ = orig_global_plan.back();
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

bool HumapPlanner::checkGoalReached(
	const geometry_msgs::PoseStamped& pose,
	const geometry_msgs::PoseStamped& velocity,
	const geometry_msgs::PoseStamped& goal
) {
	goal_reached_ = state_ptr_->isGoalReached();
	return goal_reached_;
}

geometry::Vector HumapPlanner::computeForceAtPosition(const Vector& pos) {
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

	if (generator_social_.getGeneratedTrajectories().empty()) {
		return geometry::Vector();
	}
	return generator_social_.getGeneratedTrajectories().at(0).force_total;
}

// static
bool HumapPlanner::enlargeObstacle(
	const Pose& robot_closest_to_obstacle_pose,
	Pose& obstacle_closest_to_robot_pose,
	double extension_distance,
	double distance_collision_imminent
) {
	// first, check if any action will be valid (whether distance vector could be extended)
	if (extension_distance <= 0.0) {
		return false;
	}

	Vector dist_init(obstacle_closest_to_robot_pose.getPosition() - robot_closest_to_obstacle_pose.getPosition());
	// check if initial distance is smaller than collision distance; if true, artificial movement of the obstacle will not do any better
	if (dist_init.calculateLength() <= distance_collision_imminent) {
		return false;
	}

	/*
	 * Main stage
	 *
	 * Move the initial `obstacle_closest_to_robot_pose` by the distance given by `extension_distance`.
	 *
	 * It is expected that resultant vector connecting closest points of the obstacle and the robot will have the same
	 * direction as the original one.
	 */
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

	/*
	 * Fallback stage
	 *
	 * Executed only when the resultant vector from the main stage changes direction (i.e. robot is very close
	 * to the obstacle and shifting causes a 'flip'). Therefore, try to shift the obstacle's edge point (closest
	 * to the robot) just a little.
	 *
	 * Again, it is expected that the new resultant vector connecting closest points of the obstacle and the robot will
	 * have the same direction as the original one.
	 */
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

	// evaluate direction of the new vector with the tolerance of 1 degree compared to the original one
	if (dist_angle_diff <= IGN_DTOR(1.0)) {
		obstacle_closest_to_robot_pose = obstacle_pose_hypothesis;
		return true;
	}
	return false;
}

// static
ObstContainer HumapPlanner::extractNonPeopleObstacles(
	const ObstContainer& obstacles,
	const People& people,
	double person_model_radius,
	double min_containment_rate
) {
	// container for obstacles that are not people's legs etc.
	ObstContainer obstacles_only;

	for (const ObstaclePtr obstacle: obstacles) {
		// create a temporary polygon to find distances between person center and its boundary points
		geometry_msgs::Polygon polygon;
		obstacle->toPolygonMsg(polygon);

		// vector of containment rates (within people radiuses) of the obstacle
		std::vector<double> containment_rates;

		for (const auto& person: people) {
			// number of polygon points that are located within a radius of person model
			int polygon_pts_within_radius = 0;

			for (const auto& pt: polygon.points) {
				double dist = std::hypot(pt.x - person.getPositionX(), pt.y - person.getPositionY());
				if (dist <= person_model_radius) {
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
			*std::max_element(containment_rates.cbegin(), containment_rates.cend()) >= min_containment_rate
		) {
			continue;
		}
		obstacles_only.push_back(obstacle);
	}
	return obstacles_only;
}

// static
double HumapPlanner::computeDistanceLimits(
	const geometry::Pose& pos_init,
	const geometry::Vector& vel_init,
	const double& sim_period,
	const double& sim_granularity,
	const double& acc_lim_x,
	const double& acc_lim_y,
	const double& acc_lim_th,
	const double& vel_min_x,
	const double& vel_min_y,
	const double& vel_min_th,
	const double& vel_max_x,
	const double& vel_max_y,
	const double& vel_max_th
) {
	// compute number of iterations for the full rollout
	unsigned int iter_num = std::ceil(sim_period / sim_granularity);

	std::vector<geometry::Vector> velocities;
	velocities.push_back(vel_init);

	// the longest distance between the first and the last position will be achieved when the linear velocities
	// are maxed out and angular rotation is ignored
	const geometry::Vector CMD_VEL_LIM(vel_max_x, vel_max_y, 0.0);

	// compute maximum velocities that are achievable within the sim_period
	for (unsigned int i = 0; i < iter_num; i++) {
		auto vel = velocities.back();

		// this is forward-focused check, backward motions are not investigated
		auto cmd_vel = CMD_VEL_LIM;
		// modifies `cmd_vel` so it conforms to acceleration limits
		adjustTwistWithAccLimits(
			vel,
			acc_lim_x,
			acc_lim_y,
			acc_lim_th,
			vel_min_x,
			vel_min_y,
			vel_min_th,
			vel_max_x,
			vel_max_y,
			vel_max_th,
			sim_granularity,
			cmd_vel,
			false
		);

		velocities.push_back(cmd_vel);
	}

	// predict poses
	std::vector<geometry::Pose> poses;
	poses.push_back(pos_init);
	for (const auto& vel: velocities) {
		auto pose = computeNextPoseBaseVel(poses.back(), vel, sim_granularity);
		poses.push_back(pose);
	}

	// distance between first and last pose
	double dist_pts = Vector(poses.back().getRawPosition() - poses.front().getRawPosition()).calculateLength();
	return dist_pts;
}

void HumapPlanner::updateCostParameters() {
	// update cost scales (adjust them with costmap resolution)
	double cm_resolution = planner_util_->getCostmap()->getResolution();
	scales_cm_costs_ = ScalesCmCostFunctions(*cfg_->getCost(), cm_resolution);
	path_costs_.setScale(scales_cm_costs_.path_distance_scale);
	goal_costs_.setScale(scales_cm_costs_.goal_distance_scale);
	goal_front_costs_.setScale(scales_cm_costs_.goal_front_scale);
	alignment_costs_.setScale(scales_cm_costs_.alignment_scale);

	obstacle_costs_.setScale(cfg_->getCost()->occdist_scale);
	unsaturated_trans_costs_.setScale(cfg_->getCost()->unsaturated_translation_scale);
	backward_costs_.setScale(cfg_->getCost()->backward_scale);
	ttc_costs_.setScale(cfg_->getCost()->ttc_scale);
	heading_change_smoothness_costs_.setScale(cfg_->getCost()->heading_change_smoothness_scale);
	velocity_smoothness_costs_.setScale(cfg_->getCost()->velocity_smoothness_scale);
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
		cfg_->getCost()->scaling_speed,
		cfg_->getCost()->occdist_separation,
		cfg_->getCost()->occdist_separation_kernel
	);
	obstacle_costs_.setSumScores(cfg_->getCost()->occdist_sum_scores);
	goal_front_costs_.setXShift(cfg_->getCost()->forward_point_distance);
	alignment_costs_.setXShift(cfg_->getCost()->forward_point_distance);
	unsaturated_trans_costs_.setParameters(
		cfg_->getLimits()->max_vel_trans,
		cfg_->getLimits()->max_vel_x,
		cfg_->getLimits()->max_vel_y,
		cfg_->getCost()->unsaturated_translation_compute_whole_horizon
	);
	backward_costs_.setPenalty(cfg_->getCost()->backward_penalty);
	ttc_costs_.setParameters(cfg_->getCost()->ttc_rollout_time, cfg_->getCost()->ttc_collision_distance);
	heading_disturbance_costs_.setParameters(
		// creates value of the full FOV
		2.0 * cfg_->getGeneral()->person_fov,
		cfg_->getGeneral()->person_model_radius,
		// TODO: for non-circular robots this won't be a valid circumradius
		robot_model_->getInscribedRadius(),
		cfg_->getLimits()->max_vel_trans,
		cfg_->getCost()->heading_dir_compute_whole_horizon
	);
	personal_space_costs_.setParameters(cfg_->getCost()->personal_space_compute_whole_horizon);
	fformation_space_costs_.setParameters(cfg_->getCost()->fformation_space_compute_whole_horizon);
	passing_speed_costs_.setParameters(
		cfg_->getLimits()->max_vel_trans,
		// TODO: for non-circular robots this won't be a valid circumradius
		robot_model_->getInscribedRadius(),
		cfg_->getCost()->passing_speed_compute_whole_horizon
	);
}

void HumapPlanner::createEnvironmentModel(const Pose& pose_ref, World& world_model) {
	// erase people from original set of obstacles set
	auto obstacles = HumapPlanner::extractNonPeopleObstacles(
		*obstacles_,
		*people_,
		cfg_->getGeneral()->person_model_radius,
		HumapPlanner::PERSON_POLYGON_CONTAINMENT_RATE
	);

	// selects N closest obstacles from @ref obstacles
	obstacles_env_model_ = selectRelevant<ObstaclePtr>(
		obstacles,
		[=](const ObstaclePtr& obj) -> double {
			// check the validity of the pointer
			if (!obj) {
				// make it negligible
				return std::numeric_limits<double>::max();
			}
			// obstacles are in the same frame as the robot -> transform not needed
			// FIXME robot is simplified to the point representation
			Eigen::Vector2d robot_pos(pose_.getX(), pose_.getY());
			return obj->getMinimumDistance(robot_pos);
		},
		cfg_->getGeneral()->obstacles_closest_polygons_num
	);
	// selects N closest humans from @ref people_
	people_env_model_ = selectRelevant<Person>(
		*people_,
		[=](const Person& obj) -> double {
			// FIXME objects are simplified to the points
			Vector v(obj.getPositionX() - pose_.getX(), obj.getPositionY() - pose_.getY());
			return v.calculateLength();
		},
		cfg_->getGeneral()->people_closest_num
	);
	// selects N closest groups from @ref groups_
	groups_env_model_ = selectRelevant<Group>(
		*groups_,
		[=](const Group& obj) -> double {
			// FIXME objects are simplified to the points
			Vector v(obj.getPositionX() - pose_.getX(), obj.getPositionY() - pose_.getY());
			return v.calculateLength();
		},
		cfg_->getGeneral()->groups_closest_num
	);

	// fill sparse world model with obstacles
	teb_local_planner::PoseSE2 pose = pose_ref.getAsTebPose();
	for (const ObstaclePtr obstacle: obstacles_env_model_) {
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
		HumapPlanner::enlargeObstacle(
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
	for (const auto& person: people_env_model_) {
		ObstaclePtr person_model_ptr = std::make_shared<CircularObstacle>(
			person.getPositionX(),
			person.getPositionY(),
			cfg_->getGeneral()->person_model_radius
		);

		auto pts = robot_model_->calculateClosestPoints(pose, person_model_ptr.get());

		Pose robot_closest_to_obstacle_pose(pts.robot);
		Pose obstacle_closest_to_robot_pose(pts.obstacle);

		world_model.addObstacle(
			robot_closest_to_obstacle_pose,
			obstacle_closest_to_robot_pose,
			person.getVelocity(),
			cfg_->getSfm()->human_force_formulation_dynamic
		);
	}

	ROS_DEBUG_NAMED(
		"HumapPlanner",
		"Created environment model from %2lu obstacles (out of %2lu total extracted from world), "
		"%2lu people (out of %2lu total), and %2lu people groups (out of %2lu total). "
		"Environment model consists of %2lu static and %2lu dynamic objects.",
		obstacles_env_model_.size(),
		obstacles_->size(),
		people_env_model_.size(),
		people_->size(),
		groups_env_model_.size(),
		groups_->size(),
		world_model.getStaticObjectsData().size(),
		world_model.getDynamicObjectsData().size()
	);
}

void HumapPlanner::updateLocalCosts(const std::vector<geometry_msgs::Point>& footprint_spec) {
	// some cost functions require variable scales based on the distance to the goal
	double dist_to_goal = (goal_.getPosition() - pose_.getPosition()).calculateLength();

	// update robot footprint so it won't crash with anything
	obstacle_costs_.setFootprint(footprint_spec);
	// costs for going away from path
	path_costs_.setTargetPoses(global_plan_);
	// costs for not going towards the local goal as much as possible
	goal_costs_.setTargetPoses(global_plan_);

	// costs for robot being aligned with path (nose on path)
	alignment_costs_.setTargetPoses(global_plan_);

	// limit the distance to the forward checkpoint if there are people nearby
	double min_gap_human_robot = std::numeric_limits<double>::max();
	for (const auto& person: people_env_model_) {
		// compute Euclidean distance between the human and the robot
		double distance = std::hypot(
			person.getPositionX() - pose_.getX(),
			person.getPositionY() - pose_.getY()
		);

		// estimate the actual space between the human and the robot
		double gap = distance - cfg_->getGeneral()->person_model_radius - robot_model_->getInscribedRadius();

		// select minimal but positive
		min_gap_human_robot = std::min(std::max(0.0, gap), min_gap_human_robot);
	}
	// do not allow the cost functions to score points that may be located within personal spaces or so
	double forward_point_distance = std::min(min_gap_human_robot, cfg_->getCost()->forward_point_distance);

	/*
	 * Based on DWAPlanner::updatePlanAndLocalCosts authored by Eitan Marder-Eppstein
	 * We want the robot nose to be drawn to its final position before robot turns towards goal orientation
	 * Also, `goal_front_costs_` uses `Last` as `CostAggregationType`
	 */
	auto plan_fwd_pose = getPoseFromPlan(forward_point_distance, false, false);
	std::vector<geometry_msgs::PoseStamped> front_global_plan;
	for (const auto& pose: global_plan_) {
		double diff_x = std::abs(pose.pose.position.x - plan_fwd_pose.getX());
		double diff_y = std::abs(pose.pose.position.y - plan_fwd_pose.getY());
		// target position tolerance
		static constexpr double POS_TOLERANCE = 1e-03;
		if (diff_x <= POS_TOLERANCE && diff_y <= POS_TOLERANCE) {
			break;
		}
		front_global_plan.push_back(pose);
	}
	goal_front_costs_.setTargetPoses(front_global_plan);
	goal_front_costs_.setXShift(forward_point_distance);

	// reduce scales of MapGridCostFunction-based cost functions when the robot is close to the goal
	if (dist_to_goal <= cfg_->getCost()->forward_point_distance) {
		alignment_costs_.setScale(0.0);
	} else {
		alignment_costs_.setScale(scales_cm_costs_.alignment_scale);
	}
	alignment_costs_.setXShift(forward_point_distance);

	// reset TTC datasets collected during previous iteration
	ttc_costs_.reset();

	if (!group_intrusion_.isGlobalGoalWithinGroup()) {
		fformation_space_costs_.setScale(cfg_->getCost()->fformation_space_scale);
	} else {
		// scale the cost based on the distance between the closest edges: of the group and the footprint
		double dist_to_group = std::max(
			group_intrusion_.getDistanceToGroupEdgeGoalWithin() - robot_model_->getInscribedRadius(),
			0.0
		);
		static const double EXPONENT_FACTOR = -0.7;
		double multiplier = 1.0 - std::exp(EXPONENT_FACTOR * dist_to_group);
		fformation_space_costs_.setScale(multiplier * cfg_->getCost()->fformation_space_scale);
	}

	// reduce the scale of cost function that favours higher linear velocities - prevents overshooting the goal pos.
	double unsaturated_trans_costs_dist_threshold = 3.0 * robot_model_->getInscribedRadius();
	if (dist_to_goal > unsaturated_trans_costs_dist_threshold) {
		unsaturated_trans_costs_.setScale(cfg_->getCost()->unsaturated_translation_scale);
	} else {
		double lin_slope_a = cfg_->getCost()->unsaturated_translation_scale / unsaturated_trans_costs_dist_threshold;
		double lin_val = lin_slope_a * dist_to_goal;
		// trim just in case
		double dynamic_scale = std::min(std::max(lin_val, 0.0), cfg_->getCost()->unsaturated_translation_scale);
		unsaturated_trans_costs_.setScale(dynamic_scale);
	}
}

Pose HumapPlanner::getPoseFromPlan(
	const double& dist_from_current_pose,
	bool allow_exceeding_plan,
	bool allow_poses_behind
) const {
	if (global_plan_.empty()) {
		ROS_ERROR_NAMED("HumapPlanner", "Cannot choose pose from the plan because global plan is empty");
		return Pose();
	}

	// helper function that computes squared distance between poses given by {x1, y1} and {x2, y2}
	auto computeSqDist = [](double x1, double y1, double x2, double y2) {
		double dist_x = x2 - x1;
		double dist_y = y2 - y1;
		double dist_sq = dist_x * dist_x + dist_y * dist_y;
		return dist_sq;
	};

	// computes Euclidean distance between 2 points
	auto computeDist = [computeSqDist](double x1, double y1, double x2, double y2) {
		return std::sqrt(computeSqDist(x1, y1, x2, y2));
	};

	// helper function that computes direction of a line constructed from subsequent poses {x1, y1} and {x2, y2}
	auto computeDirFromPositions = [](double x1, double y1, double x2, double y2) {
		Vector p1(x1, y1, 0.0);
		Vector p2(x2, y2, 0.0);
		Angle dir_v = (p2 - p1).calculateDirection();
		return dir_v.getRadian();
	};

	// how far we should land from the current pose
	double dist_target = dist_from_current_pose;
	// stores sum of distances between path poses
	double dist_sum_along_path = 0.0;

	// flag indicating that the global plan's poses that require pruning are not considered (skipped, not deleted);
	// typically, the global plan is not updated as frequently as the pose, so there will be an offset at the start
	bool pruned = allow_poses_behind; // typically initiates to false, but this prevents an extra 'if'

	// find a point far enough so the social force can drive the robot towards instead of produce oscillations
	for (
		// NOTE: starting from the [1] instead of [0]
		std::vector<geometry_msgs::PoseStamped>::const_iterator it = std::next(global_plan_.begin());
		it != global_plan_.end();
		++it
	) {
		// the iterator is always valid as we start from the second element
		auto it_prev = std::prev(it);

		// skip some poses if necessary
		if (!pruned) {
			// perform local 'pruning' - procedure below makes sure that the plan point lays in front of the robot;
			// position of the robot and its orientation is compared to the position of a plan point
			auto dir_to_plan_pos = computeDirFromPositions(
				pose_.getX(),
				pose_.getY(),
				it_prev->pose.position.x,
				it_prev->pose.position.y
			);
			// defines angle between the robot's heading and the location of the evaluated position
			double relative_loc_of_plan_pos = Angle(dir_to_plan_pos - pose_.getYaw()).getRadian();
			// turns true when the it->pose is not located behind the robot
			pruned = std::abs(relative_loc_of_plan_pos) <= IGN_PI_2;
			if (!pruned) {
				continue;
			}
		}

		// accumulate distances
		double dist = computeDist(
			it_prev->pose.position.x,
			it_prev->pose.position.y,
			it->pose.position.x,
			it->pose.position.y
		);
		dist_sum_along_path += dist;
		if (dist_sum_along_path > dist_target) {
			// global path does not account orientation - interpolate
			// orientation of the local goal is determined based on direction that allows to pass
			// through 2 consecutive path points (local goal and the previous one)
			double dir_v_angle = computeDirFromPositions(
				it_prev->pose.position.x,
				it_prev->pose.position.y,
				it->pose.position.x,
				it->pose.position.y
			);
			return Pose(
				Vector(it->pose.position.x, it->pose.position.y, it->pose.position.z),
				Quaternion(dir_v_angle)
			);
		}
	}
	// description of the conditions used below:
	// (1) do not proceed with plan extrapolation when it is not necessary
	// (2) even if `allow_exceeding_plan` is set to true, when the plan is too short, the procedure cannot be executed
	// (3) if the plan ends at the global goal, do not try to extend (here, distance to the global goal is compared
	//     with the requested distance from the robot)
	double dist_to_global_goal = computeDist(
		goal_.getX(),
		goal_.getY(),
		global_plan_.back().pose.position.x,
		global_plan_.back().pose.position.y
	);
	if (!allow_exceeding_plan || global_plan_.size() < 2 || dist_to_global_goal <= dist_from_current_pose) {
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

	// let's try to approximate poses beyond the plan -
	// at this point we are sure that the global plan consists of at least 2 poses;
	//
	// firstly, find out how far the requested pose should be from the last pose in the plan
	double dist_to_plan_end = computeDist(
		pose_.getX(), pose_.getY(),
		global_plan_.back().pose.position.x, global_plan_.back().pose.position.y
	);
	double dist_beyond_plan = dist_from_current_pose - dist_to_plan_end;

	// secondly, extrapolate plan, maintaining the direction of its end;
	// retrieve the second to last and the last elements of the plan, ref: https://stackoverflow.com/a/18540244
	double dir_v = computeDirFromPositions(
		global_plan_.end()[-2].pose.position.x, global_plan_.end()[-2].pose.position.y,
		global_plan_.end()[-1].pose.position.x, global_plan_.end()[-1].pose.position.y
	);
	// compute vector directed along the direction of the end of the plan
	Angle dir_v_angle(dir_v);
	Vector v_extension(dir_v_angle);
	// let the vector length and distance to the last achievable plan pose be equal to @ref dist_from_current_pose
	v_extension = v_extension * dist_beyond_plan;
	// find the target pose
	Pose pose_extrapolate_target(
		global_plan_.back().pose.position.x + v_extension.getX(),
		global_plan_.back().pose.position.y + v_extension.getY(),
		dir_v_angle.getRadian()
	);
	return pose_extrapolate_target;
}

bool HumapPlanner::planMovingRobot() {
	if (
		!cfg_->getTrajectoryGeneration()->use_social_trajectory_generator
		&& !cfg_->getTrajectoryGeneration()->use_equisampled_velocities_generator
	) {
		ROS_ERROR_NAMED(
			"HumapPlanner",
			"Both trajectory generators are disabled - no valid velocity command will be found"
		);
	}

	if (cfg_->getTrajectoryGeneration()->use_social_trajectory_generator) {
		// initialize generator with updated parameters
		generator_social_.initialise(
			world_model_,
			vel_,
			*cfg_->getTrajectorySampling(),
			cfg_->getLimits(),
			cfg_->getSfm()->mass,
			true // discretize by time
		);
	}

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

	// diagnostics only
	logTrajectoriesDetails();
	if (collectTrajectoryMotionData(result_traj_)) {
		trajectories_num_social_gen_++;
	}
	trajectories_num_total_++;

	return traj_valid;
}

bool HumapPlanner::planOrientationAdjustment(const Pose& goal) {
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
			&HumapPlanner::checkTrajectory,
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
	// add at least the current pose so the trajectory can be somehow scored
	result_traj_.addPoint(pose_.getX(), pose_.getY(), pose_.getYaw());

	// update explored trajectories (1 valid trajectory will be added to the list)
	traj_explored_.clear();
	traj_explored_.push_back(result_traj_);
	return traj_valid;
}

bool HumapPlanner::planYieldWayCrossing() {
	// update the class that helps performing the yielding routine
	if (!yield_way_crossing_.update(pose_, crossing_.getClosestSafeDirection())) {
		ROS_WARN_NAMED(
			"HumapPlanner",
			"Cannot find a feasible pose for yielding the way to a person that crosses the robot's planned path. "
			"The person might also become unobservable."
		);
		yield_way_crossing_.finish();
		return false;
	}

	base_local_planner::LocalPlannerLimits limits = *cfg_->getLimits();

	// investigate all cases when exiting from the state is possible (hoping that the global plan has been adjusted
	// and the robot won't be moving toward a person anymore)
	if (yield_way_crossing_.getGoalDistanceXy() < limits.xy_goal_tolerance) {
		yield_way_crossing_.finish();
		return true;
	}
	if (std::abs(yield_way_crossing_.getGoalDistanceYaw()) < limits.yaw_goal_tolerance) {
		yield_way_crossing_.finish();
		return true;
	}
	if (yield_way_crossing_.getTraveledDistance() > cfg_->getPlannerBehaviors()->yield_way_max_travel_distance) {
		yield_way_crossing_.finish();
		return true;
	}
	// gap will be "Inf" if no "dynamic" people nearby are detected
	if (crossing_.getGapToClosestPerson() > 2.0 * cfg_->getGeneral()->person_model_radius) {
		yield_way_crossing_.finish();
		return true;
	}

	// NOTE: LatchedStopRotateController assumes that all received poses are expressed in the same frame,
	// headers of stamped types are not required then
	geometry_msgs::PoseStamped velocity;
	velocity.pose = Pose(vel_.getX(), vel_.getY(), vel_.getZ()).getAsMsgPose();

	geometry_msgs::PoseStamped pose;
	pose.pose = pose_.getAsMsgPose();

	// base_local_planner::LocalPlannerLimits::getAccLimits is not marked as `const`...
	Eigen::Vector3f acc_limits;
	acc_limits[0] = limits.acc_lim_x;
	acc_limits[1] = limits.acc_lim_y;
	acc_limits[2] = limits.acc_lim_theta;

	// output
	geometry_msgs::Twist cmd_vel_rot;

	// reused the code that performs rotation including the acceleration limits - just to obtain the angular velocity
	bool rotate_valid = stop_rotate_controller_.rotateToGoal(
		pose,
		velocity,
		yield_way_crossing_.getGoalDirection(), // target orientation
		cmd_vel_rot,
		acc_limits,
		cfg_->getGeneral()->sim_period,
		limits,
		std::bind(
			&HumapPlanner::checkTrajectory,
			this,
			std::placeholders::_1,
			std::placeholders::_2,
			std::placeholders::_3
		)
	);

	// decelerate and start turning towards the safe position
	double vel_x_forward = std::abs(limits.min_vel_x);
	// but don't move faster than at the start of the routine
	vel_x_forward = std::min(vel_.getX(), vel_x_forward);

	// routine does not require strafing
	geometry::Vector cmd_vel(
		vel_x_forward,
		0.0,
		cmd_vel_rot.angular.z
	);

	// make the new velocity command be compliant with the kinematic and dynamic constraints
	adjustTwistWithAccLimits(
		vel_,
		limits.acc_lim_x,
		limits.acc_lim_y,
		limits.acc_lim_theta,
		limits.min_vel_x,
		limits.min_vel_y,
		-limits.max_vel_theta,
		limits.max_vel_x,
		limits.max_vel_y,
		limits.max_vel_theta,
		cfg_->getGeneral()->sim_period,
		cmd_vel,
		false // hard-coded maintain_vel_components_rate
	);

	// check the trajectory obstacle-wise
	Eigen::Vector3f vel_sample;
	vel_sample[0] = cmd_vel.getX();
	vel_sample[1] = cmd_vel.getY();
	vel_sample[2] = cmd_vel.getZ();
	bool traj_valid = checkTrajectory(
		pose_.getAsEigen2D(),
		vel_.getAsEigen<Eigen::Vector3f>(),
		vel_sample
	);

	// terminate behaviour execution if trajectories are not collision-free
	if (!traj_valid || !rotate_valid) {
		yield_way_crossing_.finish();
	}

	result_traj_.resetPoints();
	result_traj_.time_delta_ = cfg_->getGeneral()->sim_period;
	result_traj_.xv_ = cmd_vel.getX();
	result_traj_.yv_ = cmd_vel.getY();
	result_traj_.thetav_ = cmd_vel.getZ();
	// negative cost when trajectory is invalid
	result_traj_.cost_ = traj_valid ? 0.0 : -1.0;
	// add at least the current pose so the trajectory can be somehow scored
	result_traj_.addPoint(pose_.getX(), pose_.getY(), pose_.getYaw());
	// update explored trajectories (1 valid trajectory will be added to the list)
	traj_explored_.clear();
	traj_explored_.push_back(result_traj_);
	return traj_valid && rotate_valid;
}

bool HumapPlanner::planRecoveryRotateAndRecede() {
	if (!recovery_.planRecoveryRotateAndRecede(pose_.getX(), pose_.getY(), pose_.getYaw(), obstacle_costs_)) {
		result_traj_.resetPoints();
		result_traj_.time_delta_ = cfg_->getGeneral()->sim_period;
		result_traj_.xv_ = 0.0;
		result_traj_.yv_ = 0.0;
		result_traj_.thetav_ = 0.0;
		result_traj_.cost_ = 0.0;
		// add at least the current pose so the trajectory can be somehow scored
		result_traj_.addPoint(pose_.getX(), pose_.getY(), pose_.getYaw());
		// update explored trajectories (1 valid trajectory will be added to the list)
		traj_explored_.clear();
		ROS_ERROR_THROTTLE_NAMED(
			5.0,
			"HumapPlanner",
			"Cannot find a feasible pose for the recovery routine. "
			"Waiting for the navigation stack's recovery behaviour."
		);
		return false;
	}

	// get a recovery goal pose from the recovery pose search
	geometry::Pose goal_recovery = recovery_.getRotateAndRecedeRecoveryGoal();

	// let's make the goal tolerances and acceleration limits pretty strict here
	base_local_planner::LocalPlannerLimits limits = *cfg_->getLimits();
	limits.xy_goal_tolerance /= 10.0;
	limits.yaw_goal_tolerance /= 10.0;
	limits.acc_lim_x /= 5.0;
	limits.acc_lim_y /= 5.0;
	limits.acc_lim_theta /= 5.0;
	// allow very slow rotations to avoid overshooting the goal orientation
	limits.min_vel_theta /= 5.0;

	// whether robot already rotated towards the recovery goal (expect that it should be done )
	bool oriented_towards_goal = std::abs(pose_.getYaw() - goal_recovery.getYaw()) <= limits.yaw_goal_tolerance;

	// planned velocity command - the result of recovery routine
	geometry_msgs::Twist cmd_vel;
	bool traj_valid = false;

	// 2 stage procedure: first rotate in place towards the goal pose, then move straight towards that pose
	if (!oriented_towards_goal) {
		// rotate implementing strategy similar to the one in @ref planOrientationAdjustment but ignore strict cost
		// functions that will probably not allow, e.g., rotating very close to obstacles (particularly when
		// the separation distance is non-zero)
		auto checkTrajectory = std::function<bool(Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f)>(
			[](
				const Eigen::Vector3f pos,
				const Eigen::Vector3f vel,
				const Eigen::Vector3f vel_samples
			) -> bool {
				// no constraints, all trajectories is treated as valid
				return true;
			}
		);

		// NOTE: LatchedStopRotateController assumes that all received poses are expressed in the same frame,
		// headers of stamped types are not required then
		geometry_msgs::PoseStamped velocity;
		velocity.pose = Pose(vel_.getX(), vel_.getY(), vel_.getZ()).getAsMsgPose();

		geometry_msgs::PoseStamped pose;
		pose.pose = pose_.getAsMsgPose();

		geometry_msgs::PoseStamped goal_pose;
		goal_pose.pose = goal_recovery.getAsMsgPose();

		traj_valid = stop_rotate_controller_.computeVelocityCommandsStopRotate(
			cmd_vel,
			limits.getAccLimits(),
			cfg_->getGeneral()->sim_period,
			goal_pose,
			velocity,
			pose,
			limits,
			std::bind(
				checkTrajectory,
				std::placeholders::_1,
				std::placeholders::_2,
				std::placeholders::_3
			)
		);
	} else {
		// Seems that the robot reached the goal orientation but it is not guaranteed that its angular velocity is 0,
		// once getting there. Therefore, check if angular velocity from the rotation step should be shut off first.
		bool angular_vel_nonzero = std::abs(vel_.getZ()) >= 1e-02;

		// start moving with the minimum velocity regarding the acceleration limits;
		// in case `min_vel_x` is set to a negative value, provide that we'll be going forward
		double vel_x_forward = std::max(limits.min_vel_x, 0.1);
		geometry::Vector cmd_vel_forward(vel_x_forward, 0.0, 0.0);

		adjustTwistWithAccLimits(
			vel_,
			limits.acc_lim_x,
			limits.acc_lim_y,
			limits.acc_lim_theta,
			limits.min_vel_x,
			limits.min_vel_y,
			-limits.max_vel_theta,
			limits.max_vel_x,
			limits.max_vel_y,
			limits.max_vel_theta,
			cfg_->getGeneral()->sim_period,
			cmd_vel_forward,
			false // hard-coded maintain_vel_components_rate
		);

		// all velocity components zeroed at first
		cmd_vel.linear.x = 0.0;
		cmd_vel.linear.y = 0.0;
		cmd_vel.linear.z = 0.0;
		cmd_vel.angular.x = 0.0;
		cmd_vel.angular.y = 0.0;
		cmd_vel.angular.z = 0.0;

		if (angular_vel_nonzero) {
			// see note at the top of the branch;
			// if we overshoot, we'll get back to the rotation-in-place stage
			cmd_vel.angular.z = cmd_vel_forward.getZ();
		} else {
			// safe to move with a linear motion only
			cmd_vel.linear.x = cmd_vel_forward.getX();
			cmd_vel.linear.y = cmd_vel_forward.getY();
		}
		traj_valid = true;
	}

	result_traj_.resetPoints();
	result_traj_.time_delta_ = cfg_->getGeneral()->sim_period;
	result_traj_.xv_ = cmd_vel.linear.x;
	result_traj_.yv_ = cmd_vel.linear.y;
	result_traj_.thetav_ = cmd_vel.angular.z;
	// negative cost when trajectory is invalid
	result_traj_.cost_ = traj_valid ? 0.0 : -1.0;
	// add at least the current pose so the trajectory can be somehow scored
	result_traj_.addPoint(pose_.getX(), pose_.getY(), pose_.getYaw());

	// update explored trajectories (1 valid trajectory will be added to the list)
	traj_explored_.clear();
	traj_explored_.push_back(result_traj_);
	return traj_valid;
}

bool HumapPlanner::planRecoveryLookAround() {
	// wrapped in a lambda as this sequence is used multiple times in this function
	auto setStopCommandAsResult = [this]() {
		result_traj_.resetPoints();
		result_traj_.time_delta_ = cfg_->getGeneral()->sim_period;
		result_traj_.xv_ = 0.0;
		result_traj_.yv_ = 0.0;
		result_traj_.thetav_ = 0.0;
		result_traj_.cost_ = 0.0;
		// add at least the current pose so the trajectory can be somehow scored
		result_traj_.addPoint(pose_.getX(), pose_.getY(), pose_.getYaw());
		// update explored trajectories (1 valid trajectory will be added to the list)
		traj_explored_.clear();
	};

	if (!recovery_.planRecoveryLookAround(pose_.getX(), pose_.getY(), pose_.getYaw(), obstacle_costs_)) {
		setStopCommandAsResult();
		ROS_ERROR_THROTTLE_NAMED(
			5.0,
			"HumapPlanner",
			"Cannot find a feasible pose for the look around recovery routine. "
			"Waiting for the navigation stack's recovery behaviour."
		);
		return false;
	}

	// let's make the goal tolerances and acceleration limits strict here
	base_local_planner::LocalPlannerLimits limits = *cfg_->getLimits();
	limits.xy_goal_tolerance /= 5.0;
	limits.acc_lim_x /= 5.0;
	limits.acc_lim_y /= 5.0;
	limits.acc_lim_theta /= 2.0;
	limits.min_vel_theta /= 2.0;

	// get a recovery goal pose from the recovery pose search
	geometry::Pose goal_recovery = recovery_.getLookAroundRecoveryGoal();

	// whether robot already rotated towards the recovery goal (expect that it should be done)
	auto calculateAngularDiffToGoal = [this, goal_recovery]() -> double {
		return std::abs(pose_.getYaw() - goal_recovery.getYaw());
	};
	double dist_to_goal_orient = calculateAngularDiffToGoal();
	bool goal_orientation_reached = dist_to_goal_orient <= limits.yaw_goal_tolerance;

	if (goal_orientation_reached) {
		// select next goal as the valid one
		recovery_.reachedGoalRecoveryLookAround();
		goal_recovery = recovery_.getLookAroundRecoveryGoal();
		// recalculate angular diff
		dist_to_goal_orient = calculateAngularDiffToGoal();
	}

	// goal has been updated above, possibly we're out of other goals and should finish here
	if (!recovery_.isLookAroundRecoveryActive()) {
		setStopCommandAsResult();
		ROS_INFO_NAMED(
			"HumapPlanner",
			"Look around recovery routine has finished operation - returning a stop command."
		);
		return true;
	}

	double distance_to_goal_pos = (pose_.getPosition() - goal_recovery.getPosition()).calculateLength();
	bool goal_position_reached = distance_to_goal_pos <= limits.xy_goal_tolerance;
	auto bup_dist = (pose_.getPosition() - recovery_.getLookAroundRecoveryStartPose().getPosition()).calculateLength();
	bool goal_position_overshoot = bup_dist >= RecoveryManager::BACKING_UP_DISTANCE;
	// reached or slightly overshoot - does not matter here, in fact
	goal_position_reached |= goal_position_overshoot;

	// planned velocity command - the result of recovery routine
	geometry_msgs::Twist cmd_vel;
	bool traj_valid = false;

	// 2 stage procedure: first slowly moving backwards to the goal position, then rotate to the goal orientation
	if (!goal_position_reached) {
		// It is not guaranteed that its angular velocity is 0, once getting there. Therefore, check if angular
		// velocity from the rotation step should be shut off first.
		bool angular_vel_nonzero = std::abs(vel_.getZ()) >= 1e-02;

		// move with the minimum velocity regarding the acceleration limits
		double vel_x_backward = limits.min_vel_x < 0.0 ? std::max(limits.min_vel_x, -0.1) : -0.1;
		geometry::Vector cmd_vel_bwd(vel_x_backward, 0.0, 0.0);

		adjustTwistWithAccLimits(
			vel_,
			limits.acc_lim_x,
			limits.acc_lim_y,
			limits.acc_lim_theta,
			limits.min_vel_x,
			limits.min_vel_y,
			-limits.max_vel_theta,
			limits.max_vel_x,
			limits.max_vel_y,
			limits.max_vel_theta,
			cfg_->getGeneral()->sim_period,
			cmd_vel_bwd,
			false // hard-coded maintain_vel_components_rate
		);

		// all velocity components zeroed at first
		cmd_vel.linear.x = 0.0;
		cmd_vel.linear.y = 0.0;
		cmd_vel.linear.z = 0.0;
		cmd_vel.angular.x = 0.0;
		cmd_vel.angular.y = 0.0;
		cmd_vel.angular.z = 0.0;

		if (angular_vel_nonzero) {
			// see note at the top of the branch; wait until angular velocity is zeroed
			cmd_vel.angular.z = cmd_vel_bwd.getZ();
		} else {
			// safe to move with a linear motion only
			cmd_vel.linear.x = cmd_vel_bwd.getX();
			cmd_vel.linear.y = cmd_vel_bwd.getY();
		}
		traj_valid = true;

	} else {
		// NOTE: LatchedStopRotateController assumes that all received poses are expressed in the same frame,
		// headers of stamped types are not required then
		geometry_msgs::PoseStamped velocity;
		velocity.pose = Pose(vel_.getX(), vel_.getY(), vel_.getZ()).getAsMsgPose();

		geometry_msgs::PoseStamped pose;
		pose.pose = pose_.getAsMsgPose();

		geometry_msgs::PoseStamped goal_pose;
		goal_pose.pose = goal_recovery.getAsMsgPose();

		traj_valid = stop_rotate_controller_.computeVelocityCommandsStopRotate(
			cmd_vel,
			limits.getAccLimits(),
			cfg_->getGeneral()->sim_period,
			goal_pose,
			velocity,
			pose,
			limits,
			std::bind(
				&HumapPlanner::checkTrajectory,
				this,
				std::placeholders::_1,
				std::placeholders::_2,
				std::placeholders::_3
			)
		);
	}

	result_traj_.resetPoints();
	result_traj_.time_delta_ = cfg_->getGeneral()->sim_period;
	result_traj_.xv_ = cmd_vel.linear.x;
	result_traj_.yv_ = cmd_vel.linear.y;
	result_traj_.thetav_ = cmd_vel.angular.z;
	// negative cost when trajectory is invalid
	result_traj_.cost_ = traj_valid ? 0.0 : -1.0;
	// add at least the current pose so the trajectory can be somehow scored
	result_traj_.addPoint(pose_.getX(), pose_.getY(), pose_.getYaw());

	// update explored trajectories (1 valid trajectory will be added to the list)
	traj_explored_.clear();
	traj_explored_.push_back(result_traj_);

	return traj_valid;
}

void HumapPlanner::collectTrajectoryMotionData() {
	if (generator_social_.getGeneratedTrajectories().empty()) {
		motion_data_.behaviour_active_ = "unknown";
		motion_data_.force_internal_ = geometry::Vector();
		motion_data_.force_interaction_ = geometry::Vector();
		motion_data_.force_social_ = geometry::Vector();
		motion_data_.force_combined_ = geometry::Vector();
		motion_data_.closest_points_static_.clear();
		motion_data_.closest_points_dynamic_.clear();
	}

	// this version should be used when only 1 trajectory is available (no-planning approach)
	const auto traj_data = generator_social_.getGeneratedTrajectories().at(0);
	motion_data_.behaviour_active_ = traj_data.behaviour_active;
	motion_data_.force_internal_ = traj_data.force_internal;
	motion_data_.force_interaction_ = traj_data.force_interaction_static + traj_data.force_interaction_dynamic;
	motion_data_.force_social_ = traj_data.force_social;
	motion_data_.force_combined_ = traj_data.force_total;
	motion_data_.closest_points_static_ = traj_data.closest_points_static;
	motion_data_.closest_points_dynamic_ = traj_data.closest_points_dynamic;
}

bool HumapPlanner::collectTrajectoryMotionData(const base_local_planner::Trajectory& best_traj) {
	motion_data_.behaviour_active_ = "unknown";
	motion_data_.force_internal_ = geometry::Vector();
	motion_data_.force_interaction_ = geometry::Vector();
	motion_data_.force_social_ = geometry::Vector();
	motion_data_.force_combined_ = geometry::Vector();
	motion_data_.closest_points_static_.clear();
	motion_data_.closest_points_dynamic_.clear();

	// we can't tell directly whether the best_traj was generated by the @ref generator_social_
	for (const auto& traj_data: generator_social_.getGeneratedTrajectories()) {
		// closest points can be saved even if the best traj will not be matched
		if (
			motion_data_.closest_points_static_.empty()
			&& !traj_data.closest_points_static.empty()
		) {
			motion_data_.closest_points_static_ = traj_data.closest_points_static;
			motion_data_.closest_points_dynamic_ = traj_data.closest_points_dynamic;
		}

		// find the best_traj among generated ones
		if (
			traj_data.traj.xv_ != best_traj.xv_
			|| traj_data.traj.thetav_ != best_traj.thetav_
			|| traj_data.traj.yv_ != best_traj.yv_
		) {
			continue;
		}

		// check equality of traj's dt [this may be off even when trajectories are equal, thus commented out]
		// if (traj_data.traj.time_delta_ != best_traj.time_delta_) {
		// 	continue;
		// }

		// check points size
		if (traj_data.traj.getPointsSize() != best_traj.getPointsSize()) {
			continue;
		}

		// seed velocities match, check points
		bool points_mismatch = false;
		for (int i = 0; i < traj_data.traj.getPointsSize(); i++) {
			double px = 0.0;
			double py = 0.0;
			double pth = 0.0;
			double best_px = 0.0;
			double best_py = 0.0;
			double best_pth = 0.0;

			traj_data.traj.getPoint(i, px, py, pth);
			best_traj.getPoint(i, best_px, best_py, best_pth);
			if (
				best_px != px
				|| best_py != py
				|| best_pth != pth
			) {
				points_mismatch = true;
				break;
			}
		}
		if (points_mismatch) {
			continue;
		}

		// points match too, this seems to be our best trajectory
		motion_data_.behaviour_active_ = traj_data.behaviour_active;
		motion_data_.force_internal_ = traj_data.force_internal;
		motion_data_.force_interaction_ = traj_data.force_interaction_static + traj_data.force_interaction_dynamic;
		motion_data_.force_social_ = traj_data.force_social;
		motion_data_.force_combined_ = traj_data.force_total;
		// actually, no need to overwrite `closest_points_...`
		return true;
	}
	return false;
}

void HumapPlanner::logTrajectoriesDetails() {
	bool invalid_traj = false;

	if (result_traj_.cost_ < 0) {
		ROS_ERROR_COND_NAMED(
			cfg_->getDiagnostics()->log_explored_trajectories,
			"HumapPlanner",
			"Explored %lu trajectories, but couldn't find a valid one (cost %2.2f)",
			traj_explored_.size(),
			result_traj_.cost_
		);
		// set the flag to log the cause of the invalidity (cost details)
		invalid_traj = true;
	} else {
		ROS_INFO_COND_NAMED(
			cfg_->getDiagnostics()->log_explored_trajectories,
			"HumapPlanner",
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

	// those are conditions of ROS_INFO_COND below - checked to possibly return earlier
	if (
		!cfg_->getDiagnostics()->log_explored_trajectories
		&& !(invalid_traj || cfg_->getDiagnostics()->log_trajectory_cost_details)
		&& !cfg_->getDiagnostics()->log_pts_of_explored_trajectories
	) {
		return;
	}

	// investigate costs and velocities of all explored trajectories
	int traj_num = 0;
	for (const auto& traj: traj_explored_) {
		// increment at the beginning is more intuitive when comparing to the full size
		++traj_num;

		double traj_x, traj_y, traj_th = 0.0;
		traj.getEndpoint(traj_x, traj_y, traj_th);

		// print basic trajectory info
		ROS_INFO_COND_NAMED(
			cfg_->getDiagnostics()->log_explored_trajectories,
			"HumapPlanner",
			"%sExplored trajectory %3d / %3lu: cost %2.5f, vel {x %2.3f, y %2.3f, th %2.3f}, "
			"points %u, end {x %2.2f, y %2.2f, th %2.2f}%s",
			result_traj_.cost_ == traj.cost_ ? "\033[32m" : "", // mark the best trajectory green
			traj_num,
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
			invalid_traj || cfg_->getDiagnostics()->log_trajectory_cost_details,
			"HumapPlanner",
			"%sExplored trajectory %3d / %3lu cost details: "
			"obstacle %5.2f, oscillation %5.2f, path %8.2f, goal %8.2f, goal_front %8.2f, alignment %8.2f, utrans. %8.2f, "
			"backward %5.2f, TTC %5.2f, HSM %5.2f, vel_smoothness %5.2f, "
			"head_dir %5.2f, PSI %5.2f, FSI %5.2f, PSPD %5.2f%s",
			// mark the best trajectory green and all non-feasible red
			(result_traj_.cost_ == traj.cost_) ? "\033[32m" : (traj.cost_ < 0.0) ? "\x1B[31m" : "",
			traj_num,
			traj_explored_.size(),
			obstacle_costs_.getScale() * obstacle_costs_.scoreTrajectory(traj_copy),
			oscillation_costs_.getScale() * oscillation_costs_.scoreTrajectory(traj_copy),
			path_costs_.getScale() * path_costs_.scoreTrajectory(traj_copy),
			goal_costs_.getScale() * goal_costs_.scoreTrajectory(traj_copy),
			goal_front_costs_.getScale() * goal_front_costs_.scoreTrajectory(traj_copy),
			alignment_costs_.getScale() * alignment_costs_.scoreTrajectory(traj_copy),
			unsaturated_trans_costs_.getScale() * unsaturated_trans_costs_.scoreTrajectory(traj_copy),
			backward_costs_.getScale() * backward_costs_.scoreTrajectory(traj_copy),
			ttc_costs_.getScale() * ttc_costs_.scoreTrajectory(traj_copy),
			heading_change_smoothness_costs_.getScale() * heading_change_smoothness_costs_.scoreTrajectory(traj_copy),
			velocity_smoothness_costs_.getScale() * velocity_smoothness_costs_.scoreTrajectory(traj_copy),
			heading_disturbance_costs_.getScale() * heading_disturbance_costs_.scoreTrajectory(traj_copy),
			personal_space_costs_.getScale() * personal_space_costs_.scoreTrajectory(traj_copy),
			fformation_space_costs_.getScale() * fformation_space_costs_.scoreTrajectory(traj_copy),
			passing_speed_costs_.getScale() * passing_speed_costs_.scoreTrajectory(traj_copy),
			"\033[0m" // reset the (possibly) colorized output
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
				"HumapPlanner",
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

}; // namespace humap_local_planner
