#include <humap_local_planner/recovery_manager.h>

#include <costmap_2d/cost_values.h>

namespace humap_local_planner {

RecoveryManager::RecoveryManager():
	buffer_(10),
	oscillating_(false),
	stuck_(false),
	deviating_from_global_plan_(false),
	near_collision_(false),
	prev_near_collision_(near_collision_),
	can_recover_from_collision_(true),
	rr_recovery_goal_(geometry::Pose(NAN, NAN, NAN, NAN, NAN, NAN, NAN)),
	prev_cmd_vx(0.0),
	prev_cmd_omega(0.0),
	global_path_plan_outdated_(false),
	global_path_plan_max_age_(3.0),
	global_path_plan_update_timestamp_(0.0)
{}

void RecoveryManager::setOscillationBufferLength(int length) {
	buffer_.set_capacity(length);
}

void RecoveryManager::setGlobalPathPlanMaxAge(double global_path_plan_max_age) {
	global_path_plan_max_age_ = global_path_plan_max_age;
}

void RecoveryManager::checkOscillation(
	double v_x,
	double v_th,
	double v_max,
	double v_backwards_max,
	double omega_max,
	double v_eps,
	double omega_eps
) {
	if (buffer_.capacity() == 0) {
		return;
	}

	RecoveryManager::VelMeasurement measurement;
	// just consider linear velocity in x-direction in the robot frame for now
	measurement.v = prev_cmd_vx;
	measurement.omega = prev_cmd_omega;

	// buffer stores normalized velocities
	if (measurement.v > 0 && v_max > 0) {
		measurement.v /= v_max;
	} else if (measurement.v < 0 && v_backwards_max > 0) {
		measurement.v /= v_backwards_max;
	}

	if (omega_max > 0) {
		measurement.omega /= omega_max;
	}
	buffer_.push_back(measurement);

	// immediately compute new state
	detectOscillation(v_eps, omega_eps);

	prev_cmd_vx = v_x;
	prev_cmd_omega = v_th;
}

void RecoveryManager::checkDeviation(
	base_local_planner::Trajectory traj,
	MapGridCostFunction& goal_front_costfun,
	MapGridCostFunction& alignment_costfun
) {
	double cost_alignment = alignment_costfun.scoreTrajectory(traj);
	double goal_front_cost = goal_front_costfun.scoreTrajectory(traj);

	if (
		cost_alignment == alignment_costfun.obstacleCosts()
		|| cost_alignment == alignment_costfun.unreachableCellCosts()
		|| goal_front_cost == goal_front_costfun.obstacleCosts()
		|| goal_front_cost == goal_front_costfun.unreachableCellCosts()
	) {
		deviating_from_global_plan_ = true;
		return;
	}
	deviating_from_global_plan_ = false;
}

void RecoveryManager::checkCollision(
	base_local_planner::Trajectory traj,
	ObstacleSeparationCostFunction& obstacle_costfun
) {
	double occdist_cost = obstacle_costfun.scoreTrajectory(traj);
	if (RecoveryManager::isPoseCollisionFree(occdist_cost)) {
		// we're safe
		prev_near_collision_ = near_collision_;
		near_collision_ = false;
		can_recover_from_collision_ = true;
		resetRotateAndRecedeRecoveryGoal();
		return;
	}
	if (occdist_cost >= costmap_2d::LETHAL_OBSTACLE) {
		// something went wrong
		prev_near_collision_ = near_collision_;
		near_collision_ = true;
		can_recover_from_collision_ = false;
		return;
	}
	if (occdist_cost < 0.0) {
		prev_near_collision_ = near_collision_;
		// footprint seems to be in collision with some object marked on the costmap (may be outdated)
		near_collision_ = true;
		// we can attempt to recover from a collision once the separation distance is positive (there should still
		// be some gap between the obstacle and the robot)
		double obstacle_keepout_distance = obstacle_costfun.getSeparationDistance();
		can_recover_from_collision_ = obstacle_keepout_distance > 0.0;
	}
}

double RecoveryManager::computeGlobalPlanAge() {
	double current_time = ros::Time::now().toSec();
	double path_age = current_time - global_path_plan_update_timestamp_.load();
	global_path_plan_outdated_ = path_age > global_path_plan_max_age_;
	return path_age;
}

bool RecoveryManager::planRecoveryRotateAndRecede(
	double x,
	double y,
	double th,
	const ObstacleSeparationCostFunction& obstacle_costfun
) {
	// first, check whether it is possible to stick to the previous recovery pose
	if (RecoveryManager::isPoseValid(rr_recovery_goal_)) {
		auto rgoal_cost = obstacle_costfun.getFootprintCost(
			rr_recovery_goal_.getX(),
			rr_recovery_goal_.getY(),
			rr_recovery_goal_.getYaw()
		);
		if (RecoveryManager::isPoseCollisionFree(rgoal_cost)) {
			// we'll use the previously found goal pose for the recovery routine
			return true;
		}
	}

	// check the cost at the current position but without separation distance
	double pos_cost_wo_separation = obstacle_costfun.getFootprintCost(x, y, th, 0.0);
	if (pos_cost_wo_separation == costmap_2d::LETHAL_OBSTACLE || pos_cost_wo_separation < 0.0) {
		resetRotateAndRecedeRecoveryGoal();
		return false;
	}

	// Given the current pose, let's try to find a point somewhere around the mobile base, so the robot could align
	// towards it (adjust orientation in place) and slowly escape from a stuck pose.
	// Prepare a vector of recovery directions - aim is to check the "front ones" first
	auto generateMultipliersSequence = [](size_t length) -> std::vector<int> {
		int start = 0;
		int end = length - 1;
		std::vector<int> container;
		for (int i = 0; i <= (length / 2); i++) {
			if (start <= end) {
				container.push_back(start++);
			}
			if (start <= end) {
				container.push_back(end--);
			}
		}
		return container;
	};
	// stores a sequence of multipliers in a following order (for 8 pts), e.g., 0,1,8,2,7,3,6,4,5
	auto angle_multipliers = generateMultipliersSequence(RECOVERY_ROTATE_AND_RECEDE_SURROUNDING_PTS);

	// vector of directions ordered according to the sequence above - the "front ones" first;
	// "front ones" are expressed according to the robot's local coordinate system
	std::vector<double> checked_directions;
	double th_delta = (M_PI + M_PI) / static_cast<double>(RECOVERY_ROTATE_AND_RECEDE_SURROUNDING_PTS);
	for (const auto& mult: angle_multipliers) {
		geometry::Angle angle(th + static_cast<double>(mult) * th_delta);
		checked_directions.push_back(angle.getRadian());
	}

	// let's assume that we'll try to travel N cm plus the separation distance
	double dist = RECOVERY_ROTATE_AND_RECEDE_DISTANCE + obstacle_costfun.getSeparationDistance();

	// iterate over the investigated directions - choose the first one (the most upfront)
	for (int i = 0; i < RECOVERY_ROTATE_AND_RECEDE_SURROUNDING_PTS; i++) {
		// find a pose to evaluate against the cost function - search among the straight paths
		auto poses_eval = RecoveryManager::computeCoordsAtDistanceAndDirection(
			x,
			y,
			th,
			checked_directions.at(i),
			dist
		);
		// flag indicating that any intermediate pose is in collision
		bool in_collision = false;
		// iterate over the poses of the straight path
		for (const auto& pose: poses_eval) {
			auto cost = obstacle_costfun.getFootprintCost(pose.getX(), pose.getY(), pose.getYaw(), 0.0);
			if (cost < 0.0 || cost >= costmap_2d::LETHAL_OBSTACLE) {
				in_collision = true;
				break;
			}
		}
		if (in_collision) {
			continue;;
		}
		// found an obstacle-free pose lying nearby along the straight path
		rr_recovery_goal_ = geometry::Pose(
			poses_eval.back().getX(),
			poses_eval.back().getY(),
			poses_eval.back().getYaw()
		);
		return true;
	}

	// not found a valid recovery pose
	resetRotateAndRecedeRecoveryGoal();
	return false;
}

bool RecoveryManager::planRecoveryLookAround(
	double x,
	double y,
	double th,
	const ObstacleSeparationCostFunction& obstacle_costfun
) {
	double back_up_distance = BACKING_UP_DISTANCE;

	// This procedure is only planned once - when a valid set of goals is determined, it is only checked whether
	// the initially computed poses are still valid
	if (!la_recovery_goals_.empty()) {
		bool goals_still_valid = true;
		for (const auto& pose: la_recovery_goals_) {
			auto cost = obstacle_costfun.getFootprintCost(pose.getX(), pose.getY(), pose.getYaw());
			if (!RecoveryManager::isPoseCollisionFree(cost)) {
				goals_still_valid = false;
				break;
			}
		}
		if (goals_still_valid) {
			return true;
		}
		// let's skip back-up stage that we'd do at the start (non-negative back_up_distance might lead to a receding
		// horizon problem here)
		back_up_distance = 0.0;
	}

	resetLookAroundRecoveryGoals();

	/*
	 * Stage 1: Back up a bit, if possible
	 */
	auto back_up_poses = RecoveryManager::computeCoordsAtDistanceAndDirection(
		x,
		y,
		th,
		geometry::Angle(th + M_PI).getRadian(), // sample backwards
		back_up_distance,
		obstacle_costfun.getSeparationDistance()
	);

	// stores the furthest collision-free position for backing up
	geometry::Vector recovery_pos(NAN, NAN, NAN);
	bool back_up_possible = false;
	// iterate over the poses of the straight path to check if any intermediate pose is in collision
	for (const auto& pose: back_up_poses) {
		auto cost = obstacle_costfun.getFootprintCost(pose.getX(), pose.getY(), pose.getYaw());
		if (!RecoveryManager::isPoseCollisionFree(cost)) {
			break;
		}
		recovery_pos = geometry::Vector(pose.getX(), pose.getY(), pose.getZ());
		back_up_possible = true;
	}
	if (!back_up_possible) {
		// make sure that the current pose is collision-free
		auto cost = obstacle_costfun.getFootprintCost(x, y, th);
		if (!RecoveryManager::isPoseCollisionFree(cost)) {
			resetLookAroundRecoveryGoals();
			return false;
		}
		recovery_pos = geometry::Vector(x, y, th);
	}

	/*
	 * Stage 2: Find a direction opposite compared to the angular velocities stored in the velocity buffer
	 * If can't tell, choose left due to the right-hand side
	 */
	bool start_with_left_rotation = true;
	if (!buffer_.empty()) {
		auto newest_vel_measurement = buffer_.at(buffer_.size() - 1);
		bool last_rotation_left = newest_vel_measurement.omega >= 0.0;
		start_with_left_rotation = !last_rotation_left;
	}

	/*
	 * Stage 3: Collect subsequent goal poses that should be reached during the recovery procedure:
	 * - orientate with 60 degrees offset (from the current angle) in the first direction
	 * - orientate with 120 degrees offset (from the current angle) in the second direction
	 * - orientate with 120 degrees offset (from the current angle) in the first direction
	 */
	static const double ANGULAR_DIST_BASE = IGN_DTOR(60);
	la_recovery_goals_.emplace_back(
		recovery_pos.getX(),
		recovery_pos.getY(),
		start_with_left_rotation
			? geometry::Angle(th + ANGULAR_DIST_BASE).getRadian()
			: geometry::Angle(th - ANGULAR_DIST_BASE).getRadian()
	);
	la_recovery_goals_.emplace_back(
		recovery_pos.getX(),
		recovery_pos.getY(),
		start_with_left_rotation
			? geometry::Angle(th - 2.0 * ANGULAR_DIST_BASE).getRadian()
			: geometry::Angle(th + 2.0 * ANGULAR_DIST_BASE).getRadian()
	);
	la_recovery_goals_.emplace_back(
		recovery_pos.getX(),
		recovery_pos.getY(),
		start_with_left_rotation
			? geometry::Angle(th + 2.0 * ANGULAR_DIST_BASE).getRadian()
			: geometry::Angle(th - 2.0 * ANGULAR_DIST_BASE).getRadian()
	);

	bool poses_valid = std::all_of(
		la_recovery_goals_.cbegin(),
		la_recovery_goals_.cend(),
		[](const auto& pose) -> bool {
			return RecoveryManager::isPoseValid(pose);
		}
	);
	if (!poses_valid) {
		resetLookAroundRecoveryGoals();
		return false;
	}
	la_recovery_start_pose_ = geometry::Pose(x, y, th);
	return true;
}

bool RecoveryManager::reachedGoalRecoveryLookAround() {
	if (la_recovery_goals_.empty()) {
		resetLookAroundRecoveryGoals();
		return false;
	}
	la_recovery_goals_.pop_front();
	return true;
}

void RecoveryManager::clear() {
	buffer_.clear();
	oscillating_ = false;
}

void RecoveryManager::markGlobalPlanUpdate() {
	global_path_plan_update_timestamp_.store(ros::Time::now().toSec());
}

bool RecoveryManager::isPoseCollisionFree(double costmap_cost) {
	return costmap_cost >= 0.0 && costmap_cost < costmap_2d::LETHAL_OBSTACLE;
}

bool RecoveryManager::isPoseValid(const geometry::Pose& pose) {
	return !std::isinf(pose.getX())
		&& !std::isnan(pose.getX())
		&& !std::isinf(pose.getY())
		&& !std::isnan(pose.getY())
		&& !std::isinf(pose.getYaw())
		&& !std::isnan(pose.getYaw());
}

std::vector<geometry::Pose> RecoveryManager::computeCoordsAtDistanceAndDirection(
	double x,
	double y,
	double th,
	double direction,
	double distance,
	double waypoint_separation
) {
	geometry::Angle dir_angle(direction);
	geometry::Vector vec_towards_target(dir_angle);
	vec_towards_target *= waypoint_separation;
	size_t num_of_poses = std::ceil(distance / waypoint_separation);

	std::vector<geometry::Pose> container;
	container.emplace_back(x, y, th);
	for (size_t i = 0; i < num_of_poses; i++) {
		double x_shifted = container.back().getX() + vec_towards_target.getX();
		double y_shifted = container.back().getY() + vec_towards_target.getY();
		double th_shifted = geometry::Angle(direction).getRadian();
		container.emplace_back(x_shifted, y_shifted, th_shifted);
	}
	return container;
}

void RecoveryManager::detectOscillation(double v_eps, double omega_eps) {
	oscillating_ = false;

	// // we start detecting only as soon as we have the buffer filled at least half
	if (buffer_.size() < (buffer_.capacity() / 2)) {
		return;
	}

	double n = (double)buffer_.size();

	// helper
	auto sign = [](double x) -> int {
		return (x < 0.0) ? -1 : +1;
	};

	// compute mean for v and omega
	double v_mean = 0;
	double omega_mean = 0;
	int omega_zero_crossings = 0;
	for (int i = 0; i < n; ++i) {
		v_mean += buffer_.at(i).v;
		omega_mean += buffer_.at(i).omega;
		if (i == 0) {
			continue;
		}
		if (sign(buffer_.at(i).omega) != sign(buffer_.at(i-1).omega)) {
			++omega_zero_crossings;
		}
	}
	v_mean /= n;
	omega_mean /= n;

	// oscillated
	if (omega_zero_crossings > 1 && std::abs(v_mean) < v_eps && std::abs(omega_mean) < omega_eps) {
		oscillating_ = true;
	}
	// stuck completely
	if (omega_zero_crossings == 0 && std::abs(v_mean) < 1e-03 && std::abs(omega_mean) < 1e-03) {
		stuck_ = true;
	}
}

void RecoveryManager::resetRotateAndRecedeRecoveryGoal() {
	rr_recovery_goal_ = geometry::Pose(NAN, NAN, NAN, NAN, NAN, NAN, NAN);
}

void RecoveryManager::resetLookAroundRecoveryGoals() {
	la_recovery_goals_.clear();
	la_recovery_start_pose_ = geometry::Pose(NAN, NAN, NAN, NAN, NAN, NAN, NAN);
}

} // namespace humap_local_planner
