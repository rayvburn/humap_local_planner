#include <hubero_local_planner/yield_way_crossing_manager.h>

#include <angles/angles.h>

namespace hubero_local_planner {

YieldWayCrossingManager::YieldWayCrossingManager():
	routine_active_(false),
	goal_pos_(NAN, NAN, NAN),
	goal_direction_(NAN),
	distance_to_safe_linear_(NAN),
	distance_to_safe_angular_(NAN),
	robot_pos_prev_(NAN, NAN, NAN),
	distance_traveled_(NAN)
{}

bool YieldWayCrossingManager::update(
	const geometry::Pose& robot_pose,
	const std::pair<geometry::Vector, double>& safe_direction
) {
	// 1) update only if valid
	// 2) check if the old "pair" is valid
	// 3) perform necessary calculations to use the "old" pair
	if (
		!std::isnan(safe_direction.first.getX())
		&& !std::isnan(safe_direction.first.getY())
		&& !std::isnan(safe_direction.second)
	) {
		goal_pos_ = safe_direction.first;
		goal_direction_ = geometry::Angle(safe_direction.second);
	} else if (
		std::isnan(goal_pos_.getX())
		|| std::isnan(goal_pos_.getY())
		|| std::isnan(goal_direction_.getRadian())
	) {
		routine_active_ = false;
		return false;
	} else {
		// goal direction may need to be updated if an old "pair" is used
		goal_direction_ = (goal_pos_ - robot_pose.getPosition()).calculateDirection();
	}

	distance_to_safe_linear_ = (robot_pose.getPosition() - goal_pos_).calculateLength();
	distance_to_safe_angular_ = angles::shortest_angular_distance(
		robot_pose.getYaw(),
		goal_direction_.getRadian()
	);

	if (!std::isnan(robot_pos_prev_.getX()) && !std::isnan(robot_pos_prev_.getX())) {
		// previous position properly defined
		double displacement = std::hypot(
			robot_pose.getX() - robot_pos_prev_.getX(),
			robot_pose.getY() - robot_pos_prev_.getY()
		);
		distance_traveled_ += displacement;
	} else {
		// the stored previous position is invalid
		distance_traveled_ = 0.0;
	}

	robot_pos_prev_ = robot_pose.getPosition();
	routine_active_ = true;
	return true;
}

void YieldWayCrossingManager::finish() {
	routine_active_ = false;
	goal_pos_ = geometry::Vector(NAN, NAN, NAN);
	goal_direction_ = geometry::Angle(NAN);
	distance_to_safe_linear_ = NAN;
	distance_to_safe_angular_ = NAN;
	robot_pos_prev_ = geometry::Vector(NAN, NAN, NAN);
	distance_traveled_ = NAN;
}

} // namespace hubero_local_planner
