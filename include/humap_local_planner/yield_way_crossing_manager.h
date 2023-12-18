#pragma once

#include <humap_local_planner/geometry/angle.h>
#include <humap_local_planner/geometry/pose.h>
#include <humap_local_planner/geometry/vector.h>

#include <utility>

namespace humap_local_planner {

/**
 * Manages the execution of yielding way to a human that crosses the robot's path
 */
class YieldWayCrossingManager {
public:
	YieldWayCrossingManager();

	/**
	 * Updates members of the class that are used to perform the yielding routine
	 *
	 * @param robot_pose current pose of the robot
	 * @param safe_direction information about the intermediate goal position
	 * @return true If routine is active, false otherwise
	 */
	bool update(
		const geometry::Pose& robot_pose,
		const std::pair<geometry::Vector, double>& safe_direction
	);

	/// Call to this indicates that the routine will be marked as inactive
	void finish();

	bool isActive() const {
		return routine_active_;
	}

	/// Returns the intermediate goal position that should be reached to effectively yield way
	geometry::Vector getGoalPosition() const {
		return goal_pos_;
	}

	/// Returns the direction from the current pose to the intermediate goal position
	double getGoalDirection() const {
		return goal_direction_.getRadian();
	}

	/// Returns Euclidean distance to the intermediate goal position
	double getGoalDistanceXy() const {
		return distance_to_safe_linear_;
	}

	/// Returns angular distance to align the robot's nose with the intermediate goal position
	double getGoalDistanceYaw() const {
		return distance_to_safe_angular_;
	}

	/// Distance travelled since the activation (by a call to a @ref update with a valid arguments)
	double getTraveledDistance() const {
		return distance_traveled_;
	}

protected:
	bool routine_active_;
	geometry::Vector goal_pos_;
	geometry::Angle goal_direction_;

	double distance_to_safe_linear_;
	double distance_to_safe_angular_;

	geometry::Vector robot_pos_prev_;
	double distance_traveled_;
};

} // namespace humap_local_planner
