#pragma once

#include <humap_local_planner/geometry/pose.h>

#include <map>
#include <string>

namespace humap_local_planner {

/**
 * Implements a simple state machine for the planner that operates in different modes
 */
class PlannerState {
public:
	/// Planner state
	enum State {
		/// Mobile base rotates in place towards the local goal after reception of a new goal
		INITIATE_EXECUTION = 0,
		/// Mobile base moves in a normal way (most likely forward along the global plan) after execution initiation
		MOVE,
		/// Mobile base adjusts rotates in place to reach the goal orientation (local planner does not account for RPY)
		ADJUST_ORIENTATION,
		/// Once sideways crossing path is detected, the robot stops or adjusts its orientation to yield a way
		YIELD_WAY_CROSSING,
		/// Mobile base looks for a safe pose to reach (and moves to this pose) once got stuck
		RECOVERY_ROTATE_AND_RECEDE,
		/// Mobile base backs-up and rotates to the left and right to update the environment model
		RECOVERY_LOOK_AROUND,
		/// Noop state after started after reaching the latest goal
		STOPPED
	};

	/**
	 * @brief PlannerState
	 *
	 * Default implementations of recovery-related functions act as if the recovery is never needed
	 *
	 * @param pose reference to the current pose of robot
	 * @param goal reference to the global goal
	 * @param goal_local reference to the local goal (located approx approx 1 meter away from current pose of robot)
	 * @param goal_initiation reference to the motion initiation goal - pose close to the current one, but contains
	 * orientation that follows global path poses
	 * @param pos_reached_fun function that allows to check if goal position is reached
	 * @param goal_reached_fun function that allows to check if goal orientation is reached
	 * @param crossing_detected_fun checks if yielding way to a human crossing the robot's path is necessary
	 * @param yield_way_crossing_finished_fun checks if yielding routine has finished
	 * @param look_around_finished_fun function that checks if "look around" routine has finished
	 * @param oscillating_fun function that allows to check whether the robot is oscillating
	 * @param stuck_fun function that allows to check whether the robot is stuck
	 * @param deviating_fp_fun function that allows to check whether the robot deviates From the reference Path
	 * @param near_collision_fun function that allows to check whether the robot is close to colliding with its env.
	 * @param can_recover_fun function that allows to check whether the robot can recover from being stuck
	 * @param global_plan_outdated_fun function that allows to check whether the global path plan is outdated
	 * @param initiation_yaw_threshold
	 */
	PlannerState(
		const geometry::Pose& pose,
		const geometry::Pose& goal,
		const geometry::Pose& goal_local,
		const geometry::Pose& goal_initiation,
		std::function<bool()> pos_reached_fun,
		std::function<bool()> goal_reached_fun,
		std::function<bool()> crossing_detected_fun = std::function<bool()>([]() -> bool { return false; }),
		std::function<bool()> yield_way_crossing_finished_fun = std::function<bool()>([]() -> bool { return true; }),
		std::function<bool()> look_around_finished_fun = std::function<bool()>([]() -> bool { return true; }),
		std::function<bool()> oscillating_fun = std::function<bool()>([]() -> bool { return false; }),
		std::function<bool()> stuck_fun = std::function<bool()>([]() -> bool { return false; }),
		std::function<bool()> deviating_fp_fun = std::function<bool()>([]() -> bool { return false; }),
		std::function<bool()> near_collision_fun = std::function<bool()>([]() -> bool { return false; }),
		std::function<bool()> can_recover_fun = std::function<bool()>([]() -> bool { return false; }),
		std::function<bool()> global_plan_outdated_fun = std::function<bool()>([]() -> bool { return false; }),
		double initiation_yaw_threshold	= IGN_DTOR(30)
	);

	/// Should be called at the start of each control loop
	void update(bool new_goal_received = false);

	/// Helper method
	geometry::Angle computeDirectionToPose(const geometry::Pose& pose_check) const;

	inline State getState() const {
		return state_;
	}

	std::string getStateName() const;

	/**
	 * Checks if goal has been reached
	 */
	bool isGoalReached() const {
		return goal_reached_fun_();
	}

protected:
	/// State-name map for conversion from enum to string
	const std::map<State, std::string> STATE_NAMES = {
		{State::INITIATE_EXECUTION, "init"},
		{State::MOVE, "move"},
		{State::ADJUST_ORIENTATION, "adjust"},
		{State::YIELD_WAY_CROSSING, "yield"},
		{State::RECOVERY_ROTATE_AND_RECEDE, "recovery_rr"},
		{State::RECOVERY_LOOK_AROUND, "recovery_la"},
		{State::STOPPED, "stop"}
	};

	State state_;

	/// Angle expressed in radians
	double initiation_yaw_threshold_;

	const geometry::Pose& pose_;
	const geometry::Pose& goal_;
	const geometry::Pose& goal_initiation_;
	const geometry::Pose& goal_local_;

	/// Event checker
	std::function<bool()> pos_reached_fun_;
	/// Event checker
	std::function<bool()> goal_reached_fun_;
	/// Event checker
	std::function<bool()> crossing_detected_fun_;
	/// Event checker
	std::function<bool()> yield_way_crossing_finished_fun_;
	/// Event checker
	std::function<bool()> look_around_finished_fun_;
	/// Event checker
	std::function<bool()> oscillating_fun_;
	/// Event checker
	std::function<bool()> stuck_fun_;
	/// Event checker
	std::function<bool()> deviating_fp_fun_;
	/// Event checker
	std::function<bool()> near_collision_fun_;
	/// Event checker
	std::function<bool()> can_recover_fun_;
	/// Event checker
	std::function<bool()> global_plan_outdated_fun_;
};

} // namespace humap_local_planner
