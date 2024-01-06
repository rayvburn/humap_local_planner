#include <humap_local_planner/planner_state.h>

namespace humap_local_planner {

using namespace geometry;

PlannerState::PlannerState(
	const geometry::Pose& pose,
	const geometry::Pose& goal,
	const geometry::Pose& goal_local,
	const geometry::Pose& goal_initiation,
	std::function<bool()> pos_reached_fun,
	std::function<bool()> goal_reached_fun,
	std::function<bool()> crossing_detected_fun,
	std::function<bool()> yield_way_crossing_finished_fun,
	std::function<bool()> oscillating_fun,
	std::function<bool()> stuck_fun,
	std::function<bool()> deviating_fp_fun,
	std::function<bool()> near_collision_fun,
	std::function<bool()> can_recover_fun,
	double initiation_yaw_threshold):
	state_(STOPPED),
	initiation_yaw_threshold_(initiation_yaw_threshold),
	pose_(pose),
	goal_(goal),
	goal_local_(goal_local),
	goal_initiation_(goal_initiation),
	pos_reached_fun_(pos_reached_fun),
	goal_reached_fun_(goal_reached_fun),
	crossing_detected_fun_(crossing_detected_fun),
	yield_way_crossing_finished_fun_(yield_way_crossing_finished_fun),
	oscillating_fun_(oscillating_fun),
	stuck_fun_(stuck_fun),
	deviating_fp_fun_(deviating_fp_fun),
	near_collision_fun_(near_collision_fun),
	can_recover_fun_(can_recover_fun)
{}

void PlannerState::update(bool new_goal_received) {
	bool pointing_towards_goal = false;
	// when goal position is reached robot won't move in XY plane - let's just rotate to face goal orientation
	if (pos_reached_fun_()) {
		pointing_towards_goal = goal_reached_fun_();
	} else {
		auto dir_to_init_pose = std::abs(computeDirectionToPose(goal_initiation_).getRadian());
		pointing_towards_goal = dir_to_init_pose <= initiation_yaw_threshold_;
	}
	bool crossing_detected = crossing_detected_fun_();
	// recovery-related
	bool oscillating = oscillating_fun_();
	bool stuck = stuck_fun_();
	bool near_collision = near_collision_fun_();
	bool can_recover = can_recover_fun_();

	switch (getState()) {
		case INITIATE_EXECUTION: {
			// emergency transition has a higher priority than normal transitions
			if (oscillating || stuck || near_collision) {
				// recovery should be started
				state_ = RECOVERY_ROTATE_AND_RECEDE;
				break;
			}

			// check if any state transition got activated
			if (new_goal_received && pointing_towards_goal) {
				// already pointing towards the local goal -> let's skip INITIATE_EXECUTION
				state_ = MOVE;
			} else if (new_goal_received && !pointing_towards_goal) {
				// initial adjustment of the orientation is required
				state_ = INITIATE_EXECUTION;
			} else if (!new_goal_received && pointing_towards_goal) {
				// already rotated to point base's nose towards goal position
				state_ = MOVE;
			} else if (!new_goal_received && pos_reached_fun_()) {
				state_ = ADJUST_ORIENTATION;
			} else if (!new_goal_received && goal_reached_fun_()) {
				state_ = STOPPED;
			}
			break;
		}

		case MOVE: {
			// emergency transition has a higher priority than normal transitions
			if (oscillating || stuck || near_collision) {
				// recovery should be started
				state_ = RECOVERY_ROTATE_AND_RECEDE;
				break;
			}

			bool pos_reached = pos_reached_fun_();

			// angle towards the local goal (when the robot has the goal behind, it switches to INITIATE_EXECUTION)
			auto dir_to_local_goal = Angle(std::abs(computeDirectionToPose(goal_local_).getRadian())).getRadian();
			// more strict threshold than in 'initiation' case
			bool local_goal_behind_robot = dir_to_local_goal >= (IGN_PI - initiation_yaw_threshold_ / 2.0);

			// check if any state transition got activated
			if (crossing_detected && !new_goal_received && !pos_reached) {
				// yield a way but only when the goal position is not reached yet and the goal did not change
				state_ = YIELD_WAY_CROSSING;
			} else if (new_goal_received && pointing_towards_goal) {
				// already pointing towards the local goal -> let's skip INITIATE_EXECUTION
				state_ = MOVE;
			} else if ((new_goal_received && !pointing_towards_goal) || local_goal_behind_robot) {
				// initial adjustment of the orientation is required
				state_ = INITIATE_EXECUTION;
			} else if (!new_goal_received && pos_reached) {
				state_ = ADJUST_ORIENTATION;
			}
			break;
		}

		case ADJUST_ORIENTATION: {
			// emergency transition has a higher priority than normal transitions
			if (oscillating || stuck || near_collision) {
				// recovery should be started
				state_ = RECOVERY_ROTATE_AND_RECEDE;
				break;
			}

			bool pos_reached = pos_reached_fun_();
			bool goal_reached = goal_reached_fun_();

			// check if any state transition got activated
			if ((new_goal_received && pointing_towards_goal) || !pos_reached) {
				// 1) already pointing towards the local goal -> let's skip INITIATE_EXECUTION
				// 2) somehow the goal has been relocated (target position is not reached anymore)
				state_ = MOVE;
			} else if ((new_goal_received || !pos_reached) && !pointing_towards_goal) {
				// initial adjustment of the orientation is required
				state_ = INITIATE_EXECUTION;
			} else if (!new_goal_received && pos_reached && goal_reached) {
				state_ = STOPPED;
			}
			break;
		}

		case YIELD_WAY_CROSSING: {
			bool routine_finished = yield_way_crossing_finished_fun_();

			if (oscillating || stuck || near_collision) {
				// recovery should be started
				state_ = RECOVERY_ROTATE_AND_RECEDE;
				break;
			}

			if (routine_finished) {
				state_ = MOVE;
			}
			break;
		}

		case RECOVERY_ROTATE_AND_RECEDE: {
			if (!can_recover) {
				state_ = STOPPED;
			} else if (!oscillating && !near_collision) {
				// recovery finished or robot is unstuck, moving further is possible
				state_ = MOVE;
			} else if (!stuck && !near_collision) {
				state_ = INITIATE_EXECUTION;
			}
			break;
		}

		case STOPPED: {
			if (new_goal_received && pointing_towards_goal) {
				// already pointing towards the local goal -> let's skip INITIATE_EXECUTION
				state_ = MOVE;
			} else if (new_goal_received && !pointing_towards_goal) {
				// initial adjustment of the orientation is required
				state_ = INITIATE_EXECUTION;
			} else {
				// just waiting for `newGoalReceived` call
			}
			break;
		}
	}
}

geometry::Angle PlannerState::computeDirectionToPose(const geometry::Pose& pose_check) const {
	auto angle_pos_vector = (pose_check.getPosition() - pose_.getPosition()).calculateDirection();
	return geometry::Angle(angle_pos_vector.getRadian() - pose_.getYaw());
}

std::string PlannerState::getStateName() const {
	auto it = STATE_NAMES.find(state_);
	if (it == STATE_NAMES.end()) {
		return std::string("unknown");
	}
	return it->second;
}

} // namespace humap_local_planner
