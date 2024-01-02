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
	double initiation_yaw_threshold):
	state_(STOPPED),
	initiation_yaw_threshold_(initiation_yaw_threshold),
	pose_(pose),
	goal_(goal),
	goal_local_(goal_local),
	goal_initiation_(goal_initiation),
	pos_reached_fun_(pos_reached_fun),
	goal_reached_fun_(goal_reached_fun)
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

	switch (getState()) {
		case INITIATE_EXECUTION: {
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
			}
			break;
		}

		case MOVE: {
			bool pos_reached = pos_reached_fun_();

			// check if any state transition got activated
			if (new_goal_received && pointing_towards_goal) {
				// already pointing towards the local goal -> let's skip INITIATE_EXECUTION
				state_ = MOVE;
			} else if (new_goal_received && !pointing_towards_goal) {
				// initial adjustment of the orientation is required
				state_ = INITIATE_EXECUTION;
			} else if (!new_goal_received && pos_reached) {
				state_ = ADJUST_ORIENTATION;
			}
			break;
		}

		case ADJUST_ORIENTATION: {
			bool goal_reached = goal_reached_fun_();

			// check if any state transition got activated
			if (new_goal_received && pointing_towards_goal) {
				// already pointing towards the local goal -> let's skip INITIATE_EXECUTION
				state_ = MOVE;
			} else if (new_goal_received && !pointing_towards_goal) {
				// initial adjustment of the orientation is required
				state_ = INITIATE_EXECUTION;
			} else if (!new_goal_received && goal_reached) {
				state_ = STOPPED;
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
