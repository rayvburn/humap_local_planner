#pragma once

#include <people_msgs_utils/group.h>
#include <hubero_local_planner/trajectory.h>

namespace hubero_local_planner {

/**
 * Extension of the @ref people_msgs_utils::Group providing trajectory prediction of the group
 */
class Group: public people_msgs_utils::Group {
public:
	/**
	 * Constructor extending basic @ref group with trajectory prediction each @ref dt for @ref steps_num steps forward
	 *
	 * @note This class is currently more of a placeholder as groups are assumed to be static ATM, thus @ref dt ignored
	 */
	Group(const people_msgs_utils::Group& group, double /*dt*/, unsigned int steps_num):
		people_msgs_utils::Group(group),
		traj_prediction_(Trajectory(group, steps_num))
	{}

	Trajectory getTrajectoryPrediction() const {
		return traj_prediction_;
	}

protected:
	Trajectory traj_prediction_;
};

// Basic container
typedef std::vector<Group> Groups;

} // namespace hubero_local_planner
