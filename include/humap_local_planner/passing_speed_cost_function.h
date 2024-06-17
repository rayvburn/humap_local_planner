#pragma once

#include <base_local_planner/trajectory_cost_function.h>
#include <humap_local_planner/person.h>

#include <vector>

namespace humap_local_planner {

/**
 * @brief Trajectory cost function to penalize robot for not being compliant with proper speeds when passing humans
 */
class PassingSpeedCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
	PassingSpeedCostFunction(const std::vector<Person>& people);

	/**
	 * @brief Updates the internal parameters
	 *
	 * @param max_speed maximum speed achievable by the robot (used for normalization)
	 * @param min_dist minimum distance between the centers of the robot and a human (human is assumed to be a point)
	 * (used for normalization)
	 * @param compute_whole_horizon set to true if the cost function should be computed for each entry
	 * of the trajectory
	 */
	void setParameters(double max_speed, double min_dist, bool compute_whole_horizon);

	/**
	 * @brief General updating of context values if required.
	 * Subclasses may overwrite. Return false in case there is any error.
	 */
	virtual bool prepare() override;

	/**
	 * @brief Returns a score for trajectory @ref traj
	 */
	virtual double scoreTrajectory(base_local_planner::Trajectory& traj) override;

protected:
	/// A reference to a dataset containing people detections
	const std::vector<Person>& people_;
	double max_speed_;
	double min_dist_;
	bool compute_whole_horizon_;
};

} // namespace humap_local_planner
