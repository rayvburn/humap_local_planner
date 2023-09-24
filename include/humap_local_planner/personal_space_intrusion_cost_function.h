#pragma once

#include <base_local_planner/trajectory_cost_function.h>
#include <humap_local_planner/person.h>

#include <vector>

namespace humap_local_planner {

/**
 * @brief Trajectory cost function to penalize robot for human's personal space intrusions
 */
class PersonalSpaceIntrusionCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
	PersonalSpaceIntrusionCostFunction(const std::vector<Person>& people);

	/**
	 * @brief Updates the internal parameters
	 *
	 * @param compute_whole_horizon set to true if the cost function should be computed for each entry
	 * of the trajectory
	 */
	void setParameters(bool compute_whole_horizon);

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
	bool compute_whole_horizon_;
};

} // namespace humap_local_planner
