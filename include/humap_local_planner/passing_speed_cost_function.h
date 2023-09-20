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
};

} // namespace humap_local_planner
