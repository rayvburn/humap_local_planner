#pragma once

#include <base_local_planner/trajectory_cost_function.h>

namespace humap_local_planner {

/**
 * @brief Cumulative Heading Changes trajectory cost function
 *
 * Penalizes robot heading changes
 */
class HeadingChangeSmoothnessCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
	HeadingChangeSmoothnessCostFunction() = default;

	/**
	 * @brief General updating of context values if required.
	 * Subclasses may overwrite. Return false in case there is any error.
	 */
	virtual bool prepare() override;

	/**
	 * @brief Returns a score for trajectory @ref traj
	 */
	virtual double scoreTrajectory(base_local_planner::Trajectory& traj) override;
};

} // namespace humap_local_planner
