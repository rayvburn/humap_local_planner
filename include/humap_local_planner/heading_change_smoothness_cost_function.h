#pragma once

#include <base_local_planner/trajectory_cost_function.h>
#include <humap_local_planner/geometry/vector.h>

namespace humap_local_planner {

/**
 * @brief Cumulative Heading Changes trajectory cost function
 *
 * Penalizes robot heading changes throughout trajectory (current 'omega' inclusive)
 */
class HeadingChangeSmoothnessCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
	HeadingChangeSmoothnessCostFunction(const geometry::Vector& velocity_base);

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
	/// Velocity of the robot base in base coordinate system
	const geometry::Vector& velocity_base_;
};

} // namespace humap_local_planner
