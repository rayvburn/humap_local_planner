#pragma once

#include <base_local_planner/trajectory_cost_function.h>
#include <humap_local_planner/geometry/vector.h>

namespace humap_local_planner {

/**
 * @brief Trajectory Velocity Smoothnesss cost function
 *
 * Penalizes robot velocity changes throughout trajectory (current velocity inclusive)
 */
class VelocitySmoothnessCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
	VelocitySmoothnessCostFunction(const geometry::Vector& velocity_base);

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
