#pragma once

#include <base_local_planner/trajectory_cost_function.h>
#include <humap_local_planner/geometry/pose.h>

namespace humap_local_planner {

/**
 * @brief Cost function that penalizes high speeds at close distances to global goal
 */
class SpeedyGoalCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
	SpeedyGoalCostFunction(const geometry::Pose& global_goal);

	void setParameters(const double& distance_slow_down, const double vel_trans_ideal);

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
	/// Pose of the global goal
	const geometry::Pose& global_goal_;

	double distance_slow_down_;
	double vel_trans_ideal_;
};

} // namespace humap_local_planner
