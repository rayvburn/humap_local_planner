#pragma once

#include <base_local_planner/trajectory_cost_function.h>

namespace humap_local_planner {

/**
 * @brief Cost function penalizing the unsaturated translational movements
 *
 * Penalizes robot's for not keeping its velocity close to the maximum allowable
 */
class UnsaturatedTranslationCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
	UnsaturatedTranslationCostFunction();

	/**
	 * @brief Updates internal parameters
	 *
	 * @param max_trans_vel max translational speed of the robot
	 * @param compute_whole_horizon set to true if the cost function should be computed for each entry
	 * of the trajectory
	 */
	void setParameters(
		double max_trans_vel,
		double max_vel_x,
		double max_vel_y,
		bool compute_whole_horizon = true
	);

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
	/// Max translational speed of the robot
	double max_trans_vel_;
	double max_vel_x_;
	double max_vel_y_;

	bool compute_whole_horizon_;
};

} // namespace humap_local_planner
