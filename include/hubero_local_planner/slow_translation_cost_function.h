#pragma once

#include <base_local_planner/trajectory_cost_function.h>

namespace hubero_local_planner {

/**
 * @brief Slow Translational Movement cost function
 *
 * Penalizes robot's for not keeping its velocity close to the maximum allowable
 */
class SlowTranslationCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
	SlowTranslationCostFunction() = default;

	/**
	 * @brief Updates internal parameters
	 *
	 * @param max_trans_vel max translational speed of the robot
	 */
	void setParameters(double max_trans_vel, double max_vel_x, double max_vel_y);

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
};

} // namespace hubero_local_planner
