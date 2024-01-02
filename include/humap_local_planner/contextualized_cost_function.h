#pragma once

#include <base_local_planner/trajectory_cost_function.h>
#include <humap_local_planner/geometry/pose.h>
#include <costmap_2d/costmap_2d.h>

namespace humap_local_planner {

/**
 * @brief Cost function that penalizes trajectories based on costmap costs that are resulted from different contexts
 *
 * Since layered costmap approach is used, different contexts can be embedded into a resultant costmap.
 * This cost function aims to introduce those contextualized penalties in trajectory scoring.
 */
class ContextualizedCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
	ContextualizedCostFunction(costmap_2d::Costmap2D* costmap);

	void setSumScores(bool score_sums);

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
	costmap_2d::Costmap2D* costmap_;
	bool sum_scores_;
};

} // namespace humap_local_planner
