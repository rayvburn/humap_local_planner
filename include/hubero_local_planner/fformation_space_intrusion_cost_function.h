#pragma once

#include <base_local_planner/trajectory_cost_function.h>
#include <people_msgs_utils/group.h>

#include <vector>

namespace hubero_local_planner {

/**
 * @brief Trajectory cost function to penalize robot for F-formation's O-space intrusions
 */
class FformationSpaceIntrusionCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
	FformationSpaceIntrusionCostFunction();

	/**
	 * @brief Updates dataset containing F-formations (groups) detections
	 */
	void setFformationsDetections(const std::vector<people_msgs_utils::Group>& groups);

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
	std::vector<people_msgs_utils::Group> groups_;
};

} // namespace hubero_local_planner
