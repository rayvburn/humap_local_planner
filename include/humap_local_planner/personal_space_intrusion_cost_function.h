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
	PersonalSpaceIntrusionCostFunction();

	/**
	 * @brief Updates dataset containing people detections
	 */
	void setPeopleDetections(const std::vector<Person>& people);

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
	std::vector<Person> people_;
};

} // namespace humap_local_planner
