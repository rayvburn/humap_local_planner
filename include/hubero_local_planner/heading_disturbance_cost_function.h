#pragma once

#include <base_local_planner/trajectory_cost_function.h>
#include <hubero_local_planner/person.h>

#include <vector>

namespace hubero_local_planner {

/**
 * @brief Trajectory cost function to penalize disturbances introduced by robot's motion to nearby people
 *
 * Penalizes robot motion that e.g. drives it towards person
 */
class HeadingDisturbanceCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
	HeadingDisturbanceCostFunction(const std::vector<Person>& people);

	/**
	 * @brief Updates the internal parameters
	 *
	 * @param fov_person person's field of view (full) that is used in disturbance calculations
	 * @param person_model_radius radius that defines circle that models physical area occupied by a person
	 * @param robot_circumradius defines dimensions of the robot, required for cost normalization
	 * @param max_speed max speed of the robot, required for cost normalization
	 * @param compute_whole_horizon set to true if the cost function should be computed for each entry
	 * of the trajectory
	 */
	void setParameters(
		double fov_person,
		double person_model_radius,
		double robot_circumradius,
		double max_speed,
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
	/// A reference to a dataset containing people detections
	const std::vector<Person>& people_;
	double fov_person_;
	double person_model_radius_;
	double robot_circumradius_;
	double max_speed_;
	bool compute_whole_horizon_;
};

} // namespace hubero_local_planner
