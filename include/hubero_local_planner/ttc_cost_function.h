#pragma once

#include <base_local_planner/trajectory_cost_function.h>
#include <hubero_local_planner/world.h>

namespace hubero_local_planner {

/**
 * @brief Time to Collision trajectory cost function
 */
class TTCCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
	/**
	 * @brief Ctor of TTC cost function
	 *
	 * @param world_model world model const reference
	 */
	TTCCostFunction(const World& world_model);

	/**
	 * @brief Set the Parameter object
	 *
	 * @param max_sim_time maximum time that world state is predicted forward, if no collision is found,
	 * TTC is set to infinity, cost to 0
	 * @param collision_distance distance threshold for collision detection
	 */
	void setParameters(const double& max_sim_time, const double& collision_distance);

	/**
	 * @brief Clears dataset with poses of robot and world objects
	 */
	void reset();

	/**
	 * @brief General updating of context values if required.
	 * Subclasses may overwrite. Return false in case there is any error.
	 */
	virtual bool prepare() override;

	/**
	 * @brief Returns a score for trajectory @ref traj
	 */
	virtual double scoreTrajectory(base_local_planner::Trajectory& traj) override;

	/**
	 * @brief Returns const reference to dataset with poses of robot and world static objects
	 */
	inline const std::vector<std::vector<std::vector<Distance>>>& getPredictionDynamicObjects() const {
		return robot_dyn_obj_v_;
	}

	/**
	 * @brief Returns const reference to dataset with poses of robot and world dynamic objects
	 */
	inline const std::vector<std::vector<std::vector<Distance>>>& getPredictionStaticObjects() const {
		return robot_stat_obj_v_;
	}

protected:
	/**
	 * @brief Extends @ref prediction_data with world objects data for a given timestamp
	 */
	void collectWorldStateData(
		const World& world_model,
		std::vector<std::vector<Distance>>& prediction_data_static,
		std::vector<std::vector<Distance>>& prediction_data_dynamic
	);

	/**
	 * @brief Checks for collision
	 *
	 * Uses simple distance checking to evaluate if a collision of robot with obstacle will occur
	 */
	bool checkForCollision(
		const World& world_model,
		const double& dist_min_static,
		const double& dist_min_dynamic
	);

	/**
	 * @brief Computes actual cost of the trajectory based on time to collision
	 */
	double computeCost(double ttc, double total_prediction_time) const;

	const World& world_model_;
	double max_sim_time_;
	double collision_distance_;

	/**
	 * @defgroup datasets Datasets with poses of robot and world objects
	 *
	 * Outer vector: evaluated trajectories
	 * Mid vector: consecutive timestamps
	 * Inner vector: robot and obstacle poses
	 *
	 * @{
	 */
	std::vector<std::vector<std::vector<Distance>>> robot_dyn_obj_v_;
	std::vector<std::vector<std::vector<Distance>>> robot_stat_obj_v_;
	/// @}

}; // class TTCCostFunction

} // namespace hubero_local_planner
