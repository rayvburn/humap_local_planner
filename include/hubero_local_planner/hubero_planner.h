#pragma once

#include <vector>
#include <mutex>
#include <memory>

#include <hubero_local_planner/hubero_config.h>
#include <hubero_local_planner/world.h>
#include <hubero_local_planner/social_trajectory_generator.h>

//for creating a local cost grid
#include <base_local_planner/map_grid_visualizer.h>
#include <pcl_ros/publisher.h>

//for obstacle data access
#include <costmap_2d/costmap_2d.h>

#include <base_local_planner/trajectory.h>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/local_planner_util.h>

#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/twirling_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>

// useful, free functions
#include <base_local_planner/goal_functions.h>

// MapGridVisualizer
#include <base_local_planner/map_grid.h>

#include <hubero_local_planner/obstacles.h>
#include <hubero_local_planner/robot_footprint_model.h>
#include <hubero_local_planner/utils/teb_utils.h>

#include <nav_msgs/Path.h>

namespace hubero_local_planner {

using namespace geometry;
/**
 * @class HuberoPlanner
 * @brief A class implementing a local planner using the HuberoPlanner
 */
class HuberoPlanner {
public:
	// both environmental and internal drivers
	struct MotionDriverData {
		Vector force_combined;
		Vector force_internal;
		Vector force_interaction;
		Vector force_social;
		std::vector<Pose> closest_points;
		std::string behaviour_active;

		MotionDriverData() {}
		MotionDriverData(
			const Vector& force_combined,
			const Vector& force_internal,
			const Vector& force_interaction,
			const Vector& force_social,
			const std::vector<Pose>& closest_points,
			const std::string& behaviour_active):
			force_combined(force_combined),
			force_internal(force_internal),
			force_interaction(force_interaction),
			force_social(force_social),
			closest_points(closest_points),
			behaviour_active(behaviour_active)
			{}
	};

	/**
	 * @brief Constructor for the planner
	 * @param name The name of the planner
	 * @param planner_util A pointer to the initialized instance of the LocalPlannerUtil class
	 */
	HuberoPlanner(
			const std::string& name,
			std::shared_ptr<base_local_planner::LocalPlannerUtil> planner_util,
			RobotFootprintModelPtr robot_model,
			std::vector<geometry_msgs::Point> footprint_spec
	);

	/**
	 * @brief  Destructor for the planner
	 */
	virtual ~HuberoPlanner();

	void reconfigure(HuberoConfigConstPtr cfg);

	/**
	 * @brief  Check if a trajectory is legal for a position/velocity pair
	 * @param pos The robot's position
	 * @param vel The robot's velocity
	 * @param vel_samples The desired velocity
	 * @return True if the trajectory is valid, false otherwise
	 */
	bool checkTrajectory(
		const Eigen::Vector3f pos,
		const Eigen::Vector3f vel,
		const Eigen::Vector3f vel_samples
	);

	/**
	 * @brief Given the current position and velocity of the robot, find the best trajectory to exectue
	 * @param pose robot pose in the global frame of the local planner (usually `odom`)
	 * @param velocity robot velocity in the base coordinate system
	 * @param goal global goal
	 * @param obstacles robot environment model
	 * @param drive_velocities The velocities to send to the robot base
	 * @return The highest scoring trajectory. A cost >= 0 means the trajectory is legal to execute.
	 */
	base_local_planner::Trajectory findBestTrajectory(
		const Pose& pose,
		const Vector& velocity,
		const Pose& goal,
		const ObstContainerConstPtr obstacles,
		geometry_msgs::PoseStamped& drive_velocities
	);

	/**
	 * @brief Performs one-shoot force calculation of next velocity without evaluation further than 1 step forward
	 * @param pose robot pose in the global frame of the local planner (usually `odom`)
	 * @param velocity robot velocity in the base coordinate system
	 * @param goal global goal
	 * @param obstacles robot environment model
	 * @param drive_velocities trajectory generation output, stores x, y and theta velocities
	 */
	base_local_planner::Trajectory findTrajectory(
		const Pose& pose,
		const Vector& velocity,
		const Pose& goal,
		const ObstContainerConstPtr obstacles,
		geometry_msgs::PoseStamped& drive_velocities
	);

	// TODO: add grid `visualization` version, ignores storing `meaningful_interactions` data

	/**
	 * @brief  Update the cost functions before planning
	 * @param  global_pose The robot's current pose
	 * @param  new_plan The new global plan
	 * @param  footprint_spec The robot's footprint
	 *
	 * The obstacle cost function gets the footprint.
	 * The path and goal cost functions get the global_plan
	 * The alignment cost functions get a version of the global plan
	 *   that is modified based on the global_pose
	 */
	void updatePlanAndLocalCosts(const geometry_msgs::PoseStamped& global_pose,
		const std::vector<geometry_msgs::PoseStamped>& new_plan,
		const std::vector<geometry_msgs::Point>& footprint_spec
	);

	/**
	 * @brief Get the period at which the local planner is expected to run
	 * @return The simulation period
	 */
	inline double getSimPeriod() {
		return cfg_->getGeneral()->sim_period;
	}

	/**
	 * @brief Compute the components and total cost for a map grid cell
	 * @param cx The x coordinate of the cell in the map grid
	 * @param cy The y coordinate of the cell in the map grid
	 * @param path_cost Will be set to the path distance component of the cost function
	 * @param goal_cost Will be set to the goal distance component of the cost function
	 * @param occ_cost Will be set to the costmap value of the cell
	 * @param total_cost Will be set to the value of the overall cost function, taking into account the scaling parameters
	 * @return True if the cell is traversible and therefore a legal location for the robot to move to
	 */
	bool getCellCosts(int cx,
		int cy,
		float &path_cost,
		float &goal_cost,
		float &occ_cost,
		float &total_cost
	);

	/**
	 * @brief Sets new plan and resets state
	 * @return True if operation was successful
	 */
	bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

	bool getVelocityCommand(double& vx, double& vy, double& omega, int look_ahead_poses) const;

	/**
	 * @brief Performs evaluation of the goal reachment
	 * @param pose
	 * @param goal
	 * @return
	 */
	bool checkGoalReached(const tf::Stamped<tf::Pose>& pose, const tf::Stamped<tf::Pose>& goal);

	/**
	 * @brief Evaluates whether goal is considered as reached
	 * @return
	 */
	inline bool isGoalReached() const {
		return goal_reached_;
	}

	/**
	 * @brief Retrieves motion data, i.e. forces and some environment information
	 * @return
	 */
	inline MotionDriverData getMotionData() const {
		return motion_data_;
	}

	const RobotFootprintModelConstPtr getRobotFootprintModel() const {
		return robot_model_;
	}

	Pose getGoalLocal() const {
		return goal_local_;
	}

	/**
	 * @brief Returns const reference to all recently explored trajectories
	 */
	inline const std::vector<base_local_planner::Trajectory>& getExploredTrajectories() const {
		return traj_explored_;
	}

private:
	// fills up the world model with static and dynamic obstacles
	void createEnvironmentModel(const Pose& pose_ref);

	/**
	 * @return True if given @ref goal_local_ was modified, False otherwise
	 */
	bool chooseGoalBasedOnGlobalPlan();

	/**
	 * @brief Updates motion driving factors based on the newest data from trajectory generator
	 */
	void collectTrajectoryMotionData();

	std::shared_ptr<base_local_planner::LocalPlannerUtil> planner_util_;

	/// Stores all trajectories explored in last @ref findBestTrajectory execution
	std::vector<base_local_planner::Trajectory> traj_explored_;

	/// Stores the trajectory with a highest score, updated by the SimpleScoredSamplingPlanner instance
	base_local_planner::Trajectory result_traj_;

	std::vector<geometry_msgs::PoseStamped> global_plan_;
	bool goal_reached_;

	std::mutex configuration_mutex_;
	/// @brief Parameters of the planner
	HuberoConfigConstPtr cfg_;
	/// @brief The most recent robot pose expressed in the planner's frame
	Pose pose_;
	/// @brief The most recent robot velocity expressed in the base frame
	Vector vel_;
	/// @brief Global goal expressed in the planner's frame
	Pose goal_;
	/// @brief Local goal expressed in the planner's frame
	Pose goal_local_;
	/// @brief The most recent environment (obstacles) model
	ObstContainerConstPtr obstacles_;
	/// @brief Robot footprint model
	RobotFootprintModelPtr robot_model_;
	/// @brief World model
	World world_model_;

	// sampling? copies for visualization
	MotionDriverData motion_data_;

	/**
	 * @defgroup planning Search-based planning utils
	 * @{
	 */
	SocialTrajectoryGenerator generator_;
	base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;
	/// @}

}; // class HuberoPlanner

typedef std::shared_ptr<HuberoPlanner> HuberoPlannerPtr;

}; // namespace hubero_local_planner
