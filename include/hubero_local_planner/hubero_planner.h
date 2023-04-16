#pragma once

#include <vector>
#include <mutex>
#include <memory>

#include <hubero_local_planner/hubero_config.h>
#include <hubero_local_planner/world.h>
#include <hubero_local_planner/social_trajectory_generator.h>
#include <hubero_local_planner/latched_stop_rotate_controller.h>
#include <hubero_local_planner/chc_cost_function.h>
#include <hubero_local_planner/contextualized_cost_function.h>
#include <hubero_local_planner/ttc_cost_function.h>
#include <hubero_local_planner/speedy_goal_cost_function.h>
#include <hubero_local_planner/velocity_smoothness_cost_function.h>
#include <hubero_local_planner/heading_disturbance_cost_function.h>
#include <hubero_local_planner/personal_space_intrusion_cost_function.h>
#include <hubero_local_planner/fformation_space_intrusion_cost_function.h>

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
#include <base_local_planner/prefer_forward_cost_function.h>
#include <base_local_planner/twirling_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>

#include <base_local_planner/simple_trajectory_generator.h>

// useful, free functions
#include <base_local_planner/goal_functions.h>

// MapGridVisualizer
#include <base_local_planner/map_grid.h>

#include <hubero_local_planner/planner_state.h>
#include <hubero_local_planner/obstacles.h>
#include <hubero_local_planner/robot_footprint_model.h>
#include <hubero_local_planner/utils/teb_utils.h>

#include <hubero_local_planner/person.h>
#include <hubero_local_planner/group.h>

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
		Vector force_combined_;
		Vector force_internal_;
		Vector force_interaction_;
		Vector force_social_;
		std::vector<Pose> closest_points_static_;
		std::vector<Pose> closest_points_dynamic_;
		std::string behaviour_active_;

		MotionDriverData() {}
		MotionDriverData(
			const Vector& force_combined,
			const Vector& force_internal,
			const Vector& force_interaction,
			const Vector& force_social,
			const std::vector<Pose>& closest_points_static,
			const std::vector<Pose>& closest_points_dynamic,
			const std::string& behaviour_active):
			force_combined_(force_combined),
			force_internal_(force_internal),
			force_interaction_(force_interaction),
			force_social_(force_social),
			closest_points_static_(closest_points_static),
			closest_points_dynamic_(closest_points_dynamic),
			behaviour_active_(behaviour_active)
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
	 * @brief Updates previously saved global plan, prunes the part located behind the robot
	 * @param  global_pose The robot's current pose in the global frame of the local planner (usually `odom`)
	 * @param  new_plan The new global plan

	 * @note This should be called before @ref findBestTrajectory or @ref findTrajectory
	 */
	bool updatePlan(const geometry_msgs::PoseStamped& global_pose);

	/**
	 * @brief Prepares cost functions for planning
	 * @param footprint_spec The robot's footprint
	 *
	 * The obstacle cost function gets the footprint.
	 * The path and goal cost functions get the global_plan
	 * The alignment cost functions get a version of the global plan that is modified based on the global_pose
	 *
	 * @note This should be called before @ref findBestTrajectory. Not needed before @ref findTrajectory
	 * @note Based on dwa_local_planner::DWAPlanner::updatePlanAndLocalCosts authored by Eitan Marder-Eppstein
	 */
	void updateLocalCosts(const std::vector<geometry_msgs::Point>& footprint_spec);

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
		const Vector& velocity,
		const ObstContainerConstPtr obstacles,
		std::shared_ptr<const People> people,
		std::shared_ptr<const Groups> groups,
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
		const Vector& velocity,
		const ObstContainerConstPtr obstacles,
		std::shared_ptr<const People> people,
		std::shared_ptr<const Groups> groups,
		geometry_msgs::PoseStamped& drive_velocities
	);

	// TODO: add grid `visualization` version, ignores storing `meaningful_interactions` data

	/**
	 * @brief Get the period at which the local planner is expected to run
	 * @return The simulation period
	 */
	inline double getSimPeriod() {
		return cfg_->getGeneral()->sim_period;
	}

	/**
	 * @brief Computes cost function for a given cell in the costmap
	 *
	 * Compute the components and total cost for a map grid cell
	 *
	 * @param cx The x coordinate of the cell in the map grid
	 * @param cy The y coordinate of the cell in the map grid
	 * @param path_cost Will be set to the path distance component of the cost function
	 * @param goal_cost Will be set to the goal distance component of the cost function
	 * @param occ_cost Will be set to the costmap value of the cell
	 * @param total_cost Will be set to the value of the overall cost function, taking into account the scaling parameters
	 * @return True if the cell is traversible and therefore a legal location for the robot to move to
	 *
	 * @note Based on Based on dwa_local_planner::DWAPlanner::getCellCosts authored by Eitan Marder-Eppstein
	 */
	bool computeCellCost(
		int cx,
		int cy,
		float& path_cost,
		float& goal_cost,
		float& occ_cost,
		float& total_cost
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
	bool checkGoalReached(
		const geometry_msgs::PoseStamped& pose,
		const geometry_msgs::PoseStamped& velocity,
		const geometry_msgs::PoseStamped& goal
	);

	/**
	 * @brief Checks force vector that would affect robot placed in @ref pos
	 *
	 * Modifies underlying generator so cannot be marked as const
	 */
	Vector computeForceAtPosition(const Vector& pos);

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
	 * @brief Returns planning state according to the internal finite state machine
	 */
	PlannerState::State getState() const {
		return state_ptr_->getState();
	}

	/**
	 * @brief Returns const reference to all recently explored trajectories
	 */
	inline const std::vector<base_local_planner::Trajectory>& getExploredTrajectories() const {
		return traj_explored_;
	}

	/**
	 * @brief Returns robot and obstacle positions for a TTC
	 *
	 * Outer vector: trajectories
	 * Mid vector: timestamps
	 * Inner vector: robot and obstacle poses
	 *
	 * @return std::vector<std::vector<std::vector<Distance>>>
	 */
	inline const std::vector<std::vector<std::vector<Distance>>>& getTTCPredictionsStatic() const {
		return ttc_costs_.getPredictionStaticObjects();
	}

	/**
	 * @see getTTCPredictionsStatic
	 */
	inline const std::vector<std::vector<std::vector<Distance>>>& getTTCPredictionsDynamic() const {
		return ttc_costs_.getPredictionDynamicObjects();
	}

	/**
	 * @brief Enlarges obstacle by moving its edge point (that is closest to the robot)
	 *
	 * The edge point is moved according to `obstacle_extension_multiplier` from parameters and robot inscribed radius
	 */
	static bool enlargeObstacle(
		const Pose& robot_closest_to_obstacle_pose,
		Pose& obstacle_closest_to_robot_pose,
		double extension_distance,
		double distance_collision_imminent
	);

protected:
	/**
	 * @brief Updates cost functions with the contents of the @ref HuberoConfig
	 * @note Should be called once dynamic reconfigure event occurred
	 */
	void updateCostParameters();

	/**
	 * @brief Fills up the world model with static and dynamic obstacles extracted from obstacle and people containers
	 *
	 * @param pose_ref pose of the robot in the world
	 * @param world_model world model that be filled up with obstacles
	 */
	void createEnvironmentModel(const Pose& pose_ref, World& world_model);

	/**
	 * @brief Retrieves pose that is @ref dist_from_current_pose far from the current pose
	 *
	 * Current pose is the first pose of the global plan.
	 * If plan length is shorter than @ref dist_from_current_pose, the goal pose (last pose of the global plan)
	 * is used instead.
	 */
	Pose getPoseFromPlan(const double& dist_from_current_pose) const;

	/**
	 * @brief Retrieves goal pose from the global plan
	 */
	Pose getGoalFromPlan() const;

	/**
	 * @brief Performs planning specific when a robot moves with significant translational velocity
	 */
	bool planMovingRobot();

	/**
	 * @brief Performs so called stop & rotate action to adjust robot orientation to the goal orientation
	 */
	bool planOrientationAdjustment(const Pose& goal);

	/**
	 * @brief Helper method for LatchedStopRotateController to check if trajectory is valid
	 *
	 * Based on dwa_local_planner::DWAPlanner::checkTrajectory by Eitan Marder-Eppstein
	 */
	bool checkInPlaceTrajectory(
		const Eigen::Vector3f pos,
		const Eigen::Vector3f vel,
		const Eigen::Vector3f vel_samples
	);

	/**
	 * @brief Updates motion driving factors based on the newest data from trajectory generator
	 */
	void collectTrajectoryMotionData();

	/**
	 * @brief Prints details of @ref traj_explored_
	 */
	void logTrajectoriesDetails();

	/**
	 * Planner state handler that introduces different behaviours of planner
	 *
	 * Behaviours depend on the navigation stage and goal placement. This was mostly introduced for non-holonomic
	 * mobile bases that suffer from chaotic rotations once new goal is placed far behind the robot
	 */
	std::unique_ptr<PlannerState> state_ptr_;

	std::shared_ptr<base_local_planner::LocalPlannerUtil> planner_util_;

	/// Stores all trajectories explored in last @ref findBestTrajectory execution
	std::vector<base_local_planner::Trajectory> traj_explored_;

	/// Stores the trajectory with a highest score, updated by the SimpleScoredSamplingPlanner instance
	base_local_planner::Trajectory result_traj_;

	/**
	 * Global plan that gets pruned in @ref updatePlan call
	 *
	 * Contains only this part of the global plan that is contained by local costmap
	 */
	std::vector<geometry_msgs::PoseStamped> global_plan_;
	/**
	 * @brief Stores the newest goal pose that is expressed in the global frame
	 * Goal expressed in the global frame does not move, in contrary to the local frame ('odom')
	 */
	geometry_msgs::PoseStamped goal_global_frame_;
	bool goal_reached_;

	/**
	 * @brief Using `LatchedStopRotateController`, robot will simply rotate after position is reached
	 * Underlying motion model does not take orientation into account.
	 */
	LatchedStopRotateController stop_rotate_controller_;

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
	/// @brief Motion initiation goal expressed in the planner's frame
	Pose goal_initiation_;
	/// @brief The most recent environment (obstacles) model
	ObstContainerConstPtr obstacles_;
	/// @brief The most recent information on people in the environment
	std::shared_ptr<const People> people_;
	/// @brief The most recent information on groups (F-formations) in the environment
	std::shared_ptr<const Groups> groups_;
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
	SocialTrajectoryGenerator generator_social_;
	base_local_planner::SimpleTrajectoryGenerator generator_vel_space_;
	base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;

	/// Cost function that discards trajectories that move into obstacles
	base_local_planner::ObstacleCostFunction obstacle_costs_;
	/// Cost function that discards oscillating motions (assisgns cost -1)
	base_local_planner::OscillationCostFunction oscillation_costs_;
	/// Cost function that prefers trajectories on global path
	base_local_planner::MapGridCostFunction path_costs_;
	/// Cost function that prefers trajectories that go towards (local) goal, based on wave propagation
	base_local_planner::MapGridCostFunction goal_costs_;
	/// Cost function that prefers trajectories that keep the robot nose on nose path
	base_local_planner::MapGridCostFunction alignment_costs_;
	/// Cost function that prefers trajectories that make the nose go towards (local) nose goal
	base_local_planner::MapGridCostFunction goal_front_costs_;
	/// Cost function that prefers forward trajectories instead of those that consist of backward motions
	base_local_planner::PreferForwardCostFunction backward_costs_;
	/// Cost function that penalizes trajectories that can cause collision in a longer horizon
	TTCCostFunction ttc_costs_;
	/// Cost function that penalizes robot heading changes
	CHCCostFunction chc_costs_;
	/// Prevents overshoot when robot approaches goal position with a high speed
	SpeedyGoalCostFunction speedy_goal_costs_;
	/// Advantages trajectories that maintain velocities similar to the current one
	VelocitySmoothnessCostFunction velocity_smoothness_costs_;
	/// Takes into account various contexts included in layered costmap (includes obstacle inflation too)
	ContextualizedCostFunction contextualized_costs_;
	/// Penalizes robot trajectories that drive it towards the center of any person
	HeadingDisturbanceCostFunction heading_disturbance_costs_;
	// Penalizes robot intrusions into people's personal spaces
	PersonalSpaceIntrusionCostFunction personal_space_costs_;
	// Penalizes robot intrusions into F-formation's O-spaces
	FformationSpaceIntrusionCostFunction fformation_space_costs_;
	/// @}

}; // class HuberoPlanner

typedef std::shared_ptr<HuberoPlanner> HuberoPlannerPtr;

}; // namespace hubero_local_planner
