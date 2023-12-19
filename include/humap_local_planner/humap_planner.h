#pragma once

#include <vector>
#include <mutex>
#include <memory>

#include <humap_local_planner/humap_config.h>
#include <humap_local_planner/world.h>
#include <humap_local_planner/social_trajectory_generator.h>
#include <humap_local_planner/latched_stop_rotate_controller.h>
#include <humap_local_planner/obstacle_separation_cost_function.h>
#include <humap_local_planner/heading_change_smoothness_cost_function.h>
#include <humap_local_planner/ttc_cost_function.h>
#include <humap_local_planner/velocity_smoothness_cost_function.h>
#include <humap_local_planner/heading_disturbance_cost_function.h>
#include <humap_local_planner/personal_space_intrusion_cost_function.h>
#include <humap_local_planner/fformation_space_intrusion_cost_function.h>
#include <humap_local_planner/passing_speed_cost_function.h>
#include <humap_local_planner/unsaturated_translation_cost_function.h>

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
#include <base_local_planner/prefer_forward_cost_function.h>
#include <base_local_planner/twirling_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>

#include <base_local_planner/simple_trajectory_generator.h>

// useful, free functions
#include <base_local_planner/goal_functions.h>

// MapGridVisualizer
#include <base_local_planner/map_grid.h>

#include <humap_local_planner/planner_state.h>
#include <humap_local_planner/obstacles.h>
#include <humap_local_planner/robot_footprint_model.h>
#include <humap_local_planner/utils/teb_utils.h>

#include <humap_local_planner/person.h>
#include <humap_local_planner/group.h>

#include <humap_local_planner/path_crossing_detector.h>
#include <humap_local_planner/yield_way_crossing_manager.h>
#include <humap_local_planner/recovery_manager.h>

#include <nav_msgs/Path.h>

namespace humap_local_planner {

using namespace geometry;
/**
 * @class HumapPlanner
 * @brief A class implementing a local planner using the HumapPlanner
 */
class HumapPlanner {
public:
	/**
	 * Percentage of obstacle points that must lie within person area to treat the obstacle as a person
	 * See @ref extractNonPeopleObstacles for details
	 */
	static constexpr double PERSON_POLYGON_CONTAINMENT_RATE = 0.667;

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
	HumapPlanner(
			const std::string& name,
			std::shared_ptr<base_local_planner::LocalPlannerUtil> planner_util,
			RobotFootprintModelPtr robot_model,
			std::vector<geometry_msgs::Point> footprint_spec
	);

	/**
	 * @brief  Destructor for the planner
	 */
	virtual ~HumapPlanner();

	void reconfigure(HumapConfigConstPtr cfg);

	/**
	 * @brief  Check if a trajectory is legal for a position/velocity pair
	 *
	 * Helper method for LatchedStopRotateController to check if trajectory is valid
	 *
	 * @param pos The robot's position
	 * @param vel The robot's velocity
	 * @param vel_samples The desired velocity
	 * @return True if the trajectory is valid, false otherwise
	 *
	 * Based on dwa_local_planner::DWAPlanner::checkTrajectory by Eitan Marder-Eppstein
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
	 * @brief Given the current position and velocity of the robot, find the best trajectory to exectue
	 * @param footprint_spec the robot's footprint
	 * @param pose robot pose in the global frame of the local planner (usually `odom`)
	 * @param velocity robot velocity in the base coordinate system
	 * @param goal global goal
	 * @param obstacles robot environment model
	 * @param drive_velocities The velocities to send to the robot base
	 * @return The highest scoring trajectory. A cost >= 0 means the trajectory is legal to execute.
	 */
	base_local_planner::Trajectory findBestTrajectory(
		const std::vector<geometry_msgs::Point>& footprint_spec,
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
	 * @brief Computes the scalarized multi objective cost function for a given cell in the costmap
	 *
	 * Compute the components and total cost for a map grid cell
	 *
	 * @param cx The x coordinate of the cell in the map grid
	 * @param cy The y coordinate of the cell in the map grid
	 * @param costs Map that will be extended according to all spatial components of the cost function, taking into
	 * account the scaling parameters.

	 * @return True if the cell is traversible and therefore a legal location for the robot to move to.
	 * If true is returned, map keys are: "path", "goal", "layered", "alignment", "goal_front", "total"
	 *
	 * @note Partially based on dwa_local_planner::DWAPlanner::getCellCosts authored by Eitan Marder-Eppstein
	 */
	bool computeCellCost(int cx, int cy, std::map<std::string, float>& costs);

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

	Pose getGoalInitiation() const {
		return goal_initiation_;
	}

	Pose getGoalRotateAndRecedeRecovery() const {
		return recovery_.getRotateAndRecedeRecoveryGoal();
	}

	/**
	 * @brief Returns planning state according to the internal finite state machine
	 */
	PlannerState::State getState() const {
		return state_ptr_->getState();
	}

	/**
	 * @brief Returns string representation of the state according to the internal finite state machine
	 */
	std::string getStateName() const {
		return state_ptr_->getStateName();
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

	/// Retrieves pruned path plan received from the global planner
	inline const std::vector<geometry_msgs::PoseStamped>& getGlobalPlanPruned() const {
		return global_plan_;
	}

	/// Retrieves a container with information about the people in the environment that are regarded in the env. model
	inline People getPeople() const {
		return people_env_model_;
	}

	/// Retrieves a container with information about the groups in the environment that are regarded in the env. model
	inline Groups getGroups() const {
		return groups_env_model_;
	}

	/**
	 * @brief Enlarges obstacle by moving its edge point (that is closest to the robot)
	 *
	 * The edge point is moved according to `obstacle_extension_multiplier` from parameters and robot's inscribed radius
	 */
	static bool enlargeObstacle(
		const Pose& robot_closest_to_obstacle_pose,
		Pose& obstacle_closest_to_robot_pose,
		double extension_distance,
		double distance_collision_imminent
	);

	/**
	 * Given a set of obstacles and set of people, this function extracts generic obstacles that are not people
	 *
	 * People are detected independently from obstacles so they must be erased from obstacles.
	 * Obstacle is treated as a person when most of its points (@ref min_containment_rate) are located within
	 * the given radius @ref person_model_radius
	 *
	 * @param obstacles a set of all obstacles (recently detected)
	 * @param people a set of people (recently detected)
	 * @param person_model_radius
	 * @param min_containment_rate when **this** percentage of obstacle's points are located within the radius of
	 * @ref person_model_radius from any person, the obstacle is treated as a person and extracted from initial set
	 * of obstacles
	 * @return ObstContainer possibly trimmed input container @ref obstacles
	 */
	static ObstContainer extractNonPeopleObstacles(
		const ObstContainer& obstacles,
		const People& people,
		double person_model_radius,
		double min_containment_rate
	);

	/// Among the @ref objects, selects @ref max_object_num with the smallest @ref scalar_objective_fun value
	/// Objects may be people or obstacles, whereas metric may be, e.g., Euclidean distance.
	/// The function argument is to provide a lambda that computes the metric.
	/// Templates must be implemented in headers
	template<typename T>
	static std::vector<T> selectRelevant(
		const std::vector<T>& objects,
		std::function<double(const T&)> scalar_objective_fun,
		size_t max_object_num
	) {
		if (objects.size() <= max_object_num) {
			// no need to select since there are too few objects
			return std::move(objects);
		}

		// some type of objects is completely neglected
		if (max_object_num == 0) {
			return std::vector<T>();
		}

		// pair with scalar metric and object
		std::vector<std::pair<double, T>> objects_sort;
		// save metrics for each object
		for (const auto& obj: objects) {
			double metric = scalar_objective_fun(obj);
			objects_sort.push_back({metric, obj});
		}
		std::sort(
			objects_sort.begin(),
			objects_sort.end(),
			[](const auto& left, const auto& right) {
				return left.first < right.first;
			}
		);
		// create objects set with the N-smallest-metrics
		std::vector<T> objects_selected;
		for (const auto& obj_wmetric: objects_sort) {
			objects_selected.push_back(obj_wmetric.second);
			// check if enough collected
			if (objects_selected.size() >= max_object_num) {
				break;
			}
		}
		return objects_selected;
	}

	/**
	 * Computes maximum distance that can be traversed during the @ref sim_period, given the kinodynamic limits
	 * and initial velocity @ref vel_init
	 *
	 * The distance is computed as Euclidean distance between first and last predicted pose
	 */
	static double computeDistanceLimits(
		const geometry::Pose& pos_init,
		const geometry::Vector& vel_init,
		const double& sim_period,
		const double& sim_granularity,
		const double& acc_lim_x,
		const double& acc_lim_y,
		const double& acc_lim_th,
		const double& vel_min_x,
		const double& vel_min_y,
		const double& vel_min_th,
		const double& vel_max_x,
		const double& vel_max_y,
		const double& vel_max_th
	);

protected:
	/// Storage for the cost scales that need to be weighed according to the raw scale and the costmap resolution
	struct ScalesCmCostFunctions {
		/// Default constructor that assigns zeros to all scales
		ScalesCmCostFunctions():
			path_distance_scale(0.0),
			goal_distance_scale(0.0),
			alignment_scale(0.0),
			goal_front_scale(0.0) {}

		/// Ctor that calculates resolution-corrected scales
		ScalesCmCostFunctions(const CostParams& cost_params, double costmap_resolution):
			path_distance_scale(cost_params.path_distance_scale * costmap_resolution),
			goal_distance_scale(cost_params.goal_distance_scale * costmap_resolution),
			alignment_scale(cost_params.alignment_scale * costmap_resolution),
			goal_front_scale(cost_params.goal_front_scale * costmap_resolution) {}

		double path_distance_scale;
		double goal_distance_scale;
		double alignment_scale;
		double goal_front_scale;
	};

	/**
	 * @brief Updates cost functions with the contents of the @ref HumapConfig
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
	 * @brief Prepares cost functions for planning
	 * @param footprint_spec The robot's footprint
	 *
	 * The obstacle cost function gets the footprint.
	 * The path and goal cost functions get the global_plan
	 * The alignment cost functions get a version of the global plan that is modified based on the global_pose
	 *
	 * @note Based on dwa_local_planner::DWAPlanner::updatePlanAndLocalCosts authored by Eitan Marder-Eppstein
	 */
	void updateLocalCosts(const std::vector<geometry_msgs::Point>& footprint_spec);

	/**
	 * @brief Retrieves pose that is @ref dist_from_current_pose far from the current pose
	 *
	 * Current pose is the first pose of the global plan.
	 * If plan length is shorter than @ref dist_from_current_pose, the goal pose (last pose of the global plan)
	 * is used instead (when @ref allow_exceeding_plan is set to false).
	 *
	 * When @ref allow_exceeding_plan is set to true, the last and second to last poses will be used
	 * to approximate the goal lying at the requested distance (beyond the plan bounds)
	 *
	 * @ref allow_exceeding_plan was introduced since the planner utils transforms and trims the plan to the costmap
	 * bounds. Reimplementing its @ref transformGlobalPlan is problematic; therefore, introduced flag aims to allow
	 * approximating the poses beyond the plan
	 *
	 * @ref allow_poses_behind simply allows selecting a pose behind a robot (useful for choosing the motion
	 * initiation goal)
	 */
	Pose getPoseFromPlan(
		const double& dist_from_current_pose,
		bool allow_exceeding_plan = false,
		bool allow_poses_behind = false
	) const;

	/**
	 * @brief Performs planning specific when a robot moves with significant translational velocity
	 */
	bool planMovingRobot();

	/**
	 * @brief Performs so called stop & rotate action to adjust robot orientation to the goal orientation
	 */
	bool planOrientationAdjustment(const Pose& goal);

	/**
	 * @brief Performs planning of a yielding way to a person that crosses the path of the robot
	 */
	bool planYieldWayCrossing();

	/**
	 * @brief Performs a recovery once the robot is stuck according to the cost functions
	 */
	bool planRecoveryRotateAndRecede();

	/**
	 * @brief Updates motion driving factors based on the newest data from the trajectory generator
	 */
	void collectTrajectoryMotionData();

	/**
	 * @brief Updates motion driving factors based on the trajectory given by @ref best_traj
	 * @return True if the @ref best_traj was generated by the @ref SocialTrajectoryGenerator
	 */
	bool collectTrajectoryMotionData(const base_local_planner::Trajectory& best_traj);

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

	/// @brief Parameters of the planner
	HumapConfigConstPtr cfg_;
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
	/// @brief The most recent observed people that are taken into consideration when creating env. model
	ObstContainer obstacles_env_model_;
	/// @brief The most recent information on people in the environment
	std::shared_ptr<const People> people_;
	/// @brief The most recent observed people that are taken into consideration when creating env. model
	People people_env_model_;
	/// @brief The most recent information on groups (F-formations) in the environment
	std::shared_ptr<const Groups> groups_;
	/// @brief The most recent observed groups that are taken into consideration when creating env. model
	Groups groups_env_model_;
	/// @brief Robot footprint model
	RobotFootprintModelPtr robot_model_;
	/// @brief World model
	World world_model_;

	// sampling? copies for visualization
	MotionDriverData motion_data_;

	/// How many trajectories were generated during the planner's lifetime
	unsigned long long trajectories_num_total_;
	/// How many trajectories were generated by the social trajectory generator
	unsigned long long trajectories_num_social_gen_;

	/**
	 * @defgroup planning Search-based planning utils
	 * @{
	 */
	SocialTrajectoryGenerator generator_social_;
	base_local_planner::SimpleTrajectoryGenerator generator_vel_space_;
	base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;

	// Stores original cost scales adjusted for the costmap resolution
	ScalesCmCostFunctions scales_cm_costs_;

	// Detector of crossing the robot's path by a nearby people
	PathCrossingDetector crossing_;
	// Helps to perform the yielding way procedure
	YieldWayCrossingManager yield_way_crossing_;

	// Detects failures and plans recoveries
	RecoveryManager recovery_;

	/**
	 * Cost function that discards trajectories that move into obstacles or other high-cost areas
	 * Investigates different contexts that can be embedded into a layered costmap, e.g. social layers
	 */
	// base_local_planner::ObstacleCostFunction obstacle_costs_;
	ObstacleSeparationCostFunction obstacle_costs_;
	/// Cost function that discards oscillating motions (assigns cost -1)
	base_local_planner::OscillationCostFunction oscillation_costs_;
	/// Cost function that prefers trajectories on global path
	base_local_planner::MapGridCostFunction path_costs_;
	/// Cost function that prefers trajectories that go towards (local) goal, based on wave propagation
	base_local_planner::MapGridCostFunction goal_costs_;
	/// Cost function that prefers trajectories that keep the robot nose on nose path
	base_local_planner::MapGridCostFunction alignment_costs_;
	/// Cost function that prefers trajectories that make the nose go towards (local) nose goal
	base_local_planner::MapGridCostFunction goal_front_costs_;
	// Penalizes robot for not saturating the trans. vels - moving slower than the maximum speed allows
	UnsaturatedTranslationCostFunction unsaturated_trans_costs_;
	/// Cost function that prefers forward trajectories instead of those that consist of backward motions
	base_local_planner::PreferForwardCostFunction backward_costs_;
	/// Cost function that penalizes trajectories that can cause collision in a longer horizon
	TTCCostFunction ttc_costs_;
	/// Cost function that penalizes robot rotational velocity changes
	HeadingChangeSmoothnessCostFunction heading_change_smoothness_costs_;
	/// Advantages trajectories that maintain velocities similar to the current one
	VelocitySmoothnessCostFunction velocity_smoothness_costs_;
	/// Penalizes robot trajectories that drive it towards the center of any person
	HeadingDisturbanceCostFunction heading_disturbance_costs_;
	// Penalizes robot intrusions into people's personal spaces
	PersonalSpaceIntrusionCostFunction personal_space_costs_;
	// Penalizes robot intrusions into F-formation's O-spaces
	FformationSpaceIntrusionCostFunction fformation_space_costs_;
	// Penalizes robot for not being compliant with proper speeds when passing humans
	PassingSpeedCostFunction passing_speed_costs_;
	/// @}

}; // class HumapPlanner

typedef std::shared_ptr<HumapPlanner> HumapPlannerPtr;

}; // namespace humap_local_planner
