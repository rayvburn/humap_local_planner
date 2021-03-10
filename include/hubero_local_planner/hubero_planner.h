#pragma once

#include <vector>
#include <mutex>
#include <memory>

#include <hubero_local_planner/external/obstacles.h>

#include <hubero_local_planner/hubero_config.h>
#include <hubero_local_planner/sfm/social_force_model.h>
#include <hubero_local_planner/fuzz/processor.h>
#include <hubero_local_planner/fuzz/social_conductor.h>

//for creating a local cost grid
#include <base_local_planner/map_grid_visualizer.h>
#include <pcl_ros/publisher.h>

//for obstacle data access
#include <costmap_2d/costmap_2d.h>

#include <base_local_planner/trajectory.h>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/simple_trajectory_generator.h>

#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/twirling_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>

// useful, free functions
#include <base_local_planner/goal_functions.h>

// MapGridVisualizer
#include <base_local_planner/map_grid.h>

#include <nav_msgs/Path.h>

namespace hubero_local_planner {
/**
 * @class HuberoPlanner
 * @brief A class implementing a local planner using the HuberoPlanner
 */
class HuberoPlanner {
public:
	// both environmental and internal drivers
	struct MotionDriverData {
		Vector3 force_combined;
		Vector3 force_internal;
		Vector3 force_interaction;
		Vector3 force_social;
		std::vector<Pose3> closest_points;
		std::string behaviour_active;

		MotionDriverData() {}
		MotionDriverData(
			const Vector3& force_combined,
			const Vector3& force_internal,
			const Vector3& force_interaction,
			const Vector3& force_social,
			const std::vector<Pose3>& closest_points,
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
	HuberoPlanner(std::string name, std::shared_ptr<base_local_planner::LocalPlannerUtil> planner_util);

	/**
	 * @brief  Destructor for the planner
	 */
    virtual ~HuberoPlanner();

    void initialize(HuberoConfigConstPtr cfg);

    bool compute(
    		const tf::Stamped<tf::Pose>& pose,
			const geometry_msgs::Twist& velocity,
			const tf::Stamped<tf::Pose>& goal,
			ObstContainerConstPtr obstacles,
			Eigen::Vector3f& force
	);

    bool compute(
			const Pose3& pose,
			const Vector3& velocity,
			const Pose3& goal,
			ObstContainerConstPtr obstacles,
			Vector3& force
	);

    bool plan();

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

    Vector3 computeForce();

	/**
	 * @brief Given the current position and velocity of the robot, find the best trajectory to exectue
	 * @param global_pose The current position of the robot
	 * @param global_vel The current velocity of the robot
	 * @param drive_velocities The velocities to send to the robot base
	 * @return The highest scoring trajectory. A cost >= 0 means the trajectory is legal to execute.
	 */
	base_local_planner::Trajectory findBestPath(
		tf::Stamped<tf::Pose> global_pose,
		tf::Stamped<tf::Pose> global_vel,
		tf::Stamped<tf::Pose>& drive_velocities
	);

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
    void updatePlanAndLocalCosts(tf::Stamped<tf::Pose> global_pose,
		const std::vector<geometry_msgs::PoseStamped>& new_plan,
		const std::vector<geometry_msgs::Point>& footprint_spec
	);

	/**
	 * @brief Get the period at which the local planner is expected to run
	 * @return The simulation period
	 */
	inline double getSimPeriod() {
		return sim_period_;
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
	 * @brief Evaluates whether goal is considered as reached
	 * @return
	 */
	inline bool isGoalReached() const {
		printf("[HuberoPlanner::isGoalReached] \r\n");
		return goal_reached_;
	}


	inline MotionDriverData getMotionData() const {
		return motion_data_;
	}

private:

	void checkGoalReached();

	std::shared_ptr<base_local_planner::LocalPlannerUtil> planner_util_;

	double sim_period_; ///< @brief The number of seconds to use to compute max/min vels for the planner
	base_local_planner::Trajectory result_traj_;

	std::vector<geometry_msgs::PoseStamped> global_plan_;
	bool goal_reached_;

	HuberoConfigConstPtr cfg_;
	ObstContainerConstPtr obstacles_;

	sfm::SocialForceModel sfm_;
	fuzz::Processor fuzzy_processor_; //!< produces segmentation fault in destructor when ran in a separated executable (Segmentation fault (core dumped))
	fuzz::SocialConductor social_conductor_;

	// sampling? copies for visualization
	MotionDriverData motion_data_;

}; // class HuberoPlanner

typedef std::shared_ptr<HuberoPlanner> HuberoPlannerPtr;

}; // namespace hubero_local_planner
