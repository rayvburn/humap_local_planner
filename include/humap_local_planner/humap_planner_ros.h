#pragma once

#include <memory>
#include <mutex>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <angles/angles.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/latched_stop_rotate_controller.h>

#include <nav_msgs/Odometry.h>
#include <base_local_planner/odometry_helper_ros.h>

#include <costmap_converter/costmap_to_polygons.h>
#include <costmap_converter/costmap_converter_interface.h>
#include <costmap_converter/ObstacleMsg.h>

#include <dynamic_reconfigure/server.h>
#include <humap_local_planner/HumapPlannerConfig.h> //!< Dynamic reconfigure
#include <humap_local_planner/humap_config_ros.h>

#include <humap_local_planner/humap_planner.h> //!< Planner
#include <humap_local_planner/visualization.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <people_msgs/People.h>
#include <humap_local_planner/person.h>
#include <humap_local_planner/group.h>

namespace humap_local_planner {
using namespace geometry;
/**
 * @class
 * @brief ROS Wrapper for the HumapPlanner that adheres to the
 * BaseLocalPlanner interface and can be used as a plugin for move_base.
 */
class HumapPlannerROS : public nav_core::BaseLocalPlanner {
public:
 	/**
     * @brief  Constructor for planner wrapper
     */
    HumapPlannerROS();

    /**
     * @brief  Destructor for the wrapper
     */
    ~HumapPlannerROS();

    /**
     * @brief  Constructs the ros wrapper
     * @param name The name to give this instance of the trajectory planner
     * @param tf_buffer A pointer to a tf2 Buffer
     * @param costmap The cost map to use for assigning costs to trajectories
     */
    virtual void initialize(std::string name, tf2_ros::Buffer* tf_buffer, costmap_2d::Costmap2DROS* costmap_ros) override;

    /**
     * @brief Set the plan that the controller is following
     * @param orig_global_plan The plan to pass to the controller
     * @return True if the plan was updated successfully, false otherwise
     */
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    /**
     * @brief  Check if the goal pose has been achieved
     * @return True if achieved, false otherwise
     */
    bool isGoalReached();

    /**
     * @brief  Given the current position, orientation, and velocity of the robot,
     * compute velocity commands to send to the base
     * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
     * @return True if a valid trajectory was found, false otherwise
     */
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    /**
     * @brief Evaluates whether planner's initialization procedure was already executed
     * @return Returns @ref initialized_ flag
     */
    bool isInitialized() {
      return initialized_;
    }

protected:
	/**
	* @brief Callback to update the local planner's parameters based on dynamic reconfigure
	*/
	void reconfigureCB(HumapPlannerConfig &config, uint32_t level);

	/**
	* @brief Callback to update the recognized humans set
	*/
	void peopleCB(const people_msgs::PeopleConstPtr& msg);

	/**
	 * @brief Propagates the parameters to relevant objects.
	 *
	 * The cfg mutex that keeps the parameters consistent throughout the execution, blocks callbacks. Therefore,
	 * updating parameters just before the planning is a safe approach is terms of concurrency and provides that
	 * callbacks are not blocked.
	 *
	 * @return true If parameters were updated
	 * @return false If parameters were not updated
	 */
	bool updateParameters();

	/**
	 * @brief Updates @ref obstacles_ with costmap converter's data
	 *
	 * @note This method is copied from TEB Local Planner
	 */
	bool updateObstacleContainerWithCostmapConverter();

	/**
	 * @brief Given the most recent people data, transforms the people poses into the global frame and aggregates them
	 * into groups
	 * @param config_updated flag indicating that the configuration parameters have been changed since the last call
	 */
	bool updatePeopleContainer(bool config_updated);

	/**
	 * @brief Get the current robot footprint/contour model
	 * @param nh const reference to the local ros::NodeHandle
	 * @return Tuple with
	 * - robot footprint model used for distance calculations
	 * - vector of points used for costmap planning
	 *
	 * @note This method is mostly copied from TEB Local Planner. Aim is to avoid reinterpret cast to the higher-level class
	 */
	std::tuple<RobotFootprintModelPtr, std::vector<geometry_msgs::Point>> getRobotFootprintFromParamServer(
		const ros::NodeHandle& nh
	);

	/**
	 * @brief Transforms Trajectory object into vector of PoseStamped to create a local plan
	 *
	 * @note Based on dwa_local_planner::DWAPlannerROS::dwaComputeVelocityCommands authored by Eitan Marder-Eppstein
	 */
	std::vector<geometry_msgs::PoseStamped> createLocalPlan(const base_local_planner::Trajectory& traj) const;

	/**
	 * @brief Create a PCL message with all explored trajectories (retrieved from planner)
	 *
	 * @note Based on dwa_local_planner::DWAPlanner::updatePlanAndLocalCosts authored by Eitan Marder-Eppstein
	 */
	sensor_msgs::PointCloud2 createExploredTrajectoriesPcl() const;

	/**
	 * @brief Create a MarkerArray message with trajectory predictions used by time to collision cost function
	 */
	visualization_msgs::MarkerArray createTTCTrajectoriesMarkers() const;

	/**
	 * @brief Create a PCL message with local map grid costs (used for scoring)
	 *
	 * @note Based on dwa_local_planner::MapGridVisualizer::publishCostCloud
	 */
	sensor_msgs::PointCloud2 createCostGridPcl() const;

	/**
	 * @brief Create a PCL message with intermediate goals for avoiding the group intrusions
	 */
	sensor_msgs::PointCloud2 createGroupIntrusionAltGoalCandidatesGridPcl() const;

	/// @brief Callback mutex
	std::mutex cb_mutex_;
	/// @brief Parameters reconfigure mutex
	std::mutex cfg_mutex_;

	/// @brief nav_core status
	bool initialized_;

	/// @section Publishers
	/// @brief Publishes predicted trajectories of people as PCL (for visualisation)
	ros::Publisher people_traj_pred_pcl_pub_;
	/// @brief Publishes predicted trajectories of people groups as PCL (for visualisation)
	ros::Publisher group_traj_pred_pcl_pub_;
	/// @brief Global plan publisher (for visualisation)
	ros::Publisher g_plan_pub_;
	/// @brief Pruned global plan publisher (for visualisation)
	ros::Publisher g_plan_pruned_pub_;
	/// @brief Local plan publisher (for visualisation)
	ros::Publisher l_plan_pub_;
	/// @brief Explored trajectories PCL publisher (for visualisation)
	ros::Publisher traj_pcl_pub_;
	/// @brief TTC trajectories publisher (for visualisation)
	ros::Publisher ttc_markers_pub_;
	/// @brief Publishes the potential field generated by the cost function, ref: base_local_planner::MapGridVisualizer
	ros::Publisher cost_grid_pcl_pub_;
	/// @brief Publishes intermediate goals for avoiding the group intrusions (only when a heuristic strategy is used)
	ros::Publisher group_intrusion_alt_pcl_pub_;
	/// @brief Publishes the state of the planner (in terms of its finite state machine)
	ros::Publisher planner_state_pub_;

	/// @section Global planning
	std::vector<geometry_msgs::PoseStamped> global_plan_;
	/// @section Local planning
	std::shared_ptr<base_local_planner::LocalPlannerUtil> planner_util_;
	HumapPlannerPtr planner_; ///< @brief The trajectory controller

	/// @section Local costmap (expressed in odometry frame)
	costmap_2d::Costmap2DROS* costmap_ros_;
	/// @subsection Costmap converter
	pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> costmap_converter_loader_; //!< Load costmap converter plugins at runtime
	boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_; //!< Store the current costmap_converter
	ObstContainerPtr obstacles_;

	/// @brief Subscriber of aggregated data with people present in the environment
	ros::Subscriber people_sub_;
	/// @brief Container with the most recent people data (unmodified)
	people_msgs::People people_orig_;
	/// @brief Flag indicating that the @ref people_orig_ was updated since the last call to @ref updatePeopleContainer
	bool people_container_updated_;
	/// @brief Stores data about the people in the environment; a container that undergone the processing
	std::shared_ptr<People> people_;
	/// @brief Stores data about the people groups in the environment; a container that undergone the processing
	std::shared_ptr<Groups> groups_;

	/// @section Dynamic reconfigure
	dynamic_reconfigure::Server<HumapPlannerConfig> *dsrv_;
	HumapPlannerConfig cfg_default_;
	HumapPlannerConfig cfg_params_;
	/// Flag indicating that the @ref cfg_params_ has been updated since the last call to @ref updateParameters
	bool cfg_updated_;
	std::shared_ptr<HumapConfigROS> cfg_;

	/// @section Odometry
	tf2_ros::Buffer* tf_buffer_;
	base_local_planner::OdometryHelperRos odom_helper_;
	std::string odom_topic_;
	// helpers
	tf2_ros::TransformBroadcaster tf_broadcaster_;

	Visualization vis_;

	/**
	 * @brief Creates a PCL with trajectory prediction points
	 *
	 * Template methods must be implemented in header files
	 *
	 * @tparam T @ref People or @ref Groups containers
	 * @param container
	 * @return sensor_msgs::PointCloud2
	 */
	template <typename T>
	sensor_msgs::PointCloud2 createTrajectoryPredictionPcl(const std::vector<T>& container) const {
		sensor_msgs::PointCloud2 traj_cloud;
		traj_cloud.header.frame_id = planner_util_->getGlobalFrame();
		traj_cloud.header.stamp = ros::Time::now();

		if (container.empty()) {
			return traj_cloud;
		}

		sensor_msgs::PointCloud2Modifier cloud_mod(traj_cloud);
		cloud_mod.setPointCloud2Fields(
			6,
			"x", 1, sensor_msgs::PointField::FLOAT32,
			"y", 1, sensor_msgs::PointField::FLOAT32,
			"z", 1, sensor_msgs::PointField::FLOAT32,
			"theta", 1, sensor_msgs::PointField::FLOAT32,
			"time", 1, sensor_msgs::PointField::FLOAT32,
			"id", 1, sensor_msgs::PointField::FLOAT32
		);

		// check how many points there will be in a point cloud
		unsigned int num_points = 0;
		for (
			auto t = container.cbegin();
			t != container.cend();
			++t
		) {
			num_points += t->getTrajectoryPrediction().getPoses().size();
		}

		cloud_mod.resize(num_points);

		// IDs for groups whose names cannot be expressed as numbers
		unsigned long int id_fallback = 0;

		sensor_msgs::PointCloud2Iterator<float> iter_x(traj_cloud, "x");
		for (
			auto t = container.cbegin();
			t != container.cend();
			++t
		) {
			double id = 0.0;
			try {
				id = std::stod(t->getName());
				// to make IDs unique
				id_fallback = std::max(std::ceil(id), static_cast<double>(id_fallback));
			} catch (const std::invalid_argument&) {
				// name could not be converted to a double
				id = static_cast<double>(++id_fallback);
			}

			// Fill out the trajectories
			for (size_t i = 0; i < t->getTrajectoryPrediction().getPoses().size(); i++) {
				auto pose = t->getTrajectoryPrediction().getPose(i);
				auto time = t->getTrajectoryPrediction().getTime(i);

				iter_x[0] = pose.getX();
				iter_x[1] = pose.getY();
				iter_x[2] = pose.getZ();
				iter_x[3] = pose.getYaw();
				iter_x[4] = time;
				iter_x[5] = id;
				++iter_x;
			}
		}
		return traj_cloud;
	}

}; // class HumapPlannerROS
}; // namespace humap_local_planner
