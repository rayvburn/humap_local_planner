#pragma once

// shared_ptr
#include <memory>

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
#include <hubero_local_planner/HuberoPlannerConfig.h> //!< Dynamic reconfigure
#include <hubero_local_planner/hubero_config_ros.h>

#include <hubero_local_planner/hubero_planner.h> //!< Planner
#include <hubero_local_planner/visualization.h>

namespace hubero_local_planner {
using namespace geometry;
/**
 * @class 
 * @brief ROS Wrapper for the HuberoPlanner that adheres to the
 * BaseLocalPlanner interface and can be used as a plugin for move_base.
 */
class HuberoPlannerROS : public nav_core::BaseLocalPlanner {
public:
 	/**
     * @brief  Constructor for planner wrapper
     */
    HuberoPlannerROS();

    /**
     * @brief  Destructor for the wrapper
     */
    ~HuberoPlannerROS();

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
	void reconfigureCB(HuberoPlannerConfig &config, uint32_t level);

	/**
	 * @brief Updates @ref obstacles_ with costmap converter's data
	 *
	 * @note This method is copied from TEB Local Planner
	 */
	bool updateObstacleContainerWithCostmapConverter();

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

	void computeTwist(
		const Pose& pose,
		const Vector& force,
		const Vector& robot_vel_glob,
		Vector& cmd_vel
	) const;

	/**
	 * @brief Transforms Trajectory object into vector of PoseStamped to create a local plan
	 *
	 * @note Based on dwa_local_planner::DWAPlannerROS::dwaComputeVelocityCommands authored by Eitan Marder-Eppstein
	 */
	std::vector<geometry_msgs::PoseStamped> createLocalPlan(const base_local_planner::Trajectory& traj) const;

	/// @brief nav_core status
	bool initialized_;

	/// @section Publishers
	/// @brief Global plan publisher (for visualisation)
	ros::Publisher g_plan_pub_;
	/// @brief Local plan publisher (for visualisation)
	ros::Publisher l_plan_pub_;
	/// @section Global planning
	std::vector<geometry_msgs::PoseStamped> global_plan_;
	/// @section Local planning
	std::shared_ptr<base_local_planner::LocalPlannerUtil> planner_util_;
	HuberoPlannerPtr planner_; ///< @brief The trajectory controller

	/// @section Local costmap (expressed in odometry frame)
	costmap_2d::Costmap2DROS* costmap_ros_;
	/// @subsection Costmap converter
	pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> costmap_converter_loader_; //!< Load costmap converter plugins at runtime
	boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_; //!< Store the current costmap_converter
	ObstContainerPtr obstacles_;

	/// @section Dynamic reconfigure
	dynamic_reconfigure::Server<HuberoPlannerConfig> *dsrv_;
	hubero_local_planner::HuberoPlannerConfig default_config_;
	std::shared_ptr<HuberoConfigROS> cfg_;

	/// @section Odometry
	tf2_ros::Buffer* tf_buffer_;
	base_local_planner::OdometryHelperRos odom_helper_;
	std::string odom_topic_;
	geometry_msgs::PoseStamped current_pose_;
	// helpers
	tf2_ros::TransformBroadcaster tf_broadcaster_;

	Visualization vis_;

}; // class HuberoPlannerROS
}; // namespace hubero_local_planner

