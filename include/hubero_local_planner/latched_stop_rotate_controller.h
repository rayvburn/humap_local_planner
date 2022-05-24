/*
 * latched_stop_rotate_controller.h
 *
 *  Created on: Apr 16, 2012
 *      Author: tkruse
 */

#pragma once

#include <string>

#include <Eigen/Core>

#include <base_local_planner/local_planner_util.h>
#include <nav_msgs/Odometry.h>

#include <functional>
#include <string>

namespace hubero_local_planner {

/**
 * @brief Implementation of the latched stop & rotate controller that avoids OdometryHelperRos instances as arguments
 *
 * Reference: https://github.com/ros-planning/navigation/blob/melodic-devel/base_local_planner/include/base_local_planner/latched_stop_rotate_controller.h
 *
 * The reason behind why this class was copied is that it cannot be easily extended (private access to flags).
 * On the other hand, a fork would be a nice way but it will cause storing a whole navigation stack
 * in the workspace (with only 1 small class changed).
 */
class LatchedStopRotateController {
public:
	LatchedStopRotateController(const std::string& name = "");

	bool isPositionReached(
		base_local_planner::LocalPlannerUtil* planner_util,
		const geometry_msgs::PoseStamped& global_pose
	);

	bool isGoalReached(
		base_local_planner::LocalPlannerUtil* planner_util,
		const geometry_msgs::PoseStamped& robot_vel,
		const geometry_msgs::PoseStamped& global_pose
	);

	void resetLatching() {
		xy_tolerance_latch_ = false;
	}

	/**
	 * @brief Stop the robot taking into account acceleration limits
	 * @param  global_pose The pose of the robot in the global frame
	 * @param  robot_vel The velocity of the robot
	 * @param  cmd_vel The velocity commands to be filled
	 * @return  True if a valid trajectory was found, false otherwise
	 */
	bool stopWithAccLimits(
		const geometry_msgs::PoseStamped& global_pose,
		const geometry_msgs::PoseStamped& robot_vel,
		geometry_msgs::Twist& cmd_vel,
		Eigen::Vector3f acc_lim,
		double sim_period,
		std::function<bool(
			Eigen::Vector3f pos,
			Eigen::Vector3f vel,
			Eigen::Vector3f vel_samples)> obstacle_check
	);

	/**
	 * @brief Once a goal position is reached... rotate to the goal orientation
	 * @param  global_pose The pose of the robot in the global frame
	 * @param  robot_vel The velocity of the robot
	 * @param  goal_th The desired th value for the goal
	 * @param  cmd_vel The velocity commands to be filled
	 * @return  True if a valid trajectory was found, false otherwise
	 */
	bool rotateToGoal(
		const geometry_msgs::PoseStamped& global_pose,
		const geometry_msgs::PoseStamped& robot_vel,
		double goal_th,
		geometry_msgs::Twist& cmd_vel,
		Eigen::Vector3f acc_lim,
		double sim_period,
		base_local_planner::LocalPlannerLimits& limits,
		std::function<bool(
			Eigen::Vector3f pos,
			Eigen::Vector3f vel,
			Eigen::Vector3f vel_samples)> obstacle_check
	);

	bool computeVelocityCommandsStopRotate(
		geometry_msgs::Twist& cmd_vel,
		Eigen::Vector3f acc_lim,
		double sim_period,
		base_local_planner::LocalPlannerUtil* planner_util,
		const geometry_msgs::PoseStamped& robot_vel,
		const geometry_msgs::PoseStamped& global_pose,
		std::function<bool(
			Eigen::Vector3f pos,
			Eigen::Vector3f vel,
			Eigen::Vector3f vel_samples)> obstacle_check
	);

protected:
	// NOTE: loosened access specifier from private to protected
	inline double sign(double x){
		return x < 0.0 ? -1.0 : 1.0;
	}

	// @rayvburn fills up odometry message based on raw position and velocity
	nav_msgs::Odometry fillOdometry(
		const geometry_msgs::PoseStamped& robot_vel,
		const geometry_msgs::PoseStamped& global_pose
	) const;

	// whether to latch at all, and whether in this turn we have already been in goal area
	bool latch_xy_goal_tolerance_, xy_tolerance_latch_;
	bool rotating_to_goal_;

	// @rayvburn frame names added to avoid OdometryHelperRos
	std::string robot_base_frame_;
	std::string global_frame_;
};

} /* namespace */
