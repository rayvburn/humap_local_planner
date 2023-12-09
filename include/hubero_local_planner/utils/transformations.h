#pragma once

#include <hubero_local_planner/geometry/pose.h>
#include <cmath>

namespace hubero_local_planner {

/**
 * @brief Computes new pose based on previous one and velocity setpoint (expressed in local=base coordinate system)
 * @note Constant velocity model applied here
 */
geometry::Pose computeNextPoseBaseVel(
	const geometry::Pose& pose,
	const geometry::Vector& vel,
	double dt
);

/**
 * @brief Computes a new pose based on previous one and velocity setpoint expressed in global coordinate system
 * @note Constant velocity model applied here
 */
geometry::Pose computeNextPose(
	const geometry::Pose& pose,
	const geometry::Vector& vel,
	double dt
);

/**
 * @defgroup velocitytransformations Velocity Transformations
 * @{
 */
/**
 * Converts 2D forces into robot forces w/ non-holonomic contraints
 */
void computeTwistHolonomic(
	const geometry::Pose& pose,
	const geometry::Vector& force,
	const geometry::Vector& robot_vel_glob,
	const double& dt,
	const double& robot_mass,
	const double& min_vel_x,
	const double& max_vel_x,
	const double& max_rot_vel,
	geometry::Vector& cmd_vel
);

/**
 * @brief Converts 2D forces into velocity of robot with non-holonomic contraints.
 *
 * @param force x-y plane force, z is assumed to be the rotational component (theta)
 * @param robot_vel_glob velocity of the robot in global coordinate system, z represents rotational component
 */
void computeTwist(
	const geometry::Pose& pose,
	const geometry::Vector& force,
	const geometry::Vector& robot_vel_glob,
	const double& dt,
	const double& robot_mass,
	const double& min_vel_x,
	const double& max_vel_x,
	const double& max_rot_vel,
	const double& twist_rotation_compensation,
	geometry::Vector& cmd_vel
);

/**
 * @brief Performs conversion from velocity expressed in local c.s. to global c.s. velocity vector
 *
 * @param vel_local input
 * @param pose input
 * @param vel_global output, x, y and theta velocities
 */
void computeVelocityGlobal(
	const geometry::Vector& vel_local,
	const geometry::Pose& pose,
	geometry::Vector& vel_global
);

/**
 * @brief Performs conversion from velocity expressed in global c.s. to local c.s. velocity vector
 *
 * This version is appropriate for robots with holonomic constraints
 *
 * @param vel_global input
 * @param pose input
 * @param vel_local output, x, y and theta velocities
 * @param holonomic set to true when nonzero v_y velocity is properly handled by the drive of the mobile base
 */
void computeVelocityLocal(
	const geometry::Vector& vel_global,
	const geometry::Pose& pose,
	geometry::Vector& vel_local,
	bool holonomic = false
);

/**
 * @brief Computes global velocity from subsequent poses, namely @ref pose1 and @ref pose2
 *
 * @param pose1 first pose
 * @param pose2 second pose
 * @param dt time delta between poses
 * @return geometry::Vector global velocity vector
 */
geometry::Vector computeVelocityFromPoses(const geometry::Pose& pose1, const geometry::Pose& pose2, double dt);

/**
 * @brief Computes local velocity from subsequent poses, namely @ref pose1 and @ref pose2
 *
 * @param pose1 first pose
 * @param pose2 second pose
 * @param dt time delta between poses
 * @return geometry::Vector local velocity vector
 */
geometry::Vector computeBaseVelocityFromPoses(const geometry::Pose& pose1, const geometry::Pose& pose2, double dt);

/**
 * @brief Adjusts velocity command to the acceleration limits
 *
 * Based on the current velocity @ref vel, acceleration limits and @ref sim_period, recomputes @ref cmd_vel.
 * Continuous acceleration assumption is used here.
 * @param maintain_vel_components_rate when set to true and any velocity component is not within the limits,
 * others will be modified according to the one that cannot be changed as requested; others will be proportionally
 * scaled; setting this to true loosely corresponds to agreeing with dynamics violation but keeping the path
 * as intended
 *
 * @return True if @ref cmd_vel was modified to comply with acceleration limits
 */
bool adjustTwistWithAccLimits(
	const geometry::Vector& vel,
	const double& acc_lim_x,
	const double& acc_lim_y,
	const double& acc_lim_th,
	const double& vel_min_x,
	const double& vel_min_y,
	const double& vel_min_th,
	const double& vel_max_x,
	const double& vel_max_y,
	const double& vel_max_th,
	const double& sim_period,
	geometry::Vector& cmd_vel,
	bool maintain_vel_components_rate = true
);

/**
 * Same as @ref adjustTwistWithAccLimits but also avoids overshooting the goal pose (limits velocity near the goal)
 *
 * @param dist_to_goal passing a valid distance to the goal will assure that the accelerations will be set
 * as not to overshoot the goal position
 */
bool adjustTwistWithAccAndGoalLimits(
	const geometry::Vector& vel,
	const double& acc_lim_x,
	const double& acc_lim_y,
	const double& acc_lim_th,
	const double& vel_min_x,
	const double& vel_min_y,
	const double& vel_min_th,
	const double& vel_max_x,
	const double& vel_max_y,
	const double& vel_max_th,
	const double& sim_period,
	geometry::Vector& cmd_vel,
	bool maintain_vel_components_rate,
	double dist_to_goal
);

/**
 * @brief Computes pure difference between all pose components
 *
 * `pose_ref` element is used as a reference (e.g. as new pose, whereas `pose_other` is the previous one).
 * Orientation is computed by subtraction of Euler angles.
 */
geometry::Pose subtractPoses(const geometry::Pose& pose_ref, const geometry::Pose& pose_other);

/**
 * @brief Computes pure addition between all pose components
 */
geometry::Pose addPoses(const geometry::Pose& pose_ref, const geometry::Pose& pose_other);

/**
 * @brief Saturate the translational and angular velocity to given limits
 *
 * Calculation method has been taken from TebLocalPlannerROS::saturateVelocity
 * https://github.com/rst-tu-dortmund/teb_local_planner/blob/3b91dcc3b9e6733363e285e096ff7c7fea42883f/include/teb_local_planner/teb_local_planner_ros.h#L359
 *
 * The limit of the translational velocity for backwards driving can be changed independently.
 * @param cmd_vel The velocity command that should be saturated.
 * @param max_vel_x Maximum translational velocity for forward driving
 * @param max_vel_y Maximum strafing velocity (for holonomic robots)
 * @param max_vel_trans Maximum translational velocity for holonomic robots
 * @param max_vel_theta Maximum (absolute) angular velocity
 * @param max_vel_x_backwards Maximum translational velocity for backwards driving
 * @return saturated velocity command
 */
geometry::Vector saturateVelocity(
	const geometry::Vector& cmd_vel,
	double max_vel_x,
	double max_vel_y,
	double max_vel_trans,
	double max_vel_theta,
	double max_vel_x_backwards
);

/**
 * @brief Trims the translational and angular velocity to given limits keeping the proportions between them
 *
 * Might reproduce the path arising from the commanded velocities slightly better than brute-force trimming
 */
geometry::Vector adjustTwistProportional(
	const geometry::Vector& vel,
	const geometry::Vector& cmd_vel,
	double min_vel_x,
	double min_vel_y,
	double min_vel_th,
	double max_vel_x,
	double max_vel_y,
	double max_vel_th
);

/** @} */ // end of velocitytransformations

} // namespace hubero_local_planner
