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
 * Converts 2D forces into robot forces w/ holonomic contraints.
 */
void computeTwistNonholonomic(
	const geometry::Pose& pose,
	const geometry::Vector& force,
	const geometry::Vector& robot_vel_glob,
	const double& sim_period,
	const double& robot_mass,
	const double& min_vel_x,
	const double& max_vel_x,
	const double& max_rot_vel,
	geometry::Vector& cmd_vel
);

/**
 * @brief computeTwist's helper that actually performs all computations, explicitly taking necessary parameters
 *
 * @details Converts 2D forces into robot forces with non-holonomic contraints.
 * Main reason to separate computeTwist from this helper method is unit testing
 *
 * @param force x-y plane force, z is assumed to be the rotational component (theta)
 * @param robot_vel_glob velocity of the robot in global coordinate system, z represents rotational component
 */
void computeTwist(
	const geometry::Pose& pose,
	const geometry::Vector& force,
	const geometry::Vector& robot_vel_glob,
	const double& sim_period,
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

/** @} */ // end of velocitytransformations

} // namespace hubero_local_planner
