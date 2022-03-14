#pragma once

#include <hubero_local_planner/geometry/pose.h>
#include <cmath>

namespace hubero_local_planner {

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
	const double& sim_period,
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
 * @param vel_global output
 */
void computeVelocityGlobal(
	const geometry::Vector& vel_local,
	const geometry::Pose& pose,
	geometry::Vector& vel_global
);

/** @} */ // end of velocitytransformations

} // namespace hubero_local_planner
