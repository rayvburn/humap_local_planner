#pragma once

#include <hubero_local_planner/geometry/pose.h>
#include <cmath>

namespace hubero_local_planner {

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
