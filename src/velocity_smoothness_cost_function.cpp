#include <hubero_local_planner/velocity_smoothness_cost_function.h>
#include <hubero_local_planner/geometry/pose.h>
#include <hubero_local_planner/utils/transformations.h>

#include <hubero_local_planner/trajectory.h>

#include <cmath>

namespace hubero_local_planner {

VelocitySmoothnessCostFunction::VelocitySmoothnessCostFunction(const geometry::Vector& velocity_base):
	velocity_base_(velocity_base) {}

bool VelocitySmoothnessCostFunction::prepare() {
	return true;
}

double VelocitySmoothnessCostFunction::scoreTrajectory(base_local_planner::Trajectory& traj) {
	if (traj.getPointsSize() == 0) {
		return 0.0;
	}

	// compute differences between the current velocity and all consecutive that belong to the trajectory
	double vx_deviation = 0.0;
	double vy_deviation = 0.0;

	// convert to a more friendly type than received
	// `false` keeps velocities in the base coordinate system
	Trajectory t(traj, false);

	// check if we can compare the current velocity with anything
	if (t.getVelocitiesNum() == 0) {
		return 0.0;
	}

	// first velocity of the trajectory is the control command, let's compare it with the current velocity
	vx_deviation += std::abs(t.getVelocity(0).getX() - velocity_base_.getX());
	vy_deviation += std::abs(t.getVelocity(0).getY() - velocity_base_.getY());

	// iterate over trajectory points (velocity must be computed)
	for (unsigned int sim_step = 1; sim_step < t.getVelocitiesNum(); sim_step++) {
		// compute deviation from the current velocity
		vx_deviation += std::abs(t.getVelocity(sim_step).getX() - t.getVelocity(sim_step - 1).getX());
		vy_deviation += std::abs(t.getVelocity(sim_step).getY() - t.getVelocity(sim_step - 1).getY());
	}

	// divide by trajectory length to make cost function's scale independent of the actual trajectory horizon
	// +1 as the current vel. is also checked against the first vel. from the trajectory
	return (vx_deviation + vy_deviation) / static_cast<double>(t.getVelocitiesNum() + 1);
}

} // namespace hubero_local_planner
