#include <humap_local_planner/heading_change_smoothness_cost_function.h>
#include <humap_local_planner/trajectory.h>

#include <cmath>

namespace humap_local_planner {

HeadingChangeSmoothnessCostFunction::HeadingChangeSmoothnessCostFunction(const geometry::Vector& velocity_base):
	velocity_base_(velocity_base) {}

bool HeadingChangeSmoothnessCostFunction::prepare() {
	return true;
}

double HeadingChangeSmoothnessCostFunction::scoreTrajectory(base_local_planner::Trajectory& traj) {
	if (traj.getPointsSize() == 0) {
		return 0.0;
	}

	// heading change smoothness sum storage
	double hcs = 0.0;

	// convert trajectory to global poses and local velocities
	// `false` keeps velocities in the base coordinate system
	Trajectory t(traj, false);

	// check if we can compare the current velocity with anything
	if (t.getVelocitiesNum() == 0) {
		return 0.0;
	}

	// first velocity of the trajectory is the control command, let's compare it with the current velocity
	hcs += std::abs(t.getVelocity(0).getZ() - velocity_base_.getZ());

	for (size_t i = 1; i < t.getVelocitiesNum(); i++) {
		double domega = t.getVelocity(i).getZ() - t.getVelocity(i - 1).getZ();
		hcs += (std::abs(domega) / t.getTimeDelta());
	}

	// divide by trajectory length to make cost function's scale independent of the actual trajectory horizon
	// +1 as the current vel. is also checked against the first vel. from the trajectory
	return hcs / static_cast<double>((t.getVelocitiesNum() + 1));
}

} // namespace humap_local_planner
