#include <hubero_local_planner/slow_translation_cost_function.h>
#include <hubero_local_planner/trajectory.h>

#include <cmath>

namespace hubero_local_planner {

void SlowTranslationCostFunction::setParameters(double max_trans_vel, double max_vel_x, double max_vel_y) {
	max_trans_vel_ = max_trans_vel;
	max_vel_x_ = max_vel_x;
	max_vel_y_ = max_vel_y;
}

bool SlowTranslationCostFunction::prepare() {
	return true;
}

double SlowTranslationCostFunction::scoreTrajectory(base_local_planner::Trajectory& traj) {
	if (traj.getPointsSize() == 0) {
		return 0.0;
	}

	// compute differences between the current velocity and maximum
	double vx_deviation = 0.0;
	double vy_deviation = 0.0;
	double vxy_deviation = 0.0;

	// convert to a more friendly type than received
	// `false` keeps velocities in the base coordinate system
	Trajectory t(traj, false);

	// check if we can compare the current velocity with anything
	if (t.getVelocitiesNum() == 0) {
		return 0.0;
	}

	// helper
	auto computeTransVel = [](double vx, double vy) -> double {
		return std::hypot(vx, vy);
	};

	// first velocity of the trajectory is the control command, let's compare it with the current velocity
	vx_deviation += std::abs(t.getVelocity(0).getX() - max_vel_x_);
	vy_deviation += std::abs(t.getVelocity(0).getY() - max_vel_y_);
	double vxy = computeTransVel(t.getVelocity(0).getX(), t.getVelocity(0).getY());
	vxy_deviation += std::abs(vxy - max_trans_vel_);

	// iterate over trajectory points (velocity must be computed)
	for (unsigned int sim_step = 0; sim_step < t.getVelocitiesNum(); sim_step++) {
		// compute deviation from the current velocity
		vx_deviation += std::abs(t.getVelocity(sim_step).getX() - max_vel_x_);
		vy_deviation += std::abs(t.getVelocity(sim_step).getY() - max_vel_y_);

		double vxy = computeTransVel(t.getVelocity(0).getX(), t.getVelocity(0).getY());
		vxy_deviation += std::abs(vxy - max_trans_vel_);
	}

	// divide by trajectory length to make cost function's scale independent of the actual trajectory horizon
	double max_deviation = std::max(std::max(vx_deviation, vy_deviation), vxy_deviation);
	return (max_deviation) / static_cast<double>(t.getVelocitiesNum());
}

} // namespace hubero_local_planner
