#include <humap_local_planner/velocity_smoothness_cost_function.h>
#include <humap_local_planner/geometry/pose.h>
#include <humap_local_planner/utils/transformations.h>

#include <cmath>

namespace humap_local_planner {

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
	double vth_deviation = 0.0;

	// first step is computed in a specific way (velocity directly accessible)
	geometry::Vector robot_vel_prev(traj.xv_, traj.yv_, traj.thetav_);
	vx_deviation += std::abs(robot_vel_prev.getX() - velocity_base_.getX());
	vy_deviation += std::abs(robot_vel_prev.getY() - velocity_base_.getY());
	vth_deviation += std::abs(robot_vel_prev.getZ() - velocity_base_.getZ());

	if (traj.getPointsSize() == 1) {
		return vx_deviation + vy_deviation + vth_deviation;
	}

	// iterate over trajectory points (velocity must be computed)
	for (unsigned int sim_step = 1; sim_step < traj.getPointsSize(); sim_step++) {
		// retrieve 2d pose from trajectory
		double traj_x, traj_y, traj_th = 0.0;
		traj.getPoint(sim_step - 1, traj_x, traj_y, traj_th);
		geometry::Pose traj_pose_prev(traj_x, traj_y, traj_th);

		traj.getPoint(sim_step, traj_x, traj_y, traj_th);
		geometry::Pose traj_pose(traj_x, traj_y, traj_th);

		// eval robot displacement
		auto robot_displacement = subtractPoses(traj_pose, traj_pose_prev);

		// compute velocity from pose difference
		geometry::Vector robot_vel(
			robot_displacement.getX() / traj.time_delta_,
			robot_displacement.getY() / traj.time_delta_,
			robot_displacement.getYaw() / traj.time_delta_
		);

		// compute deviation from the current velocity
		vx_deviation += std::abs(robot_vel.getX() - robot_vel_prev.getX());
		vy_deviation += std::abs(robot_vel.getY() - robot_vel_prev.getY());
		vth_deviation += std::abs(robot_vel.getZ() - robot_vel_prev.getZ());

		traj_pose_prev = traj_pose;
		robot_vel_prev = robot_vel;
	}

	// divide by trajectory length to make cost function's scale independent of the actual trajectory horizon
	return (vx_deviation + vy_deviation + vth_deviation) / traj.getPointsSize();
}

} // namespace humap_local_planner
