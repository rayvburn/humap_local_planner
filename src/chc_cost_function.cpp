#include <hubero_local_planner/chc_cost_function.h>

#include <hubero_local_planner/geometry/vector.h>
#include <hubero_local_planner/geometry/pose.h>
#include <hubero_local_planner/utils/transformations.h>

#include <cmath>

namespace hubero_local_planner {

bool CHCCostFunction::prepare() {
	return true;
}

double CHCCostFunction::scoreTrajectory(base_local_planner::Trajectory& traj) {
	if (traj.getPointsSize() == 0) {
		// no changes in heading as there is only 1 sample
		return 0.0;
	}

	// compute pre-initial pose (based on seed velocity)
	double traj_x0, traj_y0, traj_th0 = 0.0;
	traj.getPoint(0, traj_x0, traj_y0, traj_th0);
	geometry::Pose traj_pose_prev(
		traj_x0 - traj.xv_ * traj.time_delta_,
		traj_y0 - traj.yv_ * traj.time_delta_,
		traj_th0 - traj.thetav_ * traj.time_delta_
	);
	geometry::Vector robot_vel_prev(traj.xv_, traj.yv_, traj.thetav_);

	// CHC sum storage
	double chc = 0.0;

	// iterate over trajectory
	for (unsigned int sim_step = 0; sim_step < traj.getPointsSize(); sim_step++) {
		// retrieve 2d pose from trajectory
		double traj_x, traj_y, traj_th = 0.0;
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

		// compute CHC
		chc += std::abs(robot_vel.getZ() - robot_vel_prev.getZ());

		traj_pose_prev = traj_pose;
		robot_vel_prev = robot_vel;
	}

	return chc / traj.getPointsSize();
}

} // namespace hubero_local_planner
