#include <hubero_local_planner/speedy_goal_cost_function.h>
#include <hubero_local_planner/geometry/vector.h>

#include <cmath>

namespace hubero_local_planner {

SpeedyGoalCostFunction::SpeedyGoalCostFunction(const geometry::Pose& global_goal):
	global_goal_(global_goal) {}

void SpeedyGoalCostFunction::setParameters(const double& distance_slow_down, const double vel_trans_ideal) {
	distance_slow_down_ = distance_slow_down;
	vel_trans_ideal_ = vel_trans_ideal;
}

bool SpeedyGoalCostFunction::prepare() {
	return true;
}

double SpeedyGoalCostFunction::scoreTrajectory(base_local_planner::Trajectory& traj) {
	if (traj.getPointsSize() == 0) {
		return 0.0;
	}

	// create and operate on a copy, otherwise some deadlock occurs
	auto global_goal = global_goal_;

	// we score based on the first point and seed translational velocity
	double pt_x, pt_y, pt_th = 0.0;
	traj.getPoint(0, pt_x, pt_y, pt_th);

	// compute distance to goal pose
	geometry::Vector v_to_goal(pt_x - global_goal.getX(), pt_y - global_goal.getY(), 0.0);
	double dist_to_goal = v_to_goal.calculateLength();
	if (dist_to_goal > distance_slow_down_) {
		return 0.0;
	}

	double trans_vel_traj = std::sqrt(traj.xv_ * traj.xv_ + traj.yv_ * traj.yv_);
	if (trans_vel_traj <= vel_trans_ideal_) {
		return 0.0;
	}

	// difference from ideal is the cost
	return trans_vel_traj - vel_trans_ideal_;
}

} // namespace hubero_local_planner
