#include <humap_local_planner/fformation_space_intrusion_cost_function.h>
#include <humap_local_planner/trajectory.h>

#include <social_nav_utils/formation_space_intrusion.h>

namespace humap_local_planner {

FformationSpaceIntrusionCostFunction::FformationSpaceIntrusionCostFunction()
{}

void FformationSpaceIntrusionCostFunction::setFformationsDetections(const std::vector<Group>& groups) {
	groups_ = groups;
}

bool FformationSpaceIntrusionCostFunction::prepare() {
	return true;
}

double FformationSpaceIntrusionCostFunction::scoreTrajectory(base_local_planner::Trajectory& traj) {
	if (groups_.empty()) {
		return 0.0;
	}

	double dt = traj.time_delta_;
	unsigned int num = traj.getPointsSize();

	// storage for intrusions related to subsequent groups
	std::vector<double> groups_intrusions;

	// compare group poses against each pose of robot trajectory
	Trajectory robot_traj(traj);
	for (const auto& group: groups_) {
		// storage for robot intrusions against groups throughout the trajectory
		std::vector<double> group_intrusions;

		// check all robot trajectory points ...
		// NOTE: velocities are not considered below, therefore getSteps() can be considered instead
		// of getVelocitiesNum()
		for (unsigned int i = 0; i < robot_traj.getSteps(); i++) {
			// retrieve poses
			auto p_robot = robot_traj.getPose(i);
			auto p_group = group.getTrajectoryPrediction().getPose(i);

			// NOTE: span could change over the prediction horizon
			// extra computations for the group
			// take half of the span and apply 2 sigma rule
			// (mean is the center of the O-space, 2 times stddev corresponds to its span)
			double variance_ospace_x = std::pow((group.getSpanX() / 2.0) / 2.0, 2);
			double variance_ospace_y = std::pow((group.getSpanY() / 2.0) / 2.0, 2);

			// delegate cost computation
			social_nav_utils::FormationSpaceIntrusion fsi_penalty(
				p_group.getX(),
				p_group.getY(),
				p_group.getYaw(),
				variance_ospace_x,
				variance_ospace_y,
				group.getCovariancePoseXX(),
				group.getCovariancePoseXY(),
				group.getCovariancePoseYY(),
				p_robot.getX(),
				p_robot.getY()
			);
			fsi_penalty.normalize();
			// add to the local (current person) dataset
			group_intrusions.push_back(fsi_penalty.getScale());
		}
		// NOTE: consider checking whether a penalty value increases over the prediction horizon (but uncertainty?)

		// choose maximum disturbance throughout the trajectory as a score for a given person
		groups_intrusions.push_back(*std::max_element(group_intrusions.cbegin(), group_intrusions.cend()));
	}

	// return max overall
	return *std::max_element(groups_intrusions.cbegin(), groups_intrusions.cend());
}

} // namespace humap_local_planner
