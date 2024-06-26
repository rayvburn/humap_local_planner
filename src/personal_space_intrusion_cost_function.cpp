#include <humap_local_planner/personal_space_intrusion_cost_function.h>
#include <humap_local_planner/trajectory.h>

#include <social_nav_utils/personal_space_intrusion.h>

namespace humap_local_planner {

PersonalSpaceIntrusionCostFunction::PersonalSpaceIntrusionCostFunction(const std::vector<Person>& people):
	people_(people),
	compute_whole_horizon_(true)
{}

void PersonalSpaceIntrusionCostFunction::setParameters(bool compute_whole_horizon) {
	compute_whole_horizon_ = compute_whole_horizon;
}

bool PersonalSpaceIntrusionCostFunction::prepare() {
	return true;
}

double PersonalSpaceIntrusionCostFunction::scoreTrajectory(base_local_planner::Trajectory& traj) {
	if (people_.empty()) {
		return 0.0;
	}

	// storage for intrusions related to subsequent people
	std::vector<double> people_intrusions;

	// iterate over predicted poses of a person, compare against each pose of robot trajectory
	// against all people pose predictions ...
	Trajectory robot_traj(traj);

	// select the ending iteration number when not the whole horizon is meant to be evaluated;
	// prepare for the case when `robot_traj.getVelocitiesNum()` is 0
	unsigned int i_end_whole = static_cast<unsigned int>(robot_traj.getVelocitiesNum());
	unsigned int i_end_first = std::min(i_end_whole, static_cast<unsigned int>(1));

	for (const auto& person: people_) {
		// storage for intrusions against person throughout the trajectory
		std::vector<double> person_intrusions;

		// check all robot trajectory poses that have matching velocities
		for (
			unsigned int i = 0;
			i < (compute_whole_horizon_ ? i_end_whole : i_end_first);
			i++
		) {
			// retrieve poses and velocities
			auto p_robot = robot_traj.getPose(i);
			auto v_robot = robot_traj.getVelocity(i);
			auto p_person = person.getTrajectoryPrediction().getPose(i);
			auto v_person = person.getTrajectoryPrediction().getVelocity(i);

			// personal space model depends on the speed, according to Kirby's thesis (4.5 - 4.7)
			double vel_lin = std::hypot(v_person.getX(), v_person.getY());
			double ps_var_front = std::max(2.0 * vel_lin, 0.5);
			double ps_var_side = (2.0 / 3.0) * ps_var_front;
			double ps_var_rear = (1.0 / 2.0) * ps_var_front;

			// delegate cost computation
			social_nav_utils::PersonalSpaceIntrusion psi_penalty(
				p_person.getX(),
				p_person.getY(),
				p_person.getYaw(),
				person.getCovariancePoseXX(),
				person.getCovariancePoseXY(),
				person.getCovariancePoseYX(),
				person.getCovariancePoseYY(),
				ps_var_front,
				ps_var_rear,
				ps_var_side,
				p_robot.getX(),
				p_robot.getY(),
				true
			);
			psi_penalty.normalize();
			// add to the local (current person) dataset
			person_intrusions.push_back(psi_penalty.getScale());

		}
		// NOTE: consider checking whether a penalty value increases over the prediction horizon (but uncertainty?)

		// choose maximum disturbance throughout the trajectory as a score for a given person
		people_intrusions.push_back(*std::max_element(person_intrusions.cbegin(), person_intrusions.cend()));
	}

	// return max overall
	return *std::max_element(people_intrusions.cbegin(), people_intrusions.cend());
}

} // namespace humap_local_planner
