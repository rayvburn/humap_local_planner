#include <humap_local_planner/passing_speed_cost_function.h>
#include <humap_local_planner/trajectory.h>

#include <social_nav_utils/passing_speed_comfort.h>

namespace humap_local_planner {

PassingSpeedCostFunction::PassingSpeedCostFunction(const std::vector<Person>& people):
	people_(people),
	compute_whole_horizon_(true)
{}

void PassingSpeedCostFunction::setParameters(bool compute_whole_horizon) {
	compute_whole_horizon_ = compute_whole_horizon;
}

bool PassingSpeedCostFunction::prepare() {
	return true;
}

double PassingSpeedCostFunction::scoreTrajectory(base_local_planner::Trajectory& traj) {
	if (people_.empty()) {
		return 0.0;
	}

	// storage for comfort values related to subsequent people
	std::vector<double> people_comforts;

	// iterate over predicted poses of a person, compare against each pose of robot trajectory
	// against all people pose predictions ...
	Trajectory robot_traj(traj);

	// select the ending iteration number when not the whole horizon is meant to be evaluated;
	// prepare for the case when `robot_traj.getVelocitiesNum()` is 0
	unsigned int i_end_whole = static_cast<unsigned int>(robot_traj.getVelocitiesNum());
	unsigned int i_end_first = std::min(i_end_whole, static_cast<unsigned int>(1));

	for (const auto& person: people_) {
		// storage for intrusions against person throughout the trajectory
		std::vector<double> person_comforts;

		// check all robot trajectory points ...
		for (
			unsigned int i = 0;
			i < (compute_whole_horizon_ ? i_end_whole : i_end_first);
			i++
		) {
			// retrieve poses
			auto p_robot = robot_traj.getPose(i);
			auto v_robot = robot_traj.getVelocity(i);
			auto p_person = person.getTrajectoryPrediction().getPose(i);

			double distance = std::sqrt(
				std::pow(p_robot.getX() - p_person.getX(), 2)
				+ std::pow(p_robot.getY() - p_person.getY(), 2)
			);
			double speed_robot = std::hypot(v_robot.getX(), v_robot.getY());

			social_nav_utils::PassingSpeedComfort speed_comfort(distance, speed_robot);

			// add to the local (current person) dataset
			person_comforts.push_back(speed_comfort.getComfort());
		}

		// choose minimum comfort (checking against comfort here, not discomfort)
		// throughout the trajectory as a score for a given person
		people_comforts.push_back(*std::min_element(person_comforts.cbegin(), person_comforts.cend()));
	}

	// return min overall
	return *std::min_element(people_comforts.cbegin(), people_comforts.cend());
}

} // namespace humap_local_planner
