#include <hubero_local_planner/personal_space_intrusion_cost_function.h>
#include <hubero_local_planner/trajectory.h>

#include <social_nav_utils/personal_space_intrusion.h>

namespace hubero_local_planner {

PersonalSpaceIntrusionCostFunction::PersonalSpaceIntrusionCostFunction()
{}

void PersonalSpaceIntrusionCostFunction::setPeopleDetections(const std::vector<people_msgs_utils::Person>& people) {
	people_ = people;
}

bool PersonalSpaceIntrusionCostFunction::prepare() {
	return true;
}

double PersonalSpaceIntrusionCostFunction::scoreTrajectory(base_local_planner::Trajectory& traj) {
	if (people_.empty()) {
		return 0.0;
	}

	// first, create a set of predicted people poses; pose predictions correspond to timestamps of robot trajectory
	double dt = traj.time_delta_;
	unsigned int num = traj.getPointsSize();

	// helper to match current state of person with the predicted trajectory
	struct PersonWithTrajPrediction {
		people_msgs_utils::Person person;
		Trajectory traj;
		// Constructor
		PersonWithTrajPrediction(const people_msgs_utils::Person& person, const Trajectory& traj):
			person(person), traj(traj)
		{}
	};

	// prediction of people poses over the trajectory time is the first stage
	std::vector<PersonWithTrajPrediction> people_w_traj;
	for (const auto& person: people_) {
		// predict object's trajectory
		people_w_traj.push_back(PersonWithTrajPrediction(person, Trajectory(person, dt, num)));
	}

	// storage for intrusions related to subsequent people
	std::vector<double> people_intrusions;

	// iterate over predicted poses of a person, compare against each pose of robot trajectory
	// against all people pose predictions ...
	Trajectory robot_traj(traj);
	for (const auto& person: people_w_traj) {
		// storage for intrusions against person throughout the trajectory
		std::vector<double> person_intrusions;

		// check all robot trajectory points ...
		for (unsigned int i = 0; i < robot_traj.getSteps(); i++) {
			// retrieve poses
			auto p_robot = robot_traj.getPose(i);
			auto v_robot = robot_traj.getVelocity(i);
			auto p_person = person.traj.getPose(i);
			auto v_person = person.traj.getVelocity(i);

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
				person.person.getCovariancePoseXX(),
				person.person.getCovariancePoseXY(),
				person.person.getCovariancePoseYX(),
				person.person.getCovariancePoseYY(),
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

} // namespace hubero_local_planner
