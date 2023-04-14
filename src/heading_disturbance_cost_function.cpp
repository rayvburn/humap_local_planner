#include <hubero_local_planner/heading_disturbance_cost_function.h>
#include <hubero_local_planner/trajectory.h>

#include <social_nav_utils/heading_direction_disturbance.h>

namespace hubero_local_planner {

HeadingDisturbanceCostFunction::HeadingDisturbanceCostFunction():
	fov_person_(3.31613),
	person_model_radius_(0.28),
	robot_circumradius_(0.275),
	max_speed_(0.55)
{}

void HeadingDisturbanceCostFunction::setPeopleDetections(const std::vector<people_msgs_utils::Person>& people) {
	people_ = people;
}

void HeadingDisturbanceCostFunction::setParameters(
	double fov_person,
	double person_model_radius,
	double robot_circumradius,
	double max_speed
) {
	fov_person_ = fov_person;
	person_model_radius_ = person_model_radius;
	robot_circumradius_ = robot_circumradius;
	max_speed_ = max_speed;
}

bool HeadingDisturbanceCostFunction::prepare() {
	return true;
}

double HeadingDisturbanceCostFunction::scoreTrajectory(base_local_planner::Trajectory& traj) {
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

	// storage for disturbances related to subsequent people
	std::vector<double> people_disturbances;

	// iterate over predicted poses of a person, compare against each pose of robot trajectory
	// against all people pose predictions ...
	Trajectory robot_traj(traj);
	for (const auto& person: people_w_traj) {
		// storage for disturbances against person throughout the trajectory
		std::vector<double> person_disturbances;

		// check all robot trajectory points ...
		for (unsigned int i = 0; i < robot_traj.getSteps(); i++) {
			// retrieve poses
			auto p_robot = robot_traj.getPose(i);
			auto v_robot = robot_traj.getVelocity(i);
			auto p_person = person.traj.getPose(i);
			auto v_person = person.traj.getVelocity(i);

			// delegate cost computation
			social_nav_utils::HeadingDirectionDisturbance heading_direction_penalty(
				p_person.getX(),
				p_person.getY(),
				p_person.getYaw(),
				person.person.getCovariancePoseXX(),
				person.person.getCovariancePoseXY(),
				person.person.getCovariancePoseYY(),
				p_robot.getX(),
				p_robot.getY(),
				p_robot.getYaw(),
				v_robot.getX(),
				v_robot.getY(),
				person_model_radius_,
				fov_person_
			);
			heading_direction_penalty.normalize(robot_circumradius_, max_speed_);
			// add to the local (current person) dataset
			person_disturbances.push_back(heading_direction_penalty.getScale());
		}
		// NOTE: consider checking whether a penalty value increases over the prediction horizon (but uncertainty?)

		// choose maximum disturbance throughout the trajectory as a score for a given person
		people_disturbances.push_back(*std::max_element(person_disturbances.cbegin(), person_disturbances.cend()));
	}

	// return max overall
	return *std::max_element(people_disturbances.cbegin(), people_disturbances.cend());
}

} // namespace hubero_local_planner
