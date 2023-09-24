#include <hubero_local_planner/heading_disturbance_cost_function.h>
#include <hubero_local_planner/trajectory.h>

#include <social_nav_utils/heading_direction_disturbance.h>

namespace hubero_local_planner {

HeadingDisturbanceCostFunction::HeadingDisturbanceCostFunction(const std::vector<Person>& people):
	people_(people),
	fov_person_(3.31613),
	person_model_radius_(0.28),
	robot_circumradius_(0.275),
	max_speed_(0.55),
	compute_whole_horizon_(true)
{}

void HeadingDisturbanceCostFunction::setParameters(
	double fov_person,
	double person_model_radius,
	double robot_circumradius,
	double max_speed,
	bool compute_whole_horizon
) {
	fov_person_ = fov_person;
	person_model_radius_ = person_model_radius;
	robot_circumradius_ = robot_circumradius;
	max_speed_ = max_speed;
	compute_whole_horizon_ = compute_whole_horizon;
}

bool HeadingDisturbanceCostFunction::prepare() {
	return true;
}

double HeadingDisturbanceCostFunction::scoreTrajectory(base_local_planner::Trajectory& traj) {
	if (people_.empty()) {
		return 0.0;
	}

	// storage for disturbances related to subsequent people
	std::vector<double> people_disturbances;

	// iterate over predicted poses of a person, compare against each pose of robot trajectory
	// against all people pose predictions ...
	Trajectory robot_traj(traj);

	// select the ending iteration number when not the whole horizon is meant to be evaluated;
	// prepare for the case when `robot_traj.getVelocitiesNum()` is 0
	unsigned int i_end_whole = static_cast<unsigned int>(robot_traj.getVelocitiesNum());
	unsigned int i_end_first = std::min(i_end_whole, static_cast<unsigned int>(1));

	for (const auto& person: people_) {
		// storage for disturbances against person throughout the trajectory
		std::vector<double> person_disturbances;

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

			// delegate cost computation
			social_nav_utils::HeadingDirectionDisturbance heading_direction_penalty(
				p_person.getX(),
				p_person.getY(),
				p_person.getYaw(),
				person.getCovariancePoseXX(),
				person.getCovariancePoseXY(),
				person.getCovariancePoseYY(),
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
