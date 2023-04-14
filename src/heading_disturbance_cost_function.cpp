#include <hubero_local_planner/heading_disturbance_cost_function.h>
#include <angles/angles.h>

#include <hubero_local_planner/utils/transformations.h>

namespace hubero_local_planner {

HeadingDisturbanceCostFunction::HeadingDisturbanceCostFunction():
	fov_person_(3.31613),
	person_model_radius_(0.4),
	disturbance_spatial_factor_exp_(-0.8)
{}

void HeadingDisturbanceCostFunction::setPeopleDetections(const std::vector<people_msgs_utils::Person>& people) {
	people_ = people;
}

void HeadingDisturbanceCostFunction::setParameters(double fov_person, double person_model_radius, double spatial_factor_exp) {
	fov_person_ = fov_person;
	person_model_radius_ = person_model_radius;
	disturbance_spatial_factor_exp_ = spatial_factor_exp;
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

	/*
	 * 1st level vector - related to subsequent people
	 * 2nd level vector - related to subsequent time steps
	 * pose - pose of the person in a given time step
	 */
	std::vector<std::vector<geometry::Pose>> people_pose_pred;
	// predict people poses
	for (const auto& person: people_) {
		std::vector<geometry::Pose> person_poses;
		// current pose
		person_poses.push_back(person.getPose());

		// create future pose set for each detected person (assuming constant velocity)
		auto vel = person.getVelocity();
		for (unsigned int i = 1; i < num; i++) {
			// to predict, refer to the newest available pose and initial velocity
			person_poses.push_back(computeNextPose(person_poses.back(), vel, dt));
		}

		// set of predicted poses for the currently checked person is ready
		people_pose_pred.push_back(person_poses);
	}

	// NOTE that each nested vector in `people_pose_pred` has equal size to `num`

	// storage for disturbances related to subsequent people
	std::vector<double> people_disturbances;

	// iterate over predicted poses of a person, compare against each pose of robot trajectory
	// against all people pose predictions ...
	for (const auto& person_pose_pred: people_pose_pred) {
		// storage for disturbances against person throughout the trajectory
		std::vector<double> person_disturbances;

		// check all robot trajectory points ...
		for (unsigned int i = 0; i < traj.getPointsSize(); i++) {
			// retrieve poses
			double x_robot, y_robot, th_robot = 0.0;
			traj.getPoint(i, x_robot, y_robot, th_robot);
			double x_person = person_pose_pred.at(i).getX();
			double y_person = person_pose_pred.at(i).getY();
			double th_person = person_pose_pred.at(i).getYaw();
			// retrieve robot velocity
			double vx_robot = 0.0;
			double vy_robot = 0.0;
			if (i == 0) {
				// velocity is explicitly given in base coordinates -> convert to global coords
				geometry::Vector robot_vel;
				computeVelocityGlobal(
					geometry::Vector(traj.xv_, traj.yv_, traj.thetav_),
					geometry::Pose(x_robot, y_robot, th_robot),
					robot_vel
				);
				vx_robot = robot_vel.getX();
				vy_robot = robot_vel.getY();
			} else {
				// eval robot displacement
				double x_prev_robot, y_prev_robot, th_prev_robot = 0.0;
				traj.getPoint(i - 1, x_prev_robot, y_prev_robot, th_prev_robot);
				auto robot_displacement = subtractPoses(
					geometry::Pose(x_robot, y_robot, th_robot),
					geometry::Pose(x_prev_robot, y_prev_robot, th_prev_robot)
				);
				// compute vel from pose difference
				auto robot_vel = geometry::Vector(
					robot_displacement.getX() / dt,
					robot_displacement.getY() / dt,
					robot_displacement.getYaw() / dt
				);
				vx_robot = robot_vel.getX();
				vy_robot = robot_vel.getY();
			}

			double disturbance = HeadingDisturbanceCostFunction::calculateDirectionDisturbance(
				x_robot,
				y_robot,
				th_robot,
				vx_robot,
				vy_robot,
				x_person,
				y_person,
				th_person,
				fov_person_,
				person_model_radius_,
				disturbance_spatial_factor_exp_
			);

			// add to the local dataset
			person_disturbances.push_back(disturbance);
		}

		// choose maximum disturbance throughout the trajectory as a score for a given person
		people_disturbances.push_back(*std::max_element(person_disturbances.cbegin(), person_disturbances.cend()));
	}

	// finally, multiply disturbances by person detection reliability (actually ignores poor tracks)
	std::vector<double> disturbances_corrected;
	auto people_it = people_.cbegin();
	auto disturbances_it = people_disturbances.cbegin();
	for (
		; // vectors of different types, cannot put it in here
		people_it != people_.end() && disturbances_it != people_disturbances.end();
		people_it++, disturbances_it++
	) {
		double person_reliability = people_it->getReliability();
		double disturbance_raw = *disturbances_it;
		disturbances_corrected.push_back(person_reliability * disturbance_raw);
	}
	return *std::max_element(disturbances_corrected.cbegin(), disturbances_corrected.cend());
}

// static
double HeadingDisturbanceCostFunction::calculateDirectionDisturbance(
	double x_robot,
	double y_robot,
	double yaw_robot,
	double vx_robot,
	double vy_robot,
	double x_person,
	double y_person,
	double yaw_person,
	double fov_person,
	double person_model_radius,
	double spatial_factor_exp
) {
	double dist_vector[2] = {0.0};
	dist_vector[0] = x_robot - x_person;
	dist_vector[1] = y_robot - y_person;

	// length of the vector
	double dist_vector_length = std::sqrt(
		std::pow(dist_vector[0], 2)
		+ std::pow(dist_vector[1], 2)
	);

	// direction of vector connecting robot and person (defines where the robot is located in relation to a person [ego agent])
	double dist_vector_angle = std::atan2(dist_vector[1], dist_vector[0]);

	// relative location vector angle (defines side where the robot is located in relation to a person)
	double rel_loc_angle = angles::normalize_angle(dist_vector_angle - yaw_person);

	// old notation: alpha-beta can be mapped to: i -> robot, j -> person
	double gamma = angles::normalize_angle(rel_loc_angle - yaw_robot);

	// calculate threshold angle values, normalize angles
	/// indicates that j moves in the same direction as i
	double gamma_eq = angles::normalize_angle(dist_vector_angle - 2 * yaw_person);
	/// indicates that j moves in a direction opposite to i
	double gamma_cc = angles::normalize_angle(M_PI - 2 * yaw_person);
	/// indicates that a ray created from a centre point and a heading of j crosses the centre point of i
	double gamma_opp = angles::normalize_angle(gamma_eq - M_PI);

	/*
	* Find range of angles that indicate <opposite, crossing in front, etc> motion direction of the robot towards person
	* e.g. opposite direction adjoins with `cross behind` and `outwards` ranges.
	* Range between direction regions can be used as a variance to model gaussian cost
	*/
	// decode relative location (right/left side)
	std::string relative_location_side = "unknown";
	if (rel_loc_angle < 0.0) {
		relative_location_side = "right";
	} else if (rel_loc_angle >= 0.0) {
		relative_location_side = "left";
	}

	// not all angles are required in this method, some values are computed for future use
	double gamma_cf_start = 0.0;
	double gamma_cf_finish = 0.0;
	double gamma_cb_start = 0.0;
	double gamma_cb_finish = 0.0;
	double gamma_out_start = 0.0;
	double gamma_out_finish = 0.0;

	if (relative_location_side == "right") {
		gamma_cf_start = gamma_cc;
		gamma_cf_finish = gamma_eq;
		gamma_cb_start = gamma_opp;
		gamma_cb_finish = gamma_cc;
		gamma_out_start = gamma_eq;
		gamma_out_finish = gamma_opp;
	} else if (relative_location_side == "left") {
		gamma_cf_start = gamma_eq;
		gamma_cf_finish = gamma_cc;
		gamma_cb_start = gamma_cc;
		gamma_cb_finish = gamma_opp;
		gamma_out_start = gamma_opp;
		gamma_out_finish = gamma_eq;
	} else {
		throw std::runtime_error("Unknown value of relative location");
	}

	double gamma_cf_range = std::abs(angles::shortest_angular_distance(gamma_cf_start, gamma_cf_finish));
	double gamma_cb_range = std::abs(angles::shortest_angular_distance(gamma_cb_start, gamma_cb_finish));
	double gamma_out_range = std::abs(angles::shortest_angular_distance(gamma_out_start, gamma_out_finish));

	// we must keep arcsin argument below 1.0, otherwise NAN will be returned instead of a very big angle
	double dist_gamma_range = std::max(person_model_radius, dist_vector_length);
	double gamma_cc_range = 2.0 * std::asin(person_model_radius / dist_gamma_range);
	// Variance is computed according 68–95–99.7 rule https://en.wikipedia.org/wiki/68%E2%80%9395%E2%80%9399.7_rule
	double gamma_cc_stddev = (gamma_cc_range / 2.0) / 3.0;
	double gamma_cc_variance = std::pow(gamma_cc_stddev, 2);

	/*
	* Note that we assume that gaussian cost of disturbance exists only within bounds of following direction angles:
	* - crossing-center (i.e. opposite and moving towards the person center)
	* - crossing-in-front
	*/
	// 1D Gaussian function, note that angle domain wraps at 3.14 so we must check for maximum of gaussians
	// located at gamma_X and shifted 2 * pi to the left and right; gamma angle should already be normalized here
	double gaussian_dir_cc = HeadingDisturbanceCostFunction::calculateGaussianAngle(gamma, gamma_cc, gamma_cc_variance);

	// 3 sigma rule - let the cost spread only over the CF region
	double gamma_cf_stddev = (gamma_cf_range / 2.0) / 3.0;
	double gamma_cf_variance = std::pow(gamma_cf_stddev, 2);
	// mean - center of the cross front region
	double gamma_cf_center = angles::normalize_angle(gamma_cf_start + gamma_cf_range / 2.0);
	double gaussian_dir_cf = HeadingDisturbanceCostFunction::calculateGaussianAngle(
		gamma,
		gamma_cf_center,
		gamma_cf_variance
	);

	double gaussian_dir_result = std::max(gaussian_dir_cc, gaussian_dir_cf);

	// check whether the robot is located within person's FOV (only then affects human's behaviour);
	// again, 3 sigma rule is used here -> 3 sigma rule applied to the half of the FOV
	double fov_stddev = (fov_person / 2.0) / 3.0;
	double variance_fov = std::pow(fov_stddev, 2);
	// starting from the left side, half of the `fov_person` is located in 0.0 and rel_loc is 0.0
	// when obstacle is in front of the object
	double gaussian_fov = HeadingDisturbanceCostFunction::calculateGaussian(rel_loc_angle, 0.0, variance_fov);

	// check how fast the robot moves
	double speed_factor = std::sqrt(std::pow(vx_robot, 2) + std::pow(vy_robot, 2));

	// check how far the robot is from the person
	double eucl_dist = std::sqrt(std::pow(x_robot - x_person, 2) + std::pow(y_robot - y_person, 2));
	double dist_factor = std::exp(spatial_factor_exp * eucl_dist);

	return gaussian_dir_result * gaussian_fov * speed_factor * dist_factor;
}

// static
double HeadingDisturbanceCostFunction::calculateGaussian(double x, double mean, double variance, bool normalize) {
	double scale = 1.0;
	// with normalization, maximum possible value will be 1.0; otherwise, it depends on the value of variance
	if (!normalize) {
		scale = 1.0 / (std::sqrt(variance) * std::sqrt(2 * M_PI));
	}
	return scale * std::exp(-std::pow(x - mean, 2) / (2.0 * variance));
}

// static
double HeadingDisturbanceCostFunction::calculateGaussianAngle(double x, double mean, double variance, bool normalize) {
	double gaussian1 = calculateGaussian(x, mean             , variance, normalize);
	double gaussian2 = calculateGaussian(x, mean - 2.0 * M_PI, variance, normalize);
	double gaussian3 = calculateGaussian(x, mean + 2.0 * M_PI, variance, normalize);
	return std::max(std::max(gaussian1, gaussian2), gaussian3);
}

} // namespace hubero_local_planner
