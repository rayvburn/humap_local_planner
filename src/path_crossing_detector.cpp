#include <humap_local_planner/path_crossing_detector.h>
#include <social_nav_utils/gaussians.h>
#include <social_nav_utils/relative_location.h>

#include <algorithm>

namespace humap_local_planner {

PathCrossingDetector::PathCrossingDetector():
	person_model_radius_(0.28),
	robot_model_radius_(0.50),
	separation_threshold_(0.05),
	confidence_threshold_(0.70),
	safe_point_distance_multiplier_(SAFE_POINT_DISTANCE_MULTIPLIER_DEFAULT),
	speed_negligible_threshold_(SPEED_NEGLIGIBLE_THRESHOLD_DEFAULT),
	crossing_detected_(false),
	gap_closest_person_(NAN)
{}

void PathCrossingDetector::setParameters(
	double person_model_radius,
	double robot_model_radius,
	double separation_threshold,
	double confidence_threshold,
	double safe_point_distance,
	double speed_negligible_threshold
) {
	person_model_radius_ = person_model_radius;
	robot_model_radius_ = robot_model_radius;
	separation_threshold_ = separation_threshold;
	confidence_threshold_ = confidence_threshold;
	safe_point_distance_multiplier_ = safe_point_distance;
	speed_negligible_threshold_ = speed_negligible_threshold;
}

bool PathCrossingDetector::detect(
	const Trajectory& traj_robot,
	const std::vector<geometry_msgs::PoseStamped>& global_plan,
	const std::vector<Trajectory>& traj_people
) {
	prepareForDetecting();
	detectCrossingTrajectory(traj_robot, traj_people);
	detectCrossingPath(global_plan, traj_people);
	// no clue whether the closest detection comes from the trajectory or from the path
	return crossing_detected_;
}

bool PathCrossingDetector::detect(
	const Trajectory& traj_robot,
	const std::vector<Trajectory>& traj_people
) {
	prepareForDetecting();
	return detectCrossingTrajectory(traj_robot, traj_people);
}

bool PathCrossingDetector::detect(
	const std::vector<geometry_msgs::PoseStamped>& global_plan,
	const std::vector<Trajectory>& traj_people
) {
	prepareForDetecting();
	return detectCrossingPath(global_plan, traj_people);
}

double PathCrossingDetector::motionDirectionFilter(
	double direction,
	const geometry::Pose& pose_prev,
	const geometry::Pose& pose_current,
	double innovation_factor
) {
	auto pos_diff = pose_current.getPosition() - pose_prev.getPosition();
	auto new_dir = pos_diff.calculateDirection().getRadian();

	// complementary filters
	double filtered_dir = geometry::Angle(
		(1.0 - innovation_factor) * direction + innovation_factor * new_dir
	).getRadian();
	return filtered_dir;
}

void PathCrossingDetector::prepareForDetecting() {
	crossing_detected_ = false;
	crossing_people_data_.clear();
	safe_directions_behind_crossing_people_.clear();
	closest_safe_direction_ = {(NAN, NAN, NAN), NAN};
	gap_closest_person_ = std::numeric_limits<double>::max();
}

bool PathCrossingDetector::detectCrossingTrajectory(
	const Trajectory& traj_robot,
	const std::vector<Trajectory>& traj_people
) {
	if (traj_robot.getPoses().empty()) {
		return false;
	}
	for (const auto& traj_person: traj_people) {
		if (traj_person.getPoses().empty()) {
			return false;
		}
	}

	auto traj_robot_poses = traj_robot.getPoses();

	for (const auto& traj_person: traj_people) {
		// velocity data is required for further calculations
		if (traj_person.getVelocities().empty()) {
			continue;
		}

		auto vel_person = traj_person.getVelocities().front();
		double speed_person = std::hypot(vel_person.getX(), vel_person.getY());

		if (speed_person < speed_negligible_threshold_) {
			continue;
		}

		// iterator must be referenced to an allocated container
		auto traj_person_poses = traj_person.getPoses();
		if (traj_person_poses.empty()) {
			continue;
		}

		auto robot_pose_it = traj_robot_poses.cbegin();
		auto person_pose_it = traj_person_poses.cbegin();

		double timestamp = 0.0;
		// filtered motion directions of a robot and a person
		double direction_robot = traj_robot.getPoses().front().getYaw();
		double direction_person = std::atan2(
			traj_person.getVelocities().front().getY(),
			traj_person.getVelocities().front().getX()
		);

		bool first_iteration = true;
		// iterate over human poses and compare against robot poses
		for (
			;
			robot_pose_it != traj_robot_poses.cend() && person_pose_it != traj_person_poses.cend();
			robot_pose_it++, person_pose_it++
		) {
			if (!first_iteration) {
				direction_robot = PathCrossingDetector::motionDirectionFilter(
					direction_robot,
					*std::prev(robot_pose_it),
					*robot_pose_it,
					ROBOT_INNOV_FACTOR_DEFAULT
				);
				direction_person = PathCrossingDetector::motionDirectionFilter(
					direction_person,
					*std::prev(person_pose_it),
					*person_pose_it,
					PERSON_INNOV_FACTOR_DEFAULT
				);
			}

			// assigning direction into yaw here
			bool person_crosses = performCrossingDetection(
				geometry::Pose(robot_pose_it->getX(), robot_pose_it->getY(), direction_robot),
				geometry::Pose(person_pose_it->getX(), person_pose_it->getY(), direction_person),
				timestamp,
				first_iteration,
				true
			);

			if (!person_crosses) {
				timestamp += traj_person.getTimeDelta();
				first_iteration = false;
				continue;
			}

			// do not clear the flag if previously set
			crossing_detected_ = true;
			crossing_people_data_.push_back({traj_person.getPoses().front(), direction_person});

			timestamp += traj_person.getTimeDelta();
			first_iteration = false;
			// processing this trajectory further is not needed
			break;
		}
	}

	// second stage - find safe directions
	//
	// robot's initial pose and direction are known
	findSafeDirectionsAndClosestPair(traj_robot.getPoses().front());
	return crossing_detected_;
}

bool PathCrossingDetector::detectCrossingPath(
	const std::vector<geometry_msgs::PoseStamped>& global_plan,
	const std::vector<Trajectory>& traj_people
) {
	std::vector<geometry::Pose> global_plan_poses;
	for (const auto& pose_stamped: global_plan) {
		global_plan_poses.push_back(geometry::Pose(pose_stamped));
	}

	if (global_plan_poses.empty()) {
		return false;
	}

	// The loop is constructed as follows:
	// - for each trajectory of detected people
	//   - iterate over person's trajectory poses
	//     - and check it against each pose of the global path
	for (const auto& traj_person: traj_people) {
		// velocity data is required for further calculations
		if (traj_person.getVelocities().empty()) {
			continue;
		}

		auto vel_person = traj_person.getVelocities().front();
		double speed_person = std::hypot(vel_person.getX(), vel_person.getY());

		if (speed_person < speed_negligible_threshold_) {
			continue;
		}

		// iterator must be referenced to an allocated container
		auto traj_person_poses = traj_person.getPoses();
		if (traj_person_poses.empty()) {
			break;
		}

		// store filtered motion directions of a person
		double direction_person = std::atan2(
			traj_person.getVelocities().front().getY(),
			traj_person.getVelocities().front().getX()
		);

		double timestamp = 0.0;
		bool first_iteration_person = true;
		bool collision_w_person_detected = false;
		// iterate over human poses
		for (
			auto person_pose_it = traj_person_poses.cbegin();
			person_pose_it != traj_person_poses.cend();
			person_pose_it++
		) {
			if (!first_iteration_person) {
				direction_person = PathCrossingDetector::motionDirectionFilter(
					direction_person,
					*std::prev(person_pose_it),
					*person_pose_it,
					PERSON_INNOV_FACTOR_DEFAULT
				);
			}

			// stores filtered motion direction of a robot
			double direction_robot = NAN;
			// non empty container ensured before, seed the robot direction somehow
			if (global_plan_poses.size() >= 2) {
				direction_robot = (
					global_plan_poses.at(1).getPosition() - global_plan_poses.at(0).getPosition()
				).calculateDirection().getRadian();
			} else if (!global_plan_poses.empty()) {
				// global path might not have the orientation updated
				direction_robot = global_plan_poses.front().getYaw();
			}

			bool first_iteration_path = true;
			// iterate over robot poses
			for (
				auto robot_pose_it = global_plan_poses.cbegin();
				robot_pose_it != global_plan_poses.cend();
				robot_pose_it++
			) {
				if (!first_iteration_path) {
					direction_robot = PathCrossingDetector::motionDirectionFilter(
						direction_robot,
						*std::prev(robot_pose_it),
						*robot_pose_it,
						ROBOT_INNOV_FACTOR_DEFAULT
					);
				}

				// assigning direction into yaw here
				bool person_crosses = performCrossingDetection(
					geometry::Pose(robot_pose_it->getX(), robot_pose_it->getY(), direction_robot),
					geometry::Pose(person_pose_it->getX(), person_pose_it->getY(), direction_person),
					timestamp,
					first_iteration_path,
					true
				);

				if (!person_crosses) {
					first_iteration_path = false;
					continue;
				}

				crossing_detected_ = true;
				crossing_people_data_.push_back({traj_person.getPoses().front(), direction_person});

				first_iteration_path = false;
				collision_w_person_detected = true;
				// processing this trajectory further is not needed
				break;
			}

			// exit to outer loop
			if (collision_w_person_detected) {
				break;
			}

			timestamp += traj_person.getTimeDelta();
			first_iteration_person = false;
		}
	}

	// second stage - find safe directions
	//
	// robot's initial pose and direction are known
	findSafeDirectionsAndClosestPair(global_plan_poses.front());
	return crossing_detected_;
}

bool PathCrossingDetector::performCrossingDetection(
	const geometry::Pose& robot_pose,
	const geometry::Pose& person_pose,
	double timestamp_prediction,
	bool save_gap_to_closest_person,
	bool compute_crossing_angle_confidence
) {
	// estimated distance between the centers of the robot and human
	auto dist_center = (robot_pose.getPosition() - person_pose.getPosition()).calculateLength();
	// spatial gap between the models of the robot and human
	auto dist_gap = dist_center - person_model_radius_ - robot_model_radius_;
	// trim in case of collision detection
	dist_gap = std::max(0.0, dist_gap);

	// store only the actual (not predicted) closest gap - update this only during the first iteration
	if (save_gap_to_closest_person) {
		gap_closest_person_ = std::min(gap_closest_person_, dist_gap);
	}

	if (dist_gap > separation_threshold_) {
		return false;
	}

	// confidence of crossing based on the time of prediction ("how far from now"); approx. 50% at 2 sec pred.
	const double TIMING_EXP_FACTOR = -0.34;
	double time_confidence = std::exp(TIMING_EXP_FACTOR * timestamp_prediction);

	// evaluate the angle of approaching crossing vectors
	geometry::Angle cross_angle(robot_pose.getYaw() - person_pose.getYaw());

	// knowing whether the human approaches from the left or right is also necessary
	social_nav_utils::RelativeLocation rel_loc(
		robot_pose.getX(),
		robot_pose.getY(),
		robot_pose.getYaw(),
		person_pose.getX(),
		person_pose.getY()
	);

	double angle_confidence = 1.0;
	if (compute_crossing_angle_confidence) {
		/*
		* Detection of crossings between the paths of the robot and the human. Cases of interest are crossings
		* with the human approaching directly from the side (right or left).
		* It is modelled by the 1D Gaussian (for angle domain) with a mean at abs(M_PI_2)
		* and with a standard deviation of (M_PI_2 / 2.0) (2 sigma rule)
		*/
		// side-angle confidence; whether the crossing person approaches directly from the side
		angle_confidence = social_nav_utils::calculateGaussianAngle(
			cross_angle.getRadian(),
			rel_loc.isLeftSide() ? M_PI_2 : -M_PI_2,
			std::pow(M_PI_4 / 2.0, 2.0),
			true // normalize to 1.0 at mean
		);
	}

	/*
	 * Consider only those detections that are in front of the robot (a 240 deg "cone" in front)
	 * The relative location is expressed in the robot's local coordinate system, therefore mean is 0.0
	 */
	double front_confidence = social_nav_utils::calculateGaussianAngle(
		rel_loc.getAngle(),
		0.0,
		std::pow((2.0 / 3.0 * M_PI) / 2.0, 2.0),
		true // normalize to 1.0 at mean
	);

	double confidence = time_confidence * angle_confidence * front_confidence;
	bool person_crossing = confidence >= confidence_threshold_;
	return person_crossing;
}

bool PathCrossingDetector::findSafeDirectionsAndClosestPair(const geometry::Pose& robot_pose) {
	// second stage - find safe directions

	// find directions from the robot initial pose towards safe positions (behind a person)
	for (const auto& person_data: crossing_people_data_) {
		auto person_pose = person_data.first;
		// NOTE: this is a filtered direction based on the full traj. prediction horizon
		auto person_dir = person_data.second;

		auto person_pos = person_pose.getPosition();
		// displacement
		geometry::Vector disp_behind_person(geometry::Angle(person_dir + M_PI));
		// TODO: the displacement multiplier might as well computed based on the current velocity
		disp_behind_person *= (person_model_radius_ * safe_point_distance_multiplier_);

		geometry::Vector safe_pos_behind_person(person_pos + disp_behind_person);
		// direction from the robot position towards the safe position is the safe direction
		auto safe_direction = (safe_pos_behind_person - robot_pose.getPosition()).calculateDirection().getRadian();

		safe_directions_behind_crossing_people_.push_back({safe_pos_behind_person, safe_direction});
	}

	// Stage 3 - find the closest "pair"
	if (safe_directions_behind_crossing_people_.empty()) {
		// keep NANs
		return false;
	} else if (safe_directions_behind_crossing_people_.size() == 1) {
		closest_safe_direction_ = safe_directions_behind_crossing_people_.front();
		return true;
	}
	// multiple - must sort according to the distance (from the robot to the safe position)
	std::vector<std::pair<double, std::pair<geometry::Vector, double>>> sort_container;
	for (const auto& elem: safe_directions_behind_crossing_people_) {
		double dist = (robot_pose.getPosition() - elem.first).calculateLength();
		sort_container.push_back({dist, elem});
	}
	std::sort(
		sort_container.begin(),
		sort_container.end(),
		// sort ascending
		[](
			const std::pair<double, std::pair<geometry::Vector, double>>& el1,
			const std::pair<double, std::pair<geometry::Vector, double>>& el2
		) -> bool {
			return el1.first < el2.first;
		}
	);
	closest_safe_direction_ = sort_container.front().second;
	return true;
}

} // namespace humap_local_planner
