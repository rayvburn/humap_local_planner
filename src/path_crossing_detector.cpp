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
	const std::vector<Trajectory>& traj_people
) {
	crossing_detected_ = false;
	crossing_people_data_.clear();
	safe_directions_behind_crossing_people_.clear();
	closest_safe_direction_ = {(NAN, NAN, NAN), NAN};
	gap_closest_person_ = std::numeric_limits<double>::max();

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
		auto vel_person = traj_person.getVelocities().front();
		double speed_person = std::hypot(vel_person.getX(), vel_person.getY());

		if (speed_person < speed_negligible_threshold_) {
			continue;
		}

		// iterator must be referenced to an allocated container
		auto traj_person_poses = traj_person.getPoses();

		auto robot_pose_it = traj_robot_poses.cbegin();
		auto person_pose_it = traj_person_poses.cbegin();

		double timestamp = 0.0;
		// filtered motion directions of a robot and a person
		double direction_robot = traj_robot.getPoses().front().getYaw();
		double direction_person = traj_person.getPoses().front().getYaw();

		bool first_iteration = true;
		// iterate over human poses and compare against robot poses
		for (
			;
			robot_pose_it != traj_robot_poses.cend() && person_pose_it != traj_person_poses.cend();
			robot_pose_it++, person_pose_it++
		) {
			if (!first_iteration) {
				auto robot_pos_diff = robot_pose_it->getPosition() - std::prev(robot_pose_it)->getPosition();
				auto robot_new_dir = robot_pos_diff.calculateDirection().getRadian();

				auto person_pos_diff = person_pose_it->getPosition() - std::prev(person_pose_it)->getPosition();
				auto person_new_dir = person_pos_diff.calculateDirection().getRadian();

				// complementary filters
				const double ROBOT_INNOV_FACTOR = 0.8;
				const double PERSON_INNOV_FACTOR = 0.6;
				direction_robot = geometry::Angle(
					(1.0 - ROBOT_INNOV_FACTOR) * direction_robot + ROBOT_INNOV_FACTOR * robot_new_dir
				).getRadian();
				// person predictions have smaller confidence
				direction_person = geometry::Angle(
					(1.0 - PERSON_INNOV_FACTOR) * direction_person + PERSON_INNOV_FACTOR * person_new_dir
				).getRadian();
			}

			// estimated distance between the centers of the robot and human
			auto dist_center = (robot_pose_it->getPosition() - person_pose_it->getPosition()).calculateLength();
			// spatial gap between the models of the robot and human
			auto dist_gap = dist_center - person_model_radius_ - robot_model_radius_;
			// trim in case of collision detection
			dist_gap = std::max(0.0, dist_gap);

			// store only the actual (not predicted) closest gap - update this only during the first iteration
			if (first_iteration) {
				gap_closest_person_ = std::min(gap_closest_person_, dist_gap);
			}

			if (dist_gap > separation_threshold_) {
				timestamp += traj_person.getTimeDelta();
				first_iteration = false;
				continue;
			}

			// confidence of crossing based on the time of prediction ("how far from now"); approx. 50% at 2 sec pred.
			const double TIMING_EXP_FACTOR = -0.34;
			double cross_confidence = std::exp(TIMING_EXP_FACTOR * timestamp);

			// evaluate the angle of approaching crossing vectors
			geometry::Angle cross_angle(direction_robot - direction_person);

			// knowing whether the human approaches from the left or right is also necessary
			social_nav_utils::RelativeLocation rel_loc(
				robot_pose_it->getX(),
				robot_pose_it->getY(),
				robot_pose_it->getYaw(),
				person_pose_it->getX(),
				person_pose_it->getY()
			);

			/*
			 * Detection of crossings between the paths of the robot and the human. Cases of interest are crossings
			 * with the human approaching directly from the side (right or left).
			 * It is modelled by the 1D Gaussian (for angle domain) with a mean at abs(M_PI_2)
			 * and with a standard deviation of (M_PI_2 / 2.0) (2 sigma rule)
			 */
			// side-angle confidence; whether the crossing person approaches directly from the side
			double angle_confidence = social_nav_utils::calculateGaussianAngle(
				cross_angle.getRadian(),
				rel_loc.isLeftSide() ? M_PI_2 : -M_PI_2,
				std::pow(M_PI_2 / 2.0, 2.0),
				true // normalize to 1.0 at mean
			);

			double confidence = cross_confidence * angle_confidence;
			bool person_crossing = confidence >= confidence_threshold_;

			// do not clear the flag if previously set
			crossing_detected_ = crossing_detected_ || person_crossing;

			if (person_crossing) {
				crossing_people_data_.push_back({traj_person.getPoses().front(), direction_person});
			}

			timestamp += traj_person.getTimeDelta();
			first_iteration = false;
			break;
		}
	}

	// second stage - find safe directions
	//
	// robot's initial pose and direction are known
	auto robot_pose = traj_robot.getPoses().front();

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
		return crossing_detected_;
	} else if (safe_directions_behind_crossing_people_.size() == 1) {
		closest_safe_direction_ = safe_directions_behind_crossing_people_.front();
		return crossing_detected_;
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
	return crossing_detected_;
}

} // namespace humap_local_planner
