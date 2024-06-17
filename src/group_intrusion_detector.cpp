#include <humap_local_planner/group_intrusion_detector.h>

#include <angles/angles.h>
#include <costmap_2d/cost_values.h>

namespace humap_local_planner {

GroupIntrusionDetector::GroupIntrusionDetector():
	global_goal_within_group_(false),
	global_goal_group_dist_(NAN),
	global_goal_group_dist_edge_(NAN),
	footprint_inflated_diameter_(0.60),
	recovery_point_distance_threshold_(0.40)
{
	// for safety (otherwise "detect" might break)
	assert(PTS_NUM_CROSSING_GROUP >= 2);
}

void GroupIntrusionDetector::setParameters(
	double footprint_inflated_diameter,
	double recovery_point_distance_threshold
) {
	footprint_inflated_diameter_ = footprint_inflated_diameter;
	recovery_point_distance_threshold_ = recovery_point_distance_threshold;
}

bool GroupIntrusionDetector::detect(
	const std::vector<Group>& groups,
	const std::vector<geometry_msgs::PoseStamped>& global_plan,
	const geometry_msgs::PoseStamped& global_goal
) {
	resetAlternativeGoal();
	global_goal_within_group_ = false;
	global_goal_group_dist_ = NAN;
	global_goal_group_dist_edge_ = NAN;
	groups_intersected_plan_.clear();

	if (groups.empty()) {
		return false;
	}
	if (global_plan.empty()) {
		return false;
	}

	// iterate over detected groups
	for (const auto& group: groups) {
		// firstly, check if the global goal lies within a group that the global path might cross; if so -> do nothing
		bool global_goal_within = checkPointInEllipse(
			global_goal.pose.position.x,
			global_goal.pose.position.y,
			group.getPositionX(),
			group.getPositionY(),
			group.getOrientationYaw(),
			group.getSpanX() / 2.0,
			group.getSpanY() / 2.0
		);
		if (global_goal_within) {
			global_goal_within_group_ = true;
			// compute distances to the group that "contains" the global goal
			double robot_x_current = global_plan.front().pose.position.x;
			double robot_y_current = global_plan.front().pose.position.y;
			double group_robot_vector_x = robot_x_current - group.getPositionX();
			double group_robot_vector_y = robot_y_current - group.getPositionY();

			double dist_center = std::hypot(
				group_robot_vector_x,
				group_robot_vector_y
			);
			auto pt_intersection = findEllipseLineIntersection(
				std::atan2(group_robot_vector_y, group_robot_vector_x),
				group.getPositionX(),
				group.getPositionY(),
				group.getOrientationYaw(),
				group.getSpanX() / 2.0,
				group.getSpanY() / 2.0
			);
			double dist_edge = std::hypot(
				robot_x_current - pt_intersection.first,
				robot_y_current - pt_intersection.second
			);
			// select closest
			if (!std::isnan(global_goal_group_dist_)) {
				global_goal_group_dist_ = std::min(global_goal_group_dist_, dist_center);
				global_goal_group_dist_edge_ = std::min(global_goal_group_dist_edge_, dist_edge);
			} else {
				global_goal_group_dist_ = dist_center;
				global_goal_group_dist_edge_ = dist_edge;
			}
			continue;
		}

		int path_group_intersects_num = 0;
		double angle_crossing = NAN;

		// iterate over pose of the global path
		for (
			auto plan_pose_it = global_plan.cbegin();
			plan_pose_it != global_plan.cend();
			plan_pose_it++
		) {
			double robot_x = plan_pose_it->pose.position.x;
			double robot_y = plan_pose_it->pose.position.y;

			// evaluate whether the robot's pose of a trajectory intersects the elliptical model of the F-formation's O-space
			bool intersects = checkPointInEllipse(
				robot_x,
				robot_y,
				group.getPositionX(),
				group.getPositionY(),
				group.getOrientationYaw(),
				group.getSpanX() / 2.0,
				group.getSpanY() / 2.0
			);
			if (!intersects) {
				continue;
			}
			path_group_intersects_num++;

			// Helper lambda to compute the angle of path's initial crossing of the O-space model
			auto computeDirection = [](
				const geometry_msgs::PoseStamped& pt1,
				const geometry_msgs::PoseStamped& pt2
			) -> double {
				geometry::Vector v1(pt1);
				geometry::Vector v2(pt2);
				return (v2 - v1).calculateDirection().getRadian();
			};

			// NAN when the angle was not defined previously
			if (std::isnan(angle_crossing)) {
				// angle can only be properly defined once the current pose of the path is not the first one
				auto index = std::distance(global_plan.cbegin(), plan_pose_it);
				if (index >= 2) {
					angle_crossing = computeDirection(*std::prev(plan_pose_it), *plan_pose_it);
				}
				// otherwise - wait for the next iteration
			} else {
				// more accurate calculation of the angular transition of path into the spatial model of the group;
				// we're safe to use "prev" without additional checks here
				auto new_dir = computeDirection(*std::prev(plan_pose_it), *plan_pose_it);
				// complementary filter
				static const double INNOV_FACTOR = 0.25;
				angle_crossing = angles::normalize_angle(
					(1.0 - INNOV_FACTOR) * angle_crossing + INNOV_FACTOR * new_dir
				);
			}

			if (path_group_intersects_num < PTS_NUM_CROSSING_GROUP) {
				// switch to the next point
				continue;
			}
			if (std::isnan(angle_crossing)) {
				if (path_group_intersects_num <= (PTS_NUM_CROSSING_GROUP + PTS_NUM_CROSSING_GROUP)) {
					// this should not happen, let's check the path further (but at most a few more poses)
					continue;
				} else {
					// unable to accept this group - something is wrong
					break;
				}
			}

			groups_intersected_plan_.push_back(GroupInfo(group, angle_crossing));
			// switch to the next group
			break;
		}
	}

	if (groups_intersected_plan_.empty()) {
		return false;
	}
	return true;
}

bool GroupIntrusionDetector::planAlternativeGoal(
	const std::vector<geometry_msgs::PoseStamped>& global_plan,
	const ObstacleSeparationCostFunction& obstacle_costfun
) {
	// stored only for visualization purposes
	goal_alternative_candidates_.clear();

	// implemented "recovery" procedure only supports a situation when 1 group is being intruded
	if (groups_intersected_plan_.size() != 1) {
		resetAlternativeGoal();
		return false;
	}

	/*
	 * Stage 1
	 *
	 * verify whether the global path crosses more on the left or right
	 * count points more on the left/right?
	 * Matlab: point_in_ellipse_relative.m
	 */
	auto group = groups_intersected_plan_.front();

	for (const auto& plan_pose: global_plan) {
		double robot_x = plan_pose.pose.position.x;
		double robot_y = plan_pose.pose.position.y;

		bool intersects = checkPointInEllipse(
			robot_x,
			robot_y,
			group.getPositionX(),
			group.getPositionY(),
			group.getOrientationYaw(),
			group.getSpanX() / 2.0,
			group.getSpanY() / 2.0
		);
		if (!intersects) {
			continue;
		}
		// NOTE: crossing angle is evaluated here instead of the group orientation - we'll try to pass on the left
		// or right compared to the newest global path that crosses the group
		double angle = findPointLocationInEllipse(
			robot_x,
			robot_y,
			group.getPositionX(),
			group.getPositionY(),
			group.getCrossingAngle(),
			// TODO: span should be rescaled according to the diff between "crossing angle" and "orientation angle"
			group.getSpanX() / 2.0,
			group.getSpanY() / 2.0
		);
		angle > 0.0 ? group.incrementCrossingPointsLeft() : group.incrementCrossingPointsRight();
	}

	/*
	 * Stage 2
	 *
	 * find an intersection point of the ellipse and a line starting from the ellipse center and pointing
	 * to the left/right (90 deg angle)
	 * Matlab: point_in_ellipse_intersection.m
	 */
	// vector of recovery (temporary goal) point candidates
	std::vector<double> recovery_dir_candidates;

	// crossing angle defines how the path crosses the spatial model of the group -> it should be rotated 180 deg
	double angle_start = angles::normalize_angle(group.getCrossingAngle() + M_PI);
	double angle_step = (3.0 * M_PI_4) / static_cast<double>(ANGLE_CANDIDATES_SIDE);
	// for iterating through angles
	double angle = angle_start;
	// select directions to look for a recovery point - this step affects the priority of selecting an alternative goal
	if (group.getCrossingPointsRight() <= group.getCrossingPointsLeft()) {
		angle_step = -angle_step;
	}
	// candidates from the "major" side
	while (recovery_dir_candidates.size() < ANGLE_CANDIDATES_SIDE) {
		recovery_dir_candidates.push_back(angle);
		angle = angles::normalize_angle(angle + angle_step);
	}
	// candidates from the "minor" side
	angle = angle_start;
	angle_step = -angle_step;
	// -1 to avoid repeating the last angle in the loop above
	while (recovery_dir_candidates.size() < (2 * ANGLE_CANDIDATES_SIDE - 1)) {
		recovery_dir_candidates.push_back(angle);
		angle = angles::normalize_angle(angle + angle_step);
	}

	std::vector<std::tuple<double, double, double>> recovery_pt_candidates;
	for (const auto& dir: recovery_dir_candidates) {
		auto pt = findEllipseLineIntersection(
			dir,
			group.getPositionX(),
			group.getPositionY(),
			group.getOrientationYaw(),
			group.getSpanX() / 2.0,
			group.getSpanY() / 2.0
		);
		recovery_pt_candidates.push_back({
			pt.first,
			pt.second,
			dir
		});
	}

	/*
	 * Stage 3
	 *
	 * try to find a safe pose outside the ellipse
	 * the ellipse-line intersection + some distance -> gives a pose candidate
	 */
	// check multiple distances - few subsequent must be in a free space (first do not have to)
	double dist_granularity = footprint_inflated_diameter_ / DIST_GRANULARITY_DIVIDER;

	std::vector<std::tuple<double, double, double, double>> recovery_pt_safe_candidates;
	for (const auto& pt_wdir: recovery_pt_candidates) {
		double pt_isection_x = std::get<0>(pt_wdir);
		double pt_isection_y = std::get<1>(pt_wdir);
		double dir = std::get<2>(pt_wdir);

		double dist = 0.0;
		double dist_last_collision = dist;
		bool found_free_space = false;
		/*
		 * Store candidates and add them to goal_alternative_candidates_ only when found_free_space is OK
		 * at the end of DIST-based loop
		 */
		std::vector<std::tuple<double, double, double, double>> recovery_dirline_safe_candidates;

		/*
		 * Evaluate whether all subsequent points along a line (at a given direction),
		 * starting from a given distance from the group center, are collision-free and if so add them to candidates
		 */
		while (dist <= DIST_FROM_EDGE_MAX) {
			double pt_extension_x = pt_isection_x + dist * std::cos(dir);
			double pt_extension_y = pt_isection_y + dist * std::sin(dir);

			// TODO: orientation is arbitrary (from the current pose to the intermediate goal)
			double theta = 0.0;
			// score the candidates - evaluate whether they do not lie near the obstacles - provide that they are in a free space
			double pt_extension_cost = obstacle_costfun.getFootprintCost(pt_extension_x, pt_extension_y, theta);
			// accept only points that do not lead to collisions
			bool pt_collides = pt_extension_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE || pt_extension_cost < 0.0;
			// we'll only be checking more pts along this direction if subsequent points are collision-free (the first
			// ones don't need to)
			if (pt_collides && found_free_space) {
				// let's check if the distance from the previously detected obstacle will fit the footprint diameter
				bool footprint_fits_into_gap = (dist - dist_last_collision) >= footprint_inflated_diameter_;
				if (!footprint_fits_into_gap) {
					// when 1 point in a series is in collision - let's abort all direction-specific candidates
					recovery_dirline_safe_candidates.clear();
					break;
				}
				// the footprint fits into a gap - let's accept points along this direction;
				// TODO: ideally, we'd not break here but search further along this direction to find another "gap"
				break;
			} else if (pt_collides) {
				// waiting for the first non-colliding point
				dist_last_collision = dist;
				dist += dist_granularity;
				continue;
			}
			found_free_space = true;
			recovery_dirline_safe_candidates.push_back({pt_extension_x, pt_extension_y, dir, pt_extension_cost});
			dist += dist_granularity;
		}

		// collect ellipse intersection points (for visualization purposes)
		goal_alternative_candidates_.push_back(
			AltGoalCandidate(
				pt_isection_x,
				pt_isection_y,
				costmap_2d::LETHAL_OBSTACLE,
				goal_alternative_candidates_.size()
			)
		);

		// copy the valid dir-specific candidates into the main container
		for (const auto& dir_candidate: recovery_dirline_safe_candidates) {
			recovery_pt_safe_candidates.push_back(dir_candidate);
			// collect for visualization purposes
			goal_alternative_candidates_.push_back(
				AltGoalCandidate(
					std::get<0>(dir_candidate),
					std::get<1>(dir_candidate),
					std::get<3>(dir_candidate),
					goal_alternative_candidates_.size()
				)
			);
		}
	}

	// sort from the furthest to the closest to the point of ellipse crossing by the global path
	// this is a crucial step for the following operations
	std::sort(
		recovery_pt_safe_candidates.begin(),
		recovery_pt_safe_candidates.end(),
		[angle_start](const auto& a, const auto& b) {
			double sada = angles::shortest_angular_distance(angle_start, std::get<2>(a));
			double sadb = angles::shortest_angular_distance(angle_start, std::get<2>(b));
			return sada < sadb;
		}
	);

	if (recovery_pt_safe_candidates.empty()) {
		return false;
	}
	// find the sign of the "angle_step" for the loop further below
	double angle_step_diff_helper1 = std::get<2>(recovery_pt_safe_candidates.at(0));
	double angle_step_diff_helper2 = NAN;
	for (const auto& candidate: recovery_pt_safe_candidates) {
		if (angle_step_diff_helper1 == std::get<2>(candidate)) {
			continue;
		}
		angle_step_diff_helper2 = std::get<2>(candidate);
		break;
	}
	if (std::isnan(angle_step_diff_helper2)) {
		return false;
	}
	double angle_step_loop = angles::shortest_angular_distance(angle_step_diff_helper1, angle_step_diff_helper2);

	/*
	 * Stage 4
	 *
	 * Evaluate whether the furthest on the right (and on the left) is reachable, i.e., there are safe
	 * "direction lines" starting from the angle_start to the "angle of interest";
	 * select the furthest available
	 *
	 * Make sure that the furthest directions are feasible - there is an angular path from the "center" direction to
	 * the borders - all intermediate directions must be valid (traversable by the robot footprint)
	 */
	// it's convenient to have 2 containers - 1 with candidates on the left from the center and the other - the right
	std::vector<std::tuple<double, double, double, double>> recovery_pt_safe_candidates_side1;
	for (
		auto c_it = recovery_pt_safe_candidates.cbegin();
		c_it != recovery_pt_safe_candidates.cend();
		c_it++
	) {
		double dir = std::get<2>(*c_it);
		if (dir == angle_start) {
			break;
		}
		recovery_pt_safe_candidates_side1.push_back(*c_it);
	}

	std::vector<std::tuple<double, double, double, double>> recovery_pt_safe_candidates_side2;
	for (
		auto c_it = (--recovery_pt_safe_candidates.cend());
		c_it != recovery_pt_safe_candidates.cbegin();
		c_it--
	) {
		double dir = std::get<2>(*c_it);
		if (dir == angle_start) {
			break;
		}
		recovery_pt_safe_candidates_side2.push_back(*c_it);
	}

	if (recovery_pt_safe_candidates_side1.empty() || recovery_pt_safe_candidates_side2.empty()) {
		return false;
	}

	// looking for the furthest direction that is associated with a "traversable" angular path
	// lambda below evaluates whether all subsequent directions, starting from the "center" (angle_start),
	// towards the ref_candidate are accessible for the robot, i.e., there is a gap for the robot to move across
	auto checkDirectionAccessibility = [angle_start](
		const std::tuple<double, double, double, double>& ref_candidate,
		const std::vector<std::tuple<double, double, double, double>>& candidates,
		double angle_step_loop
	) -> bool {
		double prev_candidate_dir = NAN;
		double diff_expected = angle_step_loop;
		double diff_to_center = angles::shortest_angular_distance(std::get<2>(ref_candidate), angle_start);
		// we'll be looking for deltas with opposite sign once the "major" angle switched onto another side
		// of the central axis
		int sign_diff_center = diff_to_center >= 0.0 ? 1 : -1;
		// flag that helps to decide after the loop execution
		bool ref_dir_inaccessible = false;

		for (const auto& candidate_check: candidates) {
			if (prev_candidate_dir == std::get<2>(candidate_check)) {
				continue;
			}
			prev_candidate_dir = std::get<2>(candidate_check);

			double diff = angles::shortest_angular_distance(std::get<2>(ref_candidate), std::get<2>(candidate_check));
			if (diff == 0) {
				continue;
			}
			double diff_to_center_check = angles::shortest_angular_distance(std::get<2>(candidate_check), angle_start);
			bool diff_correct = std::abs(diff - diff_expected) <= 1e-04;
			int sign_diff_center_check = diff_to_center_check >= 0.0 ? 1 : -1;
			bool diff_inverse_sign = diff * sign_diff_center < 0.0;

			// if the "diff" is not correct, there might be an obstacle midway to the "target direction"
			if (!diff_correct) {
				ref_dir_inaccessible = true;
				break;
			}

			// check if we have already swept through the "crossing" angle
			if (sign_diff_center_check != sign_diff_center) {
				// acceptance condition
				break;
			}
			diff_expected = angles::normalize_angle(diff_expected + angle_step_loop);
		}

		if (ref_dir_inaccessible) {
			return false;
		}
		return true;
	};

	// this will only store an arbitrary point with a direction; and the direction is what we're focused on
	auto ref_candidate1 = recovery_pt_safe_candidates_side1.front();
	auto ref_candidate2 = recovery_pt_safe_candidates_side2.front();

	bool ref1_accessible = checkDirectionAccessibility(
		ref_candidate1,
		recovery_pt_safe_candidates_side1,
		+angle_step_loop
	);

	// progressing backwards (see sorting above)
	bool ref2_accessible = checkDirectionAccessibility(
		ref_candidate2,
		recovery_pt_safe_candidates_side2,
		-angle_step_loop
	);

	std::vector<std::tuple<double, double, double, double>> recovery_pt_safe_feasbile;
	auto addCandidatesToFeasible = [](
		const std::vector<std::tuple<double, double, double, double>>& candidate_container,
		const std::tuple<double, double, double, double>& ref_candidate,
		std::vector<std::tuple<double, double, double, double>>& feasible_container
	) {
		std::for_each(
			candidate_container.cbegin(),
			candidate_container.cend(),
			[ref_candidate, &feasible_container](const auto& candidate) {
				if (std::get<2>(candidate) == std::get<2>(ref_candidate)) {
					feasible_container.push_back(candidate);
				}
			}
		);
	};

	auto collectBestCandidates = [](
		const std::vector<std::tuple<double, double, double, double>>& candidate_container,
		const std::tuple<double, double, double, double>& ref_candidate,
		std::vector<std::tuple<double, double, double, double>>& feasible_container
	) {
		std::vector<std::tuple<double, double, double, double>> ref_candidate_neighbours;
		std::for_each(
			candidate_container.cbegin(),
			candidate_container.cend(),
			[ref_candidate, &ref_candidate_neighbours](const auto& candidate) {
				if (std::get<2>(candidate) == std::get<2>(ref_candidate)) {
					ref_candidate_neighbours.push_back(candidate);
				}
			}
		);
		if (ref_candidate_neighbours.empty()) {
			return;
		}
		// choose the neighbour with the lowest cost
		std::sort(
			ref_candidate_neighbours.begin(),
			ref_candidate_neighbours.end(),
			[](const auto& a, const auto& b) {
				// sort by costs, ascending
				return std::get<3>(a) < std::get<3>(b);
			}
		);
		feasible_container.push_back(ref_candidate_neighbours.front());
	};

	// copy each candidate with a matching "direction" value
	if (ref1_accessible) {
		collectBestCandidates(recovery_pt_safe_candidates_side1, ref_candidate1, recovery_pt_safe_feasbile);
	}
	if (ref2_accessible) {
		collectBestCandidates(recovery_pt_safe_candidates_side2, ref_candidate2, recovery_pt_safe_feasbile);
	}

	// Select reasonable point from the safe candidates:
	// * not too close to the current pose
	// * not too close to any obstacle
	// * grid points around the ellipse contain the area that seems to allow reaching the goal (from the `angle_start` to the target `dir`)
	for (const auto& pt: recovery_pt_safe_feasbile) {
		double pt_cost = std::get<3>(pt);
		if (pt_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
			continue;
		}
		double robot_x = global_plan.front().pose.position.x;
		double robot_y = global_plan.front().pose.position.y;
		double pt_x = std::get<0>(pt);
		double pt_y = std::get<1>(pt);
		auto dist = std::hypot(robot_x - pt_x, robot_y - pt_y);
		if (dist <= recovery_point_distance_threshold_) {
			continue;
		}
		if (
			std::isnan(pt_x)
			|| std::isnan(pt_y)
			|| pt_cost < 0.0
			|| pt_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE
		) {
			continue;
		}
		// calculate direction from the current robot pose to the target point
		double pt_yaw = std::atan2(pt_x - robot_x, pt_y - robot_y);
		// save the coordinates to class' members
		goal_alternatives_.push_back({geometry::Pose(pt_x, pt_y, pt_yaw), pt_cost});
	}
	if (goal_alternatives_.empty()) {
		return false;
	}
	// let's pursue the selected point until the global path stops crossing the o-space
	return true;
}

std::vector<GroupIntrusionDetector::AltGoalCandidate> GroupIntrusionDetector::getIntermediateGoals() const {
	std::vector<AltGoalCandidate> goals;
	for (const auto& alternative: goal_alternatives_) {
		goals.emplace_back(
			alternative.first.getX(),
			alternative.first.getY(),
			alternative.second,
			goals.size()
		);
	}
	return goals;
}

bool GroupIntrusionDetector::checkPointInEllipse(
	double x,
	double y,
	double h,
	double k,
	double theta,
	double a,
	double b
) {
	// Calculate the value of the ellipse equation for the given point
	double value =
		std::pow(((x - h) * std::cos(theta) + (y - k) * std::sin(theta)) / a, 2)
		+ std::pow(((x - h) * std::sin(theta) - (y - k) * std::cos(theta)) / b, 2);
	return value <= 1.0;
}

std::pair<double, double> GroupIntrusionDetector::findEllipseLineIntersection(
	double angle,
	double h,
	double k,
	double theta,
	double a,
	double b
) {
	// Convert the angle of interest into the ellipse's local coordinate system
	angle = angles::normalize_angle(angle - theta);

	// Calculate the intersection point
	// Eqn 4a, 4b http://quickcalcbasic.com/ellipse%20line%20intersection.pdf
	double x = h + a * std::cos(angle) * std::cos(theta) - b * std::sin(angle) * std::sin(theta);
	double y = k + a * std::cos(angle) * std::sin(theta) + b * std::sin(angle) * std::cos(theta);

	return {x, y};
}

double GroupIntrusionDetector::findPointLocationInEllipse(
	double x,
	double y,
	double h,
	double k,
	double theta,
	double a,
	double b
) {
	// Vector from center of the ellipse to the point
	double point_vector_x = x - h;
	double point_vector_y = y - k;

	// Vector representing the major axis of the ellipse
	double major_axis_vector_x = a * std::cos(theta);
	double major_axis_vector_y = b * std::sin(theta);

	return angleBetweenVectors(
		major_axis_vector_x,
		major_axis_vector_y,
		point_vector_x,
		point_vector_y
	);
}

double GroupIntrusionDetector::angleBetweenVectors(
	double u_x,
	double u_y,
	double v_x,
	double v_y
) {
	// Calculate the magnitudes of the vectors
	double magnitude_u = std::hypot(u_x, u_y);
	double magnitude_v = std::hypot(v_x, v_y);

	// Calculate the angle in radians - formula comes from "atan2(norm(cross(u,v)), dot(u,v))"
	double angle_radians = std::atan2(u_x * v_y - u_y * v_x, u_x * v_x + u_y * v_y);
	return angle_radians;
}

void GroupIntrusionDetector::resetAlternativeGoal() {
	goal_alternatives_.clear();
}

} // namespace humap_local_planner
