#pragma once

#include <humap_local_planner/trajectory.h>

#include <utility>

namespace humap_local_planner {

/**
 * Detects whether paths created from trajectories do intersect
 */
class PathCrossingDetector {
public:
	/**
	 * When the trajectory crosses through a predicted path of a person whose initial speed is lower that than,
	 * it won't be treated as a valid cross.
	 */
	static constexpr double SPEED_NEGLIGIBLE_THRESHOLD_DEFAULT = 0.04;
	/**
	 * A factor to multiply the @ref person_model_radius_ by to find the safe position to rotate towards
	 * Note that a moving person will (usually) already be gone once the robot rotates towards the safe position
	 */
	static constexpr double SAFE_POINT_DISTANCE_MULTIPLIER_DEFAULT = 0.70;

	/**
	 * Standard deviation (in radians) of the relative location (angle from robot to human)
	 * Default value reflects a 120-degree-wide bell (2-sigma rule applied)
	 */
	static constexpr double FRONT_ANGLE_STDDEV_DEFAULT = (2.0 / 3.0 * M_PI) / 2.0;

	/// Innovation factor for the motion direction filtering
	static constexpr double ROBOT_INNOV_FACTOR_DEFAULT = 0.8;
	/// Innovation factor for the motion direction filtering (person predictions have smaller confidence)
	static constexpr double PERSON_INNOV_FACTOR_DEFAULT = 0.6;

	PathCrossingDetector();

	void setParameters(
		double person_model_radius,
		double robot_model_radius,
		double separation_threshold,
		double confidence_threshold,
		double front_angle_stddev = FRONT_ANGLE_STDDEV_DEFAULT,
		double safe_point_distance_multiplier = SAFE_POINT_DISTANCE_MULTIPLIER_DEFAULT,
		double speed_negligible_threshold = SPEED_NEGLIGIBLE_THRESHOLD_DEFAULT
	);

	/**
	 * @brief Performs crossing detection and tries to find the safe directions for further motion for the robot
	 *
	 * @param traj_robot evaluated robot trajectory (with velocities expressed in global coordinates)
	 * @param global_plan global path plan of the robot
	 * @param traj_people predicted trajectories of surrounding people
	 * @return true when crossing was detected
	 */
	bool detect(
		const Trajectory& traj_robot,
		const std::vector<geometry_msgs::PoseStamped>& global_plan,
		const std::vector<Trajectory>& traj_people
	);

	/// @brief Performs crossing detection and tries to find the safe directions for further motion for the robot
	bool detect(
		const Trajectory& traj_robot,
		const std::vector<Trajectory>& traj_people
	);

	/// @brief Performs crossing detection and tries to find the safe directions for further motion for the robot
	bool detect(
		const std::vector<geometry_msgs::PoseStamped>& global_plan,
		const std::vector<Trajectory>& traj_people
	);

	bool isCrossingDetected() const {
		return crossing_detected_;
	}

	/**
	 * @brief Returns safe directions avoiding people whose paths cross the robot's path
	 *
	 * Directions are expressed in the global coordinate system (typically "odom")
	 */
	std::vector<std::pair<geometry::Vector, double>> getSafeDirections() const {
		return safe_directions_behind_crossing_people_;
	}

	/**
	 * @brief The closest point from the container returned by the @ref getSafeDirections
	 */
	std::pair<geometry::Vector, double> getClosestSafeDirection() const {
		return closest_safe_direction_;
	}

	/**
	 * @brief Returns the container with the most recent poses and motion dir. of the people that cross robot's path
	 */
	std::vector<std::pair<geometry::Pose, double>> getCrossingPeopleData() const {
		return crossing_people_data_;
	}

	/**
	 * @brief Returns the current gap between the robot model and the closest person model (moving one)
	 *
	 * "Current gap" means that the initial poses of each trajectory are considered
	 */
	double getGapToClosestPerson() const {
		return gap_closest_person_;
	}

	/**
	 * @brief Implements a complementary filter to process the motion direction (from poses)
	 *
	 * @param direction the newest motion direction
	 * @param pose_prev previous pose
	 * @param pose_current current pose
	 * @param innovation_factor the bigger the value is, the more confident we are about the new observations
	 * @return a new motion direction
	 */
	static double motionDirectionFilter(
		double direction,
		const geometry::Pose& pose_prev,
		const geometry::Pose& pose_current,
		double innovation_factor
	);

protected:
	/**
	 * @brief Resets class' members to clear any pending detections
	 */
	void prepareForDetecting();

	/**
	 * @return true if any crossing was detected
	 */
	bool detectCrossingTrajectory(
		const Trajectory& traj_robot,
		const std::vector<Trajectory>& traj_people
	);

	/**
	 * @return true if any crossing was detected
	 */
	bool detectCrossingPath(
		const std::vector<geometry_msgs::PoseStamped>& global_plan,
		const std::vector<Trajectory>& traj_people
	);

	/**
	 * @brief Performs detection of a human proximity that may indicate crossing of the robot's path
	 *
	 * Method that needs to be called in each step of the prediction horizon
	 *
	 * @param robot_pose
	 * @param person_pose
	 * @param timestamp_prediction
	 * @param save_gap_to_closest_person
	 *
	 * @return true if crossing was detected and its confidence exceeds the threshold
	 */
	bool performCrossingDetection(
		const geometry::Pose& robot_pose,
		const geometry::Pose& person_pose,
		double timestamp_prediction,
		bool save_gap_to_closest_person,
		bool compute_crossing_angle_confidence = true
	);

	/**
	 * @brief
	 *
	 * @param robot_pose the current pose of the robot (first of the predicted trajectory)
	 * @return true if safe direction has been found
	 */
	bool findSafeDirectionsAndClosestPair(const geometry::Pose& robot_pose);

	double person_model_radius_;
	double robot_model_radius_;
	double separation_threshold_;
	double confidence_threshold_;

	double safe_point_distance_multiplier_;
	double speed_negligible_threshold_;

	// The most recent poses (not predicted) and motion directions of the people that cross robot's path
	std::vector<std::pair<geometry::Pose, double>> crossing_people_data_;
	bool crossing_detected_;

	std::vector<std::pair<geometry::Vector, double>> safe_directions_behind_crossing_people_;
	std::pair<geometry::Vector, double> closest_safe_direction_;

	double gap_closest_person_;
	/// Standard deviation (in radians) of the relative location (angle from robot to human)
	double front_angle_stddev_;
};

} // namespace humap_local_planner
