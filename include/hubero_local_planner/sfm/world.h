/*
 * world.h
 *
 *  Created on: Mar 31, 2021
 *      Author: rayvburn
 */

#ifndef INCLUDE_HUBERO_LOCAL_PLANNER_SFM_WORLD_H_
#define INCLUDE_HUBERO_LOCAL_PLANNER_SFM_WORLD_H_

#include <hubero_common/typedefs.h>
#include <utility>
#include <vector>

namespace sfm {

typedef enum {
	LOCATION_FRONT = 0,
	LOCATION_RIGHT,
	LOCATION_LEFT,
	LOCATION_BEHIND,
	LOCATION_UNSPECIFIED
} RelativeLocation;

struct StaticObject {
	Pose3 robot;
	Pose3 object;
	/// \brief so called d_alpha_beta - vector connecting the object with a robot (closest points of theirs footprints)
	Vector3 dist_v;
	/// \brief Distance to the robot (length of the `dist_v` vector)
	double dist;

	/**
	 * @brief Distance comparison
	 * @param other
	 * @return
	 */
	bool operator<(const StaticObject &other) const {
		return dist < other.dist;
	}
};

struct DynamicObject : StaticObject {
	/// \brief Velocity vector of the object
	Vector3 vel;
	/// \brief Direction of movement of the dynamic obstacles
	double dir_beta;
	///
	/// \brief Relative (to the alpha) location of dynamic object
	RelativeLocation rel_loc;
	/// \brief Relative (to the alpha) location of dynamic object expressed as an angle
	double rel_loc_angle;
	///
	/// \brief Angle of a distance vector connecting alpha and beta closest points
	double dist_angle;
};

// since contents are the same
typedef StaticObject Target;

struct Robot {
	/// @brief Pose of the centroid representing robot's footprint
	Pose3 centroid;
	/// @brief Velocity of the robot
	Vector3 vel;
	/// @brief Stores target's pose and pose of the robot's footprint that is closest to the target
	Target target;
	/// @brief Heading direction of the robot, can be either deducted from velocity or orientation
	double heading_dir;
};

/**
 * @brief Class named `world` as `environment` would not express clearly its contents.
 * World contains both environment and robot data.
 */
class World {
public:
	/**
	 *
	 * @param robot_pose_centroid: pose of the center of the robot's footprint
	 * @param robot_pose: robot pose that is closest to the target
	 * @param target_pose: target pose
	 * @param robot_vel
	 */
	World(
			const Pose3& robot_pose_centroid,
			const Pose3& robot_pose,
			const Pose3& target_pose,
			const Vector3& robot_vel
	);

	/**
	 * @brief Adds obstacle with given parameters and performs necessary calculations
	 * @param robot_pose_closest
	 * @param obstacle_pose_closest
	 * @param obstacle_vel
	 */
	void addObstacle(
			const Pose3& robot_pose_closest,
			const Pose3& obstacle_pose_closest,
			const Vector3& obstacle_vel
	);

	/**
	 * @brief Clears up existing obstacles
	 * @param robot_pose_closest
	 * @param obstacle_pose_closest
	 * @param obstacle_vel
	 */
	void addObstacles(
			const std::vector<Pose3>& robot_pose_closest,
			const std::vector<Pose3>& obstacle_pose_closest,
			const std::vector<Vector3>& obstacle_vel
	);

	inline Robot getRobotData() const {
		return robot_;
	}
	inline std::vector<StaticObject> getStaticObjectsData() const {
		return obstacle_static_;
	}
	inline std::vector<DynamicObject> getDynamicObjectsData() const {
		return obstacle_dynamic_;
	}
	inline double getDistanceClosestStaticObject() const {
		return getDistanceClosest(obstacle_static_);
	}
	inline double getDistanceClosestDynamicObject() const {
		return getDistanceClosest(obstacle_dynamic_);
	}

	virtual ~World() = default;

protected:
	void addObstacleStatic(
			const Pose3& robot_pose_closest,
			const Pose3& obstacle_pose_closest
	);
	void addObstacleDynamic(
			const Pose3& robot_pose_closest,
			const Pose3& obstacle_pose_closest,
			const Vector3& obstacle_vel
	);

	void computeObjectRelativeLocation(
			const Angle &actor_yaw,
			const Vector3 &d_alpha_beta,
			RelativeLocation& beta_rel_location,
			double& beta_angle_rel,
			double& d_alpha_beta_angle
	);

	/// @brief Stores pose of point that belongs to the robot's footprint that is closest to the target
	Robot robot_;
	/// @brief Stores poses that belong to the robot's footprint and obstacle's model and are closest to each other
	std::vector<StaticObject> obstacle_static_;
	std::vector<DynamicObject> obstacle_dynamic_;

private:
	template<typename T>
	double getDistanceClosest(const std::vector<T>& objects) const {
		if (objects.size() == 0) {
			return std::numeric_limits<double>::max();
		}
		auto it = std::min_element(objects.begin(), objects.end());
		// check if found (just in case)
		if (it == objects.end()) {
			return std::numeric_limits<double>::max();
		}
		return it->dist;
	}
};

} /* namespace sfm */

#endif /* INCLUDE_HUBERO_LOCAL_PLANNER_SFM_WORLD_H_ */
