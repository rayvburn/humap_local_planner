/*
 * world.h
 *
 *  Created on: Mar 31, 2021
 *      Author: rayvburn
 */

#ifndef INCLUDE_HUBERO_LOCAL_PLANNER_SFM_WORLD_H_
#define INCLUDE_HUBERO_LOCAL_PLANNER_SFM_WORLD_H_

// geometry
#include <hubero_local_planner/geometry/vector.h>
#include <hubero_local_planner/geometry/pose.h>
#include <hubero_local_planner/geometry/angle.h>

#include <hubero_local_planner/defines.h>

#include <utility>
#include <vector>

namespace hubero_local_planner {

struct StaticObject {
	geometry::Pose robot;
	geometry::Pose object;
	/// \brief so called d_alpha_beta - vector connecting the object with a robot (closest points of theirs footprints)
	geometry::Vector dist_v;
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
	geometry::Vector vel;
	/// \brief Translational speed of the object (i.e. length of the 2D velocity vector)
	double speed;
	/// \brief Direction of movement of the dynamic obstacles
	geometry::Angle dir_beta;
	///
	/// \brief Relative (to the alpha) location of dynamic object
	RelativeLocation rel_loc;
	/// \brief Relative (to the alpha) location of dynamic object expressed as an angle
	geometry::Angle rel_loc_angle;
	///
	/// \brief Angle of a distance vector connecting alpha and beta closest points
	geometry::Angle dist_angle;
};

// since contents are the same
typedef StaticObject Target;

struct Robot {
	/// @brief Pose of the centroid representing robot's footprint
	geometry::Pose centroid;
	/// @brief Velocity of the robot
	geometry::Vector vel;
	/// \brief Translational speed of the robot (i.e. length of the 2D velocity vector)
	double speed;
	/// @brief Stores target pose and pose of the robot's footprint that is closest to the target
	Target target;
	/// @brief Stores global goal pose and pose of the robot's footprint that is closest to the target
	Target goal;
	/// @brief Heading direction of the robot, can be either deducted from velocity or orientation
	geometry::Angle heading_dir;
};

/// @brief Helper structure useful for marking closest points between robot and object
struct Distance {
	geometry::Pose robot;
	geometry::Pose object;
};

/**
 * @brief Class named `world` as `environment` would not express clearly its contents.
 * World contains both environment and robot data.
 */
class World {
public:
	static constexpr double RELATIVE_LOCATION_FRONT_THRESHOLD = IGN_DTOR(9);

	/**
	 * @brief Dummy constructor
	 */
	World() = default;

	/**
	 * @brief This version is useful if one wants to move robot's edge point (instead of center) to the target
	 * @param robot_pose_centroid: pose of the center of the robot's footprint
	 * @param robot_pose: robot pose that is closest to the target
	 * @param robot_vel: velocity of the robot
	 * @param target_pose: target pose
	 * @param goal_pose: global goal pose
	 */
	World(
			const geometry::Pose& robot_pose_centroid,
			const geometry::Pose& robot_pose,
			const geometry::Vector& robot_vel,
			const geometry::Pose& target_pose,
			const geometry::Pose& goal_pose
	);

	/**
	 *
	 * @param robot_pose: pose of the center of the robot's footprint
	 * @param robot_vel: velocity of the robot
	 * @param target_pose: target pose
	 * @param goal_pose: global goal pose
	 */
	World(
			const geometry::Pose& robot_pose,
			const geometry::Vector& robot_vel,
			const geometry::Pose& target_pose,
			const geometry::Pose& goal_pose
	);

	/**
	 * @brief Adds obstacle with given parameters and performs necessary calculations
	 * @param robot_pose_closest
	 * @param obstacle_pose_closest
	 * @param obstacle_vel
	 * @param force_dynamic_type: if set to True, obstacle interaction will be treated as dynamic one
	 */
	void addObstacle(
			const geometry::Pose& robot_pose_closest,
			const geometry::Pose& obstacle_pose_closest,
			const geometry::Vector& obstacle_vel,
			bool force_dynamic_type = false
	);

	/**
	 * @brief Clears up existing obstacles
	 * @param robot_pose_closest
	 * @param obstacle_pose_closest
	 * @param obstacle_vel
	 * @param force_dynamic_type: if set to True, obstacle interaction will be treated as dynamic one
	 */
	void addObstacles(
			const std::vector<geometry::Pose>& robot_pose_closest,
			const std::vector<geometry::Pose>& obstacle_pose_closest,
			const std::vector<geometry::Vector>& obstacle_vel,
			bool force_dynamic_type = false
	);

    /**
     * @brief Computes prediction of the world state for @ref sim_period forward
     * @details Modifies class instance
     */
    void predict(const geometry::Vector& robot_vel, const double& sim_period);

	inline const Robot& getRobotData() const {
		return robot_;
	}
	inline const std::vector<StaticObject>& getStaticObjectsData() const {
		return obstacle_static_;
	}
	inline const std::vector<DynamicObject>& getDynamicObjectsData() const {
		return obstacle_dynamic_;
	}

	/// \brief Returns a distance to the closest
	/// static obstacle based on the world configuration
	/// valid in the last algorithm iteration.
	inline double getDistanceClosestStaticObject() const {
		return getDistanceClosest(obstacle_static_);
	}
	/// \brief Returns a distance to the closest
	/// dynamic obstacle based on the world configuration
	/// valid in the last algorithm iteration.
	inline double getDistanceClosestDynamicObject() const {
		return getDistanceClosest(obstacle_dynamic_);
	}

	virtual ~World() = default;

protected:
	StaticObject createObstacleStatic(
			const geometry::Pose& robot_pose_closest,
			const geometry::Pose& obstacle_pose_closest
	) const;
	DynamicObject createObstacleDynamic(
			const geometry::Pose& robot_pose_closest,
			const geometry::Pose& obstacle_pose_closest,
			const geometry::Vector& obstacle_vel
	) const;
	Target createTarget(const geometry::Pose& robot_pose, const geometry::Pose& target_pose) const;

	/// \brief Helper function which calculates a relative
	/// location of the investigated object based on actor's
	/// facing direction (in this case it is equal to a movement
	/// direction)
	/// \return A 3-element tuple consisting of:
	/// - relative location of the \beta object (relative to \f$\alpha\f$'s direction), see \ref RelativeLocation
	/// - relative location expressed as an angle (radians)
	/// - angle of the vector connecting \f$\alpha\f$ and \beta
	void computeObjectRelativeLocation(
			const geometry::Angle& actor_yaw,
			const geometry::Vector& d_alpha_beta,
			RelativeLocation& beta_rel_location,
			geometry::Angle& beta_angle_rel,
			geometry::Angle& d_alpha_beta_angle
	) const;

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

} /* namespace hubero_local_planner */

#endif /* INCLUDE_HUBERO_LOCAL_PLANNER_SFM_WORLD_H_ */
