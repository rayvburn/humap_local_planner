/*
 * environment.cpp
 *
 *  Created on: Mar 31, 2021
 *      Author: rayvburn
 */

#include <hubero_local_planner/sfm/world.h>

namespace sfm {

World::World(
	const Pose3& robot_pose_centroid,
	const Pose3& robot_pose,
	const Pose3& target_pose,
	const Vector3& robot_vel
) {
	robot_.centroid = robot_pose_centroid;
	robot_.vel = robot_vel;

	robot_.target.robot = robot_pose;
	robot_.target.object = target_pose;
}

void World::addObstacle(
		const Pose3& robot_pose_closest,
		const Pose3& obstacle_pose_closest,
		const Vector3& obstacle_vel
) {
	if (obstacle_vel.Length() <= 1e-06) {
		addObstacleStatic(robot_pose_closest, obstacle_pose_closest);
	} else {
		addObstacleDynamic(robot_pose_closest, obstacle_pose_closest, obstacle_vel);
	}
}

void World::addObstacles(
		const std::vector<Pose3>& robot_pose_closest,
		const std::vector<Pose3>& obstacle_pose_closest,
		const std::vector<Vector3>& obstacle_vel
) {
	if ((robot_pose_closest.size() != obstacle_pose_closest.size())
		|| (obstacle_pose_closest.size() != obstacle_vel.size())
	) {
		printf("[World::addObstacles] vectors lengths are not equal! \r\n");
		return;
	}
	for (size_t i = 0; i < robot_pose_closest.size(); i++) {
		addObstacle(robot_pose_closest.at(i), obstacle_pose_closest.at(i), obstacle_vel.at(i));
	}
}

void World::addObstacleStatic(
		const Pose3& robot_pose_closest,
		const Pose3& obstacle_pose_closest
) {
	StaticObject obstacle;
	obstacle.robot = robot_pose_closest;
	obstacle.object = obstacle_pose_closest;
	obstacle.dist_v = obstacle.object.Pos() - obstacle.robot.Pos();
	// NOTE: in SFM calculations it is assumed that all objects are in the actor's plane
	obstacle.dist_v.Z(0.0);
	obstacle.dist = obstacle.dist_v.Length();

	obstacle_static_.push_back(obstacle);
}

void World::addObstacleDynamic(
		const Pose3& robot_pose_closest,
		const Pose3& obstacle_pose_closest,
		const Vector3& obstacle_vel
) {
	DynamicObject obstacle;
	obstacle.robot = robot_pose_closest;
	obstacle.object = obstacle_pose_closest;
	obstacle.dist_v = obstacle.object.Pos() - obstacle.robot.Pos();
	// it is assumed that all objects are located in the plane
	obstacle.dist_v.Z(0.0);
	obstacle.dist = obstacle.dist_v.Length();

	Angle robot_yaw(robot_pose_closest.Rot().Yaw());

	RelativeLocation beta_rel_location = LOCATION_UNSPECIFIED;
	double beta_angle_rel = 0.0;
	double d_alpha_beta_angle = 0.0;
	computeObjectRelativeLocation(robot_yaw, obstacle.dist_v, beta_rel_location, beta_angle_rel, d_alpha_beta_angle);

	obstacle.vel = obstacle_vel;
	obstacle.dir_beta = std::atan2(obstacle.vel.Y(), obstacle.vel.X());
	obstacle.rel_loc = beta_rel_location;
	obstacle.rel_loc_angle = beta_angle_rel;
	obstacle.dist_angle = d_alpha_beta_angle;
}

void World::computeObjectRelativeLocation(
			const Angle &robot_yaw,
			const Vector3 &d_alpha_beta,
			RelativeLocation& beta_rel_location,
			double& beta_angle_rel,
			double& d_alpha_beta_angle
) {
	RelativeLocation rel_loc = LOCATION_UNSPECIFIED;
	ignition::math::Angle angle_relative; 		// relative to actor's (alpha) direction
	ignition::math::Angle angle_d_alpha_beta;	// stores yaw of d_alpha_beta

	Vector3 d_alpha_beta_norm = d_alpha_beta;
	d_alpha_beta_norm.Normalize();

	// when normalized vector used with atan2 then division by euclidean distance not needed
	angle_d_alpha_beta.Radian(std::atan2(d_alpha_beta_norm.Y(), d_alpha_beta_norm.X())); 	// V2
	angle_d_alpha_beta.Normalize();

	angle_relative.Radian(angle_d_alpha_beta.Radian() - robot_yaw.Radian());
	angle_relative.Normalize(); // V4

	/* SFM FRONT or SFM_BACK to be specific - added exponentially decreasing
	 * interaction for objects that are behind so relative angle calculations
	 * extended with close to Pi value case */
	// TODO: static constexpr
	if ( std::fabs(angle_relative.Radian()) <= IGN_DTOR(9) ||
		 std::fabs(angle_relative.Radian()) >= (IGN_PI - IGN_DTOR(9)) ) {
		rel_loc = LOCATION_FRONT;
	/* // LOCATION_BEHIND DEPRECATED HERE
	} else if (IsOutOfFOV(angle_relative.Radian())) { // consider FOV
		rel_loc = LOCATION_BEHIND;
	*/
	} else if ( angle_relative.Radian() <= 0.0 ) { // 0.0 ) {
		rel_loc = LOCATION_RIGHT;
	} else if ( angle_relative.Radian() > 0.0 ) { // 0.0 ) {
		rel_loc = LOCATION_LEFT;
	}

	/* historical data considered when calculating relative location - it is crucial to not
	 * allow minor yaw rotations to switch from left to right etc. thus a relative angle
	 * must exceed certain threshold before it will be in fact switched;
	 * LOCATION_BEHIND must not be found in map_models_rel_locations! */

	// update outputs
	beta_rel_location = rel_loc;
	beta_angle_rel = angle_relative.Radian();
	d_alpha_beta_angle = angle_d_alpha_beta.Radian();
}

} /* namespace sfm */
