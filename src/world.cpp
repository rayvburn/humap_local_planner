/*
 * environment.cpp
 *
 *  Created on: Mar 31, 2021
 *      Author: rayvburn
 */

#include <hubero_local_planner/world.h>
#include <hubero_local_planner/utils/transformations.h>

namespace hubero_local_planner {

World::World(
		const geometry::Pose& robot_pose_centroid,
		const geometry::Pose& robot_pose,
		const geometry::Vector& robot_vel,
		const geometry::Pose& target_pose,
		const geometry::Pose& goal_pose
) {
	robot_.centroid = robot_pose_centroid;
	robot_.vel = robot_vel;

	// to determine heading direction, use raw orientation or rely on velocity vector (if it's significant)
	auto robot_vel_xy = geometry::Vector(robot_vel.getX(), robot_vel.getY(), 0.0);
	if (robot_vel_xy.calculateLength() <= 1e-04) {
		robot_.heading_dir = geometry::Angle(robot_.centroid.getYaw());
	} else {
		robot_.heading_dir = robot_vel_xy.calculateDirection();
	}

	robot_.target = createTarget(robot_pose, target_pose);
	robot_.goal = createTarget(robot_pose, goal_pose);
}

World::World(
		const geometry::Pose& robot_pose,
		const geometry::Vector& robot_vel,
		const geometry::Pose& target_pose,
		const geometry::Pose& goal_pose
): World(robot_pose, robot_pose, robot_vel, target_pose, goal_pose) {}

void World::addObstacle(
		const geometry::Pose& robot_pose_closest,
		const geometry::Pose& obstacle_pose_closest,
		const geometry::Vector& obstacle_vel,
		bool force_dynamic_type
) {
	if (force_dynamic_type || obstacle_vel.calculateLength() > 1e-06) {
		obstacle_dynamic_.push_back(
			createObstacleDynamic(
				robot_pose_closest,
				obstacle_pose_closest,
				obstacle_vel)
		);
	} else {
		obstacle_static_.push_back(
			createObstacleStatic(
				robot_pose_closest,
				obstacle_pose_closest)
		);
	}
}

void World::addObstacles(
		const std::vector<geometry::Pose>& robot_pose_closest,
		const std::vector<geometry::Pose>& obstacle_pose_closest,
		const std::vector<geometry::Vector>& obstacle_vel,
		bool force_dynamic_type
) {
	if ((robot_pose_closest.size() != obstacle_pose_closest.size())
		|| (obstacle_pose_closest.size() != obstacle_vel.size())
	) {
		printf("[World::addObstacles] vectors lengths are not equal! \r\n");
		return;
	}

	obstacle_static_.clear();
	obstacle_dynamic_.clear();

	for (size_t i = 0; i < robot_pose_closest.size(); i++) {
		addObstacle(robot_pose_closest.at(i), obstacle_pose_closest.at(i), obstacle_vel.at(i), force_dynamic_type);
	}
}

void World::predict(const geometry::Vector& robot_vel, const double& sim_period) {
	auto robot_centroid_new = computeNextPose(robot_.centroid, robot_vel, sim_period);
	// NOTE: for unknown reason, result of poses difference using ignition library ignored Y-dimension difference
	auto robot_centroid_diff = subtractPoses(robot_centroid_new, robot_.centroid);
	auto robot_pose_new = addPoses(robot_.target.robot, robot_centroid_diff);

	auto world_prediction = World(
		robot_centroid_new,
		robot_pose_new,
		robot_vel,
		robot_.target.object,
		robot_.goal.object
	);

	// only dynamic obstacles will change their poses - static ones do not need to be changed
	for (auto& obstacle: obstacle_dynamic_) {
		auto robot_new_pose = addPoses(obstacle.robot, robot_centroid_diff);
		geometry::Pose obstacle_new_pose = computeNextPose(obstacle.object, obstacle.vel, sim_period);
		world_prediction.addObstacle(robot_new_pose, obstacle_new_pose, obstacle.vel);
	}

	for (auto& obstacle: obstacle_static_) {
		auto robot_new_pose = addPoses(obstacle.robot, robot_centroid_diff);
		world_prediction.addObstacle(robot_new_pose, obstacle.object, geometry::Vector(0.0, 0.0, 0.0));
	}

	// override this world instance
	*this = world_prediction;
}

StaticObject World::createObstacleStatic(
		const geometry::Pose& robot_pose_closest,
		const geometry::Pose& obstacle_pose_closest
) const {
	StaticObject obstacle;
	obstacle.robot = robot_pose_closest;
	obstacle.object = obstacle_pose_closest;
	obstacle.dist_v = geometry::Vector(obstacle.object.getRawPosition() - obstacle.robot.getRawPosition());
	// NOTE: in SFM calculations it is assumed that all objects are in the actor's plane
	obstacle.dist_v.setZ(0.0);
	obstacle.dist = obstacle.dist_v.calculateLength();

	return obstacle;
}

Target World::createTarget(const geometry::Pose& robot_pose, const geometry::Pose& target_pose) const {
	Target target;
	target.robot = robot_pose;
	target.object = target_pose;
	target.dist_v = geometry::Vector(target_pose.getRawPosition() - robot_pose.getRawPosition());
	// assume planar poses
	target.dist_v.setZ(0.0);
	target.dist = target.dist_v.calculateLength();
	return target;
}

DynamicObject World::createObstacleDynamic(
		const geometry::Pose& robot_pose_closest,
		const geometry::Pose& obstacle_pose_closest,
		const geometry::Vector& obstacle_vel
) const {
	DynamicObject obstacle;
	obstacle.robot = robot_pose_closest;
	obstacle.object = obstacle_pose_closest;
	obstacle.dist_v = geometry::Vector(obstacle.object.getRawPosition() - obstacle.robot.getRawPosition());
	// it is assumed that all objects are located in the plane
	obstacle.dist_v.setZ(0.0);
	obstacle.dist = obstacle.dist_v.calculateLength();

	geometry::Angle robot_yaw(robot_pose_closest.getYaw());

	RelativeLocation beta_rel_location = LOCATION_UNSPECIFIED;
	geometry::Angle beta_angle_rel;
	geometry::Angle d_alpha_beta_angle;
	computeObjectRelativeLocation(robot_yaw, obstacle.dist_v, beta_rel_location, beta_angle_rel, d_alpha_beta_angle);

	obstacle.vel = obstacle_vel;
	obstacle.dir_beta = obstacle_vel.calculateDirection();
	obstacle.rel_loc = beta_rel_location;
	obstacle.rel_loc_angle = beta_angle_rel;
	obstacle.dist_angle = d_alpha_beta_angle;

	return obstacle;
}

void World::computeObjectRelativeLocation(
			const geometry::Angle& robot_yaw,
			const geometry::Vector& d_alpha_beta,
			RelativeLocation& beta_rel_location,
			geometry::Angle &beta_angle_rel,
			geometry::Angle &d_alpha_beta_angle
) const {
	RelativeLocation rel_loc = LOCATION_UNSPECIFIED;
	geometry::Angle angle_d_alpha_beta(d_alpha_beta);				  // stores dir of d_alpha_beta
	geometry::Angle angle_relative(angle_d_alpha_beta - robot_yaw); // relative to actor's (alpha) direction

	/* SFM FRONT or SFM_BACK to be specific - added exponentially decreasing
	 * interaction for objects that are behind so relative angle calculations
	 * extended with close to Pi value case */
	// TODO: static constexpr
	if ( std::fabs(angle_relative.getRadian()) <= World::RELATIVE_LOCATION_FRONT_THRESHOLD ||
		 std::fabs(angle_relative.getRadian()) >= (IGN_PI - World::RELATIVE_LOCATION_FRONT_THRESHOLD) ) {
		rel_loc = LOCATION_FRONT;
	/* // LOCATION_BEHIND DEPRECATED HERE
	} else if (IsOutOfFOV(angle_relative.Radian())) { // consider FOV
		rel_loc = LOCATION_BEHIND;
	*/
	} else if ( angle_relative.getRadian() <= 0.0 ) { // 0.0 ) {
		rel_loc = LOCATION_RIGHT;
	} else if ( angle_relative.getRadian() > 0.0 ) { // 0.0 ) {
		rel_loc = LOCATION_LEFT;
	}

	/* historical data considered when calculating relative location - it is crucial to not
	 * allow minor yaw rotations to switch from left to right etc. thus a relative angle
	 * must exceed certain threshold before it will be in fact switched;
	 * LOCATION_BEHIND must not be found in map_models_rel_locations! */

	// update outputs
	beta_rel_location = rel_loc;
	beta_angle_rel = angle_relative;
	d_alpha_beta_angle = angle_d_alpha_beta;
}

} /* namespace hubero_local_planner */
