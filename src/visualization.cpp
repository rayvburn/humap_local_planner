/*
 * visualization.cpp
 *
 *  Created on: Mar 3, 2021
 *      Author: rayvburn
 */

#include <humap_local_planner/visualization.h>

namespace humap_local_planner {

Visualization::Visualization(const std::string& frame, const double& marker_stack_height)
	: marker_stack_height_(marker_stack_height) {
	marker_force_.init(frame);
	marker_behaviour_.init(frame);
	marker_state_.init(frame);
	marker_closest_pts_static_.init(frame);
	marker_closest_pts_dynamic_.init(frame);
	marker_path_.header.frame_id = frame;
	marker_force_grid_.init(frame);
	marker_footprint_.init(frame);
	marker_point_.init(frame);

	marker_path_.header.frame_id = frame;

	// const parameters
	marker_behaviour_.setParameters(0.25f);
	marker_state_.setParameters(0.2f);
	marker_footprint_.setHeight(1.2f);
	marker_point_.setSize(0.25f);

	// colors
	marker_behaviour_.setColor(0.9f, 0.9f, 0.9f, 0.95f);
	marker_state_.setColor(0.0f, 0.0f, 0.0f, 1.0f);
	marker_closest_pts_static_.setColor(1.0f, 1.0f, 0.0f, 0.7f);
	marker_closest_pts_dynamic_.setColor(1.0f, 0.3f, 0.0f, 0.7f);
	marker_force_grid_.setColor(0.2f, 1.0f, 0.0f, 0.7f);
	marker_footprint_.setColor(1.0f, 1.0f, 1.0f, 0.65f);

}

void Visualization::initialize(ros::NodeHandle& nh) {
	static const std::string PREFIX = "vis/";
	pub_marker_ = nh.advertise<visualization_msgs::Marker>(PREFIX + "marker", 1);
	pub_marker_array_ = nh.advertise<visualization_msgs::MarkerArray>(PREFIX + "marker_array", 1);
	pub_grid_ = nh.advertise<visualization_msgs::MarkerArray>(PREFIX + "force_grid", 1);

	pub_path_ = nh.advertise<nav_msgs::Path>(PREFIX + "path", 3);
	pub_closest_dist_ = nh.advertise<std_msgs::Float32>(PREFIX + "dist_obstacle", 1);
}

void Visualization::reconfigure(const double& max_force) {
	// let the force marker be always the same length
	marker_force_.setParameters(1.0, max_force);
	// grid resolution hard-coded here
	marker_force_grid_.setParameters(1.0, max_force);

	marker_force_grid_.setParameters(1.0, max_force);
}

bool Visualization::publishForceInternal(const Vector& pos, const Vector& force) {
	if (pub_marker_.getNumSubscribers() == 0) {
		return false;
	}

	// internal force vector - green
	marker_force_.setNamespace("force_internal");
	marker_force_.setColor(0.0f, 1.0f, 0.0f, 1.0f);
	auto marker = marker_force_.create(Vector(pos.getX(), pos.getY(), marker_stack_height_ + 2 * MARKER_GAP), force);
	pub_marker_.publish(marker);

	return true;
}
bool Visualization::publishForceInteraction(const Vector& pos, const Vector& force) {
	if (pub_marker_.getNumSubscribers() == 0) {
		return false;
	}

	// interaction force vector - cyan
	marker_force_.setNamespace("force_interaction");
	marker_force_.setColor(0.0f, 0.8f, 1.0f, 1.0f);
	auto marker = marker_force_.create(Vector(pos.getX(), pos.getY(), marker_stack_height_ + 3 * MARKER_GAP), force);
	pub_marker_.publish(marker);

	return true;
}
bool Visualization::publishForceSocial(const Vector& pos, const Vector& force) {
	if (pub_marker_.getNumSubscribers() == 0) {
		return false;
	}

	// social force vector - orange
	marker_force_.setNamespace("force_social");
	marker_force_.setColor(1.0f, 0.6f, 0.0f, 1.0f);
	auto marker = marker_force_.create(Vector(pos.getX(), pos.getY(), marker_stack_height_ + 4 * MARKER_GAP), force);
	pub_marker_.publish(marker);

	return true;
}
bool Visualization::publishForceCombined(const Vector& pos, const Vector& force) {
	if (pub_marker_.getNumSubscribers() == 0) {
		return false;
	}

	// combined vector - red
	marker_force_.setNamespace("force_combined");
	marker_force_.setColor(1.0f, 0.0f, 0.0f, 1.0f);
	auto marker = marker_force_.create(Vector(pos.getX(), pos.getY(), marker_stack_height_), force);
	pub_marker_.publish(marker);

	return true;
}

bool Visualization::publishBehaviourActive(const Vector& pos, const std::string& description) {
	if (pub_marker_.getNumSubscribers() == 0) {
		return false;
	}

	marker_behaviour_.setNamespace("behaviour");
	auto marker = marker_behaviour_.create(Vector(pos.getX(), pos.getY(), marker_stack_height_ + 7 * MARKER_GAP), description);
	pub_marker_.publish(marker);
	return true;
}

bool Visualization::publishClosestPoints(const std::vector<Pose>& pts_static, const std::vector<Pose>& pts_dynamic) {
	if (pub_marker_.getNumSubscribers() == 0) {
		return false;
	}

	// do not publish if there is no points inside the marker, also avoid rviz complains
	if (!pts_static.empty()) {
		marker_closest_pts_static_.setNamespace("closest_static_points");
		auto marker_static = marker_closest_pts_static_.create(pts_static);
		pub_marker_.publish(marker_static);
	}

	if (!pts_dynamic.empty()) {
		marker_closest_pts_dynamic_.setNamespace("closest_dynamic_points");
		auto marker_dynamic = marker_closest_pts_dynamic_.create(pts_dynamic);
		pub_marker_.publish(marker_dynamic);
	}
	return true;
}

bool Visualization::publishVelocity(
		const Vector& pos_start,
		const double& angle_lin,
		const double& linear_x,
		const Vector& pos_lin_end,
		const double& angle_ang,
		const double& angular_z
) {
	if (!isPositionValid(pos_start)) {
		return false;
	}
	if (!isPositionValid(pos_lin_end)) {
		return false;
	}
	if (pub_marker_array_.getNumSubscribers() == 0) {
		return false;
	}

	visualization_msgs::MarkerArray marker_array;
	std_msgs::ColorRGBA color;
	color.a = 1.0;
	color.b = 1.0;
	visualization_msgs::Marker marker;

	marker.header.frame_id = "map";
	marker.ns = "velocity";
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.lifetime = ros::Duration(1.0);

	marker.scale.x = linear_x;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	marker.color = color;

	marker.pose.position.x = pos_start.getX();
	marker.pose.position.y = pos_start.getY();
	marker.pose.position.z = marker_stack_height_ - 1 * MARKER_GAP;

	// convert to quaternion
	Quaternion quaternion(angle_lin);
	marker.pose.orientation.x = quaternion.getX();
	marker.pose.orientation.y = quaternion.getY();
	marker.pose.orientation.z = quaternion.getZ();
	marker.pose.orientation.w = quaternion.getW();

	// first marker
	marker_array.markers.push_back(marker);

	marker.scale.x = angular_z;

	marker.pose.position.x = pos_lin_end.getX();
	marker.pose.position.y = pos_lin_end.getY();

	Quaternion quaternion_ang(angle_ang);
	marker.pose.orientation.x = quaternion_ang.getX();
	marker.pose.orientation.y = quaternion_ang.getY();
	marker.pose.orientation.z = quaternion_ang.getZ();
	marker.pose.orientation.w = quaternion_ang.getW();

	marker_array.markers.push_back(marker);

	// second marker
	pub_marker_array_.publish(marker_array);
	return true;
}

bool Visualization::publishPath(const Pose& new_pos) {
	if (!isPositionValid(new_pos.getPosition())) {
		return false;
	}

	marker_path_.header.seq++;
	marker_path_.header.stamp = ros::Time::now();

	geometry_msgs::PoseStamped pose;
	pose.header = marker_path_.header;

	pose.pose.position.x = new_pos.getX();
	pose.pose.position.y = new_pos.getY();
	pose.pose.position.z = new_pos.getZ();

	pose.pose.orientation.x = new_pos.getQuaternionX();
	pose.pose.orientation.y = new_pos.getQuaternionY();
	pose.pose.orientation.z = new_pos.getQuaternionZ();
	pose.pose.orientation.w = new_pos.getQuaternionW();

	marker_path_.poses.push_back(pose);

	if (pub_path_.getNumSubscribers() == 0) {
		return false;
	}

	pub_path_.publish(marker_path_);
	return true;
}

void Visualization::resetPath() {
	marker_path_.poses.clear();
}

bool Visualization::publishGrid(
		const Pose& pos_current,
		HumapPlanner& planner
) {
	if (!isPositionValid(pos_current.getPosition())) {
		return false;
	}
	if (pub_grid_.getNumSubscribers() == 0) {
		return false;
	}

	// pose where `virtual` actor will be placed in
	Pose pose = pos_current;

	marker_force_grid_.setNamespace("force_grid");

	// grid dimensions similar to the local costmap's
	marker_force_grid_.createGrid(
		pos_current.getX() - 2.0,
		pos_current.getX() + 2.0,
		pos_current.getY() - 2.0,
		pos_current.getY() + 2.0,
		0.5
	);

	// before a start, reset a grid index
	marker_force_grid_.resetGridIndex();

	while (!marker_force_grid_.isWholeGridChecked()) {
		// set an actor's virtual pose
		pose.setPosition(marker_force_grid_.getNextGridElement());

		// calculate social force for actor located in current pose hard-coded time delta
		auto force = planner.computeForceAtPosition(pose.getPosition());

		// pass a result to vector of grid forces
		marker_force_grid_.addMarker(marker_force_grid_.create(pose.getPosition(), force));
	}
	pub_grid_.publish(marker_force_grid_.getMarkerArray());
	return true;
}

bool Visualization::publishRobotFootprint(
		const Pose& pos_current,
		const RobotFootprintModelConstPtr footprint
) {
	if (!isPositionValid(pos_current.getPosition())) {
		return false;
	}
	if (pub_marker_array_.getNumSubscribers() == 0) {
		return false;
	}

	marker_footprint_.setNamespace("footprint");
	auto marker_array = marker_footprint_.create(pos_current, footprint);
	pub_marker_array_.publish(marker_array);
	return true;
}

bool Visualization::publishGoalInitiation(const Vector& pos) {
	if (!isPositionValid(pos)) {
		return false;
	}
	if (pub_marker_.getNumSubscribers() == 0) {
		return false;
	}

	marker_point_.setNamespace("goal_init");
	marker_point_.setColor(0.25f, 0.25f, 0.0f, 0.65f);
	auto marker = marker_point_.create(pos);
	pub_marker_.publish(marker);
	return true;
}

bool Visualization::publishGoalLocal(const Vector& pos) {
	if (!isPositionValid(pos)) {
		return false;
	}
	if (pub_marker_.getNumSubscribers() == 0) {
		return false;
	}

	marker_point_.setNamespace("goal_local");
	marker_point_.setColor(0.5f, 0.5f, 0.0f, 0.65f);
	auto marker = marker_point_.create(pos);
	pub_marker_.publish(marker);
	return true;
}

bool Visualization::publishGoal(const Vector& pos) {
	if (!isPositionValid(pos)) {
		return false;
	}
	if (pub_marker_.getNumSubscribers() == 0) {
		return false;
	}

	marker_point_.setNamespace("goal_global");
	marker_point_.setColor(0.75f, 0.75f, 0.0f, 0.65f);
	auto marker = marker_point_.create(pos);
	pub_marker_.publish(marker);
	return true;
}

bool Visualization::publishGoalRecoveryRotateAndRecede(const Vector& pos) {
	if (!isPositionValid(pos)) {
		return false;
	}
	if (pub_marker_.getNumSubscribers() == 0) {
		return false;
	}

	marker_point_.setNamespace("goal_rr_recovery");
	marker_point_.setColor(0.75f, 0.0f, 0.0f, 0.65f);
	auto marker = marker_point_.create(pos);
	pub_marker_.publish(marker);
	return true;
}

bool Visualization::publishGoalRecoveryLookAround(const Vector& pos) {
	if (!isPositionValid(pos)) {
		return false;
	}
	if (pub_marker_.getNumSubscribers() == 0) {
		return false;
	}

	marker_point_.setNamespace("goal_la_recovery");
	marker_point_.setColor(0.75f, 0.0f, 0.0f, 0.65f);
	auto marker = marker_point_.create(pos);
	pub_marker_.publish(marker);
	return true;
}

bool Visualization::publishPlannerState(const Vector& pos, const std::string& state) {
	if (!isPositionValid(pos)) {
		return false;
	}
	if (pub_marker_.getNumSubscribers() == 0) {
		return false;
	}

	marker_state_.setNamespace("planner_state");
	// arbitrary height but above the ground
	auto marker = marker_state_.create(Vector(pos.getX(), pos.getY(), 0.15), state);
	pub_marker_.publish(marker);
	return true;
}

bool Visualization::isPositionValid(const Vector& pos) const {
	bool inf = std::isinf(pos.getX()) || std::isinf(pos.getY()) || std::isinf(pos.getZ());
	bool nan = std::isnan(pos.getX()) || std::isnan(pos.getY()) || std::isnan(pos.getZ());
	return !inf && !nan;
}

} /* namespace humap_local_planner */
