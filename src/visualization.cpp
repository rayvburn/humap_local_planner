/*
 * visualization.cpp
 *
 *  Created on: Mar 3, 2021
 *      Author: rayvburn
 */

#include <hubero_local_planner/visualization.h>

namespace hubero_local_planner {

Visualization::Visualization(const std::string& frame): marker_stack_height_(0.0) {
	marker_force_.init(frame);
	marker_behaviour_.init(frame);
	marker_closest_pts_.init(frame);
	marker_path_.header.frame_id = frame;
	marker_force_grid_.init(frame);

	// const parameters
	marker_behaviour_.setParameters(0.5f);

	// colors
	marker_behaviour_.setColor(0.9f, 0.9f, 0.9f, 0.95f);
	marker_closest_pts_.setColor(1.0f, 1.0f, 0.0f, 0.7f);
	marker_force_grid_.setColor(0.2f, 1.0f, 0.0f, 0.7f);
}

void Visualization::initialize(ros::NodeHandle& nh) {
	static const std::string PREFIX = "vis/";
	pub_marker_ = nh.advertise<visualization_msgs::Marker>(PREFIX + "marker", 3);
	pub_marker_array_ = nh.advertise<visualization_msgs::MarkerArray>(PREFIX + "marker_array", 3);

	pub_path_ = nh.advertise<nav_msgs::Path>(PREFIX + "path", 3);
	pub_closest_dist_ = nh.advertise<std_msgs::Float32>(PREFIX + "dist_obstacle", 3);
}

void Visualization::reconfigure(const double& max_force, const double& marker_stack_height) {
	// let the force marker be always the same length
	marker_force_.setParameters(1.0, max_force);
	// grid resolution hard-coded here
	marker_force_grid_.setParameters(1.0, max_force);
	marker_stack_height_ = marker_stack_height;
}

bool Visualization::publishForceInternal(const Vector3& pos, const Vector3& force) {
	if (pub_marker_.getNumSubscribers() == 0) {
		return false;
	}

	// internal force vector - green
	marker_force_.setNamespace("force_internal");
	marker_force_.setColor(0.0f, 1.0f, 0.0f, 1.0f);
	auto marker = marker_force_.create(Vector3(pos.X(), pos.Y(), marker_stack_height_ + 2 * MARKER_GAP), force);
	pub_marker_.publish(marker);

	return true;
}
bool Visualization::publishForceInteraction(const Vector3& pos, const Vector3& force) {
	if (pub_marker_.getNumSubscribers() == 0) {
		return false;
	}

	// interaction force vector - cyan
	marker_force_.setNamespace("force_interaction");
	marker_force_.setColor(0.0f, 0.8f, 1.0f, 1.0f);
	auto marker = marker_force_.create(Vector3(pos.X(), pos.Y(), marker_stack_height_ + 3 * MARKER_GAP), force);
	pub_marker_.publish(marker);

	return true;
}
bool Visualization::publishForceSocial(const Vector3& pos, const Vector3& force) {
	if (pub_marker_.getNumSubscribers() == 0) {
		return false;
	}

	// social force vector - orange
	marker_force_.setNamespace("force_social");
	marker_force_.setColor(1.0f, 0.6f, 0.0f, 1.0f);
	auto marker = marker_force_.create(Vector3(pos.X(), pos.Y(), marker_stack_height_ + 4 * MARKER_GAP), force);
	pub_marker_.publish(marker);

	return true;
}
bool Visualization::publishForceCombined(const Vector3& pos, const Vector3& force) {
	if (pub_marker_.getNumSubscribers() == 0) {
		return false;
	}

	// combined vector - red
	marker_force_.setNamespace("force_combined");
	marker_force_.setColor(1.0f, 0.0f, 0.0f, 1.0f);
	auto marker = marker_force_.create(Vector3(pos.X(), pos.Y(), marker_stack_height_), force);
	pub_marker_.publish(marker);

	return true;
}

bool Visualization::publishBehaviourActive(const Vector3& pos, const std::string& description) {
	if (pub_marker_.getNumSubscribers() == 0) {
		return false;
	}

	marker_force_.setNamespace("behaviour");
	auto marker = marker_behaviour_.create(Vector3(pos.X(), pos.Y(), marker_stack_height_ + 7 * MARKER_GAP), description);
	pub_marker_.publish(marker);
	return true;
}

bool Visualization::publishClosestPoints(const std::vector<Pose3>& pts) {
	if (pub_marker_.getNumSubscribers() == 0) {
		return false;
	}

	marker_closest_pts_.setNamespace("closest_points");
	auto marker = marker_closest_pts_.create(pts);
	pub_marker_.publish(marker);
	return true;
}

} /* namespace hubero_local_planner */
