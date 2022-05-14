/*
 * Arrow.cpp
 *
 *  Created on: Sep 27, 2019
 *      Author: rayvburn
 */

#include <hubero_local_planner/vis/arrow.h>

namespace hubero_local_planner {
namespace vis {

Arrow::Arrow(): max_length_(1.0f), sfm_max_force_(2000.0) {}

// ------------------------------------------------------------------- //

void Arrow::setParameters(const float& length_meters, const float& sfm_max_force) {
	max_length_ = length_meters;
	sfm_max_force_ = sfm_max_force;
}

// ------------------------------------------------------------------- //

visualization_msgs::Marker Arrow::create(const Vector& pos, const Vector& vector) const {

	visualization_msgs::Marker marker;

	// NOTE: header.stamp, ns, deprecated here
	// NOTE: marker.id is necessary for MarkerArray (otherwise only 1 marker will be drawn)
	// `ADD is something of a misnomer, it really means "create or modify"`
	// http://docs.ros.org/kinetic/api/visualization_msgs/html/msg/Marker.html

	marker.header.frame_id = frame_;
	marker.ns = namespace_;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.lifetime = ros::Duration(1.0);

	// assign marker coordinates according to current point that is pointed by grid index
	marker.pose.position.x = pos.getX();
	marker.pose.position.y = pos.getY();
	marker.pose.position.z = pos.getZ();

	// marker orientation is based on force vector direction
	Angle yaw(vector);

	// convert to quaternion
	Quaternion quaternion(yaw.getRadian());

	marker.pose.orientation.x = quaternion.getX();
	marker.pose.orientation.y = quaternion.getY();
	marker.pose.orientation.z = quaternion.getZ();
	marker.pose.orientation.w = quaternion.getW();

	// scale
	// arrow's length is calculated based on max allowable force `in SFM class`
	double length = max_length_ * vector.calculateLength() / sfm_max_force_;
	marker.scale.x = length > 0.001 ? length : 0.001;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;

	marker.color = color_;

	return (marker);

}

// ------------------------------------------------------------------- //

Arrow::~Arrow() {
	// TODO Auto-generated destructor stub
}

// ------------------------------------------------------------------- //

} /* namespace vis */
} /* namespace hubero_local_planner */
