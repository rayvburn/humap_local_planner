/*
 * point.cpp
 *
 *  Created on: Apr 15, 2021
 *      Author: rayvburn
 */

#include <hubero_local_planner/vis/point.h>

namespace hubero_local_planner {
namespace vis {

Point::Point():
	size_(0.1) {
}

void Point::setSize(const double& side_length) {
	size_ = side_length;
}

visualization_msgs::Marker Point::create(const Vector& pos) const {
	visualization_msgs::Marker marker;

	// NOTE: header.stamp, ns, deprecated here
	// NOTE: marker.id is necessary for MarkerArray (otherwise only 1 marker will be drawn)
	// `ADD is something of a misnomer, it really means "create or modify"`
	// http://docs.ros.org/kinetic/api/visualization_msgs/html/msg/Marker.html

	marker.header.frame_id = frame_;
	marker.ns = namespace_;
	marker.type = visualization_msgs::Marker::POINTS;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.orientation.w = 1.0;

	// scale
	// POINTS markers use x and y scale for width/height respectively
	marker.scale.x = size_ / 2.0;
	marker.scale.y = size_ / 2.0;

	marker.color = color_;

	geometry_msgs::Point p;
	p.x = pos.getX();
	p.y = pos.getY();
	p.z = pos.getZ();

	marker.points.push_back(p);

	return (marker);
}

} /* namespace vis */
} /* namespace hubero_local_planner */
