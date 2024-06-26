/*
 * Text.cpp
 *
 *  Created on: Sep 27, 2019
 *      Author: rayvburn
 */

#include <humap_local_planner/vis/text.h>
#include <string>

namespace humap_local_planner {
namespace vis {

// ------------------------------------------------------------------- //

Text::Text(): text_size_(0.5) {}

// ------------------------------------------------------------------- //

void Text::setParameters(const float& text_size) {
	text_size_ = text_size;
}

// ------------------------------------------------------------------- //

visualization_msgs::Marker Text::create(const Vector& pos, const int& number) const {

	return (create(pos, std::to_string(number)));

}

// ------------------------------------------------------------------- //

visualization_msgs::Marker Text::create(const Vector& pos, const std::string& text) const {

	visualization_msgs::Marker marker;

	// NOTE: header.stamp deprecated here
	// NOTE: marker.id is necessary for MarkerArray (otherwise only 1 marker will be drawn)
	// `ADD is something of a misnomer, it really means "create or modify"`
	// http://docs.ros.org/kinetic/api/visualization_msgs/html/msg/Marker.html

	marker.header.frame_id = this->frame_;
	marker.ns = this->namespace_;
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker.action = visualization_msgs::Marker::ADD;
	marker.lifetime = ros::Duration(1.0);

	// assign marker coordinates according to current point that is pointed by grid index
	marker.pose.position.x = pos.getX();
	marker.pose.position.y = pos.getY();
	marker.pose.position.z = pos.getZ();

	// scale - height of the letter
	marker.scale.z = text_size_;

	marker.color = this->color_;

	marker.text = text;

	return (marker);

}

// ------------------------------------------------------------------- //

Text::~Text() { }

// ------------------------------------------------------------------- //

} /* namespace vis */
} /* namespace humap_local_planner */
