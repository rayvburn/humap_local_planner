/*
 * LineList.cpp
 *
 *  Created on: Sep 27, 2019
 *      Author: rayvburn
 */

#include <humap_local_planner/vis/line_list.h>

namespace humap_local_planner {
namespace vis {

// ------------------------------------------------------------------- //

LineList::LineList(): line_id_max_(0) {}

// ------------------------------------------------------------------- //

visualization_msgs::MarkerArray LineList::createArray(const std::vector<Pose> &poses) {

	visualization_msgs::MarkerArray array;

	/* check is there is even number of elements */
	if ( poses.size() % 2 != 0 ) {
		std::cout << "createLineListArray() - odd number of elements in vector but MUST be even!" << std::endl;
		return (array);
	}

	/* fill a marker array with lines */
	for ( size_t i = 0; i < poses.size(); i=i+2 ) {
		array.markers.push_back( create(poses.at(i), poses.at(i+1), i) );
	}

	/* FIXME: it seems that rViz somehow stores all markers internally
	 * because when some marker of a certain ID is deleted an application
	 * throws a bad_alloc exception;
	 * Deletion is needed to get rid of those lines which indicate
	 * objects which create 0 force on current actor;
	 * temporarily a blank lines will be left - FIXME issue */

	/* check if a maximum line ID is bigger than a poses' size */
	if ( (line_id_max_ > (poses.size() - 1)) || (line_id_max_ > 0 && poses.size() == 0) ) {

		// NOTE: if poses.size() == 0 then the 1st condition is not fulfilled -
		// thus a second condition added to get rid of old marker instances

		/* copy the size value, as after entering the for loop
		 * the size of a vector will be increased */
		size_t lines_size_backup = 0;
		if ( poses.size() > 0 ) {
			lines_size_backup = poses.size() - 1;
		}

		/* if true, then fill the marker array with blank markers (action DELETE);
		 * start `for` loop with poses.size() index because (size() - 1) objects
		 * were already created */
		visualization_msgs::Marker marker;
		marker.header.frame_id = frame_;
		marker.ns = namespace_;
		marker.type = visualization_msgs::Marker::LINE_LIST;
		marker.action = visualization_msgs::Marker::DELETE;

		for ( size_t i = poses.size(); i <= line_id_max_; i++ ) {

			marker.id = i;
			array.markers.push_back(marker);

		}

		// save a new max ID of a lines array
		line_id_max_ = lines_size_backup;

	} else {

		// save a new max ID of a lines array
		if ( poses.size() != 0 ) {
			line_id_max_ = poses.size() - 1;
		} else {
			line_id_max_ = 0;
		}

	}

	return (array);

}

// ------------------------------------------------------------------- //

visualization_msgs::Marker LineList::create(const std::vector<Pose> &poses) {
	visualization_msgs::Marker marker;

	marker.header.frame_id = frame_;
	marker.ns = namespace_;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.lifetime = ros::Duration(1.0);

	marker.pose.orientation.w = 1.0;

	// line width
	marker.scale.x = 0.05;

	marker.color = color_;

	geometry_msgs::Point pt;
	for (const auto& pose: poses) {
		pt.x = pose.getX();
		pt.y = pose.getY();
		pt.z = pose.getZ();
		marker.points.push_back(pt);
	}
	return marker;
}

// ------------------------------------------------------------------- //

visualization_msgs::Marker LineList::create(const Pose &p1, const Pose &p2, const unsigned int &line_id) const {
	return ( create(p1.getPosition(), p2.getPosition(), line_id) );
}

// ------------------------------------------------------------------- //

visualization_msgs::Marker LineList::create(const Vector &p1, const Vector &p2, const unsigned int &line_id) const {

	visualization_msgs::Marker marker;

	// `ADD is something of a misnomer, it really means "create or modify"`
	// http://docs.ros.org/kinetic/api/visualization_msgs/html/msg/Marker.html

	marker.header.frame_id = frame_;
	marker.ns = namespace_;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.lifetime = ros::Duration(1.0);
	marker.id = line_id;

	marker.pose.orientation.w = 1.0;

	// line width
	marker.scale.x = 0.05;

	marker.color = this->color_; // marker.color = color_line_;

	geometry_msgs::Point point;

	point.x = p1.getX();
	point.y = p1.getY();
	point.z = 0.0f;

	marker.points.push_back(point);

	point.x = p2.getX();
	point.y = p2.getY();
	point.z = 0.0f;

	marker.points.push_back(point);

	return (marker);

}

// ------------------------------------------------------------------- //

LineList::~LineList() {}

// ------------------------------------------------------------------- //

} /* namespace vis */
} /* namespace humap_local_planner */
