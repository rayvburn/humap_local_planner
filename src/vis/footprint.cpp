/*
 * footprint.cpp
 *
 *  Created on: Apr 14, 2021
 *      Author: rayvburn
 */

#include <hubero_local_planner/vis/footprint.h>

namespace hubero_local_planner {
namespace vis {

Footprint::Footprint(): MarkerBase::MarkerBase(), height_(0.0) {
}

void Footprint::setHeight(const double& height) {
	height_ = height;
}

visualization_msgs::MarkerArray Footprint::create(
		const Pose& pos_current,
		const RobotFootprintModelConstPtr footprint) const
{
	teb_local_planner::PoseSE2 pose = pos_current.getAsTebPose();
	visualization_msgs::MarkerArray markers;
	footprint->visualizeRobot(pose, markers.markers, color_);

	for (auto& marker: markers.markers) {
		marker.header.frame_id = frame_;
		marker.ns = namespace_;
		marker.action = visualization_msgs::Marker::ADD;
		// overwrite height if set to some reasonable value
		if (height_ > 1e-06) {
			marker.scale.z = height_;
			marker.pose.position.z = height_ / 2.0;
		}
	}
	return markers;
}


} /* namespace vis */
} /* namespace hubero_local_planner */
