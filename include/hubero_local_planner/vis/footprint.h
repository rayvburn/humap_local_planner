/*
 * footprint.h
 *
 *  Created on: Apr 14, 2021
 *      Author: rayvburn
 */

#ifndef INCLUDE_HUBERO_LOCAL_PLANNER_VIS_FOOTPRINT_H_
#define INCLUDE_HUBERO_LOCAL_PLANNER_VIS_FOOTPRINT_H_

#include <hubero_local_planner/vis/marker_base.h>
#include <visualization_msgs/MarkerArray.h>
#include <hubero_local_planner/robot_footprint_model.h>

namespace vis {

class Footprint : public MarkerBase {
public:
	Footprint();
	virtual void setHeight(const double& height);
	virtual visualization_msgs::MarkerArray create(
			const hubero::geometry::Pose& pos_current,
			const hubero_local_planner::RobotFootprintModelConstPtr footprint) const;
	virtual ~Footprint() = default;
protected:
	double height_;
};

} /* namespace vis */

#endif /* INCLUDE_HUBERO_LOCAL_PLANNER_VIS_FOOTPRINT_H_ */
