/*
 * footprint.h
 *
 *  Created on: Apr 14, 2021
 *      Author: rayvburn
 */

#ifndef INCLUDE_HUMAP_LOCAL_PLANNER_VIS_FOOTPRINT_H_
#define INCLUDE_HUMAP_LOCAL_PLANNER_VIS_FOOTPRINT_H_

#include <humap_local_planner/vis/marker_base.h>
#include <visualization_msgs/MarkerArray.h>
#include <humap_local_planner/robot_footprint_model.h>

namespace humap_local_planner {
namespace vis {

using namespace geometry;
class Footprint : public MarkerBase {
public:
	Footprint();
	virtual void setHeight(const double& height);
	virtual visualization_msgs::MarkerArray create(
			const Pose& pos_current,
			const RobotFootprintModelConstPtr footprint) const;
	virtual ~Footprint() = default;
protected:
	double height_;
};

} /* namespace vis */
} /* namespace humap_local_planner */

#endif /* INCLUDE_HUMAP_LOCAL_PLANNER_VIS_FOOTPRINT_H_ */
