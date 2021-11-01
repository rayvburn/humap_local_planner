/*
 * point.h
 *
 *  Created on: Apr 15, 2021
 *      Author: rayvburn
 */

#ifndef INCLUDE_HUBERO_LOCAL_PLANNER_VIS_POINT_H_
#define INCLUDE_HUBERO_LOCAL_PLANNER_VIS_POINT_H_

#include "marker_base.h"

namespace hubero_local_planner {
namespace vis {

using namespace geometry;
class Point : public MarkerBase {
public:
	Point();
	void setSize(const double& side_length);
	virtual visualization_msgs::Marker create(const Vector& pos) const;
	virtual ~Point() = default;

protected:
	double size_;
};

} /* namespace vis */
} /* namespace hubero_local_planner */

#endif /* INCLUDE_HUBERO_LOCAL_PLANNER_VIS_POINT_H_ */
