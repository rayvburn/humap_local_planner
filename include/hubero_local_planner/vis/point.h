/*
 * point.h
 *
 *  Created on: Apr 15, 2021
 *      Author: rayvburn
 */

#ifndef INCLUDE_HUBERO_LOCAL_PLANNER_VIS_POINT_H_
#define INCLUDE_HUBERO_LOCAL_PLANNER_VIS_POINT_H_

#include "marker_base.h"

namespace vis {

class Point : public MarkerBase {
public:
	Point();
	void setSize(const double &side_length);
	virtual visualization_msgs::Marker create(const Vector3 &pos) const;
	virtual ~Point() = default;

protected:
	double size_;
};

} /* namespace vis */

#endif /* INCLUDE_HUBERO_LOCAL_PLANNER_VIS_POINT_H_ */
