/*
 * Text.h
 *
 *  Created on: Sep 27, 2019
 *      Author: rayvburn
 */

#pragma once

#include <hubero_local_planner/vis/marker_base.h>

namespace hubero_local_planner {
namespace vis {

using namespace geometry;
/// \brief Implements http://wiki.ros.org/rviz/DisplayTypes/Marker#View-Oriented_Text_.28TEXT_VIEW_FACING.3D9.29_.5B1.1.2B-.5D
/// by extending Base's functionality. Overrides Base's create method.
class Text : public MarkerBase {

public:

	Text();

	void setParameters(const float &text_size);

	visualization_msgs::Marker create(const Vector& pos, const int& number) const;
	visualization_msgs::Marker create(const Vector& pos, const std::string& text) const;

	virtual ~Text();

protected:

	float text_size_;

};

} /* namespace vis */
} /* namespace hubero_local_planner */
