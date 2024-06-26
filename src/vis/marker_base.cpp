/*
 * Base.cpp
 *
 *  Created on: Apr 16, 2019
 *      Author: rayvburn
 */

#include <humap_local_planner/vis/marker_base.h>

namespace humap_local_planner {
namespace vis {

// ------------------------------------------------------------------- //

MarkerBase::MarkerBase(): frame_("map") { }

// ------------------------------------------------------------------- //

void MarkerBase::init(const std::string &parent_frame) {
	frame_ = parent_frame;
}

// ------------------------------------------------------------------- //

void MarkerBase::setColor(const float &r, const float &g, const float &b, const float &alpha) {
	setColor(&color_, r, g, b, alpha);
}

// ------------------------------------------------------------------- //

void MarkerBase::setNamespace(const std::string& ns) {
	namespace_ = ns;
}

// ------------------------------------------------------------------- //

void MarkerBase::setColor(std_msgs::ColorRGBA *color, const float &r, const float &g, const float &b, const float &alpha) {
	color->a = alpha;	color->r = r;	color->g = g;	color->b = b;
}

// ------------------------------------------------------------------- //

MarkerBase::~MarkerBase() { }

// ------------------------------------------------------------------- //

} /* namespace vis */
} /* namespace humap_local_planner */
