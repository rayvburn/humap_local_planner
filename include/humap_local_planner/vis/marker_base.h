/*
 * Base.h
 *
 *  Created on: Apr 16, 2019
 *      Author: rayvburn
 */

#pragma once

// C++ STL libraries
#include <string>

// geometry libraries
#include <humap_local_planner/geometry/geometry.h>

// ROS libraries
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

namespace humap_local_planner {
namespace vis {

/// \brief Base class for sfm::vis:: objects;
/// provides some generic methods
class MarkerBase {

public:

	/// \brief Default constructor
	MarkerBase();

	/// \brief Sets parent frame (TF), default is "map"
	void init(const std::string &parent_frame);

	/// \brief Sets color
	void setColor(const float &r, const float &g, const float &b, const float &alpha);

	void setNamespace(const std::string& ns);

	/// \brief Default destructor
	virtual ~MarkerBase();

protected:

	/// \brief Helper function to set a color of a ColorRGBA instances in derived classes
	void setColor(std_msgs::ColorRGBA *color, const float &r, const float &g, const float &b, const float &alpha);

	/// \brief Stores parent frame's name
	std::string frame_;

	/// \brief Stores current namespace
	std::string namespace_;

	/// \brief Stores a color an arrow
	std_msgs::ColorRGBA color_;

};

} /* namespace vis */
} /* namespace humap_local_planner */
