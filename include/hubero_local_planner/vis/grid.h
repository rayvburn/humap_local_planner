/*
 * Grid.h
 *
 *  Created on: Apr 16, 2019
 *      Author: rayvburn
 */

#pragma once

#include <hubero_local_planner/vis/arrow.h>
#include <hubero_local_planner/vis/marker_base.h>
#include <vector>

// ROS libraries
#include <visualization_msgs/MarkerArray.h>

namespace hubero_local_planner {
namespace vis {

using namespace geometry;
/// \brief Class that manages creation of a vector field
/// visualization; generates a grid of points and calculates
/// a social force in each of them (treats each position as
/// if an actor would in fact be located there)
class Grid {

public:

	/// \brief Default constructor
	Grid();

	/// \brief Creates a grid (std::vector
	/// of an Vector instances)
	/// \note Method made virtual to allow derived classes
	/// to perform some `initialization` operations
	virtual void createGrid(const float &x_start, const float &x_end, const float &y_start, const float &y_end, const float &resolution);

	/// \brief Add a given marker to locally stored
	/// MarkerArray; creates a copy of a marker to
	/// change an ID (necessary for MarkerArray)
	void addMarker(const visualization_msgs::Marker &marker);

	/// \brief Function used to determine whether
	/// to stop grid composition; checks if grid
	/// index is equal to grid size
	bool isWholeGridChecked() const;

	/// \brief Based on previously created grid this
	/// method returns a position of a next grid
	/// element using locally stored grid index;
	/// NOTE: by invoking it twice one will omit
	/// some grid points
	Vector getNextGridElement();

	/// \brief Sets grid index to 0; must be invoked after
	/// each grid composition finish (or just before
	/// its start)
	void resetGridIndex();

	/// \brief Returns a MarkerArray of an appropriate
	/// size based on grid's size and resolution
	visualization_msgs::MarkerArray getMarkerArray() const;

	/// \brief Default destructor
	virtual ~Grid();

private:

	/// \brief A vector of grid points
	std::vector<Vector> grid_;

	/// \brief Stores current grid point's index
	/// in a vector
	size_t grid_index_;

protected:

	/// \brief An array of Markers
	visualization_msgs::MarkerArray marker_array_;

};

} /* namespace vis */
} /* namespace hubero_local_planner */
