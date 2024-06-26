/*
 * GridForce.h
 *
 *  Created on: Dec 29, 2019
 *      Author: rayvburn
 */

#pragma once

#include <humap_local_planner/vis/grid.h>
#include <humap_local_planner/vis/arrow.h>

namespace humap_local_planner {
namespace vis {

/// \brief Arrow is treated as a base class to easily configure grid-specific configuration
/// of the markers related to the force field visualization
class GridForce : public Grid, public Arrow {

public:

	/// \brief Default constructor
	GridForce();

	/// \brief Creates a grid (std::vector
	/// of an ignition::math::Vector3d instances)
	virtual void createGrid(const float &x_start, const float &x_end, const float &y_start, const float &y_end, const float &resolution) override;

	/// \brief Destructor
	virtual ~GridForce();

};

} /* namespace vis */
} /* namespace humap_local_planner */
