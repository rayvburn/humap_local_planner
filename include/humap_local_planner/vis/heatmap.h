/*
 * Heatmap.h
 *
 *  Created on: Dec 27, 2019
 *      Author: rayvburn
 */

#pragma once

#include <humap_local_planner/vis/grid.h>
#include <humap_local_planner/vis/heat_cell.h>

namespace humap_local_planner {
namespace vis {

/// \brief Incorporates Grid and HeatCell classes methods, provides an interface that is easy to use
/// and limits number of objects to manipulate from the main class (Actor) to a single one
class Heatmap : public Grid, public HeatCell {

public:

	/// \brief Default constructor
	Heatmap();

	/// \brief Default destructor
	virtual ~Heatmap();

private:

};

} /* namespace vis */
} /* namespace humap_local_planner */
