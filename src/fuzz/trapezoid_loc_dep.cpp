/*
 * TrapezoidLocDep.cpp
 *
 *  Created on: Aug 23, 2019
 *      Author: rayvburn
 */

#include <humap_local_planner/fuzz/trapezoid_loc_dep.h>

namespace humap_local_planner {
namespace fuzz {

// ------------------------------------------------------------------- //

TrapezoidLocDep::TrapezoidLocDep(std::string name, double intersection_deg, bool dtor_deletes):
	TrapezoidParted::TrapezoidParted(name, intersection_deg, dtor_deletes) {}

// ------------------------------------------------------------------- //

bool TrapezoidLocDep::update(RelativeLocation side, const Angle& gamma_start, const Angle& gamma_end) {

	bool extra_term;

	// based on `side` argument value, let's calculate trapezoid parameters;
	// an additional term is configured internally if needed
	if ( side == RelativeLocation::LOCATION_RIGHT ) {
		extra_term = TrapezoidParted::update(gamma_start.getRadian(), gamma_end.getRadian());
	} else if ( side == RelativeLocation::LOCATION_LEFT ) {
		extra_term = TrapezoidParted::update(gamma_end.getRadian(), gamma_start.getRadian());
	} else {
		throw std::runtime_error("wrong value of the relative location!");
	}

	return (extra_term);

}

// ------------------------------------------------------------------- //

TrapezoidLocDep::~TrapezoidLocDep() {
	// TODO Auto-generated destructor stub
}

// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

} /* namespace fuzz */
} /* namespace humap_local_planner */
