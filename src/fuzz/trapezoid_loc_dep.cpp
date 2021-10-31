/*
 * TrapezoidLocDep.cpp
 *
 *  Created on: Aug 23, 2019
 *      Author: rayvburn
 */

#include <hubero_local_planner/fuzz/trapezoid_loc_dep.h>

namespace fuzz {

// ------------------------------------------------------------------- //

TrapezoidLocDep::TrapezoidLocDep(std::string name, double intersection_deg)
		: TrapezoidParted::TrapezoidParted(name, intersection_deg) {}

// ------------------------------------------------------------------- //

bool TrapezoidLocDep::update(const char &side, const Angle& gamma_start, const Angle& gamma_end) {

	bool extra_term;

	// based on `side` argument value, let's calculate trapezoid parameters;
	// an additional term is configured internally if needed
	if ( side == 'r' ) {
		extra_term = TrapezoidParted::update(gamma_start.getRadian(), gamma_end.getRadian());
	} else if ( side == 'l' ) {
		extra_term = TrapezoidParted::update(gamma_end.getRadian(), gamma_start.getRadian());
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

