/*
 * TrapezoidLocIndep.cpp
 *
 *  Created on: Aug 23, 2019
 *      Author: rayvburn
 */

#include <hubero_local_planner/fuzz/trapezoid_loc_indep.h>

namespace hubero_local_planner {
namespace fuzz {

// ------------------------------------------------------------------- //

TrapezoidLocIndep::TrapezoidLocIndep(std::string name, double intersection_deg, double length_deg)
		: TrapezoidParted::TrapezoidParted(name, intersection_deg), interval_(IGN_DTOR(length_deg)) {}

// ------------------------------------------------------------------- //

bool TrapezoidLocIndep::update(const Angle& gamma_center) {
	// 2 angles must be created (normalization performed inside ctor)
	Angle gamma_start(gamma_center.getRadian() - interval_);
	Angle gamma_end(gamma_center.getRadian() + interval_);
	return TrapezoidParted::update(gamma_start.getRadian(), gamma_end.getRadian());
}

// ------------------------------------------------------------------- //

TrapezoidLocIndep::~TrapezoidLocIndep() {}

// ------------------------------------------------------------------- //
// private
// ------------------------------------------------------------------- //


// ------------------------------------------------------------------- //

} /* namespace fuzz */
} /* namespace hubero_local_planner */
