/*
 * TrapezoidLocIndep.cpp
 *
 *  Created on: Aug 23, 2019
 *      Author: rayvburn
 */

#include <humap_local_planner/fuzz/trapezoid_loc_indep.h>

namespace humap_local_planner {
namespace fuzz {

// ------------------------------------------------------------------- //

TrapezoidLocIndep::TrapezoidLocIndep(std::string name, double intersection_deg, double length_deg, bool dtor_deletes):
	TrapezoidParted::TrapezoidParted(name, intersection_deg, dtor_deletes),
	// check whether `length_deg` is valid, let values above threshold be OK; `interval_` is half of the `length_deg`
	interval_(IGN_DTOR(std::max(length_deg, 1e-03)) / 2.0)
{}

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
} /* namespace humap_local_planner */
