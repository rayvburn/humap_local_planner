/*
 * TrapezoidLocIndep.cpp
 *
 *  Created on: Aug 23, 2019
 *      Author: rayvburn
 */

#include <hubero_local_planner/fuzz/trapezoid_loc_indep.h>

namespace fuzz {

// ------------------------------------------------------------------- //

TrapezoidLocIndep::TrapezoidLocIndep(std::string name, double intersection_deg, double length_deg)
		: TrapezoidParted::TrapezoidParted(name, intersection_deg), interval_(IGN_DTOR(length_deg)) {}

// ------------------------------------------------------------------- //

bool TrapezoidLocIndep::update(const Angle &gamma_center) {

	// 2 angles must be created (normalization needed)
	Angle gamma_start = gamma_center;
	gamma_start.Radian(gamma_start.Radian() - interval_);
	gamma_start.Normalize();

	Angle gamma_end = gamma_center;
	gamma_end.Radian(gamma_end.Radian() + interval_);
	gamma_end.Normalize();

	return (TrapezoidParted::update(gamma_start.Radian(), gamma_end.Radian()));

}

// ------------------------------------------------------------------- //

TrapezoidLocIndep::~TrapezoidLocIndep() {}

// ------------------------------------------------------------------- //
// private
// ------------------------------------------------------------------- //


// ------------------------------------------------------------------- //

} /* namespace fuzz */
