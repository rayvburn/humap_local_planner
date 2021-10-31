/*
 * TrapezoidLocIndep.h
 *
 *  Created on: Aug 23, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_FUZZ_TRAPEZOIDLOCINDEP_H_
#define INCLUDE_FUZZ_TRAPEZOIDLOCINDEP_H_

#include <hubero_local_planner/fuzz/trapezoid_parted.h>
#include <hubero_local_planner/geometry/angle.h>
using namespace hubero::geometry;

namespace fuzz {

class TrapezoidLocIndep : public TrapezoidParted {

public:

	/// @param name
	/// @param intersection_deg
	/// @param length_deg: region in fact has 0 length, so it must be artificially extended
	/// to actually be taken into consideration during reasoning
	TrapezoidLocIndep(std::string name, double intersection_deg, double length_deg);

	///
	/// @param gamma_center
	/// @return
	bool update(const Angle& gamma_center);

//	bool update

	virtual ~TrapezoidLocIndep();

private:

	double interval_;

};

} /* namespace fuzz */

#endif /* INCLUDE_FUZZ_TRAPEZOIDLOCINDEP_H_ */
