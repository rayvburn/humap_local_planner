/*
 * TrapezoidLocIndep.h
 *
 *  Created on: Aug 23, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_FUZZ_TRAPEZOIDLOCINDEP_H_
#define INCLUDE_FUZZ_TRAPEZOIDLOCINDEP_H_

#include <humap_local_planner/fuzz/trapezoid_parted.h>
#include <humap_local_planner/geometry/angle.h>

namespace humap_local_planner {
namespace fuzz {

using namespace geometry;
class TrapezoidLocIndep : public TrapezoidParted {

public:

	/// @param name
	/// @param intersection_deg: cosine of length of the rising/falling edges of the trapezoid
	/// @param length_deg: region in fact has 0 length, so it must be artificially extended
	/// to actually be taken into consideration during reasoning; values > 0 are valid
	TrapezoidLocIndep(std::string name, double intersection_deg, double length_deg, bool dtor_deletes = false);

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
} /* namespace humap_local_planner */

#endif /* INCLUDE_FUZZ_TRAPEZOIDLOCINDEP_H_ */
