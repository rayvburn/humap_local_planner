/*
 * TrapezoidLocDep.h
 *
 *  Created on: Aug 23, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_FUZZ_TRAPEZOIDLOCDEP_H_
#define INCLUDE_FUZZ_TRAPEZOIDLOCDEP_H_

#include <hubero_local_planner/fuzz/trapezoid_parted.h>
#include <hubero_local_planner/geometry/angle.h>
#include <hubero_local_planner/defines.h>

namespace hubero_local_planner {
namespace fuzz {

using namespace geometry;
class TrapezoidLocDep : public TrapezoidParted {

public:

	TrapezoidLocDep(std::string name, double intersection_deg);

	/// @brief TODO: some image needed to illustrate the situation
	/// @param side
	/// @param gamma_start
	/// @param gamma_end
	/// @return
	bool update(RelativeLocation side, const Angle& gamma_start, const Angle& gamma_end);

	virtual ~TrapezoidLocDep();

private:


};

} /* namespace fuzz */
} /* namespace hubero_local_planner */

#endif /* INCLUDE_FUZZ_TRAPEZOIDLOCDEP_H_ */
