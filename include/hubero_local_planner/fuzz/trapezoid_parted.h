/*
 * TrapezoidParted.h
 *
 *  Created on: Aug 23, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_FUZZ_TRAPEZOIDPARTED_H_
#define INCLUDE_FUZZ_TRAPEZOIDPARTED_H_

#include <string>
#include <stdint.h>
#include <vector>
#include <tuple>

#include <fl/term/Trapezoid.h>

namespace hubero_local_planner {
namespace fuzz {

class TrapezoidParted {

public:

	/// @brief Constructor
	/// @param name: name of the trapezoid term
	// TODO: change param name
	/// @param intersection_deg: cosine of length of the rising/falling edges of the trapezoid; positive values allowed
	/// @param dtor_deletes whether @ref ~TrapezoidParted deletes memory allocated in @ref trapezoid_ptrs_
	TrapezoidParted(std::string name, double intersection_deg, bool dtor_deletes = false);

	/**
	 * Overrides `intersection_deg` value passed to constructor; positive values allowed
	 */
	void setSideLength(double intersection_deg);

	/// @brief Updates both trapezoid parts
	/// @param start
	/// @param end
	/// @return
	/// @note `start` and `end` must be normalized angle values in radians
	bool update(const double &start, const double &end);

	/// @return Vector of pointers to fl::Trapezoid instances creating
	/// single region (may be wrapped around range's corners)
	std::vector<fl::Trapezoid*> getTrapezoids() const;

	/// Destructor
	virtual ~TrapezoidParted();

protected:

	std::string generateParams(const double &a, const double &b, const double &c, const double &d, const double &height = 1.0) const;

	double findHeight(const char& slope_type, const double &start, const double &end) const;

	void resetWrapped();

	/// Vector consisting of 1 or 2 elements, depending on the current
	/// location of corner points (gamma_cc etc.). Consists of 2 elements
	/// when range wraps from +PI to - PI.
	std::vector<fl::Trapezoid*> trapezoid_ptrs_;

	/// \brief Flag that indicates whether @ref trapezoid_ptrs_ contents should be deleted in this class' dtor
	/// This is related to fl::Engine specifics, see https://github.com/fuzzylite/fuzzylite/issues/93 for details
	bool dtor_delete_trapezoids_;

	/// Defines how much a given region intersects the contiguous one (in `radians`)
	double intersection_;

};

} /* namespace fuzz */
} /* namespace hubero_local_planner */

#endif /* INCLUDE_FUZZ_TRAPEZOIDPARTED_H_ */
