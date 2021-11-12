/*
 * TrapezoidParted.cpp
 *
 *  Created on: Aug 23, 2019
 *      Author: rayvburn
 */

#include <hubero_local_planner/fuzz/trapezoid_parted.h>
#include <hubero_local_planner/geometry/angle.h>
#include <stdexcept>
#include <math.h> // fabs()

namespace hubero_local_planner {
namespace fuzz {

using namespace geometry;

// ------------------------------------------------------------------- //

TrapezoidParted::TrapezoidParted(std::string name, double intersection_deg, bool dtor_deletes)
		: intersection_(IGN_DTOR(intersection_deg)), dtor_delete_trapezoids_(dtor_deletes)
{
	// create 2 trapezoid instances, rules will need both of them for deduction;
	// both trapezoids are first initialized with NaNs so they will not be recognized
	// as valid regions until become fully initialized
	std::string name_modded1 = name;
	std::string name_modded2 = name;
	name_modded1.append("A");
	name_modded2.append("B");
	trapezoid_ptrs_.push_back(new fl::Trapezoid(name_modded1)); // .append("A")
	trapezoid_ptrs_.push_back(new fl::Trapezoid(name_modded2)); // .append("B")
	// 2 terms of the same name are allowed but such operation causes problems in recognition of output region
}

// ------------------------------------------------------------------- //

TrapezoidParted::~TrapezoidParted() {
	if (!dtor_delete_trapezoids_) {
		return;
	}
	if (trapezoid_ptrs_.at(0) != nullptr) {
		delete trapezoid_ptrs_.at(0);
	}
	if (trapezoid_ptrs_.at(1) != nullptr) {
		delete trapezoid_ptrs_.at(1);
	}
}

// ------------------------------------------------------------------- //

void TrapezoidParted::setSideLength(double intersection_deg) {
	intersection_ = IGN_DTOR(intersection_deg);
}

// ------------------------------------------------------------------- //

bool TrapezoidParted::update(const double &start, const double &end) {
	/// Defines trapezoid part's extension scale, when some operations are applied
	const double RANGE_EXTENSION = IGN_DTOR(5);

	// Creates parameter configuration equal to the uninitialized fl::Trapezoid instance
	std::string params("nan nan nan nan");
	std::string params_wrap("nan nan nan nan");

	// Variable to detect whether term (trapezoid) has been `wrapped`
	bool status = false;

	// helper variables to perform initial tests
	// A is the first vertex of the trapezoid
	Angle a(start - intersection_);
	// D is the fourth vertex of the trapezoid
	Angle d(end + intersection_);

	// check whether trapezoid's A vertex is located before (smaller than) the D vertex
	if ( a.getRadian() < d.getRadian() ) {

		// if below condition is met then full term is located within allowable bounds (in <-PI; +PI> range)
		if ( a.getRadian() >= (-IGN_PI) && d.getRadian() <= (+IGN_PI) ) {

			// normal operation - only single term's parameters must be found
			params = generateParams(a.getRadian(), start, end, d.getRadian(), 1.0);

		} else {
			throw std::runtime_error("unhandled case in trapezoid update");
		}

	} else {

		/**
		 * In this branch both terms (`normal` and `wrapped`) parameters must be found,
		 * `start` is equal to the B vertex
		 * `end`   is equal to the C vertex
		 */
		// find interval which contains +PI argument - it can be achieved
		// via simple comparison of the 2 consecutive vertices locations
		if ( a.getRadian() > start ) {

			// CASE 1: trapezoid's first part (side edge) finishes somewhere between A and B vertices
			/**
			 * NOTE: fuzzylite does not complain about terms which are too wide for
			 * the allowable range - calculates the membership functions just right
			 * as long as the input variable does not exceed the given range.
			 * In the other words: terms' bounds can exceed <-PI; +PI> range
			 * as long as the `INPUT VARIABLE` stays within those bounds.
			 */
			// firstly, find how much the B vertex goes out of +PI range (i.e. the angle value before normalization)
			Angle wrap_angle_n(-IGN_PI - start);

			// argument B vertex would have if range would not be limited
			// positive side case
			double b_out_range = IGN_PI - wrap_angle_n.getRadian();

			// first part's configuration (range artificially extended by 5 degrees)
			params = generateParams(a.getRadian(), b_out_range, b_out_range + RANGE_EXTENSION, b_out_range + RANGE_EXTENSION, 1.0);

			// consider negative side case now (wrap from the positive side to the negative one)
			// `wrap2` in notebook
			Angle wrap_angle_w = Angle(IGN_PI - a.getRadian());
			double a_out_range = -IGN_PI - wrap_angle_w.getRadian();

			/**
			 * Second part's configuration
			 * if trapezoid's broadness exceeds 2*pi range, then D vertex must be changed (compared to the one
			 * initially calculated)
			 */
			double len_raw = 2 * intersection_ + (end - start);
			bool exceeds_2pi = len_raw > (2 * IGN_PI);
			if (exceeds_2pi) {
				// without normalization
				d = Angle(end + intersection_, false);
			}
			params_wrap = generateParams(a_out_range, start, end, d.getRadian(), 1.0);

		} else if ( start >= end ) {

			// FIXME: this lacks artificial extension (as in CASE 1) - why it's handled differently?
			//
			// CASE 2: trapezoid's first part (side edge) finishes somewhere between B and C vertices
			params = generateParams(a.getRadian(), start, +IGN_PI, +IGN_PI, 1.0);

			// find the second part's configuration
			params_wrap = generateParams(-IGN_PI, -IGN_PI, end, d.getRadian(), 1.0);

		} else if ( end >= d.getRadian() ) {

			// CASE 3: trapezoid's first part (side edge) finishes somewhere between C and D vertices
			//
			// firstly, find how much the D vertex goes out of +PI range (i.e. the value before normalization)
			double wrap = std::fabs(-IGN_PI - d.getRadian());

			// argument D vertex would have if range would not be limited
			// positive side case
			double d_out_range = IGN_PI + wrap;

			// first part's configuration (range artificially extended by 5 degree)
			params = generateParams(a.getRadian(), start, end, d_out_range, 1.0);

			// consider negative side case now (wrap from the positive side to the negative one)
			//
			// difference between (+IGN_PI) and C vertex location (end)
			// `wrap2` in the notebook
			wrap = std::fabs(IGN_PI - end);
			double c_out_range = -IGN_PI - wrap;

			// second part's configuration
			params_wrap = generateParams(c_out_range - RANGE_EXTENSION, c_out_range - RANGE_EXTENSION, c_out_range, d.getRadian(), 1.0);

		} else {
			throw std::runtime_error("unhandled case in trapezoid update (section 2)");
		}

		status = true;

	}

	// if `wrap` did not occur (status is False), let's try to reset the second part (the one with index of 1)
	if ( !status ) {
		// sets all fields to `nan` except for `height`
		// FIXME: this should not be necessary!
		resetWrapped();
	}

	// update both trapezoids with new parameters
	trapezoid_ptrs_.at(0)->configure(params);
	trapezoid_ptrs_.at(1)->configure(params_wrap);

	return (status);

}

// ------------------------------------------------------------------- //

std::vector<fl::Trapezoid*> TrapezoidParted::getTrapezoids() const {
	return (trapezoid_ptrs_);
}

// ------------------------------------------------------------------- //

std::string TrapezoidParted::generateParams(const double &a, const double &b, const double &c,
		const double &d, const double &height) const {

	std::string params("");

	params += std::to_string(a); 	params += " ";		// A
	params += std::to_string(b); 	params += " ";		// B
	params += std::to_string(c); 	params += " ";		// C
	params += std::to_string(d);	params += " ";		// D
	params += std::to_string(height);					// height

	return (params);

}

// ------------------------------------------------------------------- //

double TrapezoidParted::findHeight(const char& slope_type, const double &start, const double &end) const {


	double y = std::numeric_limits<double>::max();

	if ( slope_type == 'a') {

		// ascending - looking for Y-value for the +PI argument
		y = (IGN_PI - start)/(end - start);

	} else if ( slope_type == 'd' ) {

		// descending - looking for Y-value for the -PI argument
		y = (end - (-IGN_PI))/(end - start);

	}

	return (y);

}

// ------------------------------------------------------------------- //

void TrapezoidParted::resetWrapped() {

	// check first vertex of the `wrapped` trapezoid,
	// if it is equal to some valid number let's set
	// all `vertices` to NaNs and `height` to 0
	if ( trapezoid_ptrs_.at(1)->getVertexA() != fl::nan ) {
		trapezoid_ptrs_.at(1)->setVertexA(fl::nan);
		trapezoid_ptrs_.at(1)->setVertexB(fl::nan);
		trapezoid_ptrs_.at(1)->setVertexC(fl::nan);
		trapezoid_ptrs_.at(1)->setVertexD(fl::nan);
		trapezoid_ptrs_.at(1)->setHeight(0.0f);
	}

}

// ------------------------------------------------------------------- //

} /* namespace fuzz */
} /* namespace hubero_local_planner */
