/*
 * Processor.h
 *
 *  Created on: Aug 7, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_FUZZ_PROCESSOR_H_
#define INCLUDE_FUZZ_PROCESSOR_H_

#include <fl/Headers.h>
#include <hubero_local_planner/fuzz/trapezoid_loc_dep.h>
#include <hubero_local_planner/fuzz/trapezoid_loc_indep.h>
#include <hubero_local_planner/geometry/geometry.h>
#include <vector>
#include <tuple>
#include <map>

namespace hubero_local_planner {
namespace fuzz {

///
/// \note Remember to call all `setter` methods before calling `process`. Otherwise, the fuzzy
/// calculations will surely utilize old values as inputs and a result will not be properly
/// computed.
class Processor {

public:
	/// \brief Initializes fuzzy logic system (based on `fuzzylite` library)
	/// with values proper to this application.
	Processor();

	/// \brief Prints FIS configuration to console (stdout)
	void printFisConfiguration() const;

	/// \brief Setter method for `d_alpha_beta` angle (see SFM doc for details).
	/// `d_alpha_beta` angle is a direction of a vector connecting alpha and beta
	/// center positions.
	/// \param d_alpha_beta_angle is an angle described above
	bool load(const double &dir_alpha, const std::vector<double> &dir_beta_v, const std::vector<double> &rel_loc_v,
			  const std::vector<double> &dist_angle_v);

	/// \brief Executes fuzzy calculations. The `process()` call must be preceded by `updateRegions()`
	void process();

	/// \brief Based on the output variable's value, calculates the behavior needed to be applied.
	/// In case of 2 behaviors having non-zero memberships, their `strength` is different than 1.0
	/// (sums up to 1.0 though).
	/// \return Vector of tuples. Each tuple consists of behavior name (string) and membership level.
	std::vector<std::tuple<std::string, double> > getOutput() const;

	virtual ~Processor();

private:
	/// \brief Determines location of the \beta element relative to \alpha direction of motion
	char decodeRelativeLocation(const double &rel_loc) const;

	/// \brief Updates trapezoidal regions of input variables.
	/// \note Must be preceded by `setters` of input variables.
	void updateRegions(const double &alpha_dir, const double &beta_dir, const double &d_alpha_beta_angle, const double &rel_loc);

	/* ----- inputs ----- */
	/// \brief Determines \alpha 's direction of motion.
	double alpha_dir_;

	/// \brief Direction of the vector which connects \alpha 's position with \beta 's position.
	std::vector<double> d_alpha_beta_angle_;

	/// \brief Stores angle telling which \beta is located
	/// in terms of \alpha 's direction of motion.
	std::vector<double> rel_loc_;

	/// \brief Determines \beta 's direction of motion.
	std::vector<double> beta_dir_;

	/* ------- output --------- */
	std::vector<std::tuple<std::string, double> > output_v_;

	/**
	 * @defgroup fuzzy fuzzylite-related
	 *
	 * @details fl::* objects must be dynamically allocated, passing pointer of raw, e.g., @ref fl::InputVariable
	 * object to @ref engine_ produces segmentation fault in unit tests and does not indicate failures, see
	 * https://github.com/fuzzylite/fuzzylite#c
	 *
	 * @{
	 */
	/// \brief Fuzzy logic engine
	fl::Engine* engine_ptr_;

	/* ----- Input variables ----- */
	/// \brief An angle which helps specify on which hand-side the object is located relative to the actor
	fl::InputVariable* location_ptr_; 	//	double object_dir_relative_angle_;

	// direction input variable section (consists of input variable and related trapezoidal terms)
	/// \brief A relative angle between 2 objects' velocities vectors; used to determine
	/// whether objects are moving in the same direction
	fl::InputVariable* direction_ptr_; 		//	double vels_relative_angle_;

	/// \brief Trapezoidal term related to the case when `beta` object's
	/// direction points outwards relative to the `alpha` direction
	TrapezoidLocDep trapezoid_out_;

	/// \brief Trapezoidal term related to the case when `beta` object's
	/// direction crosses in front of the `alpha` center
	TrapezoidLocDep trapezoid_cf_;

	/// \brief Trapezoidal term related to the case when `beta` object's
	/// direction crosses behind the `alpha` center
	TrapezoidLocDep trapezoid_cb_;

	/// \brief Trapezoidal term related to the case when `beta` object's
	/// direction points in the same direction as the `alpha`'s
	TrapezoidLocIndep trapezoid_eq_;

	/// \brief Trapezoidal term related to the case when `beta` object's
	/// direction points in the opposite direction as the `alpha`'s
	TrapezoidLocIndep trapezoid_opp_;

	/* ----- Output variables ----- */
	fl::OutputVariable* social_behavior_ptr_;

	/*  ----- Rule block ----- */
	fl::RuleBlock* rule_block_ptr_;
	// @} // end of the group

};

} /* namespace fuzz */
} /* namespace hubero_local_planner */

#endif /* INCLUDE_FUZZ_PROCESSOR_H_ */
