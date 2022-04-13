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

class Processor {
public:
	struct FisOutput {
		/// FIS output value
		double value;
		/// degree of membership
		double membership;
		/// name of the term that contains sharpened output value
		std::string term_name;
	};

	/// \brief Initializes fuzzy logic system (based on `fuzzylite` library)
	/// with values proper to this application.
	Processor();

	/// \brief Prints FIS configuration to console (stdout)
	void printFisConfiguration() const;

	/**
	 * Executes fuzzy calculations based on data given by arguments
	 *
	 * Prepares FIS output vector based on all entries given by arguments. Event if a membership could not be computed
	 * for a specific pair \alpha-\beta, this pair is considered in output vector - marked as output with 0 membership,
	 * term name "none".
	 *
	 * @param dir_alpha determines \alpha 's direction of motion
	 * @param dir_beta_v determines \beta -s's direction of motion
	 * @param rel_loc_v stores angle that defines relative location of \beta in terms of \alpha 's motion direction
	 * @param dist_angle_v direction of the vector that connects \alpha 's position with \beta 's position
	 * @return true if successfully computed at least 1 membership
	 */
	bool process(
		const double& dir_alpha,
		const std::vector<double>& dir_beta_v,
		const std::vector<double>& rel_loc_v,
		const std::vector<double>& dist_angle_v
	);

	/// \return Vector of single FIS Outputs, see @ref FisOutput for details
	std::vector<Processor::FisOutput> getOutput() const;

	/// \brief Specifically for unit testing, must be called after @ref process
	std::vector<std::tuple<std::string, double>> membershipInputRelLoc() const;

	/// \brief Specifically for unit testing, must be called after @ref process
	std::vector<std::tuple<std::string, double>> membershipInputDirCross() const;

	/// \brief Determines location of the \beta element relative to \alpha direction of motion
	static RelativeLocation decodeRelativeLocation(const double &rel_loc);

	/// \brief Computes @ref side (defines dir cross division placement) and @ref gamma_* s
	static void computeDirCrossBorderValues(
		const double& alpha_dir,
		const double& dist_angle,
		const double& rel_loc_angle,
		Angle& gamma_eq,
		Angle& gamma_opp,
		Angle& gamma_cc,
		RelativeLocation& side
	);

	/// \brief Frees memory allocated for fl::Engine
	virtual ~Processor();

protected:
	/// \brief Parameterized method used by @ref membershipInputRelLoc and @ref membershipInputDirCross
	static std::vector<std::tuple<std::string, double>> highestMembership(const fl::InputVariable* input_ptr);

private:
	/// \brief Updates trapezoidal regions of input variables.
	/// \note Must be preceded by `setters` of input variables.
	void updateRegions(const double &alpha_dir, const double &beta_dir, const double &d_alpha_beta_angle, const double &rel_loc);

	/* ------- output --------- */
	std::vector<FisOutput> output_v_;

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
