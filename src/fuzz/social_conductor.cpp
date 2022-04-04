/*
 * SocialConductor.cpp
 *
 *  Created on: Aug 6, 2019
 *      Author: rayvburn
 */

#include <hubero_local_planner/fuzz/regions.h>
#include <hubero_local_planner/fuzz/social_conductor.h>

namespace hubero_local_planner {
namespace fuzz {

// ------------------------------------------------------------------- //

void SocialConductor::initialize(std::shared_ptr<const hubero_local_planner::FisParams> cfg) {
	cfg_ = cfg;
}

// ------------------------------------------------------------------- //

bool SocialConductor::computeBehaviourForce(
	const Pose& pose_agent,
	const std::vector<Processor::FisOutput>& fis_outputs_v,
	const std::vector<double>& dist_v
) {
	// clear out behaviour force and active behavior names
	reset();

	if (dist_v.size() != fis_outputs_v.size()) {
		std::cout << "[SocialConductor] vectors of non equal sizes passed to apply()" << std::endl;
		return false;
	}

	const size_t size_ref = dist_v.size();
	// calculate combined output - superposed outputs
	for (size_t i = 0; i < size_ref; i++) {
		// create temporary vector pointing to direction determined by FIS output
		Vector v_temp(Angle(fis_outputs_v.at(i).value));
		double membership_factor = fis_outputs_v.at(i).membership;
		double dist_based_factor = computeBehaviourStrength(dist_v.at(i));
		behaviour_force_ += v_temp * membership_factor * dist_based_factor;
		updateActiveBehaviour(fis_outputs_v.at(i).term_name);
	}

	// convert into global coordinates
	behaviour_force_.rotate(pose_agent.getYaw());

	/**
	 * NOTE: apply trick from superpose (DEPRECATED)?
	 * force `none` behaviour when `max len` is very small
	if ( max_len < 1e-03 ) {
		behaviour_active_str_.clear();
	}
	*/

	// normalize -> may easily overflow
	if (behaviour_force_.calculateLength() > 1.0) {
		behaviour_force_.normalize();
	}

	// multiply times behaviour force factor
	behaviour_force_ *= cfg_->force_factor;

	return true;
}

// ------------------------------------------------------------------- //

void SocialConductor::reset() {
	behaviour_force_ = Vector(0.0, 0.0, 0.0);
	behaviour_active_str_.clear();
}

// ------------------------------------------------------------------- //

Vector SocialConductor::getSocialVector() const {
	return (behaviour_force_);
}

// ------------------------------------------------------------------- //

std::string SocialConductor::getBehaviourActive() const {

	// NOTE: rViz goes mad when tries to publish
	// an empty string (application crashes)
	if ( behaviour_active_str_.empty() ) {
		return ("none");
	}
	return (behaviour_active_str_);

}

// ------------------------------------------------------------------- //

void SocialConductor::updateActiveBehaviour(const std::string& beh_name) {

	if ( behaviour_active_str_.empty() ) {
		behaviour_active_str_ = beh_name;
	} else {
		behaviour_active_str_.append("\n");
		behaviour_active_str_.append(beh_name);
	}

}

// ------------------------------------------------------------------- //

double SocialConductor::computeBehaviourStrength(const double& dist_to_agent) {
	// check if obstacle is too far away
	if (dist_to_agent > cfg_->human_action_range) {
		return (0.0);
	}

	// in fact (SOCIAL_BEHAVIOUR_RANGE_END - SOCIAL_BEHAVIOUR_RANGE_START) but the start is 0.0
	double a = -1.0 / cfg_->human_action_range;
	// form of a line equation for readability, the independent variable is `dist_to_agent`
	double y = a * dist_to_agent + 1.0;
	return y;
}

} /* namespace fuzz */
} /* namespace hubero_local_planner */
