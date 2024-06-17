/*
 * SocialConductor.cpp
 *
 *  Created on: Aug 6, 2019
 *      Author: rayvburn
 */

#include <humap_local_planner/fuzz/regions.h>
#include <humap_local_planner/fuzz/social_conductor.h>

#include <humap_local_planner/sfm/social_force_model.h>

namespace humap_local_planner {
namespace fuzz {

SocialConductor::SocialConductor():
	cfg_(nullptr),
	As_(0.0),
	use_exponential_formulation_(true)
{}

void SocialConductor::initialize(std::shared_ptr<const humap_local_planner::FisParams> cfg) {
	cfg_ = cfg;
}

void SocialConductor::setEquationParameters(double as) {
	As_ = as;
}

bool SocialConductor::areFuzzyBehavioursDisabled() const {
	if (cfg_ == nullptr) {
		return true;
	}
	return cfg_->force_factor <= 0.0;
}

bool SocialConductor::computeBehaviourForce(
	const Pose& pose_agent,
	const double& speed_agent,
	const std::vector<Processor::FisOutput>& fis_outputs_v,
	const std::vector<double>& speeds_v,
	const std::vector<double>& dist_v,
	const std::vector<double>& rel_loc_v
) {
	// clear out behaviour force and active behavior names
	reset();

	// forces can be determined only if both FIS output and distance vector are defined (related to a single person)
	if (dist_v.size() != fis_outputs_v.size()
		|| dist_v.size() != speeds_v.size()
		|| dist_v.size() != rel_loc_v.size()
	) {
		return false;
	}

	const size_t size_ref = dist_v.size();
	// calculate combined output - superposed outputs
	for (size_t i = 0; i < size_ref; i++) {
		// force can't be computed if membership is invalid
		if (fis_outputs_v.at(i).membership <= 0.0) {
			continue;
		}

		// create a temporary unit vector pointing to direction determined by FIS output (in a local coordinate system)
		Vector v_temp(Angle(fis_outputs_v.at(i).value));

		// scale force vector with relevant factors
		double membership_factor = fis_outputs_v.at(i).membership;
		// 2 formulations of force strength (arising from spatiotemporal aspects) calculation are available
		double spatiotemporal_factor = 0.0;
		if (use_exponential_formulation_) {
			spatiotemporal_factor = computeBehaviourStrengthExponential(
				cfg_->human_action_range,
				dist_v.at(i),
				speed_agent,
				speeds_v.at(i)
			);
		} else {
			spatiotemporal_factor = computeBehaviourStrengthLinear(
				cfg_->human_action_range,
				dist_v.at(i),
				speed_agent,
				speeds_v.at(i)
			);
		}
		// include the FOV factor (in fact, it is one of spatiotemporal factors)
		double fov_factor = computeFovFactor(rel_loc_v.at(i));

		// total magnitude
		double force_magnitude = As_ * membership_factor * spatiotemporal_factor * fov_factor;

		// force - multiplied unit vector created based on a given direction
		behaviour_force_ += (v_temp * force_magnitude);

		updateActiveBehaviour(fis_outputs_v.at(i).term_name);
	}

	// convert resultant vector into global coordinates
	behaviour_force_.rotate(pose_agent.getYaw());

	// multiply times behaviour force factor
	behaviour_force_ *= cfg_->force_factor;

	return true;
}

void SocialConductor::reset() {
	behaviour_force_ = Vector(0.0, 0.0, 0.0);
	behaviour_active_str_.clear();
}

Vector SocialConductor::getSocialVector() const {
	return (behaviour_force_);
}

std::string SocialConductor::getBehaviourActive() const {
	// NOTE: rViz goes mad when tries to publish an empty string (application crashes)
	if ( behaviour_active_str_.empty() ) {
		return ("none");
	}
	return (behaviour_active_str_);
}

void SocialConductor::updateActiveBehaviour(const std::string& beh_name) {
	if ( behaviour_active_str_.empty() ) {
		behaviour_active_str_ = beh_name;
	} else {
		behaviour_active_str_.append("\n");
		behaviour_active_str_.append(beh_name);
	}
}

double SocialConductor::computeBehaviourStrengthLinear(
	const double& action_range,
	const double& dist_to_agent,
	const double& speed_agent,
	const double& speed_obstacle
) {
	// check if obstacle is too far away
	if (dist_to_agent > action_range) {
		return (0.0);
	}

	// in fact (SOCIAL_BEHAVIOUR_RANGE_END - SOCIAL_BEHAVIOUR_RANGE_START) but the start is 0.0
	double a_dist = -1.0 / action_range;
	// form of a line equation for readability, the independent variable is `dist_to_agent`
	double dist_factor = a_dist * dist_to_agent + 1.0;

	// compute impact of speed
	double speed_arg = speed_agent + speed_obstacle;
	// trim
	if (speed_arg > RELATIVE_SPEED_MAX) {
		speed_arg = RELATIVE_SPEED_MAX;
	}

	double a_speed = 1.0 / SocialConductor::RELATIVE_SPEED_MAX;
	double speed_factor = a_speed * speed_arg;

	return dist_factor * speed_factor;
}

double SocialConductor::computeBehaviourStrengthExponential(
	const double& action_range,
	const double& dist_to_agent,
	const double& speed_agent,
	const double& speed_obstacle
) {
	// check if the social agent is too far away
	if (dist_to_agent > action_range) {
		return (0.0);
	}

	double mutual_speed = speed_agent + speed_obstacle;
	// subtracted 1 so the factor becomes 0 for stationary objects
	double speed_factor = std::exp(+mutual_speed) - 1.0;
	double dist_factor = std::exp(-dist_to_agent);

	return speed_factor * dist_factor;
}

double SocialConductor::computeFovFactor(double rel_loc_angle) {
	switch (cfg_->fov_factor_method) {
		case(FovCalculationMethod::GAUSSIAN):
			return sfm::SocialForceModel::computeFactorFOV(rel_loc_angle, cfg_->fov, true);
		case(FovCalculationMethod::LINEAR):
			return sfm::SocialForceModel::computeFactorFOV(rel_loc_angle, cfg_->fov, false);
	}
	// otherwise, do not let FOV impact the resultant vector
	return 1.0;
}

} /* namespace fuzz */
} /* namespace humap_local_planner */
