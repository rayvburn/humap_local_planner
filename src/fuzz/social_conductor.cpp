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

SocialConductor::SocialConductor() {}

// ------------------------------------------------------------------- //

void SocialConductor::apply(const Vector &force_combined, const double &dir_alpha,
							const std::vector<double> &dist_v,
							const std::vector<std::tuple<std::string, double> > &fuzz_output_v)
{

	// moved from reset
	sf_result_ = Vector();
	behaviour_active_str_.clear();

	this->setDirection(dir_alpha);
	this->setForce(force_combined);
	std::vector<Vector> forces;

	for ( size_t i = 0; i < fuzz_output_v.size(); i++ ) {

		this->setDistance(dist_v.at(i));
		std::string term_name = std::get<0>(fuzz_output_v.at(i));

		/* Behaviour selection based on the highest membership term name */

		if ( term_name == "turn_left" ) {

			// apply `turn left` behavior
			forces.push_back(this->turnLeft());
			updateActiveBehaviour("turn_left"); // create a list of activated behaviours

		} else if ( term_name == "turn_left_accelerate" ) {

			// apply `turn left and accelerate` behavior
			forces.push_back(this->turnLeftAccelerate());
			updateActiveBehaviour("turn_left_accelerate");

		} else if ( term_name == "accelerate" ) {

			// apply `accelerate` behavior
			forces.push_back(this->accelerate());
			updateActiveBehaviour("accelerate");

		} else if ( term_name == "go_along" ) {

			// apply `go along` behavior
			forces.push_back(this->goAlong());
			updateActiveBehaviour("go_along");

		} else if ( term_name == "decelerate" ) {

			// apply `decelerate` behavior
			forces.push_back(this->decelerate());
			updateActiveBehaviour("decelerate");

		} else if ( term_name == "stop" ) {

			// apply `stop` behavior
			forces.push_back(this->stop());
			updateActiveBehaviour("stop");

		} else if ( term_name == "turn_right_decelerate" ) {

			// apply `turn right and decelerate` behavior
			forces.push_back(this->turnRightDecelerate());
			updateActiveBehaviour("turn_right_decelerate");

		} else if ( term_name == "turn_right" ) {

			// apply `turn right` behavior
			forces.push_back(this->turnRight());
			updateActiveBehaviour("turn_right");

		} else if ( term_name == "turn_right_accelerate" ) {

			// apply `turn right and accelerate` behavior
			forces.push_back(this->turnRightAccelerate());
			updateActiveBehaviour("turn_right_accelerate");

		} else {

			// prevents calling `superpose` when [input] is invalid
			// do not update the string to prevent having `none`
			// among valid behaviours
			return;

		}

	}
	superpose(forces);
}

// ------------------------------------------------------------------- //

void SocialConductor::reset() {
	behaviour_active_str_ = "idle";
	sf_result_ = Vector();
}

// ------------------------------------------------------------------- //

Vector SocialConductor::getSocialVector() const {
	return (sf_result_);
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

SocialConductor::~SocialConductor() { }

// ------------------------------------------------------------------- //

void SocialConductor::superpose(const std::vector<Vector> &forces) {

	// save a max length of a component to know
	// if there are few meaningful (i.e. non-zero length-ed)
	// elements in the vector but apparently their sum
	// is near-zero;
	// used to determine which behaviours are actually activated
	double max_len = 0.0;

	// TODO: add crowd support - actual superposition
	// NOW: average value of vectors
	Vector avg;
	for ( size_t i = 0; i < forces.size(); i++ ) {
		avg += forces.at(i);
		double len = forces.at(i).calculateLength();
		if ( len > max_len ) { max_len = len; }
	}
	avg /= forces.size();

	// assign averaged vector to the resultative one
	sf_result_ = avg;

	// check validity
	if ( std::isnan(sf_result_.getX()) || std::isnan(sf_result_.getY()) ) {
		sf_result_ = Vector();
		max_len = 0.0;
	}

	// force `none` behaviour when `max len` is very small
	if ( max_len < 1e-03 ) {
		behaviour_active_str_.clear();
	}

	// at the end, let's apply multiplier
	sf_result_ *= this->cfg_->force_factor;

}

// ------------------------------------------------------------------- //

void SocialConductor::updateActiveBehaviour(const std::string &beh_name) {

	if ( behaviour_active_str_.empty() ) {
		behaviour_active_str_ = beh_name;
	} else {
		behaviour_active_str_.append("\n");
		behaviour_active_str_.append(beh_name);
	}

}

// ------------------------------------------------------------------- //

} /* namespace fuzz */
} /* namespace hubero_local_planner */
