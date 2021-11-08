/*
 * Processor.cpp
 *
 *  Created on: Aug 7, 2019
 *      Author: rayvburn
 */

#include <hubero_local_planner/fuzz/regions.h>
#include <hubero_local_planner/fuzz/processor.h>
#include <hubero_local_planner/geometry/angle.h>

// ------------------------------------------------------------------- //

namespace hubero_local_planner {
namespace fuzz {

Processor::Processor():
	alpha_dir_(0.0),
	engine_ptr_(new fl::Engine()),
	location_ptr_(new fl::InputVariable()),
	direction_ptr_(new fl::InputVariable()),
	trapezoid_out_("outwards", 10),
	trapezoid_cf_("cross_front", 10),
	trapezoid_cb_("cross_behind", 10),
	trapezoid_eq_("equal", 10, 20),
	trapezoid_opp_("opposite", 10, 20),
	social_behavior_ptr_(new fl::OutputVariable()),
	rule_block_ptr_(new fl::RuleBlock())
{
	/* Initialize engine */
	engine_ptr_->setName("SocialBehaviors");
	engine_ptr_->setDescription("");

	/* Initialize first input variable */
	location_ptr_->setName("location");
	location_ptr_->setDescription("");
	location_ptr_->setEnabled(true);
	location_ptr_->setRange(-IGN_PI, +IGN_PI);
	location_ptr_->setLockValueInRange(false);

	// `location` regions
	location_ptr_->addTerm(new fl::Triangle( "back",        IGN_DTOR(-180.0), IGN_DTOR(-180.0), IGN_DTOR(-160.0)));
	location_ptr_->addTerm(new fl::Trapezoid("back_right",  IGN_DTOR(-180.0), IGN_DTOR(-150.0), IGN_DTOR(-120.0), IGN_DTOR(-90.0)));
	location_ptr_->addTerm(new fl::Trapezoid("front_right", IGN_DTOR(-120.0), IGN_DTOR(-90.0),  IGN_DTOR(-30.0),  IGN_DTOR(0.0)));
	location_ptr_->addTerm(new fl::Triangle( "front",       IGN_DTOR(-20.0),  IGN_DTOR(0.0),    IGN_DTOR(+20.0)));
	location_ptr_->addTerm(new fl::Trapezoid("front_left",  IGN_DTOR(0.0),    IGN_DTOR(30.0),   IGN_DTOR(90.0),   IGN_DTOR(120.0)));
	location_ptr_->addTerm(new fl::Trapezoid("back_left",   IGN_DTOR(90.0),   IGN_DTOR(120.0),  IGN_DTOR(150.0),  IGN_DTOR(180.0)));
	location_ptr_->addTerm(new fl::Triangle( "back",        IGN_DTOR(+160.0), IGN_DTOR(+180.0), IGN_DTOR(+180.0)));
	engine_ptr_->addInputVariable(location_ptr_);

	/* Initialize second input variable */
	direction_ptr_->setName("direction");
	direction_ptr_->setDescription("");
	direction_ptr_->setEnabled(true);
	direction_ptr_->setRange(-IGN_PI, +IGN_PI);
	direction_ptr_->setLockValueInRange(false);

	// approach similar to VFH = vector field histogram
	// `direction` regions must be configured dynamically,
	// TODO: `cross_center`
	//
	// `outwards`
	direction_ptr_->addTerm(trapezoid_out_.getTrapezoids().at(0));
	direction_ptr_->addTerm(trapezoid_out_.getTrapezoids().at(1));
	// `cross_front`
	direction_ptr_->addTerm(trapezoid_cf_.getTrapezoids().at(0));
	direction_ptr_->addTerm(trapezoid_cf_.getTrapezoids().at(1));
	// `cross_behind`
	direction_ptr_->addTerm(trapezoid_cb_.getTrapezoids().at(0));
	direction_ptr_->addTerm(trapezoid_cb_.getTrapezoids().at(1));
	// `equal`
	direction_ptr_->addTerm(trapezoid_eq_.getTrapezoids().at(0));
	direction_ptr_->addTerm(trapezoid_eq_.getTrapezoids().at(1));
	// `opposite`
	direction_ptr_->addTerm(trapezoid_opp_.getTrapezoids().at(0));
	direction_ptr_->addTerm(trapezoid_opp_.getTrapezoids().at(1));
	engine_ptr_->addInputVariable(direction_ptr_);

    /* Initialize output variable */
    social_behavior_ptr_->setName("behavior");
    social_behavior_ptr_->setDescription("");
    social_behavior_ptr_->setEnabled(true);
    social_behavior_ptr_->setRange(-IGN_PI, +IGN_PI);
    social_behavior_ptr_->setLockValueInRange(false);
    social_behavior_ptr_->setAggregation(new fl::Maximum);
    social_behavior_ptr_->setDefuzzifier(new fl::Centroid());
    social_behavior_ptr_->setDefaultValue(fl::nan);
    social_behavior_ptr_->setLockPreviousValue(false);

	social_behavior_ptr_->addTerm(new fl::Trapezoid("accelerate",            IGN_DTOR(-30),  IGN_DTOR(-15),  IGN_DTOR(-15),  IGN_DTOR(+30)));
	social_behavior_ptr_->addTerm(new fl::Trapezoid("turn_right_accelerate", IGN_DTOR(+15),  IGN_DTOR(+30),  IGN_DTOR(+60),  IGN_DTOR(+75)));
	social_behavior_ptr_->addTerm(new fl::Trapezoid("turn_right",            IGN_DTOR(+60),  IGN_DTOR(+75),  IGN_DTOR(+105), IGN_DTOR(+120)));
	social_behavior_ptr_->addTerm(new fl::Trapezoid("turn_right_decelerate", IGN_DTOR(+105), IGN_DTOR(+120), IGN_DTOR(+140), IGN_DTOR(+155)));
	social_behavior_ptr_->addTerm(new fl::Trapezoid("decelerateA",           IGN_DTOR(+140), IGN_DTOR(+155), IGN_DTOR(+165), IGN_DTOR(+180)));
	social_behavior_ptr_->addTerm(new fl::Triangle( "stopA",                 IGN_DTOR(+165), IGN_DTOR(+180), IGN_DTOR(+195)));
	social_behavior_ptr_->addTerm(new fl::Trapezoid("decelerateB",           IGN_DTOR(-180), IGN_DTOR(-165), IGN_DTOR(-155), IGN_DTOR(-140)));
	social_behavior_ptr_->addTerm(new fl::Triangle( "stopB",                 IGN_DTOR(-195), IGN_DTOR(-180), IGN_DTOR(-165)));
	social_behavior_ptr_->addTerm(new fl::Trapezoid("turn_left_decelerate",  IGN_DTOR(-155), IGN_DTOR(-140), IGN_DTOR(-120), IGN_DTOR(-105)));
	social_behavior_ptr_->addTerm(new fl::Trapezoid("turn_left",             IGN_DTOR(-120), IGN_DTOR(-105), IGN_DTOR(-75),  IGN_DTOR(-60)));
	social_behavior_ptr_->addTerm(new fl::Trapezoid("turn_left_accelerate",  IGN_DTOR(-75),  IGN_DTOR(-60),  IGN_DTOR(-30),  IGN_DTOR(-15)));
	engine_ptr_->addOutputVariable(social_behavior_ptr_);

    /* Initialize rule block */
    rule_block_ptr_->setName("mamdani");
    rule_block_ptr_->setDescription("");
    rule_block_ptr_->setEnabled(true);
    rule_block_ptr_->setConjunction(new fl::Minimum); 			// fuzzylite/fuzzylite/fl/norm/t
    rule_block_ptr_->setDisjunction(new fl::AlgebraicSum);		// fuzzylite/fuzzylite/fl/norm/s
    // FIXME: AlgebraicProduct -> seems to choose not the highest membership term? IS THIS THE CAUSE?
    rule_block_ptr_->setImplication(new fl::Minimum); 			// AlgebraicProduct);
    rule_block_ptr_->setActivation(new fl::General);				// https://fuzzylite.github.io/fuzzylite/df/d4b/classfl_1_1_activation.html
    /*
	terminate called after throwing an instance of 'fl::Exception'
		what():  [conjunction error] the following rule requires a conjunction operator:
	location is front_right and direction is cross_front
	{at /src/rule/Antecedent.cpp::activationDegree() [line:114]}
     */

    /*
     * NOTE: It seems that there is a crucial aspect in `fuzzylite` Term naming convention.
     * The library immediately throws segfault after AddRule call was fed with a term name
     * consisting of a number (at least at the end, didn't check other cases).
     * This may be the problem with fl::Term class or Rule parser function.
     * The program crashes also when a Rule contains a name of a Term which has not
     * been defined yet (whose name is unknown).
	 * Template:
	 * rule_block_ptr_->addRule(fl::Rule::parse("if location is X and direction is Y then behavior is Z", engine_ptr_));
     */
	// location - `front`
    rule_block_ptr_->addRule(fl::Rule::parse("if location is front 	     and (direction is oppositeA     or direction is oppositeB    ) then behavior is turn_right",            engine_ptr_));
	rule_block_ptr_->addRule(fl::Rule::parse("if location is front 	     and (direction is outwardsA     or direction is outwardsB    ) then behavior is accelerate",            engine_ptr_));
    rule_block_ptr_->addRule(fl::Rule::parse("if location is front 	     and (direction is equalA        or direction is equalB       ) then behavior is decelerateA",           engine_ptr_));
	rule_block_ptr_->addRule(fl::Rule::parse("if location is front 	     and (direction is equalA        or direction is equalB       ) then behavior is decelerateB",           engine_ptr_));
	rule_block_ptr_->addRule(fl::Rule::parse("if location is front 	     and (direction is cross_frontA  or direction is cross_frontB ) then behavior is turn_right",            engine_ptr_));
    // location - `front_right`
    rule_block_ptr_->addRule(fl::Rule::parse("if location is front_right and (direction is cross_behindA or direction is cross_behindB) then behavior is turn_left",             engine_ptr_));
	rule_block_ptr_->addRule(fl::Rule::parse("if location is front_right and (direction is oppositeA     or direction is oppositeB    ) then behavior is turn_left",             engine_ptr_));
	rule_block_ptr_->addRule(fl::Rule::parse("if location is front_right and (direction is outwardsA     or direction is outwardsB    ) then behavior is turn_left",             engine_ptr_));
	rule_block_ptr_->addRule(fl::Rule::parse("if location is front_right and (direction is equalA        or direction is equalB       ) then behavior is turn_left_accelerate",  engine_ptr_));
	rule_block_ptr_->addRule(fl::Rule::parse("if location is front_right and (direction is cross_frontA  or direction is cross_frontB ) then behavior is turn_right",            engine_ptr_));
    // location - `back_right`
    rule_block_ptr_->addRule(fl::Rule::parse("if location is back_right  and (direction is cross_behindA or direction is cross_behindB) then behavior is turn_left_accelerate",  engine_ptr_));
	rule_block_ptr_->addRule(fl::Rule::parse("if location is back_right  and (direction is oppositeA     or direction is oppositeB    ) then behavior is turn_right",            engine_ptr_));
	rule_block_ptr_->addRule(fl::Rule::parse("if location is back_right  and (direction is equalA        or direction is equalB       ) then behavior is turn_right_accelerate", engine_ptr_));
	rule_block_ptr_->addRule(fl::Rule::parse("if location is back_right  and (direction is cross_frontA  or direction is cross_frontB ) then behavior is accelerate",            engine_ptr_));
	// location - `back`
	// none
	// location - `back_left`
	rule_block_ptr_->addRule(fl::Rule::parse("if location is back_left   and (direction is cross_behindA or direction is cross_behindB) then behavior is accelerate",            engine_ptr_));
	// location - `front_left`
    rule_block_ptr_->addRule(fl::Rule::parse("if location is front_left  and (direction is cross_behindA or direction is cross_behindB) then behavior is turn_right",            engine_ptr_));
	rule_block_ptr_->addRule(fl::Rule::parse("if location is front_left  and (direction is cross_frontA  or direction is cross_frontB ) then behavior is turn_right_accelerate", engine_ptr_));
	try {
		engine_ptr_->addRuleBlock(rule_block_ptr_);
	} catch (fl::Exception& what) {
		std::cout << what.what() << std::endl;
	}
    // FIXME: below does not overwrite rule block's settings
    engine_ptr_->configure("AlgebraicProduct", "AlgebraicSum", "AlgebraicProduct", "Maximum", "Centroid", "General");
}

// ------------------------------------------------------------------- //

void Processor::printFisConfiguration() const {
	// system basic information
	std::cout << "Fuzzy Inference System configuration" << std::endl;
	std::cout << "\tinputs: " << engine_ptr_->numberOfInputVariables() <<
		", outputs: " << engine_ptr_->numberOfOutputVariables() <<
		", rule_blocks: " << engine_ptr_->numberOfRuleBlocks() << std::endl;
	std::string status;
	std::cout << "\tFIS engine ready flag: " << engine_ptr_->isReady(&status) << " (" << status << ")" << std::endl;
	// print rules
	std::cout << "FIS 'Processor' class rule block" << std::endl << rule_block_ptr_->toString() << std::endl;
}

// ------------------------------------------------------------------- //

bool Processor::load(const double &dir_alpha, const std::vector<double> &dir_beta_v,
					 const std::vector<double> &rel_loc_v, const std::vector<double> &dist_angle_v)
{

	// clear output vector
	output_v_.clear();

	// the same length is a MUST
	if ( (dir_beta_v.size() == rel_loc_v.size()) && (rel_loc_v.size() == dist_angle_v.size()) ) {
		alpha_dir_ = dir_alpha;
		beta_dir_ = dir_beta_v;
		rel_loc_ = rel_loc_v;
		d_alpha_beta_angle_ = dist_angle_v;
		return (true);
	}

	// reset just in case
	alpha_dir_ = 0.0;
	beta_dir_.clear();
	rel_loc_.clear();
	d_alpha_beta_angle_.clear();

	return (false);

}

// ------------------------------------------------------------------- //

void Processor::process() {

	// iterate over all vector elements (all vectors have the same size);
	// beta_dir's size is an arbitrarily chosen count reference here
	for ( size_t i = 0; i < beta_dir_.size(); i++ ) {

		// reset the output region name
		std::string term_name = "";

		// update the location input variable
		location_ptr_->setValue(fl::scalar(rel_loc_.at(i)));

		// update `direction_` regions according to value previously set
		updateRegions(alpha_dir_, beta_dir_.at(i), d_alpha_beta_angle_.at(i), rel_loc_.at(i));

		// calculate the gamma angle for the current alpha-beta configuration
		Angle gamma(d_alpha_beta_angle_.at(i) - alpha_dir_ - beta_dir_.at(i));
		direction_ptr_->setValue(fl::scalar(gamma.getRadian()));

		// execute fuzzy calculations
		engine_ptr_->process();

		fl::scalar y_highest_temp = fl::nan;
		fl::Term* term_highest_ptr = social_behavior_ptr_->highestMembership(social_behavior_ptr_->getValue(), &y_highest_temp);

		// check whether proper term was found, if not - `nullptr` will be detected
		if ( term_highest_ptr != nullptr ) {
			term_name = term_highest_ptr->getName();
		}

		/**
		 * NOTE: fl::variable::fuzzyOutputValue() returns a list of available terms
		 * with a corresponding membership
		 * WHEREAS fl::variable fl::variable::fuzzify(fl::scalar) returns the
		 * same list but with NORMALIZED membership?
		 * the effect is as: 	fuzzyOutputValue()	-> 	0.222/turn_right_decelerate
		 * 					fuzzify(fl::scalar)	-> 	1.000/turn_right_decelerate
		 * when only single term has non-zero membership.
		 * When multiple (2) terms have non-zero membership then results
		 * are as follow:
		 * fuzzyOutputValue()	-> 	0.239/turn_left_accelerate + 0.266/accelerate + 0.541/go_along
		 * fuzzify(fl::scalar)	-> 	0.000/turn_left_accelerate + 0.000/accelerate + 1.000/go_along
		 */
		// FIXME: make it like the absolute function (decreasing on both side of the edge - 0.5)
		double fitness = static_cast<double>(social_behavior_ptr_->getValue());
		fitness = fitness - std::floor(fitness);

		output_v_.push_back(std::make_tuple(term_name, fitness));

	}

}

// ------------------------------------------------------------------- //

std::vector<std::tuple<std::string, double> > Processor::getOutput() const {
	return (output_v_);
}

// ------------------------------------------------------------------- //

Processor::~Processor() {
	// free memory allocated in ctor;
	// Note that fl::Engine manages deletion of all objects that it receives:
	// https://github.com/fuzzylite/fuzzylite/blob/7aee562d6ca17f3cf42588ffb5116e03017c3c50/fuzzylite/src/Engine.cpp#L96
	delete engine_ptr_;
}

// ------------------------------------------------------------------- //

std::vector<std::tuple<std::string, double>> Processor::membershipInputRelLoc() const {
	return Processor::highestMembership(location_ptr_);
}

// ------------------------------------------------------------------- //

std::vector<std::tuple<std::string, double>> Processor::membershipInputDirCross() const {
	return Processor::highestMembership(direction_ptr_);
}

// ------------------------------------------------------------------- //
// PROTECTED
std::vector<std::tuple<std::string, double>> Processor::highestMembership(const fl::InputVariable* input_ptr) {
	const std::vector<fl::Term*> terms = input_ptr->terms();
	double max_membership = 0.0;
	std::vector<std::tuple<std::string, double>> v_terms_memberships;
	for (const auto term : terms) {
		double membership = term->membership(input_ptr->getValue());
		if (membership > max_membership) {
			v_terms_memberships.clear();
			v_terms_memberships.push_back(std::make_tuple(term->getName(), membership));
			max_membership = membership;
		} else if (membership == max_membership) {
			v_terms_memberships.push_back(std::make_tuple(term->getName(), membership));
		}
	}
	return v_terms_memberships;

	// dummy version - returns first (alphabetically) term that has maximum membership
	// returns std::tuple<std::string, double>
	//
	// std::string term_name("unknown");
	// fl::scalar y_highest_temp = fl::nan;
	// fl::Term* term_highest_ptr = input_ptr->highestMembership(input_ptr->getValue(), &y_highest_temp);

	// // check whether proper term was found, if not - `nullptr` will be detected
	// if ( term_highest_ptr != nullptr ) {
	// 	term_name = term_highest_ptr->getName();
	// }
	// return std::make_tuple(term_name, static_cast<double>(y_highest_temp));
}

// ------------------------------------------------------------------- //
// PRIVATE
void Processor::updateRegions(const double &alpha_dir, const double &beta_dir, const double &d_alpha_beta_angle, const double &rel_loc) {

	// calculate threshold angle values, normalize
	Angle gamma_eq(d_alpha_beta_angle - 2 * alpha_dir);
	Angle gamma_opp(gamma_eq.getRadian() - IGN_PI);
	Angle gamma_cc(IGN_PI - alpha_dir);

	// compute relative location (`side`)
	RelativeLocation side = decodeRelativeLocation(rel_loc);

	// trapezoid's specific points (vertices), see `fuzzylite` doc for details:
	// https://fuzzylite.github.io/fuzzylite/d0/d26/classfl_1_1_trapezoid.html

	/* - - - - - Terms sensitive to side changes - - - - - */
	// `outwards`
	trapezoid_out_.update(side, gamma_eq, gamma_opp);
	// `cross_front`
	trapezoid_cf_.update(side, gamma_cc, gamma_eq);
	// `cross_behind`
	trapezoid_cb_.update(side, gamma_opp, gamma_cc);

	/* - - - - - Terms insensitive to side changes - - - - - */
	// `equal`
	trapezoid_eq_.update(gamma_eq);
	// `opposite`
	trapezoid_opp_.update(gamma_opp);

}

// ------------------------------------------------------------------- //

RelativeLocation Processor::decodeRelativeLocation(const double &rel_loc) const {

	if ( rel_loc <= 0.0 ) {
		// right side
		return RelativeLocation::LOCATION_RIGHT;
	} else if ( rel_loc > 0.0 ) {
		// left side
		return RelativeLocation::LOCATION_LEFT;
	}
	return RelativeLocation::LOCATION_UNSPECIFIED;

}

// ------------------------------------------------------------------- //

} /* namespace fuzz */
} /* namespace hubero_local_planner */
