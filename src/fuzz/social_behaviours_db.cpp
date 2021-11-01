/*
 * SocialBehavioursDb.cpp
 *
 *  Created on: Sep 15, 2019
 *      Author: rayvburn
 */

#include <hubero_local_planner/fuzz/social_behaviours_db.h>

namespace hubero_local_planner {
namespace fuzz {

// ------------------------------------------------------------------- //

SocialBehavioursDb::SocialBehavioursDb()
	: 	d_alpha_beta_length_(0.0), dir_alpha_(0.0)
{}

// ------------------------------------------------------------------- //

SocialBehavioursDb::~SocialBehavioursDb() { }

// ------------------------------------------------------------------- //
// ------------------------------------------------------------------- //
// -------PROTECTED-MEMBERS-SECTION----------------------------------- //
// ------------------------------------------------------------------- //
// ------------------------------------------------------------------- //
void SocialBehavioursDb::setDistance(const double& d_alpha_beta) {
	d_alpha_beta_length_ = d_alpha_beta;
}

 // ------------------------------------------------------------------- //

void SocialBehavioursDb::setDirection(const double& dir) {
	dir_alpha_ = dir;
}

// ------------------------------------------------------------------- //

void SocialBehavioursDb::setForce(const Vector& force) {
	force_ = force;
}

// ------------------------------------------------------------------- //

Vector SocialBehavioursDb::turnLeft() {

	Vector force_beh;

	// create desired force vector
	double dir_rot = findOrientation(dir_alpha_, 'l');
	force_beh = createDirVector(dir_rot);
	double magnitude = calculateVectorMagnitude(cfg_->turn_left);
	force_beh = setVectorLength(force_beh, magnitude);

	// make sure that SFM result added to the SocialConductor's result
	// creates a vector pointing to the proper side
//	force_beh = assertForceDirection(force_beh);

	return (force_beh);

	// create diff vector: diff = behaviour_force - `social force`
	//return (force_beh - force_);

}

// ------------------------------------------------------------------- //

Vector SocialBehavioursDb::turnLeftAccelerate() {

	Vector force_beh;

	// rotation section (turning)
	double dir_rot = findOrientation(dir_alpha_, 'l');
	force_beh = createDirVector(dir_rot);
	double magnitude = calculateVectorMagnitude(cfg_->turn_left_accelerate_turn);
	force_beh = setVectorLength(force_beh, magnitude);

	// acceleration section
	magnitude = calculateVectorMagnitude(cfg_->turn_left_accelerate_acc);
	force_beh = extendVector(force_beh, dir_alpha_, magnitude);

//	force_beh = assertForceDirection(force_beh);

	return (force_beh);
	//return (force_beh - force_);

}

// ------------------------------------------------------------------- //

Vector SocialBehavioursDb::goAlong() {

	// do nothing
	return (Vector());

}

// ------------------------------------------------------------------- //

Vector SocialBehavioursDb::accelerate() {

	Vector force_beh;

	// acceleration section
	double magnitude = calculateVectorMagnitude(cfg_->accelerate);

	force_beh = createDirVector(dir_alpha_);
	force_beh = setVectorLength(force_beh, magnitude);

//	force_beh = assertForceDirection(force_beh);

	return (force_beh);
	//return (force_beh - force_);
}

// ------------------------------------------------------------------- //

Vector SocialBehavioursDb::turnRightAccelerate() {

	Vector force_beh;

	// rotation section (turning)
	double dir_rot = findOrientation(dir_alpha_, 'r');
	force_beh = createDirVector(dir_rot);

	double magnitude = calculateVectorMagnitude(cfg_->turn_right_accelerate_turn);

	force_beh = setVectorLength(force_beh, magnitude);

	// acceleration section
	magnitude = calculateVectorMagnitude(cfg_->turn_right_accelerate_acc);
	force_beh = extendVector(force_beh, dir_alpha_, magnitude);

//	force_beh = assertForceDirection(force_beh);

	return (force_beh);
	//return (force_beh - force_);

}

// ------------------------------------------------------------------- //

Vector SocialBehavioursDb::turnRight() {

	Vector force_beh;

	double dir_rot = findOrientation(dir_alpha_, 'r');
	force_beh = createDirVector(dir_rot);

	double magnitude = calculateVectorMagnitude(cfg_->turn_right);

	force_beh = setVectorLength(force_beh, magnitude);

//	force_beh = assertForceDirection(force_beh);

	return (force_beh);
	//return (force_beh - force_);

}

// ------------------------------------------------------------------- //

Vector SocialBehavioursDb::turnRightDecelerate() {

	Vector force_beh;

	// rotation section (turning)
	double dir_rot = findOrientation(dir_alpha_, 'r');
	force_beh = createDirVector(dir_rot);

	double magnitude = calculateVectorMagnitude(cfg_->turn_right_decelerate_turn);

	force_beh = setVectorLength(force_beh, magnitude);

	// deceleration section
	dir_rot = findOrientation(dir_alpha_, 'o');
	magnitude = calculateVectorMagnitude(cfg_->turn_right_decelerate_dec);
	force_beh = extendVector(force_beh, dir_rot, magnitude);

//	force_beh = assertForceDirection(force_beh);

	return (force_beh);
	//return (force_beh - force_);

}

// ------------------------------------------------------------------- //

Vector SocialBehavioursDb::stop() {

	Vector force_beh;

	// TODO: resulting force vector is needed
	double dir_rot = findOrientation(dir_alpha_, 'o');
	force_beh = createDirVector(dir_rot);

	double magnitude = calculateVectorMagnitude(cfg_->stop);

	force_beh = setVectorLength(force_beh, magnitude);

	force_beh = assertForceDirection(force_beh);

	return (force_beh);
	//return (force_beh - force_);

}

// ------------------------------------------------------------------- //

Vector SocialBehavioursDb::decelerate() {

	Vector force_beh;

	// acceleration section
	double magnitude = calculateVectorMagnitude(cfg_->decelerate);

	double dir_opp = findOrientation(dir_alpha_, 'o');
	force_beh = createDirVector(dir_opp);
	force_beh = setVectorLength(force_beh, magnitude);

//	force_beh = assertForceDirection(force_beh);

	return (force_beh);
	//return (force_beh - force_);

}

// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //
// -------PRIVATE-MEMBERS-SECTION------------------------------------- //
// ------------------------------------------------------------------- //

Vector SocialBehavioursDb::createDirVector(const double& direction) const {

	return Vector(Angle(direction));

}

// ------------------------------------------------------------------- //

Vector SocialBehavioursDb::setVectorLength(const Vector& v,
		const double& magnitude) const {

	Vector v_new = v.normalized();
	v_new *= magnitude;
	return (v_new);

}

// ------------------------------------------------------------------- //

Vector SocialBehavioursDb::extendVector(const Vector& v, const double& dir,
		const double& magnitude) const {

	// create unit vector pointing in the direction according to `dir`
	Vector v_ext = createDirVector(dir);

	// extend the vector according to magnitude
	v_ext *= magnitude;

	// sum up 2 vectors
	return (v_ext + v);

}

// ------------------------------------------------------------------- //

double SocialBehavioursDb::findOrientation(const double& dir, const char& side) const {

	Angle dir_new(dir);

	if ( side == 'l' ) {			// LEFT
		dir_new.setRadian(dir_new.getRadian() + IGN_PI_2);
	} else if ( side == 'r' ) {		// RIGHT
		dir_new.setRadian(dir_new.getRadian() - IGN_PI_2);
	} else if ( side == 'o' ) {		// OPPOSITE
		dir_new.setRadian(dir_new.getRadian() + IGN_PI);
	}

	return dir_new.getRadian();

}

// ------------------------------------------------------------------- //

double SocialBehavioursDb::calculateVectorMagnitude(const double& max) const {

	const double X_SOCIAL_RANGE = 4.0; 					// in meters

	// no social force vector is generated if obstacle is too far away
	if ( d_alpha_beta_length_ > X_SOCIAL_RANGE ) {
		return (0.0);
	}

	double a = -(max)/(X_SOCIAL_RANGE); 			// in fact (X_SOCIAL_RANGE_END - X_SOCIAL_RANGE_START) but the start is 0.0
	double y = a * d_alpha_beta_length_ + max; 		// form of a line equation for readability,
													// the independent variable is `d_alpha_beta_length_`
	return (y);

}

// ------------------------------------------------------------------- //

Vector SocialBehavioursDb::assertForceDirection(const Vector& force_beh) const {

	Vector result = force_beh + force_;
	Vector force_beh_strengthened = force_beh;
	Angle angle_res;
	Angle angle_beh;

	for ( size_t i = 0; i < 10; i++ ) {

		result = force_beh_strengthened + force_;
		angle_res = Angle(result);
		angle_beh = Angle(force_beh);

		if ( std::abs(angle_res.getRadian() - angle_beh.getRadian()) < IGN_PI_2 ) {
			break;
		} else {
			force_beh_strengthened *= 1.20;
		}

	}
	return (force_beh_strengthened);

}

// ------------------------------------------------------------------- //

} /* namespace fuzz */
} /* namespace hubero_local_planner */
