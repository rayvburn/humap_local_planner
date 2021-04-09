/*
 * SocialBehavioursDb.h
 *
 *  Created on: Sep 15, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_FUZZ_SOCIALBEHAVIOURSDB_H_
#define INCLUDE_FUZZ_SOCIALBEHAVIOURSDB_H_

#include <hubero_common/typedefs.h>
#include <hubero_local_planner/hubero_config.h>

namespace fuzz {

class SocialBehavioursDb {

public:

	/// \brief Parametrized constructor
//	/// \param social_force_strength is a maximum length of the social force vector
	SocialBehavioursDb(); // const double &social_force_strength

	/// \brief Updates internal state according to a given structure's content
	void initialize(hubero_local_planner::HuberoConfigConstPtr cfg);

	/// \brief Destructor
	virtual ~SocialBehavioursDb();

protected:

	Vector3 turnLeft();
	Vector3 turnLeftAccelerate();
	Vector3 goAlong();
	Vector3 accelerate();
	Vector3 turnRightAccelerate();
	Vector3 turnRight();
	Vector3 turnRightDecelerate();
	Vector3 stop();
	Vector3 decelerate();

	/// \brief Updates the distance between the currently investigated
	/// person and the other dynamic object (another person or a robot).
	/// \param d_alpha_beta expresses distance between alpha and currently
	/// investigated object
	void setDistance(const double &d_alpha_beta);

	/// \brief Direction of motion of the alpha object (currently investigated)
	/// \param dir is a direction expressed in world's Z axis rotation (to match
	/// the world's X axis with the alpha's direction)
	void setDirection(const double &dir);

	/// \brief Updates the internal vector of force created by the SFM algorithm
	void setForce(const Vector3 &force);

protected:

	/// \brief The SFM-generated force vector
	Vector3 force_;

	/// \brief Distance between alpha and currently investigated object
	double d_alpha_beta_length_;

	/// \brief Direction of motion of the alpha object
	double dir_alpha_;

	/// \brief Stores i.a. maximum magnitudes of the force generated by the behaviour
	/// as well as resultant force multiplier
	hubero_local_planner::HuberoConfigConstPtr cfg_;

private:

	/// \brief Creates a unit vector pointing to the given direction
	/// \param direction is a direction of the vector
	/// \return result
	Vector3 createDirVector(const double &direction) const;

	/// \brief Changes length of a given vector.
	/// \param v is a vector whose magnitude needs to be changed
	/// \param magnitude is a new length of a vector
	/// \return resulting vector
	Vector3 setVectorLength(const Vector3 &v, const double &magnitude) const;

	/// \brief Extends the given vector `v` along the given direction `dir`.
	/// \param v is a vector whose length needs to be extended
	/// \param dir is a direction which vector needs to be extended along
	/// \param magnitude is a length of the extension
	/// \return result of the operation (another vector)
	Vector3 extendVector(const Vector3 &v, const double &dir, const double &magnitude) const;

	/// \brief Calculates orientation (direction) according to a given
	/// direction `dir` so the new dir points to the proper side (+/- 90 degrees
	/// rotation relative to current direction).
	/// \param dir is current alpha direction
	/// \param side is `r` (right) or `l` (left) side indicator
	/// \return result
	double findOrientation(const double &dir, const char &side) const;

	/// \brief Knowing the maximum allowable force, calculates the linear dependency between
	/// d_alpha_beta_length and magnitude of the social vector. The line's equation is determined
	/// by the `max` magnitude and `social force` operation range.
	/// \param max is a maximum allowable magnitude of a vector
	/// \return Vector length based on `max` and `d_alpha_beta_length`
	double calculateVectorMagnitude(const double &max) const;

	/// \brief Strengthens a force generated in the behaviour handler
	/// so a sum of that force and the SFM result creates a vector
	/// pointing to a proper direction (according to the handler)
	Vector3 assertForceDirection(const Vector3 &force_beh) const;
};

} /* namespace fuzz */

#endif /* INCLUDE_FUZZ_SOCIALBEHAVIOURSDB_H_ */