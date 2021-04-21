/*
 * SocialForceModel.h
 *
 *  Created on: Dec 4, 2018
 *      Author: rayvburn
 */

#pragma once

#include <hubero_common/typedefs.h>
#include <hubero_common/shift_register.h>

// environment obstacle representation
#include <hubero_local_planner/hubero_config.h>
#include <hubero_local_planner/sfm/world.h>

// C++ STL
#include <vector>	// closest points
#include <tuple>	// rel_loc
#include <chrono>

// Actor's data sto	rage

// ----------------------------------------------------------------------------------------------- //
/*
 * References:
 * 		- D. Helbing et al. 	- Social Force Model for Pedestrian Dynamics ‎(1998)
 * 		- Moussaid et al. 		- Experimental study of the behavioural mechanisms underlying
 * 		  						  self-organization in human crowds (2009)
 * 		- S. Seer et al. 		- Validating Social Force based Models with Comprehensive
 * 		  						  Real World Motion Data (2014)
 *
 * https://www.sciencedirect.com/science/article/pii/S2352146514001161
 * In the above paper there are results of the research presented. They point that
 * Model C (Rudloff et al. (2011) of SFM fits best the real world data.
 */
// ----------------------------------------------------------------------------------------------- //

namespace sfm {

// ---------------------------------

typedef enum {

	/** `The repulsive force from static obstacles f αi is modeled by using the functional
	 * form as given by the repulsive force for elliptical formulation` - 2014 */
	INTERACTION_ELLIPTICAL = 0,		// a.k.a. v2014

	/** `repulsion from walls uses the same formulas as the repulsion from other
	 * pedestrians` - 2011 */
	INTERACTION_REPULSIVE_EVASIVE	// a.k.a. v2011

} StaticObjectInteraction;

// ---------------------------------

/** 	φ_αβ issue
 * There is an inconsistency in papers connected with the Rudloff's version of Social Force model -
 * in Rudloff et al. 2011 - https://www.researchgate.net/publication/236149039_Can_Walking_Behavior_Be_Predicted_Analysis_of_Calibration_and_Fit_of_Pedestrian_Models
 * there is a statement that theta_alpha_beta is an "angle between velocity of pedestrian α and the displacement of pedestrian β"
 * whereas in Seer et al. 2014 - https://www.sciencedirect.com/science/article/pii/S2352146514001161
 * they say that in this model "φ αβ is the angle between n α and d αβ" (they call it phi instead of theta)
 */

/** 	n_α issue
 * Another inconsistency between 2011 and 2014 papers connected to Rudloff's SFM version is n_alpha issue.
 * In 2011 original paper there is said that n_alpha is "pointing in the opposite direction to the walking
 * direction (deceleration force)".
 * On the other hand in 2014 paper (that Rudloff is co-author of) they say: "n α is the direction of movement
 * of pedestrian α".
 */

typedef enum {

	/** " φ_αβ is an angle between velocity of pedestrian α and the displacement of pedestrian β "
	 * " n_α is pointing in the opposite direction to the walking direction (deceleration force) "
	 *   2011 */
	PARAMETER_DESCRIPTION_2011 = 0,

	/** " φ_αβ is the angle between n_α and d_αβ "
	 * " n_α is the direction of movement of pedestrian α "
	 *   2014 */
	PARAMETER_DESCRIPTION_2014,

	/** Connected only with another φ_αβ angle description:
	 * NOTE: below method of calculating the angle is only correct when both objects are:
	 * 		o dynamic,
	 * 		o currently moving,
	 * 		o already aligned with the to-target-direction,
	 * 		o there are no obstacles in the environment that will make the object not move along a straight line.
	 * NOT RECOMMENDED */
	PARAMETER_DESCRIPTION_UNKNOWN

} ParameterDescription;

// ---------------------------------

/** Related to an issue connected with a case when actor's
 * direction is opposite to the resulting force direction.
 * Most likely occurs when actor is forced by an other
 * object to turn out of the way pointing towards his
 * target. The force generated by the algorithm in such
 * situation is usually so strong that causes actor
 * 3/4 rotation first and then he keeps trying to reach
 * the goal.
 * Opposite force threshold is 0.85 * PI. */
typedef enum {

	/** Despite of the force pulling actor in the opposite direction
	  * (relative to the current movement direction) he will
	  * still try to go towards his goal. */
	OPPOSITE_FORCE_GO_TOWARDS_GOAL,

	/**
	  * The force is pulling the actor in the opposite direction but
	  * he tries to rotate left. This prevents force oscillation effect
	  * to cast down onto the actor's behaviour.
	  * Letting the raw force to lead the actor will most likely
	  * cause some 'collision' because oscillations from 1 side
	  * to the other will lock any stronger rotations
	  * and actor will accelerate going straight ahead
	  * (yes, opposite to the SFM algorithm result).
	  */
	OPPOSITE_FORCE_ROTATE_LEFT

} OppositeForceMethod;

// ---------------------------------

class SocialForceModel {

public:

	/// \brief Default constructor
	SocialForceModel();

	/// \brief Function which sets internal parameters according to loaded parameters
	inline void init(std::shared_ptr<const hubero_local_planner::SfmParams> cfg) {
		cfg_ = cfg;
	}

	/// \brief Function which calculates social force
	/// for an actor taking whole world's objects
	/// into consideration
	bool computeSocialForce(
			const World& world,
			const double &dt,
			std::vector<Distance>& meaningful_static,
			std::vector<Distance>& meaningful_dynamic
	);

	/// \brief Returns internal force vector
	inline Vector3 getForceInternal() const {
		return (force_internal_);
	}
	/// \brief Returns interaction force vector
	inline Vector3 getForceInteraction() const {
		return (force_interaction_);
	}
	/// \brief Returns combined force vector (internal
	/// and interaction components summed up)
	inline Vector3 getForceCombined() const {
		return (force_combined_);
	}

	/// \brief Default destructor
	virtual ~SocialForceModel() = default;

private:

	/// \brief Helper function which assigns randomly
	/// generated numbers (with a proper mean and std. dev)
	/// to an algorithm parameters An, ... , Bw
	void setParameters();

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Social Force Model's components section (internal acceleration,
	// interaction force (dynamic/static) )

	/// \brief Helper function which calculates the internal force
	/// term of an actor; this component describes a person's
	/// motivation to reach its current goal
	Vector3 computeInternalForce(const Robot& robot);

	/// \brief Helper function which calculates interaction
	/// force which another object (static or dynamic)
	/// exerts on the actor (tuple's first element).
	/// Additionally, a distance vector to the currently
	/// considered obstacle is returned as the tuple's second
	/// element. Third element is a length of that vector.
	Vector3 computeInteractionForce(const Robot& robot, const DynamicObject& object); // TODO: const?

	/// \brief Helper function which computes a repulsive
	/// force which static obstacle exerts on the actor;
	/// fits 2014 configuration and used only in this case.
	Vector3 computeInteractionForce(const Robot& robot, const StaticObject& object, const double &dt);

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Section covering more of a geometry-related functions
	/* Theta alpha-beta calculation */

	/// \brief Helper function which computes theta_αβ angle;
	/// fits 2011 configuration, where this angle is defined
	/// as: "an angle between velocity of pedestrian α and
	/// the displacement of pedestrian β"
	double computeThetaAlphaBetaAngle2011(const Robot& robot, const DynamicObject& object);

	/// \brief Helper function which computes theta_αβ angle;
	/// fits 2014 configuration, where this angle is defined
	/// as: "an angle between n_α and d_αβ"
	/// \[param in] n_alpha - actor's normal (based on velocity
	/// vector)
	/// \[param in] d_alpha_beta - vector between objects
	/// positions
	double computeThetaAlphaBetaAngle2014(const Vector3 &n_alpha, const Vector3 &d_alpha_beta);

	/// \brief Helper function which computes theta_αβ angle;
	/// works only for dynamically moving actors -
	/// NOT RECOMMENDED
	double computeThetaAlphaBetaAngle(const ignition::math::Angle &actor_yaw,
			const ignition::math::Angle &object_yaw
	);

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	/* Resultative vector components calculation - normal (n_alpha)
	 * and perpendicular (p_alpha) */

	/// \brief Helper function which computes a vector
	/// that is normal to alpha's direction vector;
	/// depending on the `parameter description` set
	/// the vector is calculated differently
	Vector3 computeNormalAlphaDirection(const ignition::math::Pose3d &actor_pose);

	/// \brief Helper function which takes a given n_alpha
	/// vector and based on currently investigated object's
	/// relative to the actor location calculates a perpendicular
	/// to n_alpha vector
	Vector3 computePerpendicularToNormal(const Vector3 &n_alpha,
			const RelativeLocation &beta_rel_location);

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	/* General functions section (still geometry-related) */
	/// \brief Helper function which checks whether a given
	/// angle is within actor's field of view bounds
	inline bool isOutOfFOV(const double &angle_relative);

	/// \brief Helper function which calculates relative
	/// speed based on 2 given velocity vectors
	double computeRelativeSpeed(const Vector3 &actor_vel, const Vector3 &object_vel);

	/// \brief Multiplies force components by a factor parameters
	/// and computes the combined force vector (summation).
	/// \note See \ref multiplyForces
	void factorInForceCoefficients();

	/// \brief Extends/truncates forces vectors if needed (i.e. combined force's
	/// magnitude too big / too small)
	/// \param dist_closest_static: distance to the closest static obstacle available
	/// \param dist_closest_dynamic: distance to the closest dynamic obstacle available
	void applyNonlinearOperations(const double &dist_closest_static, const double &dist_closest_dynamic);

	/// \brief Multiplies all force components (i.e. internal, interaction and social)
	/// by a given coefficient and updates the total force (`force_combined_`)
	/// according to a sum of components.
	/// \param coefficient: factor
	/// \note Truncates/extends all forces and calculates the combined one
	/// \note Non-linear operation
	void multiplyForces(const double &coefficient);

	/// \brief Resets class' internal state - forces
	void reset();

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

	/// \brief ShiftRegister is used for smoothing social force
	/// especially in close presence of the obstacles where most
	/// likely there is a highly weakened potential field.
	/// This in turn can cause a very difference force vector
	/// direction in 2 consecutive iterations.
	/// \note This saves only each `Arg2`-th sample and stores `Arg1` samples
	ShiftRegister<Vector3> sf_values_{25, 15};

	/// \section Section: Force vectors section
	///
	/// \brief Internal force vector (a.k.a. f_alpha)
	Vector3 force_internal_;
	///
	/// \brief Interaction force vector created from
	/// a set of single interactions from static and dynamic
	/// obstacles (a.k.a sum of f_alpha_beta's)
	Vector3 force_interaction_;
	///
	/// \brief Result of summation of internal
	/// and interaction force vectors.
	Vector3 force_combined_;

	/// \section Section: Social Force Model parameters
	///
	/// \brief Relaxation time
	float relaxation_time_;
	///
	/// \brief Desired speed
	float speed_desired_;

	/// \section Section: SFM parameters
	/// \note substitute for a lot of parameters
	std::shared_ptr<const hubero_local_planner::SfmParams> cfg_;

	///
	/// \brief Related to an inconsistency in symbols
	/// presented in the papers, see \ref ParameterDescription
	sfm::ParameterDescription param_description_;

	/* Model C -> Rudloff et al. (2011) model's parameters based on  S. Seer et al. (2014) */
	/* setting parameters static will create a population of actors moving in the same way */
	// TODO: make all actors share the same param values
	float An_;
	float Bn_;
	float Cn_;
	float Ap_;
	float Bp_;
	float Cp_;
	float Aw_;
	float Bw_;
};

} /* namespace sfm */
