/*
 * SocialForceModel.cpp
 *
 *  Created on: Dec 4, 2018
 *      Author: rayvburn
 */

#include <hubero_local_planner/sfm/social_force_model.h>

// additional headers
#include <cmath>			// atan2()
#include <tgmath.h>			// fabs()
#include <math.h>			// exp()
#include <algorithm>    	// std::find

// ----------------------------------------

// debugging
static bool print_info = false;
// FIXME:
#include <hubero_local_planner/sfm/sfm_debug.h>

// #define SFM_DEBUG_LARGE_VECTOR_LENGTH
// #define SFM_FUZZY_PROC_INDICATORS
// #define SFM_PRINT_FORCE_RESULTS

#include <hubero_local_planner/utils/debug.h>

#define DEBUG_BASIC 0
#define DEBUG_WARN 1
#define DEBUG_ERROR 1
#define DEBUG_VERBOSE 0

#define debug_print_basic(fmt, ...) _template_debug_print_basic_(DEBUG_BASIC, fmt, ##__VA_ARGS__)
#define debug_print_warn(fmt, ...) _template_debug_print_warn_(DEBUG_WARN, fmt, ##__VA_ARGS__)
#define debug_print_err(fmt, ...) _template_debug_print_err_(DEBUG_ERROR, fmt, ##__VA_ARGS__)
#define debug_print_verbose(fmt, ...) _template_debug_print_basic_(DEBUG_VERBOSE, fmt, ##__VA_ARGS__)

// ----------------------------------------

namespace sfm {

// ------------------------------------------------------------------- //

/* 		TUNING TIPS:
 * 	o 	the higher the desired force factor set the more actor will not notice
 * 		surrounding obstacles, trying to achieve a goal as fast as possible
 * 	o	the higher the interaction force factor set the more actor will try to
 * 		avoid obstacles - he will pass them safer
 * 	o 	the higher the yaw_increment coefficient set the less inertia actor will have
 * 		and will react immediately to social force changes - this will lead to many
 * 		rotations; on the other hand setting this parameter too low will create
 * 		a very conservative movement style (possibly cause stepping into obstacles
 * 		in cluttered world) */
SocialForceModel::SocialForceModel():
	param_description_(PARAMETER_DESCRIPTION_2014),
	cfg_(nullptr)
{
	setParameters();
}

// ------------------------------------------------------------------- //

bool SocialForceModel::computeSocialForce(
		const World& world,
		const double &dt,
		std::vector<size_t>& meaningful_static,
		std::vector<size_t>& meaningful_dynamic
) {
	reset();

	auto robot = world.getRobotData();

	// -----------------------------------------------------------------------------------------
	// ------- internal force calculation ------------------------------------------------------
	// -----------------------------------------------------------------------------------------
	force_internal_ = computeInternalForce(robot);

	// check whether it is needed to calculate interaction force or only the internal one
	// should be considered
	if (cfg_->disable_interaction_forces) {
		// multiply force vector components by parameter values
		factorInForceCoefficients();
		// artificially set obstacle distances to be very very big (in meters)
		applyNonlinearOperations(100.0, 100.0);
		return false;
	}

	// -----------------------------------------------------------------------------------------
	// ------ interaction forces calculation (repulsive, attractive) ---------------------------
	// -----------------------------------------------------------------------------------------
	// environment model
	auto objects_static = world.getStaticObjectsData();
	auto objects_dynamic = world.getDynamicObjectsData();

	// added up to the `force_interaction_`
	Vector3 f_alpha_beta;

	// investigate STATIC objects of the environment
	for (const StaticObject& object: objects_static) {
		f_alpha_beta = computeInteractionForce(robot, object, dt);
		force_interaction_ += f_alpha_beta;
		if (f_alpha_beta.Length() >= 1e-06) {
			meaningful_static.push_back(&object - &objects_static[0]);
		}
	}

	// investigate DYNAMIC objects of the environment
	for (const DynamicObject& object: objects_dynamic) {
		f_alpha_beta = computeInteractionForce(robot, object);
		force_interaction_ += f_alpha_beta;
		if (f_alpha_beta.Length() >= 1e-06) {
			meaningful_dynamic.push_back(&object - &objects_dynamic[0]);
		}
	}

	// -----------------------------------------------------------------------------------------
	// ------- additional calculations ---------------------------------------------------------
	// ----------- (i.a. smoothing) ------------------------------------------------------------
	// -----------------------------------------------------------------------------------------
	// multiply force vector components by parameter values
	factorInForceCoefficients();
	debug_print_basic("\t force_internal_: %2.3f, %2.3f, %2.3f\r\n", force_internal_.X(), force_internal_.Y(), force_internal_.Z());
	debug_print_basic("\t force_interaction_: %2.3f, %2.3f, %2.3f\r\n", force_interaction_.X(), force_interaction_.Y(), force_interaction_.Z());

	// extend or truncate force vectors if needed
	applyNonlinearOperations(world.getDistanceClosestStaticObject(), world.getDistanceClosestDynamicObject());
	debug_print_basic("\t\t force_internal_: %2.3f, %2.3f, %2.3f\r\n", force_internal_.X(), force_internal_.Y(), force_internal_.Z());
	debug_print_basic("\t\t force_interaction_: %2.3f, %2.3f, %2.3f\r\n", force_interaction_.X(), force_interaction_.Y(), force_interaction_.Z());

	return true;
}

// ------------------------------------------------------------------- //

// **********************************************************************
// **********************************************************************
// PRIVATE SECTION 	*****************************************************
//					*****************************************************
void SocialForceModel::setParameters() {

	/* Invoking this function to each actor will create a population in which there are
	 * everyone moving in a slightly other way */

	std::default_random_engine rand_gen;	// random number generator

	// desired speed (based on (Moussaid et al. (2009))
	std::normal_distribution<float> dist_spd_desired(1.29F, 0.19F);
	speed_desired_ = dist_spd_desired(rand_gen);

	// relaxation time (based on (Moussaid et al. (2009))
	std::normal_distribution<float> dist_tau(0.54F, 0.05F);
	relaxation_time_ = dist_tau(rand_gen);

	// ----------------------------- Model C ------------------------------------------------------ //
	// Rudloff et al. (2011) model's parameters based on  S. Seer et al. (2014)
	// Generate random value of mean a and standard deviation b
	std::normal_distribution<float> dist_an(0.2615F, 0.0551F);		An_ = dist_an(rand_gen);
	std::normal_distribution<float> dist_bn(0.4026F, 0.1238F);		Bn_ = dist_bn(rand_gen);
	std::normal_distribution<float> dist_cn(2.1614F, 0.3728F);		Cn_ = dist_cn(rand_gen);
	std::normal_distribution<float> dist_ap(1.5375F, 0.3084F);		Ap_ = dist_ap(rand_gen);
	std::normal_distribution<float> dist_bp(0.4938F, 0.1041F);		Bp_ = dist_bp(rand_gen);
	std::normal_distribution<float> dist_cp(0.5710F, 0.1409F);		Cp_ = dist_cp(rand_gen);
	std::normal_distribution<float> dist_aw(0.3280F, 0.1481F);		Aw_ = dist_aw(rand_gen);
	std::normal_distribution<float> dist_bw(0.1871F, 0.0563F);		Bw_ = dist_bw(rand_gen);

#ifdef DEBUG_SFM_PARAMETERS
	std::cout << "\t speed_desired: " << speed_desired_ << std::endl;
	std::cout << "\t relaxation_time: " << relaxation_time_ << std::endl;
	std::cout << "\t An: " << An_ << std::endl;
	std::cout << "\t Bn: " << Bn_ << std::endl;
	std::cout << "\t Cn: " << Cn_ << std::endl;
	std::cout << "\t Ap: " << Ap_ << std::endl;
	std::cout << "\t Bp: " << Bp_ << std::endl;
	std::cout << "\t Cp: " << Cp_ << std::endl;
	std::cout << "\t Aw: " << Aw_ << std::endl;
	std::cout << "\t Bw: " << Bw_ << std::endl;
	std::cout << std::endl;
#endif

}

// ------------------------------------------------------------------- //

Vector3 SocialForceModel::computeInternalForce(const Robot& robot) {
	Vector3 to_goal_direction = robot.target.dist_v.Normalized();
	Vector3 ideal_vel_vector = speed_desired_ * to_goal_direction;
	Vector3 f_alpha = cfg_->mass /*person_mass_*/ * (1/relaxation_time_) * (ideal_vel_vector - robot.vel);
	f_alpha.Z(0.0);

	debug_print_verbose("\t to_goal_vector: %2.3f, %2.3f, %2.3f \r\n", robot.target.dist_v.X(), robot.target.dist_v.Y(), robot.target.dist_v.Z());
	debug_print_verbose("\t to_goal_direction: %2.3f, %2.3f, %2.3f \r\n", to_goal_direction.X(), to_goal_direction.Y(), to_goal_direction.Z());
	debug_print_verbose("\t ideal_vel_vector: %2.3f, %2.3f, %2.3f \r\n", ideal_vel_vector.X(), ideal_vel_vector.Y(), ideal_vel_vector.Z());
	debug_print_verbose("\t target - x=%2.3f, y=%2.3f, z=%2.3f \r\n", robot.target.object.Pos().X(), robot.target.object.Pos().Y(), robot.target.object.Pos().Z());
	debug_print_verbose("\t robot_vel: %2.3f, %2.3f, %2.3f \r\n", robot.vel.X(), robot.vel.Y(), robot.vel.Z());
	debug_print_verbose("\t mass: %2.3f \r\n", cfg_->mass);
	debug_print_verbose("\t relaxation_time: %2.3f \r\n", relaxation_time_);
	debug_print_verbose("\t f_alpha: %2.3f, %2.3f, %2.3f   |   multiplied: %2.3f, %2.3f, %2.3f \r\n",
			f_alpha.X(), f_alpha.Y(), f_alpha.Z(),
			(cfg_->internal_force_factor * f_alpha).X(),
			(cfg_->internal_force_factor * f_alpha).Y(),
			(cfg_->internal_force_factor * f_alpha).Z()
	);

	/* FIXME: experimental
	if (cfg_->internal_force_factor * f_alpha.Length() > 1100.0) {
		debug_print_err("INTERNAL FORCE TRUNCATED from %2.3f to 1100.0\r\n", cfg_->internal_force_factor * f_alpha.Length());
		f_alpha = f_alpha.Normalized() * (1100.0 / cfg_->internal_force_factor);
	}
	*/

	return f_alpha;
}

// ------------------------------------------------------------------- //

Vector3 SocialForceModel::computeInteractionForce(
		const Robot& robot,
		const DynamicObject& object)
{
	/* actor's normal (based on velocity vector, whose direction could
	 * be also acquired from his yaw angle */
	Vector3 n_alpha = computeNormalAlphaDirection(robot.centroid);

	Vector3 f_alpha_beta(0.0, 0.0, 0.0);

	// check length to other object (beta)
	if (object.dist > 7.5) {
		debug_print_verbose("object too far away, zeroing force (distance vector length: %2.4f) \r\n", object.dist);
		return f_alpha_beta;
	}

	// compute heading directions of robot and object
	Angle robot_yaw(robot.centroid.Rot().Yaw());
	robot_yaw.Normalize();

	Angle object_yaw(std::atan2(object.vel.Y(), object.vel.X()));
	object_yaw.Normalize();

	/* FOV factor is used to make interaction of objects
	 * that are behind the actor weaker */
	double fov_factor = 1.00;

	/* check whether beta is within the field of view
	 * to determine proper factor for force in case
	 * beta is behind alpha */
	if (isOutOfFOV(object.rel_loc_angle)) {
		// exp function used: e^(-0.5*x)
		fov_factor = std::exp(-0.5 * object.dist);
		debug_print_verbose(
				"dynamic obstacle out of FOV, distance: %2.3f, FOV factor: %2.3f \r\n",
				object.dist,
				fov_factor
		);
	}

	double v_rel = computeRelativeSpeed(robot.vel, object.vel);
	if (std::fabs(v_rel) < 1e-06) {
		debug_print_verbose("abs. value of `v_rel` is close to ZERO (%2.3f), zeroing interaction force \r\n", v_rel);
		return f_alpha_beta;
	}

	// speed of the Beta object
	double speed_beta = object.vel.Length();

	// store angle between objects' (in most cases) velocities
	double theta_alpha_beta = 0.0;

	// check parameter value - theta_alpha_beta issue there
	switch (param_description_) {

	case(PARAMETER_DESCRIPTION_2014):
			theta_alpha_beta = computeThetaAlphaBetaAngle2014(n_alpha, object.dist_v);
			break;

	case(PARAMETER_DESCRIPTION_2011):
	case(PARAMETER_DESCRIPTION_UNKNOWN):
	default:
			/* angle between velocities of alpha and beta */
			theta_alpha_beta = computeThetaAlphaBetaAngle2011(robot, object);;
			break;
	}

	// actual force calculations
	//
	/* For details see `Social force equation` in the articles listed in .h file */
	// actor's perpendicular (based on velocity vector)
	Vector3 p_alpha = computePerpendicularToNormal(n_alpha, object.rel_loc);

	// original
//	double exp_normal = ( (-Bn_ * theta_alpha_beta * theta_alpha_beta) / v_rel ) - Cn_ * d_alpha_beta_length;
//	double exp_perpendicular = ( (-Bp_ * std::fabs(theta_alpha_beta) ) / v_rel ) - Cp_ * d_alpha_beta_length;
	// modded
	double exp_normal = ( (-Bn_ * theta_alpha_beta * theta_alpha_beta) / (2.0 * v_rel) ) - 0.5 * Cn_ * object.dist;
	double exp_perpendicular = ( (-0.1 * Bp_ * std::fabs(theta_alpha_beta) ) / v_rel ) - 0.5 * Cp_ * object.dist;

	// `fov_factor`: weaken the interaction force when beta is behind alpha
	// original
//	Vector3 n_alpha_scaled = n_alpha * An_ * std::exp(exp_normal) * fov_factor;
//	Vector3 p_alpha_scaled = p_alpha * Ap_ * std::exp(exp_perpendicular) * fov_factor;
	// modded
	Vector3 n_alpha_scaled = n_alpha * (-4.0) * An_ * std::exp(exp_normal) * fov_factor;
	Vector3 p_alpha_scaled = p_alpha * (+2.0) * Ap_ * std::exp(exp_perpendicular) * fov_factor;

	// -----------------------------------------------------
	// debugging large vector length ----------------
	debug_print_verbose("----interaction force----\r\n");
	debug_print_verbose("\t n_alpha: %2.3f,  %2.3f \r\n", n_alpha.X(), n_alpha.Y());
	debug_print_verbose("\t robot_yaw: %2.3f, object_yaw: %2.3f \r\n", robot_yaw.Radian(), object_yaw.Radian());
	debug_print_verbose("\t dist: %2.3f, fov_factor: %2.3f \r\n", object.dist, fov_factor);
	debug_print_verbose("\t v_rel: %2.3f (robot: %2.3f %2.3f, object: %2.3f %2.3f) \r\n",
		v_rel, robot.vel.X(), robot.vel.Y(), object.vel.X(), object.vel.Y());
	debug_print_verbose("\t Θ_αß: %2.3f \r\n", theta_alpha_beta);
	debug_print_verbose("\t expNORMAL: %2.3f, expPERP: %2.3f, FOV_factor: %2.3f \r\n",
			exp_normal,
			exp_perpendicular,
			fov_factor
	);
	debug_print_verbose("\t NORMAL: %2.3f  %2.3f, PERP: %2.3f  %2.3f \r\n",
			cfg_->interaction_force_factor * n_alpha_scaled.X(),
			cfg_->interaction_force_factor * n_alpha_scaled.Y(),
			cfg_->interaction_force_factor * p_alpha_scaled.X(),
			cfg_->interaction_force_factor * p_alpha_scaled.Y()
	);
	debug_print_verbose("\t total: %2.3f  %2.3f \r\n",
			cfg_->interaction_force_factor * (n_alpha_scaled + p_alpha_scaled).X(),
			cfg_->interaction_force_factor * (n_alpha_scaled + p_alpha_scaled).Y()
	);
	debug_print_verbose("----\r\n");
	// -----------------------------------------------------

	// factor - applicable only for dynamic objects
	// interaction strength exponentially decreases as distance between objects becomes bigger;
	// it is also artificially strengthened when distance is small
	double factor = 4.0 * std::exp(-2.0 * object.dist);
	n_alpha_scaled *= factor;
	p_alpha_scaled *= factor;

	// save interaction force vector
	f_alpha_beta = n_alpha_scaled + p_alpha_scaled;

	return f_alpha_beta;
}

// ------------------------------------------------------------------- //

Vector3 SocialForceModel::computeInteractionForce(const Robot& robot, const StaticObject& object, const double &dt) {
	/* elliptical formulation - `14 article - equations (3) and (4) */

	// distance vector
	Vector3 d_alpha_i = -object.dist_v; // note the proper direction
	d_alpha_i.Z(0.00); // planar
	double d_alpha_i_len = object.dist;

	// length (vβ * ∆t) of the stride (step size)
	Vector3 y_alpha_i = robot.vel * dt;
	y_alpha_i.Z(0.00); // planar

	// semi-minor axis of the elliptic formulation
	double w_alpha_i = 0.5 * sqrt( std::pow((d_alpha_i_len + (d_alpha_i - y_alpha_i).Length()),2) -
								   std::pow(y_alpha_i.Length(), 2) );

	// division by ~0 prevention - returning zeros vector instead of NaNs
	if ( (std::fabs(w_alpha_i) < 1e-08) || (std::isnan(w_alpha_i)) || (d_alpha_i_len < 1e-08) ) {
		debug_print_err("----static obstacle interaction error---- \r\n");
		debug_print_err("\t d_alpha_i: %2.3f %2.3f, len: %2.3f \r\n", d_alpha_i.X(), d_alpha_i.Y(), d_alpha_i_len);
		debug_print_err("\t y_alpha_i: %2.3f %2.3f, w_alpha_i: %2.3f \r\n", y_alpha_i.X(), y_alpha_i.Y(), w_alpha_i);
		debug_print_err("\t FAIL w_alpha_i small: %d \r\n", std::fabs(w_alpha_i) < 1e-08);
		debug_print_err("\t FAIL w_alpha_i NaN: %d \r\n", std::isnan(w_alpha_i));
		debug_print_err("\t d_alpha_i Length FAIL: %d \r\n", d_alpha_i_len < 1e-08);
		debug_print_err("\t returning ZERO force \r\n");
		debug_print_err("---- \r\n");
		return Vector3();
	}

	// ~force (acceleration) calculation
	Vector3 f_alpha_i;
	f_alpha_i = Aw_ * exp(-w_alpha_i/Bw_) * ((d_alpha_i_len + (d_alpha_i - y_alpha_i).Length()) /
			    2*w_alpha_i) * 0.5 * (d_alpha_i.Normalized() + (d_alpha_i - y_alpha_i).Normalized());

	// setting the `strength` (numerator) too high produces noticeable accelerations around objects
	double factor = 90.0/(std::exp(0.5 * d_alpha_i_len));
	f_alpha_i *= factor;

	debug_print_verbose("----static obstacle interaction \r\n");
	debug_print_verbose("\t d_alpha_i: %2.3f %2.3f, len: %2.3f \r\n",
			d_alpha_i.X(),
			d_alpha_i.Y(),
			d_alpha_i_len
	);
	debug_print_verbose("\t y_alpha_i: %2.3f %2.3f, w_alpha_i: %2.3f \r\n",
			y_alpha_i.X(),
			y_alpha_i.Y(),
			w_alpha_i
	);
	debug_print_verbose("\t f_alpha_i (multiplied): %2.3f %2.3f \r\n",
			cfg_->interaction_force_factor * f_alpha_i.X(),
			cfg_->interaction_force_factor * f_alpha_i.Y()
	);
	debug_print_verbose("---- \r\n");

	return f_alpha_i;
}

// ------------------------------------------------------------------- //

double SocialForceModel::computeThetaAlphaBetaAngle2011(const Robot& robot, const DynamicObject& object
		/*const ignition::math::Angle &actor_yaw, const Vector3 &object_vel,
		const ignition::math::Angle &object_yaw, const Vector3 &d_alpha_beta, const bool &is_actor*/
) {
	/* 2011 - "θ αβ - angle between velocity of pedestrian α
	 * and the displacement of pedestrian β" */
	/* `d_alpha_beta` can be interpreted as `d_actor_object` */

	// both velocities are expressed in world's coordinate system
	// formula -> Section "Examples of spatial tasks" @ https://onlinemschool.com/math/library/vector/angl/
	double cos_angle = robot.vel.Dot(object.vel) / (robot.vel.Length() * object.vel.Length());
	Angle theta(std::acos(cos_angle));
	theta.Normalize();

	return theta.Radian();
}

// ------------------------------------------------------------------- //

double SocialForceModel::computeThetaAlphaBetaAngle2014(const Vector3 &n_alpha,
		const Vector3 &d_alpha_beta) {

	/* 2014 - "φ αβ is the angle between n α and d αβ"
	 * n_alpha, 	- actor's normal (based on velocity vector)
	 * d_alpha_beta - vector between objects positions
	 */

	/* both n α and d αβ are expressed in world's coordinate system so
	 * simple angle difference should do the job */

#ifdef DEBUG_GEOMETRY_1
	if ( print_info ) {
		std::cout << "GetAngleBetweenObjectsVelocities(): ";
	}
#endif

	ignition::math::Angle angle_n_alpha( std::atan2( n_alpha.Y(), n_alpha.X() ) );
	angle_n_alpha.Normalize();

	ignition::math::Angle angle_d_alpha_beta( std::atan2( d_alpha_beta.Y(), d_alpha_beta.X() ) );
	angle_d_alpha_beta.Normalize();

	ignition::math::Angle phi_alpha_beta(angle_n_alpha.Radian() - angle_d_alpha_beta.Radian());
	phi_alpha_beta.Normalize();

#ifdef DEBUG_GEOMETRY_1
	if ( print_info ) {
		std::cout << "\tangle_n_alpha: " << angle_n_alpha.Radian() << "\tangle_d_alpha_beta: " << angle_d_alpha_beta.Radian() << "\tdiff: " << phi_alpha_beta.Radian() << std::endl;
	}
#endif

	return (phi_alpha_beta.Radian());

}

// ------------------------------------------------------------------- //

// dynamic objects interaction
double SocialForceModel::computeThetaAlphaBetaAngle(const ignition::math::Angle &actor_yaw,
		const ignition::math::Angle &object_yaw) {
    /*
	 *
	 * NOTE: below method (very simple and naive) of calculating the angle is correct
	 * only when both objects are:
	 * 		o dynamic,
	 * 		o currently moving,
	 * 		o already aligned with the to-target-direction,
	 * 		o there are no obstacles in the environment that will make the object not move along a straight line.
	 */

	// only on-plane movement considered
#ifdef DEBUG_GEOMETRY_1
	if ( print_info ) {
		std::cout << "GetAngleBetweenObjectsVelocities(): ";
	}
#endif

	ignition::math::Angle yaw_diff(actor_yaw.Radian() - object_yaw.Radian());
	yaw_diff.Normalize();

#ifdef DEBUG_GEOMETRY_1
	if ( print_info ) {
		std::cout << "\t yaw_actor: " << _actor_yaw->Radian() << "  yaw_object: " << _object_yaw->Radian() << "  yaw_diff: " << yaw_diff.Radian() << std::endl;
	}
#endif

	return ( yaw_diff.Radian() );

}

// ------------------------------------------------------------------- //

Vector3 SocialForceModel::computeNormalAlphaDirection(const Pose3 &actor_pose) {

	// when speed is 0 then there is no way of calculating the angle THETA_alpha_beta (0 vector length)
	// better way is calculating normal based on actor's yaw than velocity vector


	/* all calculations here are performed on world coordinate system data -
	 * n_alpha from actor's coordinate system is projected onto
	 * world's coordinate system axes */
	ignition::math::Angle yaw_norm(actor_pose.Rot().Yaw());
	yaw_norm.Normalize();

	// check parameter setting - n_alpha issue there
	switch (param_description_) {

	case(PARAMETER_DESCRIPTION_2011):
			// vector pointing opposite direction
			yaw_norm -= yaw_norm.Pi;
			yaw_norm.Normalize();
			break;

	case(PARAMETER_DESCRIPTION_2014):
	default:
			// do not rotate
			break;

	}

	Vector3 n_alpha;

	// rotate the vector
	n_alpha.X( +sin(yaw_norm.Radian()) );
	n_alpha.Y( -cos(yaw_norm.Radian()) ); 	// sine and cosine relation the same as in GetNewPose()
	n_alpha.Z(0.0); 						// in-plane movement only at the moment
	n_alpha.Normalize();

	if ( print_info ) {
		std::cout << "\t n_alpha: " << n_alpha << "\t";
	}

	return n_alpha;

}

// ------------------------------------------------------------------- //

Vector3 SocialForceModel::computePerpendicularToNormal(const Vector3 &n_alpha,
		const RelativeLocation &beta_rel_location)

{

	/* Depending on which side beta is on relative to n_alpha, the p_alpha (perpendicular vector)
	 * will point to direction opposite to the side where beta is */

	/*
	Vector3 p_alpha;
	if ( _beta_rel_location == RelativeLocation::LOCATION_RIGHT ) {
		p_alpha = _n_alpha.Perpendicular();
		// return (_n_alpha.Perpendicular());
	} else if ( _beta_rel_location == RelativeLocation::LOCATION_LEFT ) {

		// Vector3 p_alpha;

		// inverse-perpendicular vector calculations based on ignition library
		static const double sqr_zero = 1e-06 * 1e-06;
		Vector3 to_cross = {0, 0, 1}; // TODO: -1?
		p_alpha = _n_alpha.Cross(to_cross);

		// Check the length of the vector
		if (p_alpha.SquaredLength() < sqr_zero)
		{

			to_cross = {0, -1, 0};
			p_alpha = p_alpha.Cross(to_cross);
			std::cout << "GetPerpendicularToNormal(): " << "\t TOO SHORT! " << std::endl;

		}
	}
	*/

	Vector3 p_alpha;
	static const double sqr_zero = 1e-06 * 1e-06;
	Vector3 to_cross;

	if ( beta_rel_location == RelativeLocation::LOCATION_LEFT ) {

		// check parameter - n_alpha issue there
		switch (param_description_) {

		case(PARAMETER_DESCRIPTION_2011):
				to_cross.Set(0.0, 0.0, -1.0);
				break;

		case(PARAMETER_DESCRIPTION_2014):
		default:
				to_cross.Set(0.0, 0.0,  1.0);
				break;

		}


	} else if ( beta_rel_location == RelativeLocation::LOCATION_RIGHT ) {

		// check parameter - n_alpha issue there
		switch (param_description_) {

		case(PARAMETER_DESCRIPTION_2011):
				to_cross.Set(0.0, 0.0,  1.0);
				break;
		case(PARAMETER_DESCRIPTION_2014):
		default:
				to_cross.Set(0.0, 0.0, -1.0);
				break;

		}

	} else {

#ifdef DEBUG_GEOMETRY_1
		if ( print_info ) {
			std::cout << "BEHIND!\t" << std::endl;
		}
#endif

	}

	p_alpha = n_alpha.Cross(to_cross);

	// Check the length of the vector
	if (p_alpha.SquaredLength() < sqr_zero) {

		// this should not happen
		to_cross = {0, -1, 0};
		p_alpha = p_alpha.Cross(to_cross);
#ifdef DEBUG_GEOMETRY_1
		if ( print_info ) {
			std::cout << "GetPerpendicularToNormal(): " << "\t TOO SHORT! " << std::endl;
		}
#endif

	}

#ifdef DEBUG_GEOMETRY_1
	if ( print_info ) {
		std::cout << "GetPerpendicularToNormal(): " << "  x: " << p_alpha.X() << "  y: " << p_alpha.Y() << "  z: " << p_alpha.Z() << std::endl;
	}
#endif

	return ( p_alpha );

}

// ------------------------------------------------------------------- //

inline bool SocialForceModel::isOutOfFOV(const double &angle_relative) {
	if (std::fabs(angle_relative) >= cfg_->fov) {
		return (true);
	}
	return (false);
}

// ------------------------------------------------------------------- //

double SocialForceModel::computeRelativeSpeed(const Vector3 &actor_vel, const Vector3 &object_vel) {
	debug_print_verbose("obj_vel: %2.3f  %2.3f \r\n", object_vel.X(), object_vel.Y());
	debug_print_verbose("robot_vel: %2.3f  %2.3f \r\n", actor_vel.X(), actor_vel.Y());
	debug_print_verbose("v_rel: %2.3f  %2.3f \r\n", (object_vel - actor_vel).X(), (object_vel - actor_vel).Y());
	debug_print_verbose("spd_rel: %2.3f \r\n", (object_vel - actor_vel).Length());
	return ((object_vel - actor_vel).Length());
}

// ------------------------------------------------------------------- //

void SocialForceModel::factorInForceCoefficients() {

	force_internal_ 	*= cfg_->internal_force_factor; //factor_force_internal_;
	force_interaction_ 	*= cfg_->interaction_force_factor; //factor_force_interaction_;
//	force_social_ 		*= factor_force_social_;
	force_combined_ 	 = force_internal_ + force_interaction_;// + force_social_;

}

// ------------------------------------------------------------------- //

void SocialForceModel::applyNonlinearOperations(const double &dist_closest_static, const double &dist_closest_dynamic) {

	// FIXME: choose a closest distance to an obstacle
	double dist_closest = std::min(dist_closest_static, dist_closest_dynamic);

	// truncate the force value to max to prevent strange speedup of an actor
	double force_combined_magnitude_init = force_combined_.Length();

	// TODO: additional factor which weakens internal component when there is
	// an obstacle close to the actor
	// FIXME: experimental
	double internal_weakening_factor = std::exp(0.75 * dist_closest) - 1.0;
	if ( internal_weakening_factor > 1.0 ) {
		internal_weakening_factor = 1.0;
	}
	force_internal_ *= internal_weakening_factor;

	// evaluate force magnitude
	if (force_combined_magnitude_init >= cfg_->max_force) {

		multiplyForces(cfg_->max_force / force_combined_magnitude_init);
		if ( print_info ) {
			std::cout << "\tTRUNCATED, factor: " << (cfg_->max_force / force_combined_magnitude_init);
		}

	} else if (force_combined_magnitude_init <= cfg_->min_force) {

		// TODO: set desired force factor according to the distance to the closest obstacle -
		// the closer actor gets, the smaller the coefficient should be - this is
		// the REAL SOCIAL feature of the model */
		double force_min_factor_socialized = 1.00;

		// ------------------------------------------------------------------------------
		// V1
		// NOTE: this algorithm in fact allows the total force to stay
		// below the threshold value (force_min_).
		// Aim is to allow higher maneuverability
		// when there is some obstacle nearby.
		// Higher maneuverability is achieved because extension
		// of the force vector to the force_min length
		// creates additional `inertia` when actor got into
		// close-to-zero potential zone. The inertia pushes
		// him to the place when potential is far away from 0.
//		force_min_factor_socialized = std::exp(0.75 * dist_closest);
//		(force_min_factor_socialized > 1.00) ? (force_min_factor_socialized = 1.00) : (0);
//		sf_values_.update(force_combined_);
//		// f_total = sf_values_.getAverage().Normalize() * force_min_sf * force_min_;
//		multiplyForces((force_min_factor_socialized * force_min_) / force_combined_magnitude_init);
//		if ( print_info ) {
//			std::cout << "\tEXTENDED, factor: " << (force_min_factor_socialized * force_min_) / force_combined_magnitude_init;
//		}

		/* NOTE: the above algorithm causes total force's components
		 * to increase length dramatically to compensate too small
		 * resulting force */

		// ------------------------------------------------------------------------------
		// V2 - bad
		// FIXME: Let's extend the social force vector - in most cases
		// it will be the best way to recover from being stuck.
		// Actor usually gets stuck in close proximity to other person.
//		force_min_factor_socialized = std::exp(0.75 * (dist_closest - 1.50));
//		(force_min_factor_socialized > 1.00) ? (force_min_factor_socialized = 1.00) : (0);
//		double extension_len = std::fabs(force_combined_magnitude_init - (force_min_factor_socialized * force_min_));
//		force_social_ += (0.5 * extension_len * force_social_.Normalized());
//		force_interaction_ += (0.5 * extension_len * force_interaction_.Normalized());
//		sf_values_.update(force_internal_ + force_interaction_ + force_social_);
//
//		// make sure the average is a non-zero vector
//		Vector3 avg = sf_values_.getAverage();
//		if ( avg.Length() > 1e-06 ) {
//			force_combined_ = avg;
//		}

		// ------------------------------------------------------------------------------
		// V3 - bad
		// determines how much the internal force can be extended according to proximity
		// to the closest obstacle
//		dist_closest = 1.0; // it's better to use the shortest distance from a dynamic object here
//		double factor_force_internal_dynamic = std::exp(0.5 * dist_closest) - 1;
//		(force_min_factor_socialized > 1.00) ? (force_min_factor_socialized = 1.00) : (0);
//
//		// applies for interaction force and social force (equal division of the extension vector)
//		double factor_force_dynamic = (1.0 - factor_force_internal_dynamic) / 2.0;
//
//		// how much the combined vector should be extended
//		double extension_len = std::fabs(force_combined_magnitude_init - force_min_);
//
//		force_internal_ 	+= factor_force_internal_dynamic * extension_len * force_internal_.Normalized();
//		force_interaction_ 	+= factor_force_dynamic 		 * extension_len * force_interaction_.Normalized();
//		force_social_ 		+= factor_force_dynamic 		 * extension_len * force_social_.Normalized();
//		sf_values_.update(force_internal_ + force_interaction_ + force_social_);
//
//		// make sure the average is a non-zero vector
//		Vector3 avg = sf_values_.getAverage();
//		if ( avg.Length() > 1e-06 ) {
//			force_combined_ = avg;
//		}

		// ------------------------------------------------------------------------------
		// V4
		// IDEA: let's maintain actor's direction of motion determined by combined force
		//
		// the exp function to slow down actors when there is another person just behind
		// the one currently considered;
		// likely does not have a great impact on resulting behaviour

		// how much the combined vector should be extended
		double extension_len = std::fabs(force_combined_magnitude_init - cfg_->min_force);

		// create an extension vector
		Vector3 extension = extension_len * force_combined_.Normalized();

		// add the resulting vector to `force_interaction_`
		force_interaction_ += extension;

		// sum up
		force_combined_ = force_internal_ + force_interaction_; // + force_social_;
		sf_values_.update(force_combined_);

		// make sure the average is a non-zero vector
		Vector3 avg = sf_values_.getAverage();
		if ( avg.Length() > 1e-06 ) {
			force_combined_ = avg;
		}

	} else {

		// force in allowable range

		// clear social forces vector used for averaging
		if ( !sf_values_.isEmpty() ) {
			sf_values_.clear();
		}

	}

}

// ------------------------------------------------------------------- //

void SocialForceModel::multiplyForces(const double &coefficient) {

	force_internal_ 	*= coefficient;
	force_interaction_ 	*= coefficient;
//	force_social_ 		*= coefficient;
//	force_combined_ 	 = force_internal_ + force_interaction_ + force_social_;
	force_combined_ 	 = force_internal_ + force_interaction_;

}

// ------------------------------------------------------------------- //

void SocialForceModel::reset() {
	force_internal_ = Vector3();
	force_interaction_ = Vector3();
//	force_social_ = Vector3();
	force_combined_ = Vector3();
}

// ------------------------------------------------------------------- //

} /* namespace sfm */
