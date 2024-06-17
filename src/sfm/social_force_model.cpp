/*
 * SocialForceModel.cpp
 *
 *  Created on: Dec 4, 2018
 *      Author: rayvburn
 */

#include <humap_local_planner/sfm/social_force_model.h>

// additional headers
#include <cmath>			// atan2()
#include <tgmath.h>			// fabs()
#include <math.h>			// exp()
#include <algorithm>    	// std::find
#include <random>           // std::normal_distribution
#include <stdexcept>

#include <social_nav_utils/gaussians.h>

// ----------------------------------------

// debugging
static bool print_info = false;
// FIXME:
#include <humap_local_planner/sfm/sfm_debug.h>

// #define SFM_DEBUG_LARGE_VECTOR_LENGTH
// #define SFM_FUZZY_PROC_INDICATORS
// #define SFM_PRINT_FORCE_RESULTS

#include <humap_local_planner/utils/debug.h>

#define DEBUG_BASIC 0
#define DEBUG_WARN 1
#define DEBUG_ERROR 1
#define DEBUG_VERBOSE 0

#define debug_print_basic(fmt, ...) _template_debug_print_basic_(DEBUG_BASIC, fmt, ##__VA_ARGS__)
#define debug_print_warn(fmt, ...) _template_debug_print_warn_(DEBUG_WARN, fmt, ##__VA_ARGS__)
#define debug_print_err(fmt, ...) _template_debug_print_err_(DEBUG_ERROR, fmt, ##__VA_ARGS__)
#define debug_print_verbose(fmt, ...) _template_debug_print_basic_(DEBUG_VERBOSE, fmt, ##__VA_ARGS__)

// ----------------------------------------

namespace humap_local_planner {
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
{}

void SocialForceModel::init(std::shared_ptr<const humap_local_planner::SfmParams> cfg) {
	cfg_ = cfg;
	if (cfg_ == nullptr) {
		throw std::runtime_error("Given nullptr to SFM instead of a valid configuration parameter struct");
	}
	setParameters();
}

void SocialForceModel::setEquationParameters(
	double an,
	double bn,
	double cn,
	double ap,
	double bp,
	double cp,
	double aw,
	double bw
) {
	An_ = an;
	Bn_ = bn;
	Cn_ = cn;
	Ap_ = ap;
	Bp_ = bp;
	Cp_ = cp;
	Aw_ = aw;
	Bw_ = bw;
}

void SocialForceModel::setEquationParameters(
	double an,
	double bn,
	double cn,
	double ap,
	double bp,
	double cp,
	double aw,
	double bw,
	double speed_desired
) {
	setEquationParameters(an, bn, cn, ap, bp, cp, aw, bw);
	speed_desired_ = speed_desired;
}

void SocialForceModel::setEquationParameters(
	double an,
	double bn,
	double cn,
	double ap,
	double bp,
	double cp,
	double aw,
	double bw,
	double speed_desired,
	double relaxation_time
) {
	setEquationParameters(an, bn, cn, ap, bp, cp, aw, bw, speed_desired);
	relaxation_time_ = relaxation_time;
}

bool SocialForceModel::areInteractionForcesDisabled() const {
	// enabled by default
	if (cfg_ == nullptr) {
		return false;
	}
	return cfg_->disable_interaction_forces;
}

// ------------------------------------------------------------------- //

bool SocialForceModel::computeSocialForce(
		const World& world,
		const double &dt,
		std::vector<Distance>& meaningful_static,
		std::vector<Distance>& meaningful_dynamic
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
		if (cfg_->filter_forces) {
			// artificially set obstacle distances to be very very big (in meters)
			applyNonlinearOperations(100.0, 100.0);
		}
		return false;
	}

	// -----------------------------------------------------------------------------------------
	// ------ interaction forces calculation (repulsive, attractive) ---------------------------
	// -----------------------------------------------------------------------------------------
	// environment model
	auto objects_static = world.getStaticObjectsData();
	auto objects_dynamic = world.getDynamicObjectsData();

	// added up to the `force_interaction_`
	Vector f_alpha_beta;

	// investigate STATIC objects of the environment
	for (const StaticObject& object: objects_static) {
		f_alpha_beta = computeInteractionForce(robot, object, dt);
		force_interaction_static_ += f_alpha_beta;
		if (f_alpha_beta.calculateLength() >= 1e-06) {
			meaningful_static.push_back(Distance {.robot = object.robot, .object = object.object});
		}
	}

	// investigate DYNAMIC objects of the environment
	for (const DynamicObject& object: objects_dynamic) {
		f_alpha_beta = computeInteractionForce(robot, object);
		force_interaction_dynamic_ += f_alpha_beta;
		if (f_alpha_beta.calculateLength() >= 1e-06) {
			meaningful_dynamic.push_back(Distance {.robot = object.robot, .object = object.object});
		}
	}

	// -----------------------------------------------------------------------------------------
	// ------- additional calculations ---------------------------------------------------------
	// ----------- (i.a. smoothing) ------------------------------------------------------------
	// -----------------------------------------------------------------------------------------
	// multiply force vector components by parameter values
	factorInForceCoefficients();

	if (cfg_->filter_forces) {
		// extend or truncate force vectors if needed
		applyNonlinearOperations(world.getDistanceClosestStaticObject(), world.getDistanceClosestDynamicObject());
	}

	return true;
}

// ------------------------------------------------------------------- //

// static
double SocialForceModel::computeFactorFOV(double angle_relative, double fov, bool gaussian) {
	if (!gaussian) {
		double fov_half = fov / 2.0;
		// outside FOV - linear decrease, minimum of 0.0
		if (angle_relative < -fov_half) {
			return (IGN_PI + angle_relative) / (IGN_PI - fov_half);
		} else if (angle_relative > fov_half) {
			return (IGN_PI - angle_relative) / (IGN_PI - fov_half);
		}
		// within FOV - perfectly OK
		return 1.0;
	}

	// 2 sigma rule applied
	double fov_variance = std::pow(fov / 2.0, 2.0);
	// 0.0 - axis of sight in the center of the span of FOV angles
	return social_nav_utils::calculateGaussianAngle(angle_relative, 0.0, fov_variance);
}

// **********************************************************************
// **********************************************************************
// PROTECTED SECTION*****************************************************
//					*****************************************************
void SocialForceModel::setParameters() {
	/*
	* `heterogenous_population` set to true will produce slightly different parameters in each system launch.
	* This will cause robot to move in a slightly different way each time.
	*/
	if (cfg_->heterogenous_population) {
		// random number generator
		std::default_random_engine rand_gen;

		// desired speed (based on (Moussaid et al. (2009))
		std::normal_distribution<float> dist_spd_desired(cfg_->speed_desired, cfg_->speed_desired_stddev);
		speed_desired_ = dist_spd_desired(rand_gen);

		// relaxation time (based on (Moussaid et al. (2009))
		std::normal_distribution<float> dist_tau(cfg_->relaxation_time, cfg_->relaxation_time_stddev);
		relaxation_time_ = dist_tau(rand_gen);

		// ----------------------------- Model C ------------------------------------------------------ //
		// Rudloff et al. (2011) model's parameters based on  S. Seer et al. (2014)
		// Generate random value of mean a and standard deviation b
		std::normal_distribution<float> dist_an(cfg_->an, cfg_->an_stddev);
		An_ = dist_an(rand_gen);
		std::normal_distribution<float> dist_bn(cfg_->bn, cfg_->bn_stddev);
		Bn_ = dist_bn(rand_gen);
		std::normal_distribution<float> dist_cn(cfg_->cn, cfg_->cn_stddev);
		Cn_ = dist_cn(rand_gen);
		std::normal_distribution<float> dist_ap(cfg_->ap, cfg_->ap_stddev);
		Ap_ = dist_ap(rand_gen);
		std::normal_distribution<float> dist_bp(cfg_->bp, cfg_->bp_stddev);
		Bp_ = dist_bp(rand_gen);
		std::normal_distribution<float> dist_cp(cfg_->cp, cfg_->cp_stddev);
		Cp_ = dist_cp(rand_gen);
		std::normal_distribution<float> dist_aw(cfg_->aw, cfg_->aw_stddev);
		Aw_ = dist_aw(rand_gen);
		std::normal_distribution<float> dist_bw(cfg_->bw, cfg_->bw_stddev);
		Bw_ = dist_bw(rand_gen);
		printf(
			"SFM parameters computed in a non-deterministic way. Values are as follows:\r\n"
			"\tspeed_desired %2.5f\r\n"
			"\trelaxation_time %2.5f\r\n"
			"\tAn %2.5f\r\n"
			"\tBn %2.5f\r\n"
			"\tCn %2.5f\r\n"
			"\tAp %2.5f\r\n"
			"\tBp %2.5f\r\n"
			"\tCp %2.5f\r\n"
			"\tAw %2.5f\r\n"
			"\tBw %2.5f\r\n",
			speed_desired_,
			relaxation_time_,
			An_,
			Bn_,
			Cn_,
			Ap_,
			Bp_,
			Cp_,
			Aw_,
			Bw_
		);
		return;
	}

	// homogenous (deterministic) mode selected
	speed_desired_ = cfg_->speed_desired;
	relaxation_time_ = cfg_->relaxation_time;
	An_ = cfg_->an;
	Bn_ = cfg_->bn;
	Cn_ = cfg_->cn;
	Ap_ = cfg_->ap;
	Bp_ = cfg_->bp;
	Cp_ = cfg_->cp;
	Aw_ = cfg_->aw;
	Bw_ = cfg_->bw;
}

// ------------------------------------------------------------------- //

Vector SocialForceModel::computeInternalForce(const Robot& robot) {
	return SocialForceModel::computeInternalForce(robot.vel, robot.target.dist_v, cfg_->mass, speed_desired_, relaxation_time_);
}

Vector SocialForceModel::computeInternalForce(
	const Vector& vel_robot,
	const Vector& d_robot_object,
	const double& mass,
	const double& speed_desired,
	const double& relaxation_time
) {
	Vector to_goal_direction = d_robot_object.normalized();
	Vector ideal_vel_vector = speed_desired * to_goal_direction;
	Vector f_alpha = mass * (1 / relaxation_time) * (ideal_vel_vector - vel_robot);
	f_alpha.setZ(0.0);

	debug_print_verbose("\t to_goal_vector: %2.3f, %2.3f, %2.3f \r\n", d_robot_object.getX(), d_robot_object.getY(), d_robot_object.getZ());
	debug_print_verbose("\t to_goal_direction: %2.3f, %2.3f, %2.3f \r\n", to_goal_direction.getX(), to_goal_direction.getY(), to_goal_direction.getZ());
	debug_print_verbose("\t ideal_vel_vector: %2.3f, %2.3f, %2.3f \r\n", ideal_vel_vector.getX(), ideal_vel_vector.getY(), ideal_vel_vector.getZ());
	debug_print_verbose("\t robot_vel: %2.3f, %2.3f, %2.3f \r\n", vel_robot.getX(), vel_robot.getY(), vel_robot.getZ());
	debug_print_verbose("\t mass: %2.3f \r\n", mass);
	debug_print_verbose("\t relaxation_time: %2.3f \r\n", relaxation_time);
	debug_print_verbose("\t f_alpha: %2.3f, %2.3f, %2.3f\r\n",//   |   multiplied: %2.3f, %2.3f, %2.3f \r\n",
			f_alpha.getX(), f_alpha.getY(), f_alpha.getZ()
	);

	return f_alpha;
}

// ------------------------------------------------------------------- //

Vector SocialForceModel::computeInteractionForce(
		const Robot& robot,
		const DynamicObject& object)
{
	/* actor's normal (based on velocity vector, whose direction could
	 * be also acquired from his yaw Angle */
	Vector n_alpha = computeNormalAlphaDirection(robot.centroid);

	Vector f_alpha_beta(0.0, 0.0, 0.0);

	// check length to other object (beta)
	if (object.dist > 7.5) {
		debug_print_verbose("object too far away, zeroing force (distance vector length: %2.4f) \r\n", object.dist);
		return f_alpha_beta;
	}

	// compute heading directions of robot and object
	Angle robot_yaw(robot.centroid.getYaw());
	Angle object_yaw(object.vel);

	/* FOV factor is used to make interaction of objects
	 * that are behind the actor weaker */
	double fov_factor = computeFactorFOV(object.rel_loc_angle);

	double v_rel = computeRelativeSpeed(robot.vel, object.vel);
	if (std::abs(v_rel) < 1e-06) {
		debug_print_verbose("abs. value of `v_rel` is close to ZERO (%2.3f), zeroing interaction force \r\n", v_rel);
		return f_alpha_beta;
	}

	// speed of the Beta object
	double speed_beta = object.vel.calculateLength();

	// store Angle between objects' (in most cases) velocities
	Angle theta_alpha_beta;

	// check parameter value - theta_alpha_beta issue there
	switch (param_description_) {

	case(PARAMETER_DESCRIPTION_2014):
			theta_alpha_beta = computeThetaAlphaBetaAngle2014(n_alpha, object.dist_v);
			break;

	case(PARAMETER_DESCRIPTION_2011):
	case(PARAMETER_DESCRIPTION_UNKNOWN):
	default:
			/* Angle between velocities of alpha and beta */
			theta_alpha_beta = computeThetaAlphaBetaAngle2011(robot.vel, object.vel);;
			break;
	}

	// actual force calculations
	//
	/* For details see `Social force equation` in the articles listed in .h file */
	// actor's perpendicular (based on velocity vector)
	Vector p_alpha = computePerpendicularToNormal(n_alpha, object.rel_loc);

	double th_ab_angle = theta_alpha_beta.getRadian();
	double exp_normal = ((-Bn_ * th_ab_angle * th_ab_angle) / v_rel) - Cn_ * object.dist;
	double exp_perpendicular = ((-Bp_ * std::abs(th_ab_angle)) / v_rel) - Cp_ * object.dist;

	Vector n_alpha_scaled = n_alpha * An_ * std::exp(exp_normal);
	Vector p_alpha_scaled = p_alpha * Ap_ * std::exp(exp_perpendicular);

	// `fov_factor`: weakens the interaction force when beta is behind alpha
	n_alpha_scaled *= fov_factor;
	p_alpha_scaled *= fov_factor;

	// -----------------------------------------------------
	// debugging large vector length ----------------
	debug_print_verbose("----interaction force----\r\n");
	debug_print_verbose("\t n_alpha: %2.3f,  %2.3f \r\n", n_alpha.getX(), n_alpha.getY());
	debug_print_verbose("\t robot_yaw: %2.3f, object_yaw: %2.3f \r\n", robot_yaw.getRadian(), object_yaw.getRadian());
	debug_print_verbose("\t dist: %2.3f, fov_factor: %2.3f \r\n", object.dist, fov_factor);
	debug_print_verbose("\t v_rel: %2.3f (robot: %2.3f %2.3f, object: %2.3f %2.3f) \r\n",
		v_rel, robot.vel.getX(), robot.vel.getY(), object.vel.getX(), object.vel.getY());
	debug_print_verbose("\t Θ_αß: %2.3f \r\n", th_ab_angle);
	debug_print_verbose("\t expNORMAL: %2.3f, expPERP: %2.3f, FOV_factor: %2.3f \r\n",
			exp_normal,
			exp_perpendicular,
			fov_factor
	);
	debug_print_verbose("\t NORMAL: %2.3f  %2.3f, PERP: %2.3f  %2.3f \r\n",
			cfg_->dynamic_interaction_force_factor * n_alpha_scaled.getX(),
			cfg_->dynamic_interaction_force_factor * n_alpha_scaled.getY(),
			cfg_->dynamic_interaction_force_factor * p_alpha_scaled.getX(),
			cfg_->dynamic_interaction_force_factor * p_alpha_scaled.getY()
	);
	debug_print_verbose("\t total: %2.3f  %2.3f \r\n",
			cfg_->dynamic_interaction_force_factor * (n_alpha_scaled + p_alpha_scaled).getX(),
			cfg_->dynamic_interaction_force_factor * (n_alpha_scaled + p_alpha_scaled).getY()
	);
	debug_print_verbose("----\r\n");
	// -----------------------------------------------------
	// save interaction force vector
	f_alpha_beta = n_alpha_scaled + p_alpha_scaled;

	return f_alpha_beta;
}

// ------------------------------------------------------------------- //

Vector SocialForceModel::computeInteractionForce(const Robot& robot, const StaticObject& object, const double &dt) {
	/* elliptical formulation - `14 article - equations (3) and (4) */

	// distance vector
	Vector d_alpha_i = -object.dist_v; // note the proper direction
	d_alpha_i.setZ(0.00); // planar (this could be ommitted)
	double d_alpha_i_len = object.dist;

	// length (vβ * ∆t) of the stride (step size)
	Vector y_alpha_i = robot.vel * dt;
	y_alpha_i.setZ(0.00); // planar (this could be ommitted)

	// helper to avoid duplicate operations
	auto dy_alpha_i = d_alpha_i - y_alpha_i;
	auto dy_alpha_i_len = dy_alpha_i.calculateLength();

	// semi-minor axis of the elliptic formulation
	double w_alpha_i = 0.5 * sqrt( std::pow((d_alpha_i_len + dy_alpha_i_len), 2) -
								   std::pow(y_alpha_i.calculateLength(), 2) );

	// division by ~0 prevention - returning zeros vector instead of NaNs
	bool w_alpha_i_small = std::abs(w_alpha_i) < 1e-08;
	bool w_alpha_i_nan = std::isnan(w_alpha_i);
	bool d_alpha_i_short = d_alpha_i_len < 1e-08;
	if (w_alpha_i_small && !w_alpha_i_nan && !d_alpha_i_short) {
		debug_print_verbose("----static obstacle interaction abort ---- \r\n");
		debug_print_verbose("\t FAIL w_alpha_i small \r\n");
		debug_print_verbose("---- \r\n");
		return Vector();
	}
	if (w_alpha_i_small || w_alpha_i_nan || d_alpha_i_short) {
		debug_print_err("----static obstacle interaction error---- \r\n");
		debug_print_err("\t d_alpha_i: %2.3f %2.3f, len: %2.3f \r\n", d_alpha_i.getX(), d_alpha_i.getY(), d_alpha_i_len);
		debug_print_err("\t y_alpha_i: %2.3f %2.3f, w_alpha_i: %2.3f \r\n", y_alpha_i.getX(), y_alpha_i.getY(), w_alpha_i);
		debug_print_err("\t FAIL w_alpha_i small: %d \r\n", w_alpha_i_small);
		debug_print_err("\t FAIL w_alpha_i NaN: %d \r\n", w_alpha_i_nan);
		debug_print_err("\t d_alpha_i Length FAIL: %d \r\n", d_alpha_i_short);
		debug_print_err("\t returning ZERO force \r\n");
		debug_print_err("---- \r\n");
		return Vector();
	}

	// ~force (acceleration) calculation
	Vector f_alpha_i;
	f_alpha_i = Aw_ * exp(-w_alpha_i/Bw_) * ((d_alpha_i_len + dy_alpha_i_len) /
			    2*w_alpha_i) * 0.5 * (d_alpha_i.normalized() + (dy_alpha_i).normalized());

	/*
	 * Check whether beta is within the field of view to determine a proper factor for force in case
	 * "beta" is behind "alpha"
	 */
	geometry::Angle angle_relative(object.dist_v.calculateDirection().getRadian() - robot.heading_dir.getRadian());
	double fov_factor = computeFactorFOV(angle_relative);

	f_alpha_i *= fov_factor;

	debug_print_verbose("----static obstacle interaction \r\n");
	debug_print_verbose("\t d_alpha_i: %2.3f %2.3f, len: %2.3f \r\n",
			d_alpha_i.getX(),
			d_alpha_i.getY(),
			d_alpha_i_len
	);
	debug_print_verbose("\t y_alpha_i: %2.3f %2.3f, w_alpha_i: %2.3f \r\n",
			y_alpha_i.getX(),
			y_alpha_i.getY(),
			w_alpha_i
	);
	debug_print_verbose("\t f_alpha_i (multiplied): %2.3f %2.3f \r\n",
			cfg_->static_interaction_force_factor * f_alpha_i.getX(),
			cfg_->static_interaction_force_factor * f_alpha_i.getY()
	);
	debug_print_verbose("---- \r\n");

	return f_alpha_i;
}

// ------------------------------------------------------------------- //

Angle SocialForceModel::computeThetaAlphaBetaAngle2011(const Vector& robot_vel, const Vector& object_vel) {

	/* 2011 - "θ αβ - Angle between velocity of pedestrian α
	 * and the displacement of pedestrian β" */
	/* `d_alpha_beta` can be interpreted as `d_actor_object` */

	// both velocities are expressed in world's coordinate system
	// formula -> Section "Examples of spatial tasks" @ https://onlinemschool.com/math/library/vector/angl/
	double cos_angle = robot_vel.dot(object_vel) / (robot_vel.calculateLength() * object_vel.calculateLength());
	Angle theta(std::acos(cos_angle));
	return theta;
}

// ------------------------------------------------------------------- //

Angle SocialForceModel::computeThetaAlphaBetaAngle2014(const Vector& n_alpha, const Vector& d_alpha_beta) {

	/* 2014 - "φ αβ is the Angle between n α and d αβ"
	 * n_alpha, 	- actor's normal (based on velocity vector)
	 * d_alpha_beta - vector between objects positions
	 */

	/* both n α and d αβ are expressed in world's coordinate system so
	 * simple Angle difference should do the job */

	Angle angle_n_alpha(n_alpha);
	Angle angle_d_alpha_beta(d_alpha_beta);

	Angle phi_alpha_beta(angle_n_alpha - angle_d_alpha_beta);
	phi_alpha_beta.normalize();

	return phi_alpha_beta;

}

// ------------------------------------------------------------------- //

// dynamic objects interaction
Angle SocialForceModel::computeThetaAlphaBetaAngle(const Angle& actor_yaw, const Angle& object_yaw) {
    /*
	 *
	 * NOTE: below method (very simple and naive) of calculating the Angle is correct
	 * only when both objects are:
	 * 		o dynamic,
	 * 		o currently moving,
	 * 		o already aligned with the to-target-direction,
	 * 		o there are no obstacles in the environment that will make the object not move along a straight line.
	 */

	// only on-plane movement considered

	Angle yaw_diff(actor_yaw - object_yaw);
	yaw_diff.normalize();


	return yaw_diff;//.getRadian();

}

// ------------------------------------------------------------------- //

Vector SocialForceModel::computeNormalAlphaDirection(const Pose &actor_pose) {
	return SocialForceModel::computeNormalAlphaDirection(Angle(actor_pose.getYaw()), param_description_);
}

Vector SocialForceModel::computeNormalAlphaDirection(const Angle &robot_yaw, ParameterDescription param_description) {

	// when speed is 0 then there is no way of calculating the Angle THETA_alpha_beta (0 vector length)
	// better way is calculating normal based on actor's yaw than velocity vector


	/* all calculations here are performed on world coordinate system data -
	 * n_alpha from actor's coordinate system is projected onto
	 * world's coordinate system axes */
	Angle yaw_norm = robot_yaw;

	// check parameter setting - n_alpha issue there
	switch (param_description) {

	case(PARAMETER_DESCRIPTION_2011):
			// vector pointing opposite direction
			yaw_norm -= Angle(IGN_PI);
			yaw_norm.normalize();
			break;

	case(PARAMETER_DESCRIPTION_2014):
	default:
			// do not rotate
			break;

	}

	Vector n_alpha(yaw_norm);
	return n_alpha;

}

// ------------------------------------------------------------------- //

Vector SocialForceModel::computePerpendicularToNormal(const Vector &n_alpha, const RelativeLocation &beta_rel_location) {
	return SocialForceModel::computePerpendicularToNormal(n_alpha, beta_rel_location, param_description_);
}

Vector SocialForceModel::computePerpendicularToNormal(
		const Vector &n_alpha,
		const RelativeLocation &beta_rel_location,
		ParameterDescription param_description)

{

	/* Depending on which side beta is on relative to n_alpha, the p_alpha (perpendicular vector)
	 * will point to direction opposite to the side where beta is */

	/*
	Vector p_alpha;
	if ( _beta_rel_location == RelativeLocation::LOCATION_RIGHT ) {
		p_alpha = _n_alpha.Perpendicular();
		// return (_n_alpha.Perpendicular());
	} else if ( _beta_rel_location == RelativeLocation::LOCATION_LEFT ) {

		// Vector p_alpha;

		// inverse-perpendicular vector calculations based on ignition library
		static const double sqr_zero = 1e-06 * 1e-06;
		Vector to_cross = {0, 0, 1}; // TODO: -1?
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

	Vector p_alpha;
	static const double sqr_zero = 1e-06 * 1e-06;
	Vector to_cross;

	if ( beta_rel_location == RelativeLocation::LOCATION_LEFT ) {

		// check parameter - n_alpha issue there
		switch (param_description) {

		case(PARAMETER_DESCRIPTION_2011):
				to_cross = Vector(0.0, 0.0, -1.0);
				break;

		case(PARAMETER_DESCRIPTION_2014):
		default:
				to_cross = Vector(0.0, 0.0,  1.0);
				break;

		}


	} else if ( beta_rel_location == RelativeLocation::LOCATION_RIGHT ) {

		// check parameter - n_alpha issue there
		switch (param_description) {

		case(PARAMETER_DESCRIPTION_2011):
				to_cross = Vector(0.0, 0.0,  1.0);
				break;
		case(PARAMETER_DESCRIPTION_2014):
		default:
				to_cross = Vector(0.0, 0.0, -1.0);
				break;

		}

	} else {

	}

	p_alpha = n_alpha.cross(to_cross);

	// Check the length of the vector
	if (p_alpha.calculateLength() < sqr_zero) {

		// this should not happen
		to_cross = Vector(0, -1, 0);
		p_alpha = p_alpha.cross(to_cross);

	}


	return ( p_alpha );

}

// ------------------------------------------------------------------- //

inline bool SocialForceModel::isOutOfFOV(const Angle& angle_relative) {
	if (std::abs(angle_relative.getRadian()) >= cfg_->fov) {
		return (true);
	}
	return (false);
}

// ------------------------------------------------------------------- //

double SocialForceModel::computeFactorFOV(const Angle& angle_relative) {
	// full FOV is passed to the static method
	return SocialForceModel::computeFactorFOV(
		angle_relative.getRadian(),
		2.0 * cfg_->fov,
		cfg_->fov_factor_method == sfm::FovCalculationMethod::GAUSSIAN
	);
}

// ------------------------------------------------------------------- //

double SocialForceModel::computeRelativeSpeed(const Vector& actor_vel, const Vector& object_vel) {
	Vector actor_vel_no_angular = actor_vel;
	actor_vel_no_angular.setZ(0.0);
	Vector object_vel_no_angular = object_vel;
	object_vel_no_angular.setZ(0.0);
	return ((object_vel_no_angular - actor_vel_no_angular).calculateLength());
}

// ------------------------------------------------------------------- //

void SocialForceModel::factorInForceCoefficients() {

	force_internal_ *= cfg_->internal_force_factor;
	force_interaction_static_ *= cfg_->static_interaction_force_factor;
	force_interaction_dynamic_ *= cfg_->dynamic_interaction_force_factor;
	computeCombinedForce();

}

// ------------------------------------------------------------------- //

void SocialForceModel::applyNonlinearOperations(const double &dist_closest_static, const double &dist_closest_dynamic) {

	// FIXME: choose a closest distance to an obstacle
	double dist_closest = std::min(dist_closest_static, dist_closest_dynamic);

	// truncate the force value to max to prevent strange speedup of an actor
	double force_combined_magnitude_init = force_combined_.calculateLength();

	// evaluate force magnitude
	if (force_combined_magnitude_init >= cfg_->max_force) {

		multiplyForces(cfg_->max_force / force_combined_magnitude_init);

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
//		Vector avg = sf_values_.getAverage();
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
//		Vector avg = sf_values_.getAverage();
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
		Vector extension = extension_len * force_combined_.normalized();

		// add the resulting vector to `force_interaction_dynamic_`
		force_interaction_dynamic_ += extension;

		// sum up
		computeCombinedForce();
		sf_values_.update(force_combined_);

		// make sure the average is a non-zero vector
		Vector avg = sf_values_.getAverage();
		if ( avg.calculateLength() > 1e-06 ) {
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

	force_internal_ *= coefficient;
	force_interaction_dynamic_ *= coefficient;
	force_interaction_static_ *= coefficient;
	computeCombinedForce();

}

// ------------------------------------------------------------------- //

void SocialForceModel::reset() {
	force_internal_ = Vector();
	force_interaction_static_ = Vector();
	force_interaction_dynamic_ = Vector();
	force_combined_ = Vector();
}

void SocialForceModel::computeCombinedForce() {
	force_combined_ = force_internal_ + force_interaction_dynamic_ + force_interaction_static_;
}

// ------------------------------------------------------------------- //

} /* namespace sfm */
} /* namespace humap_local_planner */
