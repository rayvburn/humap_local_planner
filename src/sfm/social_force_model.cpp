/*
 * SocialForceModel.cpp
 *
 *  Created on: Dec 4, 2018
 *      Author: rayvburn
 */

// #include <actor/core/Target.h>	// isModelNegligible static function

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

//

#include <hubero_local_planner/debug.h>

#define DEBUG_BASIC 0
#define DEBUG_WARN 0
#define DEBUG_ERROR 0

#define debug_print_basic(fmt, ...) _template_debug_print_basic_(DEBUG_BASIC, fmt, ##__VA_ARGS__)
#define debug_print_warn(fmt, ...) _template_debug_print_warn_(DEBUG_WARN, fmt, ##__VA_ARGS__)
#define debug_print_err(fmt, ...) _template_debug_print_err_(DEBUG_ERROR, fmt, ##__VA_ARGS__)

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
	/*
	fov_(2.00),
	speed_max_(1.50),
	person_mass_(1),
	factor_force_internal_(100.0), // desired_force_factor(200.0),
	factor_force_interaction_(3000.0), // interaction_force_factor(6000.0),
	force_max_(2000.0), force_min_(300.0), // force_min(800.0)
	inflation_type_(INFLATION_ELLIPSE),
	interaction_static_type_(INTERACTION_ELLIPTICAL),
	*/
	param_description_(PARAMETER_DESCRIPTION_2014),
	/*
	opposite_force_method_(OPPOSITE_FORCE_GO_TOWARDS_GOAL),
	factor_maneuverability_(6.5),
	*/
	dir_alpha_(0.0),
	dist_closest_dynamic_(std::numeric_limits<double>::max()),
	dist_closest_static_(std::numeric_limits<double>::max()),
	cfg_(nullptr)
{
	setParameters();

	// raw value used instead
	//factor_maneuverability_ /= 1000;

	closest_points_.clear();
}

// ------------------------------------------------------------------- //

void SocialForceModel::init(hubero_local_planner::HuberoConfigConstPtr cfg) {
	cfg_ = std::make_shared<hubero_local_planner::HuberoConfig::SfmParams>(cfg->sfm);
	// FIXME: this was missing in previous ->getSfmParams
	// inflation_type_ = InflationType::INFLATION_ELLIPSE;
}
//void SocialForceModel::init(std::shared_ptr<const actor::ros_interface::ParamLoader> params_ptr,
//		  const InflationType &inflation_type, const std::string &actor_name,
//		  const gazebo::physics::WorldPtr &world_ptr)
//{
//
//	params_ptr_ = params_ptr;
//
//	factor_force_internal_ = params_ptr_->getSfmParams().internal_force_factor;
//	factor_force_interaction_ = params_ptr_->getSfmParams().interaction_force_factor;
//	person_mass_ = static_cast<unsigned short int>(params_ptr_->getSfmParams().mass);
//	factor_maneuverability_ = params_ptr_->getSfmParams().maneuverability / 1000.0;
//	speed_max_ = params_ptr_->getSfmParams().max_speed;
//	fov_ = params_ptr_->getSfmParams().fov;
//	force_min_ = params_ptr_->getSfmParams().min_force;
//	force_max_ = params_ptr_->getSfmParams().max_force;
//	interaction_static_type_ = static_cast<sfm::StaticObjectInteraction>(params_ptr_->getSfmParams().static_obj_interaction);
//
//	opposite_force_method_ = static_cast<sfm::OppositeForceMethod>(params_ptr_->getSfmParams().opposite_force);
//	disable_interaction_forces_ = params_ptr_->getSfmParams().disable_interaction_forces;
//
//	owner_name_ = actor_name;
//
//	// initialize historical relative locations map with arbitrary values,
//	// discard objects that should be ignored
//	for ( unsigned int i = 0; i < world_ptr->ModelCount(); i++ ) {
//
//		if ( isModelNegligible(world_ptr->ModelByIndex(i)->GetName()) ) {
//			continue;
//		}
//
//		map_models_rel_locations_[ world_ptr->ModelByIndex(i)->GetName() ] = LOCATION_UNSPECIFIED;
//
//	}
//
//	if ( !boundary_.init(params_ptr, world_ptr) ) {
//		std::cout << "[SocialForceModel::init] WorldBoundary class initialization failed or model name does not exist in the database. Check the 'world_dictionary/world_model' parameter (located among SFM params), if needed" << std::endl;
//	}
//
//}

// ------------------------------------------------------------------- //

bool SocialForceModel::computeSocialForce(
		const hubero_local_planner::ObstContainerConstPtr obstacles,
		const Pose3 &pose,
		const Vector3 &velocity,
		const Vector3 &target,
		const double &dt) {
	// reset internal state at the start of computations
	reset();

	// RosService-driven?
	(SfmGetPrintData()) ? (print_info = true) : (0);



	// -----------------------------------------------------------------------------------------
	// ------- internal force calculation ------------------------------------------------------
	// -----------------------------------------------------------------------------------------
	force_internal_ = computeInternalForce(pose, velocity, target);

	// check whether it is needed to calculate interaction force or only the internal one
	// should be considered
	//if (disable_interaction_forces_ ) {
	if (cfg_->disable_interaction_forces) {
		factorInForceCoefficients(); 	// multiply force vector components by parameter values
		return (false);
	}

	// -----------------------------------------------------------------------------------------
	// ------ interaction forces calculation (repulsive, attractive) ---------------------------
	// -----------------------------------------------------------------------------------------
	// added up to the `force_interaction_`
	Vector3 f_alpha_beta;

	// whether obstacle is moving or not (different computation method selected)
	bool is_dynamic;

	for (const auto& obstacle: *obstacles) {
		// check if the obstacle is static or dynamic (force calculation differs)
		is_dynamic = obstacle->isDynamic();

		// TODO: consider actor personal space even when he is standing!

		// model_closest i.e. closest to a robot
		Pose3 robot_closest_to_obstacle_pose = pose;

		// no inflation, `closest points` are objects centers (done above)
		Eigen::Vector2d obstacle_closest = obstacle->getClosestPoint(Eigen::Vector2d(pose.Pos().X(), pose.Pos().Y()));
//		std::cout << "\t obstacle_closest: " << obstacle_closest[0] << "  " << obstacle_closest[1] << std::endl;
		Pose3 obstacle_closest_to_robot_pose = Pose3(obstacle_closest[0], obstacle_closest[1], 0.0, 0.0, 0.0, 0.0);
//		std::cout << "\t obstacle_closest_to_robot_pose: " << obstacle_closest_to_robot_pose << std::endl;
		auto distance_v_eig = obstacle_closest - Eigen::Vector2d(pose.Pos().X(), pose.Pos().Y());
//		std::cout << "\t distance_v_eig: " << distance_v_eig[0] << "  " << distance_v_eig[1] << std::endl;
		Vector3 distance_v(distance_v_eig[0], distance_v_eig[1], 0.0);
//		std::cout << "\t distance_v: " << distance_v << std::endl;
		double distance = obstacle->getMinimumDistance(Eigen::Vector2d(pose.Pos().X(), pose.Pos().Y()));
//		std::cout << "\t distance: " << distance << std::endl;
		Vector3 model_vel = Vector3((obstacle->getCentroidVelocity())[0], (obstacle->getCentroidVelocity())[1], 0.0);
//		std::cout << "\t model_vel: " << model_vel << std::endl;
		is_dynamic = model_vel.Length() > 1e-05;

		if (is_dynamic) {
			debug_print_basic(ANSI_COLOR_RED "DYNAMIC - obs: %d, vel-based: %d / vel: %2.4f, %2.4f" ANSI_COLOR_RESET "\r\n",
				obstacle->isDynamic(),
				is_dynamic,
				model_vel.X(),
				model_vel.Y()
			);
		}

		// from interaction force internals

		// `d_alpha_beta` is a vector between objects positions.
		Vector3 d_alpha_beta = obstacle_closest_to_robot_pose.Pos() - robot_closest_to_obstacle_pose.Pos();
		d_alpha_beta.Z(0.0); // NOTE: in SFM calculations it is assumed that all objects are in the actor's plane
		double d_alpha_beta_length = d_alpha_beta.Length(); // Length calculates vector's distance with each call

		Angle robot_yaw(getYawFromPose(pose));
		robot_yaw.Normalize();

		RelativeLocation beta_rel_location = LOCATION_UNSPECIFIED;
		double beta_angle_rel = 0.0;
		double d_alpha_beta_angle = 0.0;
		std::tie(beta_rel_location, beta_angle_rel, d_alpha_beta_angle) = computeObjectRelativeLocation(
				robot_yaw,
				d_alpha_beta
		);

		// based on a parameter and an object type - calculate a force from a static object properly
		if (is_dynamic || cfg_->static_obj_interaction == INTERACTION_REPULSIVE_EVASIVE) {

			// calculate interaction force
			std::tie(f_alpha_beta, distance_v, distance) = computeInteractionForce(
				robot_closest_to_obstacle_pose,
				velocity,
				obstacle_closest_to_robot_pose,
				model_vel,
				//
				d_alpha_beta,
				d_alpha_beta_length,
				beta_rel_location,
				beta_angle_rel,
				d_alpha_beta_angle,
				//
				is_dynamic
			); // FIXME: is an actor



//			if ( is_actor ) {
//				dir_beta_dynamic_v_.push_back(/*convertActorToWorldOrientation(*/object_pose.Rot().Yaw()/*)*/);
//			} else {
//				ignition::math::Angle angle(std::atan2(object_vel.Y(), object_vel.X()));
//				angle.Normalize();
//				// TODO: major change!
//				//dir_beta_dynamic_v_.push_back(angle.Radian());
//			}

//			std::cout << "\t DYNAMIC f_alpha_beta: " << f_alpha_beta << std::endl;
//			std::cout << "\t DYNAMIC distance_v: " << distance_v << std::endl;
//			std::cout << "\t DYNAMIC distance: " << distance << std::endl;

			// truncate to `FORCE_INTERACTION_MAX` length
			const double FORCE_INTERACTION_MAX = 1200; 	// 900.0; (intersection occurs)
			if (f_alpha_beta.Length() * cfg_->interaction_force_factor > FORCE_INTERACTION_MAX) {
				f_alpha_beta = f_alpha_beta.Normalized() * (FORCE_INTERACTION_MAX/(f_alpha_beta.Length() * cfg_->interaction_force_factor));
			}

		} else {

			std::tie(f_alpha_beta, distance_v, distance) = computeForceStaticObstacle(
				robot_closest_to_obstacle_pose,
				velocity,
				obstacle_closest_to_robot_pose,
				dt
			);

//			std::cout << "\t STATIC f_alpha_beta: " << f_alpha_beta << std::endl;
//			std::cout << "\t STATIC distance_v: " << distance_v << std::endl;
//			std::cout << "\t STATIC distance: " << distance << std::endl;

		}

		if (is_dynamic) {
			// ---- fuzzylite-related
			dir_alpha_ = std::atan2(velocity.Y(), velocity.X()); // /*convertActorToWorldOrientation(*/actor_pose.Rot().Yaw()/*)*/;
			// TODO: major change
			rel_loc_dynamic_v_.push_back(beta_angle_rel);
			dist_angle_dynamic_v_.push_back(d_alpha_beta_angle);
			dist_dynamic_v_.push_back(d_alpha_beta_length);
			dir_beta_dynamic_v_.push_back(std::atan2(model_vel.Y(), model_vel.X()));
		}

		/* debug closest points
		 * a pair must be added to vector, pair consists
		 * of model closest point's pose and an actor's
		 * pose that is closest to the model;
		 * note that actor's pose lies on its bounding
		 * figure's border (in most cases) */
		if (f_alpha_beta.Length() > 1e-06) {
			closest_points_.push_back(obstacle_closest_to_robot_pose);
			closest_points_.push_back(robot_closest_to_obstacle_pose);
		}

		// save distance to the closest obstacle (evaluates if the given one is smaller
		// than the one considered as `the closest` so far);
		// NOTE `actor` is treated as a dynamic object all the time (even if currently doesn't move)
		updateClosestObstacleDistance(
			obstacle->isDynamic() ? dist_closest_dynamic_ : dist_closest_static_,
			distance
		);

		// sum all forces
		force_interaction_ += f_alpha_beta;

//		std::cout << "\t sum so far: f_alpha_beta: " << force_interaction_ << std::endl;

//		std::cout << "===============================================" << std::endl;

	} /* for loop ends here (iterates over all world models) */

	// multiply force vector components by parameter values
	factorInForceCoefficients();
	debug_print_basic("\t force_internal_: %2.3f, %2.3f, %2.3f\r\n", force_internal_.X(), force_internal_.Y(), force_internal_.Z());
	debug_print_basic("\t force_interaction_: %2.3f, %2.3f, %2.3f\r\n", force_interaction_.X(), force_interaction_.Y(), force_interaction_.Z());

#ifdef SFM_PRINT_FORCE_RESULTS
	if ( print_info ) {
		std::cout << "-----------------------\n";
		std::cout << owner_name_ << " | SocialForce: " << force_combined_ << "\tinternal: " << force_internal_ << "\tinteraction: " << force_interaction_ << "\tsocial: " << force_social_;
	}
#endif

#ifdef DEBUG_FORCE_PRINTING_SF_TOTAL_AND_NEW_POSE
	( SfmGetPrintData() ) ? (print_info = false) : (0);
#endif

	// extend or truncate force vectors if needed
	applyNonlinearOperations(dist_closest_static_, dist_closest_dynamic_);

	debug_print_basic("\t\t\t force_internal_: %2.3f, %2.3f, %2.3f\r\n", force_internal_.X(), force_internal_.Y(), force_internal_.Z());
	debug_print_basic("\t\t\t force_interaction_: %2.3f, %2.3f, %2.3f\r\n", force_interaction_.X(), force_interaction_.Y(), force_interaction_.Z());

#ifdef DEBUG_FORCE_PRINTING_SF_TOTAL_AND_NEW_POSE
	( SfmGetPrintData() ) ? (print_info = true) : (0);
#endif

#ifdef SFM_PRINT_FORCE_RESULTS
	if ( print_info ) {
		std::cout << "\n" << owner_name_ << " | finalValue: " << force_combined_ << "\tlength: " << force_combined_.Length() << std::endl;
	}
#endif

#ifdef DEBUG_FORCE_PRINTING_SF_TOTAL_AND_NEW_POSE
	( SfmGetPrintData() ) ? (print_info = false) : (0);
#endif

#ifdef SFM_PRINT_FORCE_RESULTS
	std::cout << "\n\t\tINTERNAL: \t" << force_internal_ << std::endl;
	std::cout << "\t\tINTERACTION: \t" << force_interaction_ << std::endl;
	std::cout << "\t\tSOCIAL: \t" <<  force_social_ << std::endl;
	std::cout << "\t\tTOTAL: \t\t" << force_combined_ << std::endl;
	std::cout << "**************************************************************************\n\n";
#endif

	return (true);
}

//
//bool SocialForceModel::computeSocialForce(const gazebo::physics::WorldPtr &world_ptr,
//		const Pose3 &actor_pose, const Vector3 &actor_velocity,
//		const Vector3 &actor_target, const actor::core::CommonInfo &actor_info,
//		const double &dt, const std::vector<std::string> &ignored_models_v)
//{
//
//	// reset internal state at the start of computations
//	reset();
//
//	// RosService-driven?
//	(SfmGetPrintData()) ? (print_info = true) : (0);
//
//	// -----------------------------------------------------------------------------------------
//	// ------- internal force calculation ------------------------------------------------------
//	// -----------------------------------------------------------------------------------------
//	force_internal_ = computeInternalForce(actor_pose, actor_velocity, actor_target);
//
//	// check whether it is needed to calculate interaction force or only the internal one
//	// should be considered
//	//if (disable_interaction_forces_ ) {
//	if (cfg_->disable_interaction_forces) {
//		factorInForceCoefficients(); 	// multiply force vector components by parameter values
//		return (false);
//	}
//
//	// -----------------------------------------------------------------------------------------
//	// ------ interaction forces calculation (repulsive, attractive) ---------------------------
//	// -----------------------------------------------------------------------------------------
//	// added up to the `force_interaction_`
//	Vector3 f_alpha_beta;
//
//	// whether obstacle is moving or not (different computation method selected)
//	bool is_dynamic;
//
//	/* model_vel contains model's velocity (world's object or actor) - for the actor this is set differently
//	 * it was impossible to set actor's linear velocity by setting it by the model's class method */
//	Vector3 model_vel;
//
//	/* Border pointer which will carry cases related to repulsive force calculations
//	 * associated with other actors borders (i.e. box/circle/ellipse) stored
//	 * in the `actor::core::CommonInfo` 'container' class.
//	 * A raw (`naked`) pointer is used instead of the shared_ptr instance because
//	 * of Border->Box casting problems (`dynamic_pointer_cast` tested with no luck -
//	 * the Box pointer still executes Border virtual methods)
//	 */
//	inflation::Border* model_border_ptr = nullptr; // std::shared_ptr<inflation::Border> model_border_ptr;
//
//	/* The `dynamic_cast` operation does not allow to convert `inflation::Border`
//	 * to the `inflation::Box` class. Because of that, a new instance
//	 * is created which carries the Box configuration methods (which are not virtual).
//	 * It is used only as a helper in the process of assigning a proper pointer
//	 * to the `model_border_ptr` (see @ref preprocessTypicalModel and @ref preprocessWorldBoundary)
//	 * Default constructor called below.
//	 */
//	inflation::Box model_border_box;
//
//	/* the flag introduced below is used as a workaround for the problem connected with inability
//	 * to set actor velocity and acceleration in the gazebo::physics::WorldPtr */
//	bool is_an_actor = false;
//	Pose3 model_raw_pose;
//	std::string model_name; // TODO: can be deleted in the final version
//
//	/* store distance vector (connecting actor's and obstacle's `closest` points) and its length */
//	Vector3 distance_v;
//	double distance = 0.0;
//
//	// ============================================================================
//
//	// iterate over all world's objects + 4 walls (at least try)
//	for ( unsigned int i = 0; i < (world_ptr->ModelCount() + 4); i++ ) {
//
//		// ***** preprocessing section - can break the iteration if the model is negligible/invalid *****
//
//		// evaluate if the object to consider is a valid Gazebo model or only the artificial one,
//		// i.e. that is represented by a bounding box and is not recognized
//		// in the Gazebo model list (an artificial boundary generated in the operation of a division
//		// of the 'single model of the world'
//		if ( isTypicalModel(i, world_ptr->ModelCount()) ) {
//
//			// flag storing the output of the preprocessing routine
//			bool is_valid = false;
//			std::tie(is_valid, is_an_actor, model_name, model_raw_pose, model_vel, model_border_ptr) =
//					preprocessTypicalModel(world_ptr, i, ignored_models_v, actor_info, model_border_box);
//
//			if ( !is_valid ) {
//				// the current model is negligible, skip the iteration
//				continue;
//			}
//
//		} else {
//
//			// flag storing the output of the preprocessing routine
//			bool is_valid = false;
//
//			// `wall_num` range: 0-3
//			std::tie(is_valid, is_an_actor, model_name, model_raw_pose, model_vel, model_border_ptr) =
//					preprocessWorldBoundary(world_ptr->ModelCount() + 3 - i, model_border_box);
//
//			if ( !is_valid ) {
//				// the current model (boundary) is invalid, skip the iteration
//				continue;
//			}
//
//		}
//
//		// ***************** end of the preprocessing section *******************************************
//
//		// check if the obstacle is static or dynamic (force calculation differs)
//		is_dynamic = isDynamicObstacle(model_vel);
//
//		// TODO: consider actor personal space even when he is standing!
//
//		// model_closest i.e. closest to an actor or the actor's bounding
//		Pose3 actor_closest_to_model_pose = actor_pose;
//		Pose3 model_closest_point_pose = model_raw_pose;
//
//		// calculate closest points
//		switch(inflation_type_) {
//
//		case(INFLATION_BOX_OTHER_OBJECTS):
//
//				actor_closest_to_model_pose = actor_pose;
//				model_closest_point_pose.Pos() = inflator_.findClosestPointsModelBox(actor_pose, model_raw_pose, *model_border_ptr);
//				break;
//
//		case(INFLATION_BOX_ALL_OBJECTS):
//
//				std::tie( actor_closest_to_model_pose, model_closest_point_pose.Pos() ) =
//						inflator_.findClosestPointsBoxes(actor_pose, *(actor_info.getBorderPtr()),
//														  model_raw_pose, *(model_border_ptr), model_name);
//				break;
//
//		case(INFLATION_CIRCLE):
//
//				if ( is_an_actor ) {
//					std::tie( actor_closest_to_model_pose, model_closest_point_pose.Pos() ) =
//							inflator_.findClosestPointsCircles(actor_pose, *(actor_info.getBorderPtr()),
//															  model_raw_pose, *(model_border_ptr), model_name );
//				} else {
//					std::tie( actor_closest_to_model_pose, model_closest_point_pose.Pos() ) =
//							inflator_.findClosestPointsActorBox(actor_pose, *(actor_info.getBorderPtr()),
//															  model_raw_pose, *(model_border_ptr), model_name );
//				}
//				break;
//
//		case(INFLATION_ELLIPSE):
//
//				if ( is_an_actor ) {
//					std::tie( actor_closest_to_model_pose, model_closest_point_pose.Pos() ) =
//							inflator_.findClosestPointsEllipses(actor_pose, *(actor_info.getBorderPtr()),
//															  model_raw_pose, *(model_border_ptr), model_name );
//				} else {
//					std::tie( actor_closest_to_model_pose, model_closest_point_pose.Pos() ) =
//							inflator_.findClosestPointsActorBox(actor_pose, *(actor_info.getBorderPtr()),
//															  model_raw_pose, *(model_border_ptr), model_name );
//
//				}
//				break;
//
//		default:
//
//				// no inflation, `closest points` are objects centers (done above)
//				break;
//
//		}
//
//		// based on a parameter and an object type - calculate a force from a static object properly
//		if ( is_dynamic || interaction_static_type_ == INTERACTION_REPULSIVE_EVASIVE ) {
//
//			// calculate interaction force
//			std::tie(f_alpha_beta, distance_v, distance) = computeInteractionForce(	actor_closest_to_model_pose, actor_velocity,
//													model_closest_point_pose, model_vel, is_an_actor);
//
//			// truncate to `FORCE_INTERACTION_MAX` length
//			const double FORCE_INTERACTION_MAX = 1200; 	// 900.0; (intersection occurs)
//			if ( f_alpha_beta.Length() * factor_force_interaction_ > FORCE_INTERACTION_MAX ) {
//				f_alpha_beta = f_alpha_beta.Normalized() * (FORCE_INTERACTION_MAX/(f_alpha_beta.Length() * factor_force_interaction_));
//			}
//
//		} else {
//
//			std::tie(f_alpha_beta, distance_v, distance) = computeForceStaticObstacle(actor_closest_to_model_pose, actor_velocity,
//													  model_closest_point_pose, dt);
//
//		}
//
//		/* debug closest points
//		 * a pair must be added to vector, pair consists
//		 * of model closest point's pose and an actor's
//		 * pose that is closest to the model;
//		 * note that actor's pose lies on its bounding
//		 * figure's border (in most cases) */
//		if ( f_alpha_beta.Length() > 1e-06 ) {
//			closest_points_.push_back(model_closest_point_pose);
//			closest_points_.push_back(actor_closest_to_model_pose);
//		}
//
//		// save distance to the closest obstacle (evaluates if the given one is smaller
//		// than the one considered as `the closest` so far);
//		// NOTE `actor` is treated as a dynamic object all the time (even if currently doesn't move)
//		updateClosestObstacleDistance(((is_an_actor || is_dynamic) ? dist_closest_dynamic_ : dist_closest_static_), distance);
//
//		// sum all forces
//		force_interaction_ += f_alpha_beta;
//
//	} /* for loop ends here (iterates over all world models) */
//
//	// ============================================================================
//
//	// multiply force vector components by parameter values
//	factorInForceCoefficients();
//
//#ifdef SFM_PRINT_FORCE_RESULTS
//	if ( print_info ) {
//		std::cout << "-----------------------\n";
//		std::cout << owner_name_ << " | SocialForce: " << force_combined_ << "\tinternal: " << force_internal_ << "\tinteraction: " << force_interaction_ << "\tsocial: " << force_social_;
//	}
//#endif
//
//#ifdef DEBUG_FORCE_PRINTING_SF_TOTAL_AND_NEW_POSE
//	( SfmGetPrintData() ) ? (print_info = false) : (0);
//#endif
//
//	// extend or truncate force vectors if needed
//	applyNonlinearOperations(dist_closest_static_, dist_closest_dynamic_);
//
//#ifdef DEBUG_FORCE_PRINTING_SF_TOTAL_AND_NEW_POSE
//	( SfmGetPrintData() ) ? (print_info = true) : (0);
//#endif
//
//#ifdef SFM_PRINT_FORCE_RESULTS
//	if ( print_info ) {
//		std::cout << "\n" << owner_name_ << " | finalValue: " << force_combined_ << "\tlength: " << force_combined_.Length() << std::endl;
//	}
//#endif
//
//#ifdef DEBUG_FORCE_PRINTING_SF_TOTAL_AND_NEW_POSE
//	( SfmGetPrintData() ) ? (print_info = false) : (0);
//#endif
//
//#ifdef SFM_PRINT_FORCE_RESULTS
//	std::cout << "\n\t\tINTERNAL: \t" << force_internal_ << std::endl;
//	std::cout << "\t\tINTERACTION: \t" << force_interaction_ << std::endl;
//	std::cout << "\t\tSOCIAL: \t" <<  force_social_ << std::endl;
//	std::cout << "\t\tTOTAL: \t\t" << force_combined_ << std::endl;
//	std::cout << "**************************************************************************\n\n";
//#endif
//
//	return (true);
//
//}

// ------------------------------------------------------------------- //

Pose3 SocialForceModel::computeNewPose(const Pose3 &actor_pose,
			const Vector3 &actor_vel, const Vector3 &social_force,
			const Vector3 &target, const double &dt)
{

	/* II Newton's law equation			a = F / m
	 * Straight-line movement equation 	v = a * t
	 * with the use of 2 above - calculate the resulting ideal velocity caused by social forces */
	Vector3 result_vel = (social_force / cfg_->mass) * dt;
	Vector3 result_vel_backup = result_vel; // debugging purposes

#ifdef DEBUG_NEW_POSE
	if ( print_info ) {
		std::cout << "GetNewPose(): ";
		std::cout << "\tresult_vel: " << result_vel;
	}
#endif

	/* if calculated speed value is bigger than max_speed then perform normalization
	 * leave velocity direction as is, shorten the vector to max possible */
	if (result_vel.Length() > cfg_->max_speed) {

		result_vel = result_vel.Normalize() * cfg_->max_speed;

		#ifdef DEBUG_NEW_POSE
		if ( print_info ) {
			std::cout << "\t vel TRUNCATED!: " << result_vel;
		}
		#endif

	}

#ifdef DEBUG_NEW_POSE
	Vector3 result_vel_init = result_vel;
	// temporary vector caused by social-force-based displacements
	Vector3 new_position(_actor_pose.Pos().X() + result_vel.X() * _dt,
										  _actor_pose.Pos().Y() + result_vel.Y() * _dt,
										  _actor_pose.Pos().Z());
	if ( print_info ) {
		std::cout << "\nPOSITION1 \torig: " << _actor_pose.Pos() << "\tdelta_x: " << result_vel.X() * _dt << "\tdelta_y: " << result_vel.Y() * _dt << "\tnew_position: " << new_position << std::endl;
	}
#endif

	// -------------------------------------------------------------------------------------------------------------------

	/* Consider the yaw angle of the actor - it is crucial to make him face his
	 * current movement direction / current target.
	 * yaw_new is calculated according to the vector of resulting velocity -
	 * which is based on social force */
	ignition::math::Angle yaw_new = computeYawMovementDirection(actor_pose, actor_vel, result_vel, target);

	/* recalculation pros:
	 *  	o smooth rotational motion,
	 *  	o prevents getting stuck in 1 place (observed few times),
	 *  	o prevents sliding (usually caused by rise of v_rel between actors
	 *  	  which causes big interaction)
	 * cons:
	 * 		o prevents immediate action when actor is moving toward an obstacle.
	 * recalculation acts as a inertia force here */

	/* calculate velocity components according to the yaw_new (that was
	 * likely truncated to prevent jumps in rotational movement -
	 * this provides smoothed rotation)
	 * only on-plane motions are supported, thus X and Y calculations */

	/* result_vel vector is projected onto world's coordinate system axes
	 * according to yaw_new (expresses actor's direction in actor's system);
	 * because actor's coord. system is rotated (-90 deg) the projection onto
	 * 	o	x axis: cos(yaw_new - 90 deg) * Len = +sin(yaw_new) * Len
	 * 	o	y axis: sin(yaw_new - 90 deg) * Len = -cos(yaw_new) * Len  */
	// V1
//	result_vel.X( +sin(yaw_new.Radian()) * result_vel.Length() );
//	result_vel.Y( -cos(yaw_new.Radian()) * result_vel.Length() );


	// ------------- V2
	result_vel.X( cos(yaw_new.Radian()) * result_vel.Length() );
	result_vel.Y( sin(yaw_new.Radian()) * result_vel.Length() );


	if ( print_info ) {
		std::cout << "\n\tSMOOTHING ROTATION - RECALCULATED VEL\tdelta_x: " << result_vel.X() * dt << "\tdelta_y: " << result_vel.Y() * dt << '\n' << std::endl;
	}

	/* calculate new pose - consider current pose, velocity and delta of time -
	 * and set the new pose component values (for position and orientation) */

	// TODO: fix forced pose.Z() and pose.Roll() according to current 'stance'
	// TODO: hard-coded value for STANDING stance
	// TODO: assuming standing pose thus roll angle is set to half-pi (STANDING)

	Pose3 new_pose;
	new_pose.Set(actor_pose.Pos().X() + result_vel.X() * dt,
				 actor_pose.Pos().Y() + result_vel.Y() * dt,
				 actor_pose.Pos().Z(), //1.2138,
				 (IGN_PI/2),
				 0,
				 yaw_new.Radian() + IGN_PI_2); 	// V2 // transform back to actor's CS
				 // yaw_new.Radian()); 			// V1


#ifdef DEBUG_NEW_POSE

	#ifdef DEBUG_JUMPING_POSITION
	if ( (new_pose.Pos() - _actor_pose.Pos()).Length() > 0.1 ) {
		std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n\n\n";
		std::cout << "\nJUMP_IN_POSITION\tinit_pose: " << _actor_pose << "\tresult_vel init: " << result_vel_init << std::endl;
		std::cout << "\t\tnew_pose: " << new_pose << "result_vel recalculated: " << result_vel << std::endl;
		std::cout << "\t\tactor_ID: " << curr_actor << std::endl;
		std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n\n\n";
	}
	#endif

	if ( print_info ) {
		std::cout << "\nPOSITION2 \torig: " << _actor_pose.Pos() << "\tdelta_x: " << result_vel.X() * _dt << "\tdelta_y: " << result_vel.Y() * _dt << "\tnew_position: " << new_pose.Pos(); // << std::endl;
		std::cout << std::endl;
		std::cout << "---------------------------------------------------------------------------------";
		std::cout << std::endl;
	}

#endif

#ifdef DEBUG_FORCE_PRINTING_SF_TOTAL_AND_NEW_POSE
	if ( SfmGetPrintData() ) {
	std::cout <<  SfmDebugGetCurrentActorName() << " | GetNewPose() CALCULATION\n";
	std::cout << "\tactor_yaw_init: " << (actor_pose.Rot().Yaw()-IGN_PI_2) << "\tactor_yaw_mod: " << yaw_new.Radian() << std::endl;
	std::cout << "\tresult_vel_angle: " << std::atan2(result_vel.Y(), result_vel.X()) << std::endl;
	std::cout << "\t\tsf: " << social_force << "\tresult_vel_init: " << result_vel_backup << "\tresult_vel_mod: " << result_vel;
	if ( result_vel.X() > 0.0 && result_vel.Y() > 0.0 ) {
		std::cout << "\tI QUARTER";
	} else if ( result_vel.X() > 0.0 && result_vel.Y() < 0.0 ) {
		std::cout << "\tIV QUARTER";
	} else if ( result_vel.X() < 0.0 && result_vel.Y() > 0.0 ) {
		std::cout << "\tII QUARTER";
	} else if ( result_vel.X() < 0.0 && result_vel.Y() < 0.0 ) {
		std::cout << "\tIII QUARTER";
	}
	std::cout << "\nPOSITION \torig: " << actor_pose.Pos() << "\tdelta_x: " << result_vel.X() * dt << "\tdelta_y: " << result_vel.Y() * dt << "\tnew_position: " << new_pose.Pos(); // << std::endl;
	std::cout << std::endl;
	std::cout << "---------------------------------------------------------------------------------\n";
	}
#endif

	return new_pose;

}

// ------------------------------------------------------------------- //

void SocialForceModel::reset() {

	closest_points_.clear();

	force_internal_ = Vector3();
	force_interaction_ = Vector3();
//	force_social_ = Vector3();
	force_combined_ = Vector3();

	dir_alpha_ = 0.0;
	rel_loc_dynamic_v_.clear();
	dist_angle_dynamic_v_.clear();
	dist_dynamic_v_.clear();
	dir_beta_dynamic_v_.clear();

	dist_closest_dynamic_ = std::numeric_limits<double>::max();
	dist_closest_static_ = std::numeric_limits<double>::max();

}

// ------------------------------------------------------------------- //

Vector3 SocialForceModel::getForceInternal() const 	{ return (force_internal_);		}
Vector3 SocialForceModel::getForceInteraction() const 	{ return (force_interaction_); 	}
Vector3 SocialForceModel::getForceCombined() const		{ return (force_combined_); 	}

double SocialForceModel::getDirectionAlpha() const 						{ return (dir_alpha_); 				}
std::vector<double> SocialForceModel::getDirectionBetaDynamic() const 	{ return (dir_beta_dynamic_v_); 	}
std::vector<double> SocialForceModel::getRelativeLocationDynamic() const{ return (rel_loc_dynamic_v_); 		}
std::vector<double> SocialForceModel::getDistanceAngleDynamic() const 	{ return (dist_angle_dynamic_v_); 	}
std::vector<double> SocialForceModel::getDistanceDynamic() const 		{ return (dist_dynamic_v_); 		}
double SocialForceModel::getDistanceClosestStaticObstacle() const 		{ return (dist_closest_static_);	}
double SocialForceModel::getDistanceClosestDynamicObstacle() const 		{ return (dist_closest_dynamic_); 	}

std::vector<Pose3> SocialForceModel::getClosestPointsVector() const { return (closest_points_); }

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

Vector3 SocialForceModel::computeInternalForce(const Pose3 &actor_pose,
		const Vector3 &actor_vel, const Vector3 &actor_target) {

	// FIXME: a lot of allocations here
	Vector3 to_goal_vector = (actor_target - actor_pose.Pos());
	Vector3 to_goal_direction = to_goal_vector.Normalize();
	Vector3 ideal_vel_vector = speed_desired_ * to_goal_direction;
	Vector3 f_alpha = cfg_->mass /*person_mass_*/ * (1/relaxation_time_) * (ideal_vel_vector - actor_vel);
	f_alpha.Z(0.0);

	debug_print_basic("\t to_goal_vector: %2.3f, %2.3f, %2.3f \r\n", to_goal_vector.X(), to_goal_vector.Y(), to_goal_vector.Z());
	debug_print_basic("\t to_goal_direction: %2.3f, %2.3f, %2.3f \r\n", to_goal_direction.X(), to_goal_direction.Y(), to_goal_direction.Z());
	debug_print_basic("\t ideal_vel_vector: %2.3f, %2.3f, %2.3f \r\n", ideal_vel_vector.X(), ideal_vel_vector.Y(), ideal_vel_vector.Z());
	debug_print_basic("\t target - x=%2.3f, y=%2.3f, z=%2.3f \r\n", actor_target.X(), actor_target.Y(), actor_target.Z());
	debug_print_basic("\t actor_vel: %2.3f, %2.3f, %2.3f \r\n", actor_vel.X(), actor_vel.Y(), actor_vel.Z());
	debug_print_basic("\t mass: %2.3f \r\n", cfg_->mass);
	debug_print_basic("\t relaxation_time: %2.3f \r\n", relaxation_time_);
	debug_print_basic("\t f_alpha: %2.3f, %2.3f, %2.3f   |   multiplied: %2.3f, %2.3f, %2.3f \r\n",
			f_alpha.X(), f_alpha.Y(), f_alpha.Z(),
			(cfg_->internal_force_factor * f_alpha).X(),
			(cfg_->internal_force_factor * f_alpha).Y(),
			(cfg_->internal_force_factor * f_alpha).Z()
	);

	// internal force truncation (causes `overdrive` in close presence of
	// another actor)
	if ( f_alpha.Length() > 1100.0 ) {
		debug_print_err("INTERNAL FORCE TRUNCATED from %2.3f to 1100.0\r\n", f_alpha.Length());
		f_alpha = f_alpha.Normalized() * 1100.0;
	}

	return f_alpha;
}

// ------------------------------------------------------------------- //

/// \return F_alpha_beta - repulsive force created by `beta` object
/// \return d_alpha_beta - distance vector pointing from `alpha` to `beta`
/// \return theta_alpha_beta - angle	////// ? is this still needed?
std::tuple<Vector3, Vector3, double>
SocialForceModel::computeInteractionForce(
		const Pose3 &actor_pose,
		const Vector3 &actor_vel,
		const Pose3 &object_pose,
		const Vector3 &object_vel,
		const Vector3 &d_alpha_beta,
		const double &d_alpha_beta_length,
		const RelativeLocation &beta_rel_location,
		const double &beta_angle_rel,
		const double &d_alpha_beta_angle,
		const bool &is_actor)
{
	/* actor's normal (based on velocity vector, whose direction could
	 * be also acquired from his yaw angle */
	Vector3 n_alpha = computeNormalAlphaDirection(actor_pose);

	// ================================================================================
	// section from "GetObjectsInteractionForce()" function which is DEPRECATED now
	//
	//
	// TODO: only 6 closest actors taken into consideration?
	Vector3 f_alpha_beta(0.0, 0.0, 0.0);

	// check length to other object (beta)
	if ( d_alpha_beta_length > 7.5 ) {

		// TODO: if no objects nearby the threshold should be increased?
		#ifdef DEBUG_INTERACTION_FORCE
		if ( print_info ) {
			std::cout << "\t OBJECT TOO FAR AWAY, ZEROING FORCE! \t d_alpha_beta_length: " << d_alpha_beta_length;
			std::cout << std::endl;
		}
		#endif
		return (std::make_tuple(f_alpha_beta, d_alpha_beta, d_alpha_beta_length));

	}

#ifdef DEBUG_SHORT_DISTANCE
	if ( !print_info && curr_actor == 0 && d_alpha_beta_length < 0.4 ) {
		print_info = true;
	}
#endif

	/* yaw of an actor is always updated in new pose calculation procedure, so setting the yaw
	 * based on world info should work - actor is always oriented in his movement direction
	 * (if linear speed is non-zero) */
	ignition::math::Angle actor_yaw(getYawFromPose(actor_pose));
	actor_yaw.Normalize();

	// OBJECT_YAW used when V2011 and V2014 not #defined
	/* this is a simplified version - object's yaw could be taken from world's info indeed,
	 * but the object's coordinate system orientation is not known, so velocity calculation
	 * still may need to be performed (if needed depending on θ αβ calculation method);
	 * when `_is_actor` flag is set no further velocity calculation needed! */
	ignition::math::Angle object_yaw(getYawFromPose(object_pose));
	object_yaw.Normalize();



	// FIXME: here



	/* FOV factor is used to make interaction of objects
	 * that are behind the actor weaker */
	double fov_factor = 1.00;

	/* check whether beta is within the field of view
	 * to determine proper factor for force in case
	 * beta is behind alpha */
	if ( isOutOfFOV(beta_angle_rel) ) {

		// exp function used: e^(-0.5*x)
		fov_factor = std::exp( -0.5 * d_alpha_beta_length );
		#ifdef DEBUG_FORCE_EACH_OBJECT
		if ( SfmGetPrintData() ) {
			#ifdef DEBUG_FORCE_PRINTING_SF_TOTAL_AND_NEW_POSE
			std::cout << SfmDebugGetCurrentActorName();
			#endif
			std::cout << "\nDYNAMIC OBSTACLE (*) --- OUT OF FOV!" << std::endl;
			std::cout << "\t" << SfmDebugGetCurrentObjectName() << " is BEHIND, dist: " << d_alpha_beta_length << ",   force will be multiplied by: " << fov_factor << std::endl;
		}
		#endif

	}


	double v_rel = computeRelativeSpeed(actor_vel, object_vel);

	if ( v_rel < 1e-06 ) {

		#ifdef DEBUG_INTERACTION_FORCE
		if ( print_info ) {
			std::cout << "  v_rel = 0, ZEROING FORCE!";
			std::cout << std::endl;
		}
		#endif

		return (std::make_tuple(f_alpha_beta, d_alpha_beta, d_alpha_beta_length));

	}

	// speed of the Beta object
	double speed_beta = object_vel.Length();

	/* angle between velocities of alpha and beta is needed for Fuzzifier */
	double velocities_angle = computeThetaAlphaBetaAngle(actor_vel, actor_yaw, object_vel, object_yaw, d_alpha_beta, is_actor);

	// store angle between objects' (in most cases) velocities
	double theta_alpha_beta = 0.0;

	// check parameter value - theta_alpha_beta issue there
	switch (param_description_) {

	case(PARAMETER_DESCRIPTION_2011):
			//theta_alpha_beta = computeThetaAlphaBetaAngle(_actor_vel, actor_yaw, _object_vel, object_yaw, _is_actor);
			theta_alpha_beta = velocities_angle;
			break;

	case(PARAMETER_DESCRIPTION_2014):
			theta_alpha_beta = computeThetaAlphaBetaAngle(n_alpha, d_alpha_beta);
			break;

	case(PARAMETER_DESCRIPTION_UNKNOWN):
	default:
			theta_alpha_beta = computeThetaAlphaBetaAngle(actor_yaw, object_yaw);
			break;

	}

	/* For details see `Social force equation` in the articles listed in .h file */
	Vector3 p_alpha = computePerpendicularToNormal(n_alpha, beta_rel_location); 	// actor's perpendicular (based on velocity vector)
	// original
//	double exp_normal = ( (-Bn_ * theta_alpha_beta * theta_alpha_beta) / v_rel ) - Cn_ * d_alpha_beta_length;
//	double exp_perpendicular = ( (-Bp_ * std::fabs(theta_alpha_beta) ) / v_rel ) - Cp_ * d_alpha_beta_length;
	// modded
	double exp_normal = ( (-Bn_ * theta_alpha_beta * theta_alpha_beta) / (2.0 * v_rel) ) - 0.5 * Cn_ * d_alpha_beta_length;
	double exp_perpendicular = ( (-0.1 * Bp_ * std::fabs(theta_alpha_beta) ) / v_rel ) - 0.5 * Cp_ * d_alpha_beta_length;

	// `fov_factor`: weaken the interaction force when beta is behind alpha
	// original
//	Vector3 n_alpha_scaled = n_alpha * An_ * std::exp(exp_normal) * fov_factor;
//	Vector3 p_alpha_scaled = p_alpha * Ap_ * std::exp(exp_perpendicular) * fov_factor;
	// modded
	Vector3 n_alpha_scaled = n_alpha * (-4.0) * An_ * std::exp(exp_normal) * fov_factor;
	Vector3 p_alpha_scaled = p_alpha * (+2.0) * Ap_ * std::exp(exp_perpendicular) * fov_factor;

	// -----------------------------------------------------
	// FIXME: debugging large vector length ----------------
#ifdef SFM_DEBUG_LARGE_VECTOR_LENGTH
	std::cout << "\t-  -  -  - interaction force -  -  -  -  -  -  -  " << owner_name_ << std::endl;
	std::cout << "\t" << owner_name_ << " yaw: " << actor_yaw.Radian() << "\t" << SfmDebugGetCurrentObjectName() << " yaw: " << object_yaw.Radian() << std::endl;
	std::cout << "\tΘ_αß: " << theta_alpha_beta << "\tv_rel: " << v_rel << "\tdist: " << d_alpha_beta_length << std::endl;
	std::cout << "\texpNORMAL: " << exp_normal << "\texpPERP: " << exp_perpendicular << "\tFOV_factor: " << fov_factor << std::endl;
	std::cout << "\tNORMAL: " << factor_force_interaction_ * n_alpha_scaled.X() << " " << factor_force_interaction_ * n_alpha_scaled.Y() << "\tPERP: " << factor_force_interaction_ * p_alpha_scaled.X() << " " << factor_force_interaction_ * p_alpha_scaled.Y() << std::endl;
	std::cout << "\ttotal: " << factor_force_interaction_ * (n_alpha_scaled + p_alpha_scaled).X() << " " << factor_force_interaction_ * (n_alpha_scaled + p_alpha_scaled).Y() << std::endl;
	std::cout << std::endl;
#endif
	// -----------------------------------------------------

	// extra factor - applicable only for dynamic objects
	if ( speed_beta > 1e-06 ) {

		// (n_alpha_scaled + p_alpha_scaled).Length() > 1e-06 (condition deprecated, when got here then
		// force will surely be non-zero)

		// interaction strength exponentially decreases as distance between objects becomes bigger;
		// it is also artificially strengthened when distance is small
		double factor = 4.0 * std::exp(-2.0 * d_alpha_beta_length);
		n_alpha_scaled *= factor;
		p_alpha_scaled *= factor;

	}

	// save interaction force vector
	f_alpha_beta = n_alpha_scaled + p_alpha_scaled;

#ifdef DEBUG_FORCE_EACH_OBJECT

	if ( SfmGetPrintData() ) {
	#ifdef DEBUG_FORCE_PRINTING_SF_TOTAL_AND_NEW_POSE
	std::cout << "\n-----------------------\n" << SfmDebugGetCurrentActorName();
	#else
	if ( SfmDebugGetCurrentActorName() == "actor1" ) {
	#endif
		std::cout << "\nDYNAMIC OBSTACLE" << std::endl;
		std::cout << "\t" << SfmDebugGetCurrentObjectName() << ": ";
		std::cout << "\tv_rel: " << v_rel << std::endl;
		std::cout << "\tn_alpha: " << n_alpha;
		std::cout << "\texp_n: " << exp_normal << "\ttheta_alpha_beta: " << theta_alpha_beta << "\td_alpha_beta_len: " << d_alpha_beta_length << std::endl;
		std::cout << "\tp_alpha: " << p_alpha;
		std::cout << "\texp_p: " << exp_perpendicular;

		std::string location_str;
		if ( beta_rel_location == LOCATION_FRONT ) {
			location_str = "FRONT";
		} else if ( beta_rel_location == LOCATION_BEHIND ) {
			location_str = "BEHIND";
		} else if ( beta_rel_location == LOCATION_RIGHT ) {
			location_str = "RIGHT SIDE";
		} else if ( beta_rel_location == LOCATION_LEFT ) {
			location_str = "LEFT SIDE";
		} else if ( beta_rel_location == LOCATION_UNSPECIFIED ) {
			location_str = "UNKNOWN";
		}
		std::cout << "\t\trel_location: " << location_str << std::endl;

		std::cout << "\tf_alpha_beta: " << f_alpha_beta * factor_force_interaction_ << "\t\tvec len: " << factor_force_interaction_ * f_alpha_beta.Length() << std::endl;
	#ifndef DEBUG_FORCE_PRINTING_SF_TOTAL_AND_NEW_POSE
	}
	#endif
	}
#endif



#ifdef DEBUG_INTERACTION_FORCE
	if ( print_info ) {
		std::cout << "Interaction";
		std::cout << "\tv_rel: " << v_rel;
		std::cout << "\texp_n: " << exp_normal;
		std::cout << "\texp_p: " << exp_perpendicular;
		std::cout << "\tf_alpha_beta: " << f_alpha_beta;
		std::cout << std::endl;
	}
#endif


//	return (f_alpha_beta);
	return (std::make_tuple(f_alpha_beta, d_alpha_beta, d_alpha_beta_length));

	/* Algorithm INPUTS are:
	 * - dt
	 * - goal reaching component (acceleration term)
	 * 		- target's pose
	 * 		- current actor's pose
	 * 		- actual speed
	 * 		- desired speed
	 * 		- relaxation time
	 * - dynamic object repulsion component (other people or obstacles)
	 * 		- object's pose
	 * 		- current actor's pose
	 * 		- object's speed
	 * - static object repulsion component (borders)
	 * 		- object's pose
	 * 		- current actor's pose
	 *	- object attraction component (other people or objects)
	 *		- field decrease factor
	 *		- object's pose
	 *		- current actor's pose
	 *
	 */

}

// ------------------------------------------------------------------- //

std::tuple<Vector3, Vector3, double>
SocialForceModel::computeForceStaticObstacle(const Pose3 &actor_pose,
		const Vector3 &actor_velocity, const Pose3 &object_pose,
		const double &dt)
{

	/* elliptical formulation - `14 article - equations (3) and (4) */

	// distance vector
	Vector3 d_alpha_i = actor_pose.Pos() - object_pose.Pos(); // proper direction
	d_alpha_i.Z(0.00); // planar
	double d_alpha_i_len = d_alpha_i.Length();

	// length (vβ * ∆t) of the stride (step size)
	Vector3 y_alpha_i = actor_velocity * dt;
	y_alpha_i.Z(0.00); // planar

	// semi-minor axis of the elliptic formulation
	double w_alpha_i = 0.5 * sqrt( std::pow((d_alpha_i_len + (d_alpha_i - y_alpha_i).Length()),2) -
								   std::pow(y_alpha_i.Length(), 2) );

	// division by ~0 prevention - returning zeros vector instead of NaNs
	if ( (std::fabs(w_alpha_i) < 1e-08) || (std::isnan(w_alpha_i)) || (d_alpha_i_len < 1e-08) ) {

		#ifdef DEBUG_FORCE_EACH_OBJECT
		if ( SfmGetPrintData() ) {
			std::cout << "\n-----------------------\n" << SfmDebugGetCurrentActorName();
			std::cout << "\nSTATIC OBSTACLE ============= ERROR ===================" << std::endl;
			std::cout << "\t" << SfmDebugGetCurrentObjectName() << ": ";
			std::cout << "\td_alpha_i: " << d_alpha_i << " \tlen: " << d_alpha_i_len << std::endl;
			std::cout << "\ty_alpha_i: " << y_alpha_i;
			std::cout << "\tw_alpha_i: " << w_alpha_i << std::endl;
			std::cout << "\tFAIL w_alpha_i small: " << (std::fabs(w_alpha_i) < 1e-08) << std::endl;
			std::cout << "\tFAIL w_alpha_i NaN: " << (std::isnan(w_alpha_i)) << std::endl;
			std::cout << "\td_alpha_i Length FAIL: " << (d_alpha_i_len < 1e-08) << std::endl;
			std::cout << "\tf_alpha_i: " << Vector3(0.0, 0.0, 0.0) << std::endl;
		}
		#endif

		return (std::make_tuple(Vector3(), d_alpha_i, d_alpha_i_len));
	}

	// ~force (acceleration) calculation
	Vector3 f_alpha_i;
	f_alpha_i = Aw_ * exp(-w_alpha_i/Bw_) * ((d_alpha_i_len + (d_alpha_i - y_alpha_i).Length()) /
			    2*w_alpha_i) * 0.5 * (d_alpha_i.Normalized() + (d_alpha_i - y_alpha_i).Normalized());

	// setting the `strength` (numerator) too high produces noticeable accelerations around objects
	double factor = 90.0/(std::exp(0.5 * d_alpha_i_len));
	f_alpha_i *= factor;

	// -----------------------------------------------------
	// FIXME: debugging large vector length ----------------
#ifdef SFM_DEBUG_LARGE_VECTOR_LENGTH
	if ( factor_force_interaction_ * f_alpha_i.Length() > 5.0 ) {
		std::cout << "\t-  -  -  - static obstacle force -  -  -  -  -  -  -  " << std::endl;
		std::cout << "\t-  -  -  - " << SfmDebugGetCurrentObjectName() << "-  -  -  -  -  -  -  " << std::endl;
		std::cout << "\td_alpha_i: " << d_alpha_i << "\td_alpha_i_len: " << d_alpha_i_len << "\ty_alpha_i: " << y_alpha_i << "\tw_alpha_i: " << w_alpha_i << std::endl;
		std::cout << "\tf_alpha_i: " << factor_force_interaction_*f_alpha_i.X() << " " << factor_force_interaction_*f_alpha_i.Y() << std::endl;
		std::cout << std::endl;
	}
#endif
	// -----------------------------------------------------

#ifdef DEBUG_FORCE_EACH_OBJECT
	if ( SfmGetPrintData() ) {
	#ifdef DEBUG_FORCE_PRINTING_SF_TOTAL_AND_NEW_POSE
	std::cout << "\n-----------------------\n" << SfmDebugGetCurrentActorName();
	#else
	if ( SfmDebugGetCurrentActorName() == "actor1" ) { // && SfmDebugGetCurrentObjectName() == "table1") {
	#endif
		std::cout << "\nSTATIC OBSTACLE" << std::endl;
		std::cout << "\t" << SfmDebugGetCurrentObjectName() << ": ";
		std::cout << "\td_alpha_i: " << d_alpha_i << " \tlen: " << d_alpha_i_len << std::endl;
		std::cout << "\ty_alpha_i: " << y_alpha_i;
		std::cout << "\tw_alpha_i: " << w_alpha_i << std::endl;
		std::cout << "\tf_alpha_i: " << f_alpha_i * factor_force_interaction_ << std::endl;
//		std::cout << "\tactor_pos: " << actor_pose.Pos() << "\t\tobj_pos: " << object_pose.Pos() << std::endl;
//		std::cout << "\tdiff__pos: " << (actor_pose.Pos() - object_pose.Pos()) << "\t\tlen: " << (actor_pose.Pos() - object_pose.Pos()).Length() << std::endl;
	#ifndef DEBUG_FORCE_PRINTING_SF_TOTAL_AND_NEW_POSE
	}
	#endif
	}
#endif

	return (std::make_tuple(f_alpha_i, d_alpha_i, d_alpha_i_len));

}

// ------------------------------------------------------------------- //

/**
 *
 * @param actor_vel
 * @param actor_yaw: raw value, need to be modified to obtain orientation in the world coordinate system
 * @param object_vel
 * @param object_yaw
 * @param d_alpha_beta
 * @param is_actor
 * @return
 */
double SocialForceModel::computeThetaAlphaBetaAngle(const Vector3 &actor_vel,
		const ignition::math::Angle &actor_yaw, const Vector3 &object_vel,
		const ignition::math::Angle &object_yaw, const Vector3 &d_alpha_beta, const bool &is_actor)
{

	#ifdef DBG_COMPUTE_THETA_ALPHA_BETA_ANGLE_2011
	// FIXME: debugging
	static int counter_theta = 0;
	static const int FREQ_DBG = 10;
	#endif

	/* 2011 - "θ αβ - angle between velocity of pedestrian α
	 * and the displacement of pedestrian β" */
	/* `d_alpha_beta` can be interpreted as `d_actor_object` */

#if defined(DEBUG_GEOMETRY_1) || defined(DEBUG_FUZZIFIER_VEL_ANGLE)
	if ( SfmDebugGetCurrentActorName() == "actor1" ) {
		std::cout << "\ncomputeThetaAlphaBetaAngle(): ";
	}
#endif


	/* Actor's info is based on his world's pose - it is easier to treat the current actor
	 * as an object that always keeps aligned to his movement direction.
	 * On the other hand the other object's orientation is not known and its velocity
	 * will be used to determine it's yaw. */

	/* The problem is that actor's linear velocity couldn't be set so it can't be visible
	 * from the world_ptr (always 0, no matter what was set, same in model_ptr) - so when
	 * calculating interaction between 2 actors the other one (so-called beta)
	 * has a linear velocity of (0,0,0). */

	ignition::math::Angle yaw_diff;

	#ifdef DBG_COMPUTE_THETA_ALPHA_BETA_ANGLE_2011
	// FIXME: debugging
	double yaw_diff_d = 0.0;
	double actor_yaw_d = actor_yaw.Radian();
	double object_yaw_d = object_yaw.Radian();

	static double yaw_diff_prev;
	static Vector3 actor_vel_prev;
	static double actor_yaw_prev;
	static Vector3 object_vel_prev;
	static double object_yaw_prev;

	// FIXME:
	ignition::math::Angle yaw_diff_diff;
	double yaw_diff_diff_d = 0.0;
	#endif

	// TODO: change conditions structure, check
	// TODO: consider separately `is_actor` and `is_dynamic_object`
	// if ( is_actor || object_vel.Length() < 1e-06 )

	// check if the other object is an actor
	if ( is_actor ) {

		#if defined(DEBUG_GEOMETRY_1) || defined(DEBUG_FUZZIFIER_VEL_ANGLE)
		if ( SfmDebugGetCurrentActorName() == "actor1" ) {
			std::cout << "\t++another ACTOR++\t" << SfmDebugGetCurrentObjectName();
		}
		#endif

		/* both actors coordinate systems are oriented the same
		 * so no extra calculations need to be performed (relative angle) */

		/* this is a `lightweight` version - another one is based on dot product
		 * and arccos calculation (used in the dynamic object case) - result would
		 * be equal here (when both actors are moving);
		 * with a use of this version there is an ability to calculate the angle
		 * even when one of the actors is currently only rotating */
		// yaw_diff.Radian((actor_yaw.Radian() - object_yaw.Radian())); // V1
		yaw_diff.Radian(object_yaw.Radian() - actor_yaw.Radian()); // OK
		yaw_diff.Normalize();

		#ifdef DBG_COMPUTE_THETA_ALPHA_BETA_ANGLE_2011
		// FIXME
		if ( counter_theta == FREQ_DBG ) {
			yaw_diff_d = yaw_diff.Radian();

			// - - - - - -
			yaw_diff_diff.Radian(yaw_diff_prev - yaw_diff_d);
			yaw_diff_diff.Normalize();

			yaw_diff_diff_d = yaw_diff_diff.Radian();
			if ( std::fabs(yaw_diff_diff_d) > IGN_PI_2 ) {
				int b = 0;
				b++;
			// - - - - - -
			}
		}
		#endif

	} else {

		/* so first let's check if the considered object is static or dynamic;
		 * if it is static then the only thing that could be done is calculating
		 * the yaw_diff as in `_is_actor` case */
		if ( object_vel.Length() < 1e-06 ) {

			#if defined(DEBUG_GEOMETRY_1) || defined(DEBUG_FUZZIFIER_VEL_ANGLE)
			if ( SfmDebugGetCurrentActorName() == "actor1" ) {
				std::cout << "\t++STATIC object++\t" << SfmDebugGetCurrentObjectName(); // THIS SHOULD NOT HAPPEN AFTER ADDING COMPONENT FOR STATIC OBSTACLES!
			}
			#endif

			// yaw_diff.Radian((_actor_yaw.Radian() - _object_yaw.Radian()));
			// transform actor's yaw to the world's coordinate system
			yaw_diff.Radian((actor_yaw.Radian() - IGN_PI_2 - object_yaw.Radian()));
			yaw_diff.Normalize();

			#ifdef DBG_COMPUTE_THETA_ALPHA_BETA_ANGLE_2011
			// FIXME
			if ( counter_theta == FREQ_DBG ) {
				yaw_diff_d = yaw_diff.Radian();

				// - - - - - -
				yaw_diff_diff.Radian(yaw_diff_prev - yaw_diff_d);
				yaw_diff_diff.Normalize();

				yaw_diff_diff_d = yaw_diff_diff.Radian();
				if ( std::fabs(yaw_diff_diff_d) > IGN_PI_2 ) {
					int b = 0;
					b++;
				// - - - - - -
				}
			}
			#endif

			// TODO: angle calculation version based on d_alpha_beta
			ignition::math::Angle actor_yaw_w(actor_yaw.Radian() - IGN_PI_2); // world coord system
			actor_yaw_w.Normalize();

			ignition::math::Angle d_alpha_beta_angle( std::atan2(d_alpha_beta.Y(), d_alpha_beta.X()) );
			double d_alpha_beta_angle_d = d_alpha_beta_angle.Radian();
			d_alpha_beta_angle.Radian(d_alpha_beta_angle.Radian() + ignition::math::Angle::Pi.Radian());
			double d_alpha_beta_angle_inv_d = d_alpha_beta_angle.Radian();

			yaw_diff = actor_yaw_w - d_alpha_beta_angle;
			yaw_diff.Normalize();
			double yaw_diff_double = yaw_diff.Radian();
			int c = 0;
			c++;

		} else {

			#if defined(DEBUG_GEOMETRY_1) || defined(DEBUG_FUZZIFIER_VEL_ANGLE)
			if ( SfmDebugGetCurrentActorName() == "actor1" ) {
				std::cout << "\t++!!! DYNAMIC object !!!++\t" << SfmDebugGetCurrentObjectName();
			}
			#endif

//			 TODO: debug this, NOT TESTED!
//			 velocities are expressed in world's coordinate system
//
//			 V1
//			 transform object's yaw to actor's coordinate system by adding 90 deg
//			ignition::math::Angle yaw_temp( std::atan2( _object_vel.Y(), _object_vel.X() ) + (IGN_PI/2) );
//			yaw_temp.Normalize();
//
//			yaw_diff.Radian( _actor_yaw.Radian() - yaw_temp.Radian() );

			// V2 - OK
			// both velocities are expressed in world's coordinate system
			// formula -> Section "Examples of spatial tasks" @ https://onlinemschool.com/math/library/vector/angl/
			yaw_diff.Radian( std::acos( actor_vel.Dot(object_vel) / (actor_vel.Length() * object_vel.Length()) ) );
			yaw_diff.Normalize();

			#ifdef DBG_COMPUTE_THETA_ALPHA_BETA_ANGLE_2011
			// FIXME
			if ( counter_theta == FREQ_DBG ) {
				yaw_diff_d = yaw_diff.Radian();

				// - - - - - -
				yaw_diff_diff.Radian(yaw_diff_prev - yaw_diff_d);
				yaw_diff_diff.Normalize();

				yaw_diff_diff_d = yaw_diff_diff.Radian();
				if ( std::fabs(yaw_diff_diff_d) > IGN_PI_2 ) {
					int b = 0;
					b++;
				// - - - - - -
				}
			}
			#endif

		}

	}


#if defined(DEBUG_GEOMETRY_1) || defined(DEBUG_FUZZIFIER_VEL_ANGLE)
	if ( SfmDebugGetCurrentActorName() == "actor1" ) {
		std::cout << "\tyaw_actor: " << actor_yaw.Radian() << "  yaw_object: " << object_yaw.Radian() << "\n\tyaw_diff: " << yaw_diff.Radian() << "\tactor_vel: " << actor_vel << "\tobj_vel: " << object_vel << std::endl << std::endl;
	}
#endif



	#ifdef DBG_COMPUTE_THETA_ALPHA_BETA_ANGLE_2011
	if ( counter_theta++ == FREQ_DBG ) {

		counter_theta = 0;
		std::cout << "\n[computeInteractionForce] theta_a_b: " << yaw_diff.Radian() << std::endl << std::endl;

		// FIXME: debugging
		yaw_diff_prev = yaw_diff.Radian();
		actor_vel_prev = actor_vel;
		actor_yaw_prev = actor_yaw.Radian();
		object_vel_prev = object_vel;
		object_yaw_prev = object_yaw.Radian();

	}
	#endif


	return ( yaw_diff.Radian() );

}

// ------------------------------------------------------------------- //

double SocialForceModel::computeThetaAlphaBetaAngle(const Vector3 &n_alpha,
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
	ignition::math::Angle yaw_norm(getYawFromPose(actor_pose));
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
	if ( _beta_rel_location == LOCATION_RIGHT ) {
		p_alpha = _n_alpha.Perpendicular();
		// return (_n_alpha.Perpendicular());
	} else if ( _beta_rel_location == LOCATION_LEFT ) {

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

	if ( beta_rel_location == LOCATION_LEFT ) {

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


	} else if ( beta_rel_location == LOCATION_RIGHT ) {

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

std::tuple<RelativeLocation, double, double> SocialForceModel::computeObjectRelativeLocation(const ignition::math::Angle &actor_yaw,
		const Vector3 &d_alpha_beta)
{

	RelativeLocation rel_loc = LOCATION_UNSPECIFIED;
	ignition::math::Angle angle_relative; 		// relative to actor's (alpha) direction
	ignition::math::Angle angle_d_alpha_beta;	// stores yaw of d_alpha_beta

	Vector3 d_alpha_beta_norm = d_alpha_beta;
	d_alpha_beta_norm.Normalize();

	// when normalized vector used with atan2 then division by euclidean distance not needed
	//angle_d_alpha_beta.Radian( std::atan2(d_alpha_beta_norm.X(), d_alpha_beta_norm.Y()) );	// V1
	angle_d_alpha_beta.Radian( std::atan2(d_alpha_beta_norm.Y(), d_alpha_beta_norm.X()) ); 	// V2
	angle_d_alpha_beta.Normalize();

	//angle_relative = _actor_yaw + angle_d_alpha_beta;	// V1

	// V2
	//angle_relative = _actor_yaw - angle_d_alpha_beta; 	// actor's coordinate system rotation considered (relative to world coord. sys.)

	/* V2 NOTES:
	 *
	 */
	angle_relative = actor_yaw - angle_d_alpha_beta;

	// V3
	// corrected in terms of transforming to world's coordinate system
	ignition::math::Angle actor_yaw_corrected(actor_yaw.Radian() - IGN_PI_2);
	actor_yaw_corrected.Normalize(); // V4
//	angle_relative.Radian( angle_d_alpha_beta.Radian() - actor_yaw_corrected() ); // () <- ???
	angle_relative.Radian(angle_d_alpha_beta.Radian() - actor_yaw_corrected.Radian());
	angle_relative.Normalize(); // V4

	// angle_relative.Normalize(); // V1

#ifdef DEBUG_GEOMETRY_2
	if ( print_info ) {
		// ?
//		std::cout << "\n GetBetaRelativeLocation() " << "ACTOR yaw: " << _actor_yaw.Radian() << "  ANGLE d_alpha_beta: " << angle_d_alpha_beta.Radian() << "  ANGLE sum: " << angle_relative.Radian();
	}
	std::string txt_dbg;
#endif

	// LOCATION_FRONT ~ hysteresis regulator
	/*
	if ( angle_relative.Radian() >= -IGN_DTOR(9) &&
		 angle_relative.Radian() <= +IGN_DTOR(9) ) {
	*/

	/* SFM FRONT or SFM_BACK to be specific - added exponentially decreasing
	 * interaction for objects that are behind so relative angle calculations
	 * extended with close to Pi value case */
	if ( std::fabs(angle_relative.Radian()) <= IGN_DTOR(9) ||
		 std::fabs(angle_relative.Radian()) >= (IGN_PI - IGN_DTOR(9)) ) {

		rel_loc = LOCATION_FRONT;
#ifdef DEBUG_GEOMETRY_2
		txt_dbg = "FRONT";
#endif
	/* // LOCATION_BEHIND DEPRECATED HERE
	} else if ( IsOutOfFOV(angle_relative.Radian() ) ) { // consider FOV

		rel_loc = LOCATION_BEHIND;
#ifdef DEBUG_GEOMETRY_2
		txt_dbg = "BEHIND";
#endif
	*/
	} else if ( angle_relative.Radian() <= 0.0 ) { // 0.0 ) {

		rel_loc = LOCATION_RIGHT;
#ifdef DEBUG_GEOMETRY_2
		txt_dbg = "RIGHT";
#endif

	} else if ( angle_relative.Radian() > 0.0 ) { // 0.0 ) {

		rel_loc = LOCATION_LEFT;
#ifdef DEBUG_GEOMETRY_2
		txt_dbg = "LEFT";
#endif

	}

#ifdef DEBUG_GEOMETRY_2
	print_info = true; // FIXME
	if ( print_info ) {
		// std::cout << "  " << txt_dbg << std::endl;
		std::cout << "\t" << owner_name_ << " | " << txt_dbg << "\trel: " << angle_relative.Radian() << "\td_ab: " << angle_d_alpha_beta.Radian() << "\talpha_dir: " << actor_yaw_corrected.Radian() << std::endl;
	}
#endif

	/* historical data considered when calculating relative location - it is crucial to not
	 * allow minor yaw rotations to switch from left to right etc. thus a relative angle
	 * must exceed certain threshold before it will be in fact switched;
	 * LOCATION_BEHIND must not be found in map_models_rel_locations! */

#ifdef DEBUG_OSCILLATIONS
	if ( SfmDebugGetCurrentObjectName() == "table1" && SfmDebugGetCurrentActorName() == "actor1" ) {
		std::string location_str;
		if ( rel_loc == LOCATION_FRONT ) {
			location_str = "FRONT";
		} else if ( rel_loc == LOCATION_BEHIND ) {
			location_str = "BEHIND";
		} else if ( rel_loc == LOCATION_RIGHT ) {
			location_str = "RIGHT SIDE";
		} else if ( rel_loc == LOCATION_LEFT ) {
			location_str = "LEFT SIDE";
		} else if ( rel_loc == LOCATION_UNSPECIFIED ) {
			location_str = "UNKNOWN";
		}
		std::cout << "\t" << SfmDebugGetCurrentObjectName() << "'s relative location: " << location_str <<  "\tactor: " << SfmDebugGetCurrentActorName();
		std::cout << "\n\t\td_alpha_beta: x " << d_alpha_beta_norm.X() << "  y " << d_alpha_beta_norm.Y() << "\tangle_actor_yaw_RAW: " << _actor_yaw.Radian() << "\tangle_actor_yaw_CORR: " << actor_yaw_corrected.Radian() << "\tangle_d_ab: " << angle_d_alpha_beta.Radian() << /* "\tangle_diff: " << (angle_d_alpha_beta.Radian()-actor_yaw_corrected.Radian())<< */ "\tangle_rel: " << angle_relative.Radian();
	}
#endif

	/* if the angle_relative is above few degrees value then the historical value may be discarded */
	if ( rel_loc != LOCATION_FRONT ) {
		map_models_rel_locations_[SfmDebugGetCurrentObjectName()] = rel_loc;
	} else {
#ifdef DEBUG_OSCILLATIONS
		std::cout << "\t:::::::::::::::::::::::::HIST REL ANG!:::::::::::::::::::::::::::::::::::::::::::\t";
#endif
		return ( std::make_tuple(map_models_rel_locations_[SfmDebugGetCurrentObjectName()], angle_relative.Radian(), angle_d_alpha_beta.Radian()) );
	}

	return ( std::make_tuple(rel_loc, angle_relative.Radian(), angle_d_alpha_beta.Radian()) );

}

// ------------------------------------------------------------------- //

inline bool SocialForceModel::isOutOfFOV(const double &angle_relative) {

	if (std::fabs(angle_relative) >= cfg_->fov) {
		return (true);
	}
	return (false);

}

// ------------------------------------------------------------------- //

inline double SocialForceModel::getYawFromPose(const Pose3 &pose) {

	// the actor's offset yaw is considered
	// return (_actor_pose.Rot().Yaw() + (IGN_PI / 2));

	// when offset already considered
	return (pose.Rot().Yaw());
}

// ------------------------------------------------------------------- //

double SocialForceModel::computeRelativeSpeed(const Vector3 &actor_vel,
		const Vector3 &object_vel) {

#ifdef DEBUG_REL_SPEED

	Vector3 rel_vel = _object_velocity - _actor_velocity;
	if ( print_info ) {
		std::cout << std::endl;
		std::cout << "GetRelativeSpeed(): ";
		std::cout << "  obj_vel: " << _object_velocity;
		std::cout << "  act_vel: " << _actor_velocity;
		std::cout << "  v_rel:  " << rel_vel;
		std::cout << "  spd_rel: " << rel_vel.Length();
		std::cout << std::endl;
	}

	return rel_vel.Length();

#else

	return ( (object_vel - actor_vel).Length() );

#endif

}

// ------------------------------------------------------------------- //

ignition::math::Angle SocialForceModel::computeYawMovementDirection(const Pose3 &actor_pose,
		const Vector3 &actor_vel,	const Vector3 &sf_vel,
		const Vector3 &target) {

	// actor's coordinate system transformed into world's coordinate system (Social Force's)

	// 1st step
	// social force's angle
	ignition::math::Angle yaw_sf( std::atan2(sf_vel.Y(), sf_vel.X()) );
	yaw_sf.Normalize();

	// actor's angle transformed to world coordinate system
	/* NOTE: not treating yaw_alpha_w as Angle class object (double instead)
	 * was the main mistake why actors' behavior was sometimes quite off (like
	 * going in the direction opposite to the one pointed by social force). */
	ignition::math::Angle yaw_alpha_w(getYawFromPose(actor_pose) - IGN_PI_2);
	yaw_alpha_w.Normalize();

	// difference used to determine angular rotation direction
	ignition::math::Angle yaw_diff = yaw_sf - yaw_alpha_w;
	yaw_diff.Normalize();

	// 2nd step
	/* `yaw_increment` as a function of speed - the faster actor goes
	 * the less maneuverability he has and less rotations will make	*/
	/// @param yaw_increment - the less the value is the more reluctant
	/// to immediate rotations the actor will be
	/// @note correlated with parameter @param force_min
	/// @note 0.009 before - many rotations
	/// @note 0.003 too much inertia?
	/// @note 0.013 near obstacles orientation oscillates very much
	double yaw_increment = cfg_->maneuverability * std::exp( -actor_vel.Length() );

	// sign determines angle increment direction
	short int sign = -1;

	// desired in terms of calculated SF or `to_goal_direction`
	double yaw_desired_change = 0.0;

	/* Check if yaw_diff is in the `breakdown` range. Breakdown range is related to values
	 * close to -PI or PI. In such a situation it's safer (USUALLY!) to take direction to goal
	 * into consideration. This will avoid rotations in place which usually are performed
	 * in the wrong direction (CW/CCW).
	 * Very primitive target-reach algorithm is presented below (simple going in the
	 * target direction) */
	if ( std::fabs(yaw_diff.Radian()) >= (0.85 * IGN_PI) ) {

		// variables valid only for `OPPOSITE_FORCE_GO_TOWARDS_GOAL`
		Vector3 to_goal_dir;
		ignition::math::Angle to_goal_ang;
		ignition::math::Angle yaw_alpha_goal_diff;

		//switch (opposite_force_method_) {
		switch (cfg_->opposite_force_method) {

		case(OPPOSITE_FORCE_GO_TOWARDS_GOAL):
				// `breakdown` range
				/* Try to find whether to rotate CW or CCW for slightly better alignment
				 * with the target (goal) direction */
				to_goal_dir = Vector3(target.X(), target.Y(), 0.0);
				to_goal_dir.X(to_goal_dir.X() - actor_pose.Pos().X());
				to_goal_dir.Y(to_goal_dir.Y() - actor_pose.Pos().Y());
				to_goal_dir.Z(0.0);
				to_goal_dir.Normalize();
				//std::cout << "[breakdown] to_goal_dir: " << to_goal_dir << std::endl;

				to_goal_ang.Radian(std::atan2(to_goal_dir.Y(), to_goal_dir.X()));
				to_goal_ang.Normalize();
				//std::cout << "[breakdown] yaw_alpha_w: " << yaw_alpha_w.Radian() << std::endl;
				//std::cout << "[breakdown] to_goal_ang: " << to_goal_ang.Radian() << std::endl;

				yaw_alpha_goal_diff = to_goal_ang - yaw_alpha_w;
				yaw_alpha_goal_diff.Normalize();
				//std::cout << "[breakdown] yaw_alpha_goal_diff: " << yaw_alpha_goal_diff.Radian() << std::endl;

				// based on calculated angle value determine the sign
				( yaw_alpha_goal_diff.Radian() >= 0.0 ) ? (sign = +1) : (0);
				yaw_desired_change = yaw_alpha_goal_diff.Radian();
				break;

		case(OPPOSITE_FORCE_ROTATE_LEFT):
				sign = +1;
				break;

		}


	} else { /* std::fabs(yaw_diff.Radian()) < (0.85 * IGN_PI) */

		// typical operation
		( yaw_diff.Radian() >= 0.0 ) ? (sign = +1) : (0);
		yaw_desired_change = yaw_diff.Radian();

	}

	// stores the angle increment based on current SF
	double angle_change = 0.0;
	if ( std::fabs(yaw_desired_change) < yaw_increment ) {
		// only small rotation adjustment needed
		angle_change = static_cast<double>(sign) * yaw_desired_change;
	} else {
		// truncate big angle change desired by SF
		angle_change = static_cast<double>(sign) * yaw_increment;
	}

	// 3rd step
	// calculate new yaw
	ignition::math::Angle yaw_new( yaw_alpha_w.Radian() + angle_change );
	yaw_new.Normalize();
	return (yaw_new);

}

// ------------------------------------------------------------------- //

//bool SocialForceModel::isModelNegligible(const std::string &model_name) const {
//
//	if ( actor::core::Target::isModelNegligible(model_name, params_ptr_->getSfmDictionary().ignored_models_) ||
//		 model_name == owner_name_ ) {
//		// do not save data for objects that should be ignored and for itself
//		return (true);
//	}
//	return (false);
//
//}

// ------------------------------------------------------------------- //

bool SocialForceModel::isModelNegligible(const std::string &model_name,
		const std::vector<std::string> ignored_models_v) const {

	if ( ignored_models_v.size() == 0 ) {
		return (false);
	}

	std::vector<std::string>::const_iterator it;
	it = std::find(ignored_models_v.begin(), ignored_models_v.end(), model_name);
	if ( it != ignored_models_v.end() ) {
		// found
		return (true);
	}
	// not found on the ignored models list -> non-ignorable = non-negligible
	return (false);

}

// ------------------------------------------------------------------- //

bool SocialForceModel::isDynamicObstacle(const Vector3 &vel) const {

	// NOTE: static robot can reach up to 0.008 m/s - is it the ODE issue?
	if (vel.Length() >= 0.01) {
		return (true);
	}
	return (false);

}

// ------------------------------------------------------------------- //

double SocialForceModel::convertActorToWorldOrientation(const double &yaw_actor) const {

	ignition::math::Angle yaw_world(yaw_actor - IGN_PI_2);
	yaw_world.Normalize();

	return (yaw_world.Radian());

}

// ------------------------------------------------------------------- //

// in world coordinate system
double SocialForceModel::computeVectorDirection(const Vector3 &v) const {

	ignition::math::Angle angle_v;
	Vector3 v_norm = v;
	v_norm.Normalize();

	// when normalized vector used with atan2 then division by euclidean distance not needed
	angle_v.Radian(std::atan2(v_norm.Y(), v_norm.X()));
	angle_v.Normalize();

	return (angle_v.Radian());

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

bool SocialForceModel::updateClosestObstacleDistance(double &dist_compare, const double &dist) const {

	// note: non-linear modifications
	if ( dist < dist_compare ) {
		dist_compare = dist;
		return (true);
	}
	return (false);

}

// ------------------------------------------------------------------- //

bool SocialForceModel::isTypicalModel(const unsigned int &count_curr, const unsigned int &models_total) {

	if ( count_curr >= models_total ) {
		return (false);
	}
	return (true);

}

// ------------------------------------------------------------------- //

//std::tuple<bool, bool, std::string, Pose3, Vector3, inflation::Border* /* std::shared_ptr<inflation::Border> */ >
//SocialForceModel::preprocessTypicalModel(const gazebo::physics::WorldPtr &world_ptr,
//		const size_t &model_num,
//		const std::vector<std::string> &ignored_models_v,
//		const actor::core::CommonInfo &actor_info,
//		inflation::Box &model_box)
//{
//
//	// save a pointer to the new model
//	gazebo::physics::ModelPtr model_ptr = world_ptr->ModelByIndex(model_num);
//
//	// check whether social force calculation is necessary (list given by system parameters)
//	if ( isModelNegligible(model_ptr->GetName()) ) {
//		// in fact only the first `false` is meaningful here
//		return (std::make_tuple(false, false, std::string(), Pose3(), Vector3(), nullptr));
//	}
//
//	// check model name's presence in the dynamic list of negligible objects
//	// (usually the list is not empty only in `following` state)
//	if ( isModelNegligible(model_ptr->GetName(), ignored_models_v) ) {
//		// in fact only the first `false` is meaningful here
//		return (std::make_tuple(false, false, std::string(), Pose3(), Vector3(), nullptr));
//	}
//
//	//////////////////////////////////////////////////////////////////////////
//	/// catch model name - debugging /////////////////////////////////////////
//	SfmDebugSetCurrentObjectName(model_ptr->GetName()); //////////////////////
//	SfmDebugSetCurrentActorName(owner_name_); ////////////////////////////////
//	//////////////////////////////////////////////////////////////////////////
//
//	// some of the outputs
//	bool is_an_actor = false;
//	Vector3 model_vel;
//
////	std::shared_ptr<inflation::Border> model_border_ptr;
//	inflation::Border* model_border_ptr;
//
//	// decode information if the current model is of `Actor` type
//	if ( (is_an_actor = actor_decoder_.isActor(model_ptr->GetType())) ) {
//
//		// decoder of the CommonInfo class
//		actor_decoder_.setID(model_ptr->GetName(), actor_info.getNameIDMap());
//
//		// load data from CommonInfo based on actor's id
//		model_vel = actor_decoder_.getData(actor_info.getLinearVelocitiesVector());
//		model_border_ptr = actor_decoder_.getData(actor_info.getBorderPtrsVector()).get(); // returns a naked pointer
//
//
//	} else {
//
//		model_vel = model_ptr->WorldLinearVel();
//
//		// `dynamic_cast` does not work in this case, `model_border_box` acts as a helper instance here
//		model_box.setBox(model_ptr->BoundingBox());
//		model_border_ptr = &model_box;
//
//	}
//
//	// rest of the outputs
//	std::string model_name = model_ptr->GetName();
//	Pose3 model_raw_pose = model_ptr->WorldPose();
//
//	return (std::make_tuple(true, is_an_actor, model_name, model_raw_pose, model_vel, model_border_ptr));
//
//}
//
//
//
//// ------------------------------------------------------------------- //
//
//std::tuple<bool, bool, std::string, Pose3, Vector3, inflation::Border* /* std::shared_ptr<inflation::Border> */ >
//SocialForceModel::preprocessWorldBoundary(const size_t &wall_num, inflation::Box &model_box) {
//
//	// evaluate validity of the boundary box
//	if ( !boundary_.isValid(wall_num) ) {
//		return (std::make_tuple(false, false, std::string(), Pose3(), Vector3(), nullptr));
//	}
//
//	// model_name stands for the one of the method outputs
//	std::string model_name = "world_boundary_";
//	model_name += std::to_string(wall_num);
//
//	//////////////////////////////////////////////////////////////////////////
//	/// catch model ARTIFICIAL name - debugging //////////////////////////////
//	SfmDebugSetCurrentObjectName(model_name);   //////////////////////////////
//	SfmDebugSetCurrentActorName(owner_name_);   //////////////////////////////
//	//////////////////////////////////////////////////////////////////////////
//
//	// rest of the outputs
//	bool is_an_actor = false;
//	Vector3 model_vel;
//	Pose3 model_raw_pose;
//
////	std::shared_ptr<inflation::Border> model_border_ptr;
//	inflation::Border* model_border_ptr = nullptr;
//
//	// processing section
//	model_vel = Vector3(0.0, 0.0, 0.0);
//
//	model_box.setBox(*boundary_.getBoundingBoxPtr(wall_num));
////	model_border_ptr = std::make_shared<inflation::Border>(&model_box);
////	model_border_ptr = std::make_shared<inflation::Border>(dynamic_cast<inflation::Border>(&model_box));
//	model_border_ptr = &model_box;
//
//	// a trivial rotation component
//	model_raw_pose.Rot() = Quaternion(0.0, 0.0, 0.0);
//	model_raw_pose.Pos() = model_border_ptr->getCenter();
////	model_raw_pose.Pos().Z() = 0.0;
//
//	return (std::make_tuple(true, is_an_actor, model_name, model_raw_pose, model_vel, model_border_ptr));
//
//}

// ------------------------------------------------------------------- //

SocialForceModel::~SocialForceModel() { }

// ------------------------------------------------------------------- //

} /* namespace sfm */