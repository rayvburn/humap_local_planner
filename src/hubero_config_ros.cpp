/*
 * hubero_config_ros.cpp
 *
 *  Created on: Feb 23, 2021
 *      Author: rayvburn
 */

#include <hubero_local_planner/hubero_config_ros.h>

namespace hubero_local_planner {

void HuberoConfigROS::loadFromParamServer(const ros::NodeHandle& nh) {
	nh.param("odom_topic", odom_topic, odom_topic);
	nh.param("map_frame", map_frame, map_frame);

	// GeneralParams
	// subsection to delete?
	nh.param("init_pose", general.init_pose, general.init_pose);
	nh.param("init_target", general.init_target, general.init_target);
	nh.param("init_stance", general.init_stance, general.init_stance);

	nh.param("global_frame_name", general.global_frame_name, general.global_frame_name);
	nh.param("animation_factor", general.animation_factor, general.animation_factor);
	nh.param("animation_speed_rotation", general.animation_speed_rotation, general.animation_speed_rotation);
	nh.param("target_tolerance", general.target_tolerance, general.target_tolerance);
	nh.param("target_reach_max_time", general.target_reach_max_time, general.target_reach_max_time);
	nh.param("target_reachable_check_period", general.target_reachable_check_period, general.target_reachable_check_period);
	nh.param("limit_actors_workspace", general.limit_actors_workspace, general.limit_actors_workspace);
	nh.param("world_bound_x", general.world_bound_x, general.world_bound_x);
	nh.param("world_bound_y", general.world_bound_y, general.world_bound_y);
	//

	// InflatorParams
	nh.param("bounding_type", inflator.bounding_type, inflator.bounding_type);
	nh.param("circle_radius", inflator.circle_radius, inflator.circle_radius);
	nh.param("box_size", inflator.box_size, inflator.box_size);
	nh.param("ellipse", inflator.ellipse, inflator.ellipse);
	nh.param("inflation_radius", inflator.inflation_radius, inflator.inflation_radius);

	// SfmParams
	nh.param("fov", sfm.fov, sfm.fov);
	nh.param("max_speed", sfm.max_speed, sfm.max_speed);
	nh.param("mass", sfm.mass, sfm.mass);
	nh.param("maneuverability", sfm.maneuverability, sfm.maneuverability);
	nh.param("internal_force_factor", sfm.internal_force_factor, sfm.internal_force_factor);
	nh.param("interaction_force_factor", sfm.interaction_force_factor, sfm.interaction_force_factor);
	nh.param("min_force", sfm.min_force, sfm.min_force);
	nh.param("max_force", sfm.max_force, sfm.max_force);
	nh.param("heterogenous_population", sfm.heterogenous_population, sfm.heterogenous_population);
	nh.param("static_obj_interaction", sfm.static_obj_interaction, sfm.static_obj_interaction);
	nh.param("box_inflation_type", sfm.box_inflation_type, sfm.box_inflation_type);
	nh.param("opposite_force", sfm.opposite_force, sfm.opposite_force);
	nh.param("disable_interaction_forces", sfm.disable_interaction_forces, sfm.disable_interaction_forces);

	// BehaviourParams
	nh.param("force_factor", behaviour.force_factor, behaviour.force_factor);
	nh.param("turn_left", behaviour.turn_left, behaviour.turn_left);
	nh.param("turn_left_accelerate_turn", behaviour.turn_left_accelerate_turn, behaviour.turn_left_accelerate_turn);
	nh.param("turn_left_accelerate_acc", behaviour.turn_left_accelerate_acc, behaviour.turn_left_accelerate_acc);
	nh.param("accelerate", behaviour.accelerate, behaviour.accelerate);
	nh.param("turn_right_accelerate_turn", behaviour.turn_right_accelerate_turn, behaviour.turn_right_accelerate_turn);
	nh.param("turn_right_accelerate_acc", behaviour.turn_right_accelerate_acc, behaviour.turn_right_accelerate_acc);
	nh.param("turn_right", behaviour.turn_right, behaviour.turn_right);
	nh.param("turn_right_decelerate_turn", behaviour.turn_right_decelerate_turn, behaviour.turn_right_decelerate_turn);
	nh.param("turn_right_decelerate_dec", behaviour.turn_right_decelerate_dec, behaviour.turn_right_decelerate_dec);
	nh.param("stop", behaviour.stop, behaviour.stop);
	nh.param("decelerate", behaviour.decelerate, behaviour.decelerate);
}

void HuberoConfigROS::reconfigure(HuberoPlannerConfig& cfg) {
	printf("HuberoConfigROS::reconfigure() called but no actions will be performed!\r\n");
	//std::lock_guard lock(config_mutex_);

	/*
	general.init_pose = cfg
	general.init_target;
	general.init_stance;

	general.global_frame_name				= "world";
	general.animation_factor 				= 4.5;				/// \brief Time scaling factor. Used to coordinate translational motion with the actor_ptr_'s walking animation.
	general.animation_speed_rotation 		= 0.007;
	general.target_tolerance 				= 1.25;
	general.target_reach_max_time 			= 60.0;
	general.target_reachable_check_period 	= 2.0;
	general.limit_actors_workspace			= true;
	general.world_bound_x					{-3.20, +3.80};
	general.world_bound_y					{-10.20, +3.80};

	inflator.bounding_type 					= 2;
	inflator.circle_radius					= 0.5;
	inflator.box_size						{0.45, 0.45, 1.00};
	inflator.ellipse							{1.00, 0.80, 0.35, 0.00};
	inflator.inflation_radius				= 0.45; // the `worst` case from the default values

	sfm.fov 						= 2.00;
	sfm.max_speed 					= 1.50;
	sfm.mass 						= 80.0;
	sfm.maneuverability				= 6.5;
	sfm.internal_force_factor 		= 100.0;
	sfm.interaction_force_factor 	= 3000.0;
	sfm.min_force 					= 300.0;
	sfm.max_force 					= 2000.0;
	sfm.heterogenous_population 	= false;
	sfm.static_obj_interaction 		= 1;
	sfm.box_inflation_type 			= 0;
	sfm.opposite_force				= 0;
	sfm.disable_interaction_forces	= false;

	behaviour.force_factor					= 1.0;
	behaviour.turn_left 						= 500.0;
	behaviour.turn_left_accelerate_turn 		= 500.0;
	behaviour.turn_left_accelerate_acc 		= 625.0;
	behaviour.accelerate 						= 500.0;
	behaviour.turn_right_accelerate_turn 		= 500.0;
	behaviour.turn_right_accelerate_acc 		= 625.0;
	behaviour.turn_right 						= 800.0;
	behaviour.turn_right_decelerate_turn 		= 500.0;
	behaviour.turn_right_decelerate_dec 		= 625.0;
	behaviour.stop 							= 500.0;
	behaviour.decelerate 						= 500.0;
	*/
}

} /* namespace hubero_local_planner */
