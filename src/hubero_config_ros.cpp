/*
 * hubero_config_ros.cpp
 *
 *  Created on: Feb 23, 2021
 *      Author: rayvburn
 */

#include <hubero_local_planner/hubero_config_ros.h>

namespace hubero_local_planner {

void HuberoConfigROS::loadFromParamServer(const ros::NodeHandle& nh) {
	ROS_INFO("[HuberoConfigROS] loadFromParamServer()");

	nh.param("odom_topic", odom_topic, odom_topic);
	nh.param("map_frame", map_frame, map_frame);

//	// GeneralParams
//	// subsection to delete?
//	nh.param("init_pose", general_->init_pose, general_->init_pose);
//	nh.param("init_target", general_->init_target, general_->init_target);
//	//
//	int general_init_stance = general_->init_stance;
//	nh.param("init_stance", general_init_stance, general_init_stance);
//	general_->init_stance = static_cast<unsigned short int>(general_init_stance);
//
//	nh.param("global_frame_name", general_->global_frame_name, general_->global_frame_name);
//	nh.param("animation_factor", general_->animation_factor, general_->animation_factor);
//	nh.param("animation_speed_rotation", general_->animation_speed_rotation, general_->animation_speed_rotation);
//	nh.param("target_tolerance", general_->target_tolerance, general_->target_tolerance);
//	nh.param("target_reach_max_time", general_->target_reach_max_time, general_->target_reach_max_time);
//	nh.param("target_reachable_check_period", general_->target_reachable_check_period, general_->target_reachable_check_period);
//	nh.param("limit_actors_workspace", general_->limit_actors_workspace, general_->limit_actors_workspace);
//	nh.param("world_bound_x", general_->world_bound_x, general_->world_bound_x);
//	nh.param("world_bound_y", general_->world_bound_y, general_->world_bound_y);

	// GeneralParams
	nh.param("sim_period", general_->sim_period, general_->sim_period);
	nh.param("sim_time", general_->sim_time, general_->sim_time);
	nh.param("sim_granularity", general_->sim_granularity, general_->sim_granularity);
	nh.param("angular_sim_granularity", general_->angular_sim_granularity, general_->angular_sim_granularity);

	// Limits
	printf("Parameter section: `LIMITS` \r\n");
	printf("\t - max_trans_vel: %5.5f \r\n", limits_->max_trans_vel);
	printf("\t - min_trans_vel: %5.5f \r\n", limits_->min_trans_vel);
	printf("\t - max_vel_x: %5.5f \r\n", limits_->max_vel_x);
	printf("\t - min_vel_x: %5.5f \r\n", limits_->min_vel_x);
	printf("\t - max_vel_y: %5.5f \r\n", limits_->max_vel_y);
	printf("\t - min_vel_y: %5.5f \r\n", limits_->min_vel_y);
	printf("\t - max_rot_vel: %5.5f \r\n", limits_->max_rot_vel);
	printf("\t - min_rot_vel: %5.5f \r\n", limits_->min_rot_vel);
	printf("\t - acc_lim_x: %5.5f \r\n", limits_->acc_lim_x);
	printf("\t - acc_lim_y: %5.5f \r\n", limits_->acc_lim_y);
	printf("\t - acc_lim_theta: %5.5f \r\n", limits_->acc_lim_theta);
	printf("\t - acc_limit_trans: %5.5f \r\n", limits_->acc_limit_trans);
	printf("\t - prune_plan: %d \r\n", limits_->prune_plan);
	printf("\t - xy_goal_tolerance: %5.5f \r\n", limits_->xy_goal_tolerance);
	printf("\t - yaw_goal_tolerance: %5.5f \r\n", limits_->yaw_goal_tolerance);
	printf("\t - trans_stopped_vel: %5.5f \r\n", limits_->trans_stopped_vel);
	printf("\t - rot_stopped_vel: %5.5f \r\n", limits_->rot_stopped_vel);
//	nh.param("max_trans_vel", limits_->max_trans_vel, limits_->max_trans_vel);
//	nh.param("min_trans_vel", limits_->min_trans_vel, limits_->min_trans_vel);
	nh.param("max_vel_x", limits_->max_vel_x, limits_->max_vel_x);
	nh.param("min_vel_x", limits_->min_vel_x, limits_->min_vel_x);
//	nh.param("max_vel_y", limits_->max_vel_y, limits_->max_vel_y);
//	nh.param("min_vel_y", limits_->min_vel_y, limits_->min_vel_y);
	nh.param("max_rot_vel", limits_->max_rot_vel, limits_->max_rot_vel);
	nh.param("min_rot_vel", limits_->min_rot_vel, limits_->min_rot_vel);
//	nh.param("acc_lim_x", limits_->acc_lim_x, limits_->acc_lim_x);
//	nh.param("acc_lim_y", limits_->acc_lim_y, limits_->acc_lim_y);
//	nh.param("acc_lim_theta", limits_->acc_lim_theta, limits_->acc_lim_theta);
//	nh.param("acc_limit_trans", limits_->acc_limit_trans, limits_->acc_limit_trans);
	nh.param("prune_plan", limits_->prune_plan, limits_->prune_plan);
	nh.param("xy_goal_tolerance", limits_->xy_goal_tolerance, limits_->xy_goal_tolerance);
	nh.param("yaw_goal_tolerance", limits_->yaw_goal_tolerance, limits_->yaw_goal_tolerance);
	nh.param("trans_stopped_vel", limits_->trans_stopped_vel, limits_->trans_stopped_vel);
	nh.param("rot_stopped_vel", limits_->rot_stopped_vel, limits_->rot_stopped_vel);

	printf("Parameter section: `LIMITS` \r\n");
	printf("\t - max_trans_vel: %5.5f \r\n", limits_->max_trans_vel);
	printf("\t - min_trans_vel: %5.5f \r\n", limits_->min_trans_vel);
	printf("\t - max_vel_x: %5.5f \r\n", limits_->max_vel_x);
	printf("\t - min_vel_x: %5.5f \r\n", limits_->min_vel_x);
	printf("\t - max_vel_y: %5.5f \r\n", limits_->max_vel_y);
	printf("\t - min_vel_y: %5.5f \r\n", limits_->min_vel_y);
	printf("\t - max_rot_vel: %5.5f \r\n", limits_->max_rot_vel);
	printf("\t - min_rot_vel: %5.5f \r\n", limits_->min_rot_vel);
	printf("\t - acc_lim_x: %5.5f \r\n", limits_->acc_lim_x);
	printf("\t - acc_lim_y: %5.5f \r\n", limits_->acc_lim_y);
	printf("\t - acc_lim_theta: %5.5f \r\n", limits_->acc_lim_theta);
	printf("\t - acc_limit_trans: %5.5f \r\n", limits_->acc_limit_trans);
	printf("\t - prune_plan: %d \r\n", limits_->prune_plan);
	printf("\t - xy_goal_tolerance: %5.5f \r\n", limits_->xy_goal_tolerance);
	printf("\t - yaw_goal_tolerance: %5.5f \r\n", limits_->yaw_goal_tolerance);
	printf("\t - trans_stopped_vel: %5.5f \r\n", limits_->trans_stopped_vel);
	printf("\t - rot_stopped_vel: %5.5f \r\n", limits_->rot_stopped_vel);

	// InflatorParams
	// unsigned short int - needed reference so static_cast will not do
	int inflator_bounding_type = inflator_->bounding_type;
	nh.param("bounding_type", inflator_bounding_type, inflator_bounding_type);
	inflator_->bounding_type = static_cast<unsigned short int>(inflator_bounding_type);
	//
	nh.param("circle_radius", inflator_->circle_radius, inflator_->circle_radius);
	nh.param("box_size", inflator_->box_size, inflator_->box_size);
	nh.param("ellipse", inflator_->ellipse, inflator_->ellipse);
	nh.param("inflation_radius", inflator_->inflation_radius, inflator_->inflation_radius);

	// SfmParams
	nh.param("fov", sfm_->fov, sfm_->fov);
	nh.param("max_speed", sfm_->max_speed, sfm_->max_speed);
	nh.param("mass", sfm_->mass, sfm_->mass);
	nh.param("maneuverability", sfm_->maneuverability, sfm_->maneuverability);
	nh.param("internal_force_factor", sfm_->internal_force_factor, sfm_->internal_force_factor);
	nh.param("interaction_force_factor", sfm_->interaction_force_factor, sfm_->interaction_force_factor);
	nh.param("min_force", sfm_->min_force, sfm_->min_force);
	nh.param("max_force", sfm_->max_force, sfm_->max_force);
	nh.param("heterogenous_population", sfm_->heterogenous_population, sfm_->heterogenous_population);
	//
	int sfm_static_obj_interaction = sfm_->static_obj_interaction;
	nh.param("static_obj_interaction", sfm_static_obj_interaction, sfm_static_obj_interaction);
	sfm_->static_obj_interaction = static_cast<unsigned short int>(sfm_static_obj_interaction);
	//
	int sfm_box_inflation_type = sfm_->box_inflation_type;
	nh.param("box_inflation_type", sfm_box_inflation_type, sfm_box_inflation_type);
	sfm_->box_inflation_type = static_cast<unsigned short int>(sfm_box_inflation_type);
	//
	int sfm_opposite_force_method = sfm_->opposite_force_method;
	nh.param("opposite_force", sfm_opposite_force_method, sfm_opposite_force_method);
	sfm_->opposite_force_method = static_cast<unsigned short int>(sfm_opposite_force_method);
	//
	nh.param("disable_interaction_forces", sfm_->disable_interaction_forces, sfm_->disable_interaction_forces);

	// BehaviourParams
	nh.param("force_factor", behaviour_->force_factor, behaviour_->force_factor);
	nh.param("turn_left", behaviour_->turn_left, behaviour_->turn_left);
	nh.param("turn_left_accelerate_turn", behaviour_->turn_left_accelerate_turn, behaviour_->turn_left_accelerate_turn);
	nh.param("turn_left_accelerate_acc", behaviour_->turn_left_accelerate_acc, behaviour_->turn_left_accelerate_acc);
	nh.param("accelerate", behaviour_->accelerate, behaviour_->accelerate);
	nh.param("turn_right_accelerate_turn", behaviour_->turn_right_accelerate_turn, behaviour_->turn_right_accelerate_turn);
	nh.param("turn_right_accelerate_acc", behaviour_->turn_right_accelerate_acc, behaviour_->turn_right_accelerate_acc);
	nh.param("turn_right", behaviour_->turn_right, behaviour_->turn_right);
	nh.param("turn_right_decelerate_turn", behaviour_->turn_right_decelerate_turn, behaviour_->turn_right_decelerate_turn);
	nh.param("turn_right_decelerate_dec", behaviour_->turn_right_decelerate_dec, behaviour_->turn_right_decelerate_dec);
	nh.param("stop", behaviour_->stop, behaviour_->stop);
	nh.param("decelerate", behaviour_->decelerate, behaviour_->decelerate);
}

void HuberoConfigROS::reconfigure(HuberoPlannerConfig& cfg) {
	std::lock_guard<std::mutex> lock(config_mutex_);
	ROS_INFO("[HuberoConfigROS] reconfigure()");

	// sfm_->disable_interaction_forces = cfg.groups.sfm_->disable_interaction_forces;

	/*
	general_->init_pose = cfg
	general_->init_target;
	general_->init_stance;

	general_->global_frame_name				= "world";
	general_->animation_factor 				= 4.5;				/// \brief Time scaling factor. Used to coordinate translational motion with the actor_ptr_'s walking animation.
	general_->animation_speed_rotation 		= 0.007;
	general_->target_tolerance 				= 1.25;
	general_->target_reach_max_time 			= 60.0;
	general_->target_reachable_check_period 	= 2.0;
	general_->limit_actors_workspace			= true;
	general_->world_bound_x					{-3.20, +3.80};
	general_->world_bound_y					{-10.20, +3.80};

	inflator_->bounding_type 					= 2;
	inflator_->circle_radius					= 0.5;
	inflator_->box_size						{0.45, 0.45, 1.00};
	inflator_->ellipse							{1.00, 0.80, 0.35, 0.00};
	inflator_->inflation_radius				= 0.45; // the `worst` case from the default values

	sfm_->fov 						= 2.00;
	sfm_->max_speed 					= 1.50;
	sfm_->mass 						= 80.0;
	sfm_->maneuverability				= 6.5;
	sfm_->internal_force_factor 		= 100.0;
	sfm_->interaction_force_factor 	= 3000.0;
	sfm_->min_force 					= 300.0;
	sfm_->max_force 					= 2000.0;
	sfm_->heterogenous_population 	= false;
	sfm_->static_obj_interaction 		= 1;
	sfm_->box_inflation_type 			= 0;
	sfm_->opposite_force				= 0;
	sfm_->disable_interaction_forces	= false;

	behaviour_->force_factor					= 1.0;
	behaviour_->turn_left 						= 500.0;
	behaviour_->turn_left_accelerate_turn 		= 500.0;
	behaviour_->turn_left_accelerate_acc 		= 625.0;
	behaviour_->accelerate 						= 500.0;
	behaviour_->turn_right_accelerate_turn 		= 500.0;
	behaviour_->turn_right_accelerate_acc 		= 625.0;
	behaviour_->turn_right 						= 800.0;
	behaviour_->turn_right_decelerate_turn 		= 500.0;
	behaviour_->turn_right_decelerate_dec 		= 625.0;
	behaviour_->stop 							= 500.0;
	behaviour_->decelerate 						= 500.0;
	*/

	sfm_->static_obj_interaction = cfg.static_object_interaction_type;
	sfm_->opposite_force_method = cfg.oppsite_force_method;
	sfm_->disable_interaction_forces = cfg.disable_interaction_forces;
}

} /* namespace hubero_local_planner */
