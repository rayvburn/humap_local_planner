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

	// GeneralParams
	nh.param("sim_time", general_->sim_time, general_->sim_time);
	nh.param("sim_granularity", general_->sim_granularity, general_->sim_granularity);
	nh.param("angular_sim_granularity", general_->angular_sim_granularity, general_->angular_sim_granularity);

	// `sim_period` is handled differently - derived from `controller_frequency`
	std::string controller_frequency_param;
	nh.searchParam("controller_frequency", controller_frequency_param);
	double controller_frequency = 10.0;
	if (nh.param(controller_frequency_param, controller_frequency, controller_frequency)) {
		general_->sim_period = 1.0 / controller_frequency;
		ROS_INFO(
			"Sim period set to %6.3f s. Computed based on `controller_frequency` which is %6.3f Hz",
			general_->sim_period,
			controller_frequency
		);
	}

	// Limits
	// default values based on PAL's TIAGo config
	nh.param("max_vel_trans", limits_->max_vel_trans, 1.5);
	nh.param("min_vel_trans", limits_->min_vel_trans, 0.1);
	nh.param("max_vel_x", limits_->max_vel_x, 1.5);
	nh.param("min_vel_x", limits_->min_vel_x, -0.1);
	nh.param("max_vel_y", limits_->max_vel_y, 0.0);
	nh.param("min_vel_y", limits_->min_vel_y, 0.0);
	nh.param("max_vel_theta", limits_->max_vel_theta, 2.0);
	nh.param("min_vel_theta", limits_->min_vel_theta, 0.4);
	nh.param("acc_lim_x", limits_->acc_lim_x, 2.5);
	nh.param("acc_lim_y", limits_->acc_lim_y, 0.0);
	nh.param("acc_lim_theta", limits_->acc_lim_theta, 3.2);
	nh.param("acc_lim_trans", limits_->acc_lim_trans, 2.5);
	nh.param("prune_plan", limits_->prune_plan, true);
	nh.param("xy_goal_tolerance", limits_->xy_goal_tolerance, 0.1);
	nh.param("yaw_goal_tolerance", limits_->yaw_goal_tolerance, 0.2);
	nh.param("trans_stopped_vel", limits_->trans_stopped_vel, 0.01);
	nh.param("theta_stopped_vel", limits_->theta_stopped_vel, 0.01);

	// SfmParams
	nh.param("fov", sfm_->fov, sfm_->fov);
	nh.param("mass", sfm_->mass, sfm_->mass);
	nh.param("internal_force_factor", sfm_->internal_force_factor, sfm_->internal_force_factor);
	nh.param("static_interaction_force_factor", sfm_->static_interaction_force_factor, sfm_->static_interaction_force_factor);
	nh.param("dynamic_interaction_force_factor", sfm_->dynamic_interaction_force_factor, sfm_->dynamic_interaction_force_factor);
	nh.param("min_force", sfm_->min_force, sfm_->min_force);
	nh.param("max_force", sfm_->max_force, sfm_->max_force);
	nh.param("heterogenous_population", sfm_->heterogenous_population, sfm_->heterogenous_population);
	nh.param("use_tuned_params", sfm_->use_tuned_params, sfm_->use_tuned_params);
	int sfm_static_obj_interaction = sfm_->static_obj_interaction;
	nh.param("static_obj_interaction", sfm_static_obj_interaction, sfm_static_obj_interaction);
	sfm_->static_obj_interaction = static_cast<unsigned short int>(sfm_static_obj_interaction);
	nh.param("disable_interaction_forces", sfm_->disable_interaction_forces, sfm_->disable_interaction_forces);

	// Fuzzy Inference System parameters
	nh.param("force_factor", fis_->force_factor, fis_->force_factor);
	nh.param("human_action_range", fis_->human_action_range, fis_->human_action_range);
}

void HuberoConfigROS::reconfigure(HuberoPlannerConfig& cfg) {
	std::lock_guard<std::mutex> lock(config_mutex_);
	ROS_INFO("[HuberoConfigROS] reconfigure()");

	general_->angular_sim_granularity = cfg.angular_sim_granularity;
	general_->sim_granularity = cfg.sim_granularity;
	general_->sim_time = cfg.sim_time;
	general_->local_goal_distance = cfg.local_goal_distance;
	general_->person_model_radius = cfg.person_model_radius;
	general_->obstacle_extension_multiplier = cfg.obstacle_extension_multiplier;
	general_->planning_approach = cfg.planning_approach;
	general_->publish_traj_pcl = cfg.publish_traj_pcl;
	general_->publish_cost_grid_pcl = cfg.publish_cost_grid_pcl;

	sfm_->fov = cfg.fov;
	sfm_->mass = cfg.mass;
	sfm_->internal_force_factor = cfg.internal_force_factor;
	sfm_->static_interaction_force_factor = cfg.static_interaction_force_factor;
	sfm_->dynamic_interaction_force_factor = cfg.dynamic_interaction_force_factor;
	sfm_->min_force = cfg.min_force;
	sfm_->max_force = cfg.max_force;
	sfm_->heterogenous_population = cfg.heterogenous_population;
	sfm_->use_tuned_params = cfg.use_tuned_params;
	sfm_->static_obj_interaction = cfg.static_object_interaction_type;
	sfm_->disable_interaction_forces = cfg.disable_interaction_forces;

	fis_->force_factor = cfg.force_factor;
	fis_->human_action_range = cfg.human_action_range;
	fis_->as = cfg.as;

	limits_->acc_lim_theta = cfg.acc_lim_theta;
	limits_->acc_lim_x = cfg.acc_lim_x;
	limits_->acc_lim_y = cfg.acc_lim_y;
	limits_->acc_lim_trans = cfg.acc_lim_trans;
	limits_->max_vel_theta = cfg.max_vel_theta;
	limits_->max_vel_trans = cfg.max_vel_trans;
	limits_->max_vel_x = cfg.max_vel_x;
	limits_->max_vel_y = cfg.max_vel_y;
	limits_->min_vel_theta = cfg.min_vel_theta;
	limits_->min_vel_trans = cfg.min_vel_trans;
	limits_->min_vel_x = cfg.min_vel_x;
	limits_->min_vel_y = cfg.min_vel_y;
	limits_->prune_plan = cfg.prune_plan;
	// limits_->restore_defaults
	limits_->theta_stopped_vel = cfg.theta_stopped_vel;
	limits_->trans_stopped_vel = cfg.trans_stopped_vel;
	limits_->xy_goal_tolerance = cfg.xy_goal_tolerance;
	limits_->yaw_goal_tolerance = cfg.yaw_goal_tolerance;
	limits_->twist_rotation_compensation = cfg.twist_rotation_compensation;

	traj_sampling_->force_internal_amplifier_min = cfg.force_internal_amplifier_min;
	traj_sampling_->force_internal_amplifier_max = cfg.force_internal_amplifier_max;
	traj_sampling_->force_internal_amplifier_granularity = cfg.force_internal_amplifier_granularity;
	traj_sampling_->force_interaction_static_amplifier_min = cfg.force_interaction_static_amplifier_min;
	traj_sampling_->force_interaction_static_amplifier_max = cfg.force_interaction_static_amplifier_max;
	traj_sampling_->force_interaction_static_amplifier_granularity = cfg.force_interaction_static_amplifier_granularity;
	traj_sampling_->force_interaction_dynamic_amplifier_min = cfg.force_interaction_dynamic_amplifier_min;
	traj_sampling_->force_interaction_dynamic_amplifier_max = cfg.force_interaction_dynamic_amplifier_max;
	traj_sampling_->force_interaction_dynamic_amplifier_granularity = cfg.force_interaction_dynamic_amplifier_granularity;
	traj_sampling_->force_interaction_social_amplifier_min = cfg.force_interaction_social_amplifier_min;
	traj_sampling_->force_interaction_social_amplifier_max = cfg.force_interaction_social_amplifier_max;
	traj_sampling_->force_interaction_social_amplifier_granularity = cfg.force_interaction_social_amplifier_granularity;

	costs_->path_distance_scale = cfg.path_distance_scale;
	costs_->goal_distance_scale = cfg.goal_distance_scale;
	costs_->occdist_scale = cfg.occdist_scale;
	costs_->ttc_scale = cfg.ttc_scale;
	costs_->chc_scale = cfg.chc_scale;
	costs_->speedy_goal_scale = cfg.speedy_goal_scale;
	costs_->oscillation_reset_dist = cfg.oscillation_reset_dist;
	costs_->oscillation_reset_angle = cfg.oscillation_reset_angle;
	costs_->scaling_speed = cfg.scaling_speed;
	costs_->max_scaling_factor = cfg.max_scaling_factor;
	costs_->ttc_rollout_time = cfg.ttc_rollout_time;
	costs_->ttc_collision_distance = cfg.ttc_collision_distance;
	costs_->forward_point_distance = cfg.forward_point_distance;
	costs_->speedy_goal_distance = cfg.speedy_goal_distance;

	diagnostics_->log_trajectory_generation_samples = cfg.log_trajectory_generation_samples;
	diagnostics_->log_trajectory_generation_details = cfg.log_trajectory_generation_details;
	diagnostics_->log_trajectory_generation_fails = cfg.log_trajectory_generation_fails;
	diagnostics_->log_explored_trajectories = cfg.log_explored_trajectories;
	diagnostics_->log_pts_of_explored_trajectories = cfg.log_pts_of_explored_trajectories;
	diagnostics_->log_trajectory_cost_details = cfg.log_trajectory_cost_details;
}

} /* namespace hubero_local_planner */
