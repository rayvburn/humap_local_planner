#pragma once

#include <vector>
#include <string>
#include <mutex>
#include <memory>

#include <base_local_planner/local_planner_limits.h>

namespace humap_local_planner {
	struct PlannerLimitsParams: base_local_planner::LocalPlannerLimits {
		/**
		 * Parameter related to conversion of global force vector to twist vector
		 *
		 * Defines how much robot tries to follow force direction compared to pure transformation of force into local
		 * velocity
		 */
		double twist_rotation_compensation = 0.25;
		/**
		 * Whether to maintain the mutual rate of velocity components when transforming requested velocity
		 * to the actual command
		 */
		bool maintain_vel_components_rate = true;
	};

	struct GeneralParams {
		/// Whether to use planning (True) or proactive approach (False) for trajectory generation
		bool planning_approach = true;
		/// How often the global path is expected to be updated
		double path_planning_period = 0.5;
		/// The amount of time to roll trajectories out for in seconds
		double sim_time = 1.2;
		/// The granularity with which to check for collisions along each trajectory in meters
		double sim_granularity = 0.025;
		/// The granularity with which to check for collisions for rotations in radians
		double angular_sim_granularity = 0.1;
		/// How often planning executes
		double sim_period = 0.2;
		/// The multiplier of the distance from the center point of the robot to place the local goal for self-driven
		/// force calculation
		double local_goal_distance_multiplier = 1.1;
		/// The radius of the circular representation of person-obstacle used by the sparse environment model
		double person_model_radius = 0.4;
		/// Half of the field of view of a detected person
		double person_fov = 3.31613 / 2.0;
		/// The factor by which robot's inscribed radius will be multiplied to caluclate distance between closest
		/// points of the robot footprint and environment object (static/dynamic)
		double obstacle_extension_multiplier = 1.0;
		/// Number of closest polygons taken into consideration as obstacles; -1 represents all polygons
		int obstacles_closest_polygons_num = -1;
		/// Number of closest people taken into consideration during trajectory planning; -1 represents all people
		int people_closest_num = -1;
		/// Number of closest groups taken into consideration during trajectory planning; -1 represents all groups
		int groups_closest_num = -1;
	};

	struct PlannerBehaviorsParams {
		/// Length of a buffer to detect oscillations. Expressed as a number of seconds to keep the measurements
		double oscillation_buffer_length = 5.0;
		/// The minimum speed (m/s) of a person to consider them in path crossing calculations
		double path_crossing_person_speed_threshold = 0.1;
		/// Confidence threshold to consider geometrical arrangement between the robot and a person as path crossing
		double path_crossing_confidence_threshold = 0.6;
		/**
		 * Standard deviation (in radians) of the relative location (angle from robot to human)
		 * Taken into account while calculating the confidence of the crossing;
		 * Default value reflects a 120-degree-wide bell (2-sigma rule applied)
		 */
		double path_crossing_front_stddev = (2.0 / 3.0 * M_PI) / 2.0;
		/// Maximum distance that the robot can travel while in the YIELD_WAY_CROSSING state
		double yield_way_max_travel_distance = 0.75;
		/**
		 * The multiplier of expected path planning frequency; once the global path plan is older than
		 * 'expected_freq * multiplier', it is considered a violation
		 */
		double path_plan_update_violation_multiplier = 5.0;
	};

	/// \brief Declaration of an SfmParams typedef'ed struct;
	/// default values are provided
	struct SfmParams {
		double fov = 2.00;
		unsigned int fov_factor_method = 0; // 0: Gaussian, 1: Linear, see sfm::FovCalculationMethod enum
		double mass = 80.0;
		double internal_force_factor = 100.0;
		double static_interaction_force_factor = 100.0;
		double dynamic_interaction_force_factor = 100.0;
		double min_force = 300.0;
		double max_force = 2000.0;
		/// Setting true causes SFM params to be defined in non-deterministic way (std deviation around mean value)
		bool heterogenous_population = false;
		/// Set to true to apply SFM parameter adjustments appointed in the tuning process
		bool use_tuned_params = true;
		/// \brief Determines method of calculating force for static obstacles,
		/// see \ref sfm::StaticObjectInteraction
		unsigned short int static_obj_interaction = 0;
		/// Whether to use a force formulation designated for an interaction with dynamic objects regardless
		/// of the human's speed. When set to false, the formulation will be chosen according to the speed.
		bool human_force_formulation_dynamic = false;
		/// Whether to apply filtering operations to the resultant forces
		bool filter_forces = true;
		/// \brief Defines whether interaction forces should be calculated;
		/// setting to False will force robot to take the shortest possible path.
		bool disable_interaction_forces	= false;

		/**
		 * @defgroup eqnparams Equation Params
		 * Group contains equation parameter value pairs: mean and standard deviation
		 * @{
		 */
		// mean values (CUSTOM TUNING_MULTIPLIER * reference value from article)
		double speed_desired = 1.29;
		double relaxation_time = 0.54;
		double an = -8.0 * 0.2615;
		double bn = +5.0 * 0.4026;
		double cn = +1.5 * 2.1614;
		double ap = +1.0 * 1.5375;
		double bp = +2.0 * 0.4938;
		double cp = +0.8 * 0.5710;
		double aw = +123.139 * 0.3280;
		double bw = +1.2 * 0.1871;

		// standard deviation values
		double speed_desired_stddev = 0.19;
		double relaxation_time_stddev = 0.05;
		double an_stddev = 0.0551;
		double bn_stddev = 0.1238;
		double cn_stddev = 0.3728;
		double ap_stddev = 0.3084;
		double bp_stddev = 0.1041;
		double cp_stddev = 0.1409;
		double aw_stddev = 0.1481;
		double bw_stddev = 0.0563;
		/// @} // end of eqnparams group
	};

	/// \brief Declaration of a FuzzyInferenceSystem Params typedef'ed struct;
	/// default values are provided
	struct FisParams {
		/// Factor to multiply resultant human action force with
		double force_factor = 1.0;
		/// Distance at which any created human action force fade completely
		double human_action_range = 4.0;
		/// Full field of view of a controlled agent (NOTE: inherits the value from @ref GeneralParams to obtain
		/// the human-like behaviour)
		double fov = 3.31613;
		/// Method used for FOV factor calculation; 0: Gaussian, 1: Linear, 2: Do not include FOV,
		/// see fuzz::FovCalculationMethod for details
		unsigned int fov_factor_method = 0;
		/// Levelling factor that also directly affects amplitude of the resultant force
		double as = 1.1865;
	};

	/**
	 * @brief General parameters related to trajectory generation
	 */
	struct TrajectoryGeneration {
		/// True enables the generator that produces model-based trajectories.
		bool use_social_trajectory_generator = true;
		/// True enables generator that produces evenly spaced elements from a list of feasible velocities.
		bool use_equisampled_velocities_generator = true;
		/// If set to true, the generator will recompute feasible velocities in each step and will restrict the
		/// velocities to those that do not overshoot the goal in sim_time. Otherwise, when false, the generator will
		/// sample velocities during the first iteration and will not take the goal into account (as in DWA approach)
		/// Legacy parameter name (nav. stack) `use_dwa`
		bool equisampled_continued_acceleration = true;
		/// How many velocity samples along platform's X axis will be checked
		unsigned int equisampled_vx = 5;
		/// How many velocity samples along platform's Y axis will be checked
		unsigned int equisampled_vy = 1;
		/// How many velocity samples around platform's Z axis will be checked
		unsigned int equisampled_vth = 10;
		/// Minimum linear velocity that is used by the equisampled velocities generator
		/// Helpful if the generator is not allowed to follow general rule that allows to e.g. produce backward motions
		double equisampled_min_vel_x = 0.1;
	};

	/**
	 * @brief Set of trajectory generator parameters with default values
	 *
	 * Granularities define how many equidistant amplifiers will be investigated (in how many samples to divide the
	 * given dimension)
	 */
	struct TrajectorySamplingParams {
		double sfm_desired_speed_amplifier_min = 1.0;
		double sfm_desired_speed_amplifier_max = 1.0;
		double sfm_desired_speed_amplifier_granularity = 1.0;

		double sfm_an_amplifier_min = 1.0;
		double sfm_an_amplifier_max = 1.0;
		double sfm_an_amplifier_granularity = 1.0;

		double sfm_bn_amplifier_min = 1.0;
		double sfm_bn_amplifier_max = 1.0;
		double sfm_bn_amplifier_granularity = 1.0;

		double sfm_cn_amplifier_min = 1.0;
		double sfm_cn_amplifier_max = 1.0;
		double sfm_cn_amplifier_granularity = 1.0;

		double sfm_ap_amplifier_min = 1.0;
		double sfm_ap_amplifier_max = 1.0;
		double sfm_ap_amplifier_granularity = 1.0;

		double sfm_bp_amplifier_min = 1.0;
		double sfm_bp_amplifier_max = 1.0;
		double sfm_bp_amplifier_granularity = 1.0;

		double sfm_cp_amplifier_min = 1.0;
		double sfm_cp_amplifier_max = 1.0;
		double sfm_cp_amplifier_granularity = 1.0;

		double sfm_aw_amplifier_min = -1.25;
		double sfm_aw_amplifier_max = 1.25;
		double sfm_aw_amplifier_granularity = 0.25;

		double sfm_bw_amplifier_min = 1.0;
		double sfm_bw_amplifier_max = 1.0;
		double sfm_bw_amplifier_granularity = 1.0;

		double fis_as_amplifier_min = 1.0;
		double fis_as_amplifier_max = 1.0;
		double fis_as_amplifier_granularity = 1.0;
	};

	struct CostParams {
		/// The weight for the path distance part of the cost function
		double path_distance_scale = 0.6;
		/// The weight for the goal distance part of the cost function
		double goal_distance_scale = 0.8;
		/// The weight for the obstacle distance part of the cost function
		double occdist_scale = 0.01;
		/// Whether to sum up the scores along the path or use the maximum one (default)
		bool occdist_sum_scores = false;
		/// The minimum separation that should be kept from obstacles
		double occdist_separation = 0.05;
		/// The type of kernel used for applying the separation from environment objects
		unsigned short int occdist_separation_kernel = 0;
		/// The weight for the alignment with the global plan
		double alignment_scale = 0.6;
		/// The weight for the achievement of the additional goal point placed in front of the mobile base
		double goal_front_scale = 0.8;
		/// The weight for deviating from the maximum allowable translational velocities
		double unsaturated_translation_scale = 0.0;
		/// The weight for the backward motion penalisation
		double backward_scale = 0.5;
		/// The weight for the time to collision (TTC) cost function
		double ttc_scale = 3.0;
		/// The weight for the cost function that penalizes robot rotational velocity changes
		double heading_change_smoothness_scale = 18.0;
		/// The weight for the cost function that penalizes velocity changes along trajectory
		double velocity_smoothness_scale = 13.0;
		/// The weight for the cost function that penalizes trajectories that leading towards human center
		double heading_dir_scale = 7.0;
		/// The weight for the cost function that penalizes trajectories that intrude human personal space
		double personal_space_scale = 7.0;
		/// The weight for the cost function that penalizes trajectories that intrude F-formation space
		double fformation_space_scale = 7.0;
		/// The weight for the cost function that penalizes robot trajectories that are not compliant with proper speeds of passing people
		double passing_speed_scale = 1.0;
		/// The distance the robot must travel before oscillation flags are reset, in meters
		double oscillation_reset_dist = 0.05;
		/// The angle the robot must turn before oscillation flags are reset, in radians
		double oscillation_reset_angle = 0.2;
		/// The absolute value of the velocity at which to start scaling the robot's footprint, in m/s
		double scaling_speed = 0.25;
		/// The maximum factor to scale the robot's footprint by
		double max_scaling_factor = 0.2;
		/// Penalty (cost) value assigned to trajectory that starts with backward motion
		double backward_penalty = 20.0;
		/// The duration of maximum world state prediction to detect robot collision
		double ttc_rollout_time = 3.0;
		/// The distance threshold for collision detection in TTC prediction
		double ttc_collision_distance = 0.05;
		/// The distance from the center point of the robot to place an additional scoring point, in meters
		double forward_point_distance = 0.325;
		/// Whether to compute the cost fun. for the whole horizon or only for the first step of each traj. candidate
		bool heading_dir_compute_whole_horizon = true;
		/// Whether to compute the cost fun. for the whole horizon or only for the first step of each traj. candidate
		bool personal_space_compute_whole_horizon = true;
		/// Whether to compute the cost fun. for the whole horizon or only for the first step of each traj. candidate
		bool fformation_space_compute_whole_horizon = true;
		/// Whether to compute the cost fun. for the whole horizon or only for the first step of each traj. candidate
		bool passing_speed_compute_whole_horizon = true;
		/// Whether to compute the cost fun. for the whole horizon or only for the first step of each traj. candidate
		bool unsaturated_translation_compute_whole_horizon = true;
	};

	struct DiagnosticsParams {
		/// Whether to print info on samples used for trajectory generation
		bool log_trajectory_generation_samples = false;
		/// Whether to print info on motion drivers (forces) of a specific trajectory
		bool log_trajectory_generation_details = false;
		/// Whether to print info on unsuccessful attempt to trajectory generation
		bool log_trajectory_generation_fails = false;
		/// Whether to print info on force amplitudes for given set of parameter amplifiers
		bool log_trajectory_generation_forces = false;
		/// Whether to print info on explored trajectories
		bool log_explored_trajectories = false;
		/// Whether to print info on points of explored trajectories
		bool log_pts_of_explored_trajectories = false;
		/// Whether to print info on cost of the best trajectory
		bool log_trajectory_cost_details = false;
		/// Whether to force stoppage of robot without disabling planning (useful for cost tuning)
		bool force_robot_stop = false;
	};

class HumapConfig {
public:
	HumapConfig() {
		limits_ = std::make_shared<PlannerLimitsParams>();
		general_ = std::make_shared<GeneralParams>();
		planner_behaviors_ = std::make_shared<PlannerBehaviorsParams>();
		sfm_ = std::make_shared<SfmParams>();
		fis_ = std::make_shared<FisParams>();
		traj_gen_ = std::make_shared<TrajectoryGeneration>();
		traj_sampling_ = std::make_shared<TrajectorySamplingParams>();
		costs_ = std::make_shared<CostParams>();
		diagnostics_ = std::make_shared<DiagnosticsParams>();
	}

	std::shared_ptr<const PlannerLimitsParams> getLimits() const {
		return limits_;
	}

	std::shared_ptr<const GeneralParams> getGeneral() const {
		return general_;
	}

	std::shared_ptr<PlannerBehaviorsParams> getPlannerBehaviors() const {
		return planner_behaviors_;
	}

	std::shared_ptr<const SfmParams> getSfm() const {
		return sfm_;
	}

	std::shared_ptr<const FisParams> getFis() const {
		return fis_;
	}

	std::shared_ptr<const TrajectoryGeneration> getTrajectoryGeneration() const {
		return traj_gen_;
	}

	std::shared_ptr<const TrajectorySamplingParams> getTrajectorySampling() const {
		return traj_sampling_;
	}

	std::shared_ptr<const CostParams> getCost() const {
		return costs_;
	}

	std::shared_ptr<const DiagnosticsParams> getDiagnostics() const {
		return diagnostics_;
	}

	std::mutex& getMutex() const {
		return mutex_;
	}

	virtual ~HumapConfig() = default;

protected:
	std::shared_ptr<PlannerLimitsParams> limits_;
	std::shared_ptr<GeneralParams> general_;
	std::shared_ptr<PlannerBehaviorsParams> planner_behaviors_;
	std::shared_ptr<SfmParams> sfm_;
	std::shared_ptr<FisParams> fis_;
	std::shared_ptr<TrajectoryGeneration> traj_gen_;
	std::shared_ptr<TrajectorySamplingParams> traj_sampling_;
	std::shared_ptr<CostParams> costs_;
	std::shared_ptr<DiagnosticsParams> diagnostics_;

	/// Mutex for config accesses and changes
	mutable std::mutex mutex_;
};

typedef std::shared_ptr<const HumapConfig> HumapConfigConstPtr;

} /* namespace humap_local_planner */

