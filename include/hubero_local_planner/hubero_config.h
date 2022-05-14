#pragma once

#include <vector>
#include <string>
#include <mutex>
#include <memory>

#include <base_local_planner/local_planner_limits.h>

namespace hubero_local_planner {
	struct PlannerLimitsParams: base_local_planner::LocalPlannerLimits {
		/**
		 * Parameter related to conversion of global force vector to twist vector
		 *
		 * Defines how much robot tries to follow force direction compared to pure transformation of force into local
		 * velocity
		 */
		double twist_rotation_compensation = 0.25;
	};

	struct GeneralParams {
		/// Whether to use planning (True) or proactive approach (False) for trajectory generation
		bool planning_approach = true;
		/// The amount of time to roll trajectories out for in seconds
		double sim_time = 1.2;
		/// The granularity with which to check for collisions along each trajectory in meters
		double sim_granularity = 0.025;
		/// The granularity with which to check for collisions for rotations in radians
		double angular_sim_granularity = 0.1;
		/// How often planning executes
		double sim_period = 0.2;
		/// The distance from the center point of the robot to place a local goal
		double local_goal_distance = 1.0;
		/// The radius of the circular representation of person-obstacle used by the sparse environment model
		double person_model_radius = 0.4;
		/// Whether to publish PCL with explored trajectories
		bool publish_traj_pcl = true;
		/// Whether to publish PCL with the potential field generated by the cost function
		bool publish_cost_grid_pcl = true;
	};

	/// \brief Declaration of an SfmParams typedef'ed struct;
	/// default values are provided
	struct SfmParams {
		double fov = 2.00;
		double mass = 80.0;
		double internal_force_factor = 100.0;
		double static_interaction_force_factor = 100.0;
		double dynamic_interaction_force_factor = 100.0;
		double min_force = 300.0;
		double max_force = 2000.0;
		/// Setting true causes SFM params to be defined in non-deterministic way (std deviation around mean value)
		bool heterogenous_population = false;
		/// \brief Determines method of calculating force for static obstacles,
		/// see \ref sfm::StaticObjectInteraction
		unsigned short int static_obj_interaction = 0;
		/// \brief Defines whether interaction forces should be calculated;
		/// setting to False will force robot to take the shortest possible path.
		bool disable_interaction_forces	= false;
	};

	/// \brief Declaration of a FuzzyInferenceSystem Params typedef'ed struct;
	/// default values are provided
	struct FisParams {
		/// Factor to multiply resultant human action force with
		double force_factor = 1.0;
		/// Distance at which any created human action force fade completely
		double human_action_range = 4.0;
	};

	/**
	 * @brief Set of trajectory generator parameters with default values
	 *
	 * Granularities define how many equidistant amplifiers will be investigated (in how many samples to divide the
	 * given dimension)
	 */
	struct TrajectorySamplingParams {
		double force_internal_amplifier_min = 0.5;
		double force_internal_amplifier_max = 1.5;
		double force_internal_amplifier_granularity = 0.5;

		double force_interaction_dynamic_amplifier_min = 0.5;
		double force_interaction_dynamic_amplifier_max = 1.5;
		double force_interaction_dynamic_amplifier_granularity = 0.5;

		double force_interaction_social_amplifier_min = 0.5;
		double force_interaction_social_amplifier_max = 1.5;
		double force_interaction_social_amplifier_granularity = 0.5;

		double force_interaction_static_amplifier_min = 0.5;
		double force_interaction_static_amplifier_max = 1.5;
		double force_interaction_static_amplifier_granularity = 0.5;
	};

	struct CostParams {
		/// The weight for the path distance part of the cost function
		double path_distance_scale = 0.6;
		/// The weight for the goal distance part of the cost function
		double goal_distance_scale = 0.8;
		/// The weight for the obstacle distance part of the cost function
		double occdist_scale = 0.01;
		/// The weight for the time to collision (TTC) cost function
		double ttc_scale = 3.0;
		/// The weight for the cost function that penalizes robot heading changes
		double chc_scale = 1.0;
		/// The weight for the cost function that penalizes high speeds near global goal
		double speedy_goal_scale = 12.0;
		/// The distance the robot must travel before oscillation flags are reset, in meters
		double oscillation_reset_dist = 0.05;
		/// The angle the robot must turn before oscillation flags are reset, in radians
		double oscillation_reset_angle = 0.2;
		/// The absolute value of the velocity at which to start scaling the robot's footprint, in m/s
		double scaling_speed = 0.25;
		/// The maximum factor to scale the robot's footprint by
		double max_scaling_factor = 0.2;
		/// The duration of maximum world state prediction to detect robot collision
		double ttc_rollout_time = 3.0;
		/// The distance threshold for collision detection in TTC prediction
		double ttc_collision_distance = 0.05;
		/// The distance from the center point of the robot to place an additional scoring point, in meters
		double forward_point_distance = 0.325;
		/// The distance threshold of penalization of high speeds near global goal
		double speedy_goal_distance = 0.85;
	};

	struct DiagnosticsParams {
		/// Whether to print info on samples used for trajectory generation
		bool log_trajectory_generation_samples = false;
		/// Whether to print info on motion drivers (forces) of a specific trajectory
		bool log_trajectory_generation_details = false;
		/// Whether to print info on unsuccessful attempt to trajectory generation
		bool log_trajectory_generation_fails = false;
		/// Whether to print info on explored trajectories
		bool log_explored_trajectories = false;
		/// Whether to print info on points of explored trajectories
		bool log_pts_of_explored_trajectories = false;
		/// Whether to print info on cost of the best trajectory
		bool log_trajectory_cost_details = false;
	};

class HuberoConfig {
public:
	HuberoConfig() {
		limits_ = std::make_shared<PlannerLimitsParams>();
		general_ = std::make_shared<GeneralParams>();
		sfm_ = std::make_shared<SfmParams>();
		fis_ = std::make_shared<FisParams>();
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

	std::shared_ptr<const SfmParams> getSfm() const {
		return sfm_;
	}

	std::shared_ptr<const FisParams> getFis() const {
		return fis_;
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

	virtual ~HuberoConfig() = default;

protected:
	std::shared_ptr<PlannerLimitsParams> limits_;
	std::shared_ptr<GeneralParams> general_;
	std::shared_ptr<SfmParams> sfm_;
	std::shared_ptr<FisParams> fis_;
	std::shared_ptr<TrajectorySamplingParams> traj_sampling_;
	std::shared_ptr<CostParams> costs_;
	std::shared_ptr<DiagnosticsParams> diagnostics_;
};

typedef std::shared_ptr<const HuberoConfig> HuberoConfigConstPtr;

} /* namespace hubero_local_planner */

