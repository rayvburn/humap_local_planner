#pragma once

#include <vector>
#include <string>
#include <mutex>
#include <memory>

#include <base_local_planner/local_planner_limits.h>

namespace hubero_local_planner {
	struct GeneralParams {
		/// Whether to use planning (True) or proactive approach (False) for trajectory generation
		bool planning_approach = true;
		/// The amount of time to roll trajectories out for in seconds
		double sim_time										= 1.2;
		/// The granularity with which to check for collisions along each trajectory in meters
		double sim_granularity								= 0.025;
		/// The granularity with which to check for collisions for rotations in radians
		double angular_sim_granularity						= 0.1;
		/// How often planning executes
		double sim_period									= 0.2;
		/// The distance from the center point of the robot to place a local goal
		double forward_point_distance						= 0.5;
		/// How much robot tries to follow force direction compared to pure transformation of force into local velocity
		double twist_rotation_compensation					= 0.25;
		/// Whether to publish PCL with explored trajectories
		bool publish_traj_pcl = true;
		/// Whether to publish PCL with the potential field generated by the cost function
		bool publish_cost_grid_pcl = true;
	};

	/// \brief Declaration of an ActorParams typedef'ed struct;
	/// default values are provided
	struct InflatorParams {

		unsigned short int 	bounding_type 					= 2;
		double				circle_radius					= 0.5;
		std::vector<double> box_size						{0.45, 0.45, 1.00};
		std::vector<double> ellipse							{1.00, 0.80, 0.35, 0.00};
		double				inflation_radius			= 0.45; // the `worst` case from the default values

	};

	/// \brief Declaration of an SfmParams typedef'ed struct;
	/// default values are provided
	struct SfmParams {

		double 				fov					= 2.00;
		double 				max_speed 					= 1.50;
		double 				mass				= 80.0;
		/// \brief Determines maneuverability of the actor,
		double 				maneuverability			= 0.0065;
		double 				internal_force_factor 		= 100.0;
		double 				interaction_force_factor 	= 1200.0;
		double 				min_force 					= 300.0;
		double 				max_force 					= 2000.0;
		bool 				heterogenous_population 	= false;
		/// \brief Determines method of calculating force for static obstacles,
		/// see \ref sfm::StaticObjectInteraction
		unsigned short int 	static_obj_interaction 		= 0;
		/// \brief Determines type of inflation figure
		unsigned short int 	box_inflation_type 			= 0;
		/// \brief Method of computing a new pose while force
		/// of the opposite direction (relative to \f$\alpha\f$ is generated)
		unsigned short int	opposite_force_method		= 0;
		/// \brief Defines whether interaction forces should be calculated;
		/// setting to False will force robot to take the shortest possible path.
		bool 				disable_interaction_forces	= false;

	};

	/// \brief Declaration of a BehaviourParams typedef'ed struct;
	/// default values are provided
	struct BehaviourParams {
		double			force_factor					= 1.0;
		double			turn_left						= 500.0;
		double			turn_left_accelerate_turn 		= 500.0;
		double 			turn_left_accelerate_acc 		= 625.0;
		double 			accelerate						= 500.0;
		double 			turn_right_accelerate_turn 		= 500.0;
		double			turn_right_accelerate_acc		= 625.0;
		double			turn_right 						= 800.0;
		double 			turn_right_decelerate_turn 		= 500.0;
		double 			turn_right_decelerate_dec 		= 625.0;
		double 			stop							= 500.0;
		double			decelerate						= 500.0;
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
	};

class HuberoConfig {
public:
	HuberoConfig() {
		limits_ = std::make_shared<base_local_planner::LocalPlannerLimits>();
		general_ = std::make_shared<GeneralParams>();
		inflator_ = std::make_shared<InflatorParams>();
		sfm_ = std::make_shared<SfmParams>();
		behaviour_ = std::make_shared<BehaviourParams>();
		traj_sampling_ = std::make_shared<TrajectorySamplingParams>();
		costs_ = std::make_shared<CostParams>();
	}

	std::shared_ptr<const base_local_planner::LocalPlannerLimits> getLimits() const {
		return limits_;
	}

	std::shared_ptr<const GeneralParams> getGeneral() const {
		return general_;
	}

	std::shared_ptr<const InflatorParams> getInflator() const {
		return inflator_;
	}

	std::shared_ptr<const SfmParams> getSfm() const {
		return sfm_;
	}

	std::shared_ptr<const BehaviourParams> getBehaviour() const {
		return behaviour_;
	}

	std::shared_ptr<const TrajectorySamplingParams> getTrajectorySampling() const {
		return traj_sampling_;
	}

	std::shared_ptr<const CostParams> getCost() const {
		return costs_;
	}

	virtual ~HuberoConfig() = default;

protected:
	std::shared_ptr<base_local_planner::LocalPlannerLimits> limits_;
	std::shared_ptr<GeneralParams> general_;
	std::shared_ptr<InflatorParams> inflator_;
	std::shared_ptr<SfmParams> sfm_;
	std::shared_ptr<BehaviourParams> behaviour_;
	std::shared_ptr<TrajectorySamplingParams> traj_sampling_;
	std::shared_ptr<CostParams> costs_;
};

typedef std::shared_ptr<const HuberoConfig> HuberoConfigConstPtr;

} /* namespace hubero_local_planner */

