#pragma once

#include <vector>
#include <string>
#include <mutex>
#include <memory>

#include <base_local_planner/local_planner_limits.h>

namespace hubero_local_planner {

//	/// \brief Declaration of an ActorParams typedef'ed struct;
//	/// default values are provided
//	struct GeneralParams {
//
//		std::vector<double> init_pose; 						// if empty - there will be .world file values chosen
//		std::vector<double> init_target;					// if empty - there will be random values chosen
//		unsigned short int	init_stance						= 0;
//
//		std::string 		global_frame_name				= "world";
//		double 				animation_factor				= 4.5;				/// \brief Time scaling factor. Used to coordinate translational motion with the actor_ptr_'s walking animation.
//		double 				animation_speed_rotation 		= 0.007;
//		double 				target_tolerance				= 1.25;
//		double 				target_reach_max_time 			= 60.0;
//		double 				target_reachable_check_period 	= 2.0;
//		bool 				limit_actors_workspace			= true;
//	    std::vector<double> world_bound_x					{-3.20, +3.80};
//	    std::vector<double> world_bound_y					{-10.20, +3.80};
//
//	};

	struct GeneralParams {
		double sim_time										= 1.2;
		double sim_granularity								= 0.025;
		double angular_sim_granularity						= 0.1;
		double sim_period									= 0.2;
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
		double 				interaction_force_factor 	= 3000.0;
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

class HuberoConfig {
public:
	HuberoConfig() {
		limits_ = std::make_shared<base_local_planner::LocalPlannerLimits>();
		general_ = std::make_shared<GeneralParams>();
		inflator_ = std::make_shared<InflatorParams>();
		sfm_ = std::make_shared<SfmParams>();
		behaviour_ = std::make_shared<BehaviourParams>();
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

	virtual ~HuberoConfig() = default;

protected:
	std::shared_ptr<base_local_planner::LocalPlannerLimits> limits_;
	std::shared_ptr<GeneralParams> general_;
	std::shared_ptr<InflatorParams> inflator_;
	std::shared_ptr<SfmParams> sfm_;
	std::shared_ptr<BehaviourParams> behaviour_;
};

typedef std::shared_ptr<const HuberoConfig> HuberoConfigConstPtr;

} /* namespace hubero_local_planner */

