#pragma once

#include <vector>
#include <string>
#include <mutex>
#include <memory>

namespace hubero_local_planner {

class HuberoConfig {
public:
	/// \brief Declaration of an ActorParams typedef'ed struct;
	/// default values are provided
	struct GeneralParams {

		std::vector<double> init_pose; 						// if empty - there will be .world file values chosen
		std::vector<double> init_target;					// if empty - there will be random values chosen
		unsigned short int	init_stance; //						= 0;

		std::string 		global_frame_name;//				= "world";
		double 				animation_factor;// 				= 4.5;				/// \brief Time scaling factor. Used to coordinate translational motion with the actor_ptr_'s walking animation.
		double 				animation_speed_rotation;// 		= 0.007;
		double 				target_tolerance;// 				= 1.25;
		double 				target_reach_max_time;// 			= 60.0;
		double 				target_reachable_check_period;// 	= 2.0;
		bool 				limit_actors_workspace;//			= true;
	    std::vector<double> world_bound_x;//					{-3.20, +3.80};
	    std::vector<double> world_bound_y;//					{-10.20, +3.80};

	} general;

	/// \brief Declaration of an ActorParams typedef'ed struct;
	/// default values are provided
	struct InflatorParams {

		unsigned short int 	bounding_type;// 					= 2;
		double				circle_radius;//					= 0.5;
		std::vector<double> box_size;//						{0.45, 0.45, 1.00};
		std::vector<double> ellipse;//							{1.00, 0.80, 0.35, 0.00};
		double				inflation_radius;//				= 0.45; // the `worst` case from the default values

	} inflator;

	/// \brief Declaration of an SfmParams typedef'ed struct;
	/// default values are provided
	struct SfmParams {

		double 				fov;// 						= 2.00;
		double 				max_speed;// 					= 1.50;
		double 				mass;// 						= 80.0;
		double 				maneuverability;//				= 0.0065;
		double 				internal_force_factor;// 		= 100.0;
		double 				interaction_force_factor;// 	= 3000.0;
		double 				min_force;// 					= 300.0;
		double 				max_force;// 					= 2000.0;
		bool 				heterogenous_population;// 	= false;
		unsigned short int 	static_obj_interaction;// 		= 1;
		unsigned short int 	box_inflation_type;// 			= 0;
		unsigned short int	opposite_force_method;//		= 0;
		bool 				disable_interaction_forces;//	= false;

	} sfm;

	/// \brief Declaration of a BehaviourParams typedef'ed struct;
	/// default values are provided
	struct BehaviourParams {
		double			force_factor;//					= 1.0;
		double			turn_left;// 						= 500.0;
		double			turn_left_accelerate_turn;// 		= 500.0;
		double 			turn_left_accelerate_acc;// 		= 625.0;
		double 			accelerate;// 						= 500.0;
		double 			turn_right_accelerate_turn;// 		= 500.0;
		double			turn_right_accelerate_acc;// 		= 625.0;
		double			turn_right;// 						= 800.0;
		double 			turn_right_decelerate_turn;// 		= 500.0;
		double 			turn_right_decelerate_dec;// 		= 625.0;
		double 			stop;// 							= 500.0;
		double			decelerate;// 						= 500.0;
	} behaviour;

	HuberoConfig() = default;
	virtual ~HuberoConfig() = default;
};

typedef std::shared_ptr<const HuberoConfig> HuberoConfigConstPtr;

} /* namespace hubero_local_planner */

