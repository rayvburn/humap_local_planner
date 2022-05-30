#pragma once

#include <base_local_planner/trajectory_sample_generator.h>

#include <hubero_local_planner/world.h>
#include <hubero_local_planner/sfm/social_force_model.h>
#include <hubero_local_planner/fuzz/processor.h>
#include <hubero_local_planner/fuzz/social_conductor.h>

#include <hubero_local_planner/hubero_config.h>

#include <memory>

namespace hubero_local_planner {

/**
 * @brief Trajectory generator that produces trajectories that are complaint with common social rules pedestrian motion
 *
 * @note User must call setParameters<SPECIFIER> methods before call to @ref nextTrajectory or @ref generateTrajectory
 *
 * This trajectory generator does not use continued acceleration approach (like DWA). Instead, it uses model-based
 * approach to produce trajectories similar to the ones taken by people. It implements Social Force Model and Fuzzy
 * Inference system methods to generate robots trajectories.
 */
class SocialTrajectoryGenerator: public base_local_planner::TrajectorySampleGenerator {
public:
	/// Helper struct to pass a custom amplifier set to @ref initialise
	struct SampleAmplifierSet {
		double sfm_speed_desired_amplifier;
		double sfm_an_amplifier;
		double sfm_bn_amplifier;
		double sfm_cn_amplifier;
		double sfm_ap_amplifier;
		double sfm_bp_amplifier;
		double sfm_cp_amplifier;
		double sfm_aw_amplifier;
		double sfm_bw_amplifier;
		double fis_as_amplifier;

		/// Default ctor - unit amplifiers do not affect results
		SampleAmplifierSet():
			sfm_speed_desired_amplifier(1.0),
			sfm_an_amplifier(1.0),
			sfm_bn_amplifier(1.0),
			sfm_cn_amplifier(1.0),
			sfm_ap_amplifier(1.0),
			sfm_bp_amplifier(1.0),
			sfm_cp_amplifier(1.0),
			sfm_aw_amplifier(1.0),
			sfm_bw_amplifier(1.0),
			fis_as_amplifier(1.0) {}
	};

	SocialTrajectoryGenerator();

	/**
	 * @brief Update parameters used in trajectory generation
	 *
	 * @param sfm_params_ptr
	 * @param beh_params_ptr
	 * @param sim_time The amount of time to roll trajectories out for in seconds
	 * @param sim_granularity The granularity with which to check for collisions along each trajectory in meters
	 * @param angular_sim_granularity The granularity with which to check for collisions for rotations in radians
	 * @param sim_period How often planning executes
	 */
	void setParameters(
		std::shared_ptr<const hubero_local_planner::SfmParams> sfm_params_ptr,
		std::shared_ptr<const hubero_local_planner::FisParams> fis_params_ptr,
		double sim_time,
		double sim_granularity,
		double angular_sim_granularity,
		double sim_period,
		bool log_generation_samples,
		bool log_generation_details,
		bool log_generation_fails
	);

	inline bool isConfigured() const {
		return params_set_;
	}

	/**
	 * @brief Generate all samples that will be used to generate trajectories
	 *
	 * @param world_model stores current world model with static and dynamic obstacles (environment) and robot data
	 * @param robot_local_vel robot current velocity
	 * @param limits_amplifiers limits of amplifiers used to generate trajectories
	 * @param limits_lp_ptr current velocity limits (local planner's)
	 * @param additional_samples additional velocity samples to generate individual trajectories from.
	 * @param robot_mass mass of the robot (required for SFM calculations)
	 * @param discretize_by_time if true, the trajectory is split in chunks of the same duration, else, of same length
	 *
	 * @note Based on base_local_planner::SimpleTrajectoryGenerator::initialise
	 * Copyright (c) 2008, Willow Garage, Inc., Author: TKruse
	 */
  	void initialise(
		const World& world_model,
		const geometry::Vector& robot_local_vel,
		const TrajectorySamplingParams& limits_amplifiers,
		std::shared_ptr<const PlannerLimitsParams> limits_lp_ptr,
		const std::vector<SampleAmplifierSet>& additional_samples,
		const double& robot_mass,
		bool discretize_by_time = false
	);

	/**
	 * @param world_model stores current world model with static and dynamic obstacles (environment) and robot data
	 * @param robot_local_vel robot current velocity
	 * @param limits_amplifiers limits of amplifiers used to generate trajectories
	 * @param limits_lp_ptr current velocity limits (local planner's)
	 * @param robot_mass mass of the robot (required for SFM calculations)
	 * @param discretize_by_time if true, the trajectory is split in chunks of the same duration, else, of same length
	 *
	 * @note Based on base_local_planner::SimpleTrajectoryGenerator::initialise
	 * Copyright (c) 2008, Willow Garage, Inc., Author: TKruse
	 */
	void initialise(
		const World& world_model,
		const geometry::Vector& robot_local_vel,
		const TrajectorySamplingParams& limits_amplifiers,
		std::shared_ptr<const PlannerLimitsParams> limits_lp_ptr,
		const double& robot_mass,
		bool discretize_by_time = false
	);

	/**
	 * @brief Version of @ref initialise without @ref TrajectorySamplingParams
	 * Useful for @ref generateTrajectoryWithoutPlanning
	 */
	void initialise(
		const World& world_model,
		const geometry::Vector& robot_local_vel,
		std::shared_ptr<const PlannerLimitsParams> limits_lp_ptr,
		const double& robot_mass,
		bool discretize_by_time = false
	);

	/**
	 * Whether this generator can create more trajectories
	 *
	 * @note Based on base_local_planner::SimpleTrajectoryGenerator::hasMoreTrajectories
	 * Copyright (c) 2008, Willow Garage, Inc., Author: TKruse
	 */
	virtual bool hasMoreTrajectories() override {
		return next_sample_index_ < sample_amplifier_params_v_.size();
	}

	/**
	 * Create and return the next sample trajectory
	 *
	 * @note Based on base_local_planner::SimpleTrajectoryGenerator::hasMoreTrajectories
	 * Copyright (c) 2008, Willow Garage, Inc., Author: TKruse
	 */
	virtual bool nextTrajectory(base_local_planner::Trajectory& traj) override;

	bool generateTrajectory(
		const World& world_model,
		const SampleAmplifierSet& sample_amplifiers,
		base_local_planner::Trajectory& traj
	);

	/**
	 * @brief Prints fuzzy inference system configuration
	 */
	void printFisConfiguration() const;

	/**
	 * @brief Calculates next trajectory to the goal without deep investigation into the future
	 *
	 * Calculations are performed according to a fixed set of parameters
	 *
	 * @param world_model model of robot and its environment
	 * @param traj trajectory object that will be filled up according to calculations
	 * @return true if traj complies with local planner's velocity limits
	 * @return false if traj exceeds local planner's velocity limits
	 */
	bool generateTrajectoryWithoutPlanning(base_local_planner::Trajectory& traj);

	/**
	 * @brief Retrieves internal force vector computed during the last @ref computeForces with motion data update req.
	 */
	inline geometry::Vector getForceInternal() const {
		return diag_force_internal_;
	}

	/**
	 * @brief Retrieves interaction force vector computed during the last @ref computeForces with motion data update req.
	 */
	inline geometry::Vector getForceInteractionStatic() const {
		return diag_force_interaction_static_;
	}

	/**
	 * @brief Retrieves interaction force vector computed during the last @ref computeForces with motion data update req.
	 */
	inline geometry::Vector getForceInteractionDynamic() const {
		return diag_force_interaction_dynamic_;
	}

	/**
	 * @brief Retrieves social force vector computed during the last @ref computeForces with motion data update
	 */
	inline geometry::Vector getForceSocial() const {
		return diag_force_social_;
	}

	/**
	 * @brief Retrieves robot-obstacle poses computed during the last @ref computeForces with motion data update req.
	 */
	inline std::vector<geometry::Pose> getRobotStaticObstacleDistances() const {
		return diag_closest_points_static_;
	}

	/**
	 * @brief Retrieves robot-obstacle poses computed during the last @ref computeForces with motion data update req.
	 */
	inline std::vector<geometry::Pose> getRobotDynamicObstacleDistances() const {
		return diag_closest_points_dynamic_;
	}

	/**
	 * @brief Retrieves fuzzy behaviour computed during the last @ref computeForces with motion data update req.
	 */
	inline std::string getActiveFuzzyBehaviour() const {
		return diag_behaviour_active_;
	}

	static std::vector<double> computeAmplifierSamples(
		double amplifier_min,
		double amplifier_max,
		double granularity,
		const std::string& log_identifier
	);

protected:
	/**
	 * @details Makes sure that the robot would at least be moving with one of the required minimum velocities
	 * for translation and rotation (if set)
	 *
	 * @note Based on contents of base_local_planner::SimpleTrajectoryGenerator::generateTrajectory
	 * Copyright (c) 2008, Willow Garage, Inc., Author: TKruse
	 */
	bool areVelocityLimitsFulfilled(const double& speed_linear, const double& speed_angular, const double& eps);

	/**
	 * @note Based on contents of base_local_planner::SimpleTrajectoryGenerator::generateTrajectory
	 * Copyright (c) 2008, Willow Garage, Inc., Author: TKruse
	 */
	int computeStepsNumber(const double& speed_linear, const double& speed_angular);

	/**
	 * @brief Wraps all necessary force calculations required to compute a velocity sample
	 *
	 * @param update_motion_data should be set to true for first/single trajectory calculation
	 */
	void computeForces(
		const World& world_model,
		const double& dt,
		const SampleAmplifierSet& sample_amplifiers,
		geometry::Vector& force_internal,
		geometry::Vector& force_interaction_dynamic,
		geometry::Vector& force_interaction_static,
		geometry::Vector& force_human_action,
		bool update_motion_data = false
	);

	/// Turns true when @ref setParameters was called
	bool params_set_;

	/// Vector of investigated amplifier values
	std::vector<SampleAmplifierSet> sample_amplifier_params_v_;
	/// Determines vector index of the next sample
	unsigned int next_sample_index_;
	/// Store current set of samples that will be investigated for scoring
	TrajectorySamplingParams limits_amplifiers_;

	/// Velocity limits of the local planner
	std::shared_ptr<const PlannerLimitsParams> limits_planner_ptr_;
	Eigen::Vector3f vels_min_;
	Eigen::Vector3f vels_max_;

	/// Stores newest world model passed via @ref initialise call; values are referenced to global coordinate system
	World world_model_;
	/// Stores robot's newest velocity expressed in the base coordinate system
	geometry::Vector vel_local_;

	/// Granularity between simulation timestamps, a.k.a. time between points in the trajectory, a.k.a. `dt`
	double sim_granularity_;
	double angular_sim_granularity_;
	/// How far in the future the local plan will be generated
	double sim_time_;
	double sim_period_;

	/// Flags that enable logging
	bool log_generation_samples_;
	bool log_generation_details_;
	bool log_generation_fails_;

	/// True if number of points in the trajectory only depends on simulation granularity and not depends on distance
	bool discretize_by_time_;

	double robot_mass_;

	/**
	 * @defgroup socialtrajectory Helper classes that are used to compute velocity samples of the trajectory
	 * @{
	 */
	sfm::SocialForceModel sfm_;
	fuzz::Processor fuzzy_processor_;
	fuzz::SocialConductor social_conductor_;
	/// @}

	/**
	 * @defgroup diagnostics Diagnostics data
	 * @{
	 */
	geometry::Vector diag_force_internal_;
	geometry::Vector diag_force_interaction_static_;
	geometry::Vector diag_force_interaction_dynamic_;
	geometry::Vector diag_force_social_;
	std::vector<geometry::Pose> diag_closest_points_static_;
	std::vector<geometry::Pose> diag_closest_points_dynamic_;
	std::string diag_behaviour_active_;
	/// @}

}; // class SocialTrajectoryGenerator
} // namespace hubero_local_planner
