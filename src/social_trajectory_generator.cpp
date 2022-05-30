#include <hubero_local_planner/social_trajectory_generator.h>

#include <base_local_planner/simple_trajectory_generator.h> // static methods used in 'computeNewPositions' and 'computeNewVelocities'
#include <hubero_local_planner/hubero_planner_ros.h> // static methods (computeTwist) used in generateTrajectory
#include <hubero_local_planner/utils/transformations.h> // free functions

#include <cmath> // hypot

namespace hubero_local_planner {

using namespace base_local_planner;

SocialTrajectoryGenerator::SocialTrajectoryGenerator():
	params_set_(false),
	limits_planner_ptr_(nullptr),
	vels_min_(Eigen::Vector3f::Zero()),
	vels_max_(Eigen::Vector3f::Zero()),
	log_generation_samples_(false),
	log_generation_details_(false),
	log_generation_fails_(false) {
}

void SocialTrajectoryGenerator::setParameters(
	std::shared_ptr<const hubero_local_planner::SfmParams> sfm_params_ptr,
	std::shared_ptr<const hubero_local_planner::FisParams> fis_params_ptr,
	double sim_time,
	double sim_granularity,
	double angular_sim_granularity,
	double sim_period,
	bool log_generation_samples,
	bool log_generation_details,
	bool log_generation_fails
) {
	sfm_.init(sfm_params_ptr);
	social_conductor_.initialize(fis_params_ptr);

	sim_time_ = sim_time;
	sim_granularity_ = sim_granularity;
	angular_sim_granularity_ = angular_sim_granularity;
	sim_period_ = sim_period;
	log_generation_samples_ = log_generation_samples;
	log_generation_details_ = log_generation_details;
	log_generation_fails_ = log_generation_fails;

	params_set_ = true;
}

void SocialTrajectoryGenerator::initialise(
	const World& world_model,
	const geometry::Vector& robot_local_vel,
	const TrajectorySamplingParams& limits_amplifiers,
	std::shared_ptr<const PlannerLimitsParams> limits_lp_ptr,
	const std::vector<SampleAmplifierSet>& additional_samples,
	const double& robot_mass,
	bool discretize_by_time
) {
	initialise(
		world_model,
		robot_local_vel,
		limits_amplifiers,
		limits_lp_ptr,
		robot_mass,
		discretize_by_time
	);
	// add static samples if any
	sample_amplifier_params_v_.insert(sample_amplifier_params_v_.end(), additional_samples.begin(), additional_samples.end());
}

void SocialTrajectoryGenerator::initialise(
	const World& world_model,
	const geometry::Vector& robot_local_vel,
	const TrajectorySamplingParams& limits_amplifiers,
	std::shared_ptr<const PlannerLimitsParams> limits_lp_ptr,
	const double& robot_mass,
	bool discretize_by_time
) {
	// save copies for later use
	world_model_ = world_model;
	vel_local_ = robot_local_vel;
	limits_planner_ptr_ = limits_lp_ptr;
	limits_amplifiers_ = limits_amplifiers;
	discretize_by_time_ = discretize_by_time;

	robot_mass_ = robot_mass;

	// setup search
	next_sample_index_ = 0;
	sample_amplifier_params_v_.clear();

	// prepare all amplifier values to be evaluated later
	auto sfm_speed_amps = computeAmplifierSamples(
		limits_amplifiers_.sfm_desired_speed_amplifier_min,
		limits_amplifiers_.sfm_desired_speed_amplifier_max,
		limits_amplifiers_.sfm_desired_speed_amplifier_granularity,
		"desired speed parameter of the SFM"
	);

	auto sfm_an_amps = computeAmplifierSamples(
		limits_amplifiers_.sfm_an_amplifier_min,
		limits_amplifiers_.sfm_an_amplifier_max,
		limits_amplifiers_.sfm_an_amplifier_granularity,
		"An parameter of the SFM (dyn. objects)"
	);

	auto sfm_bn_amps = computeAmplifierSamples(
		limits_amplifiers_.sfm_bn_amplifier_min,
		limits_amplifiers_.sfm_bn_amplifier_max,
		limits_amplifiers_.sfm_bn_amplifier_granularity,
		"Bn parameter of the SFM (dyn. objects)"
	);

	auto sfm_cn_amps = computeAmplifierSamples(
		limits_amplifiers_.sfm_cn_amplifier_min,
		limits_amplifiers_.sfm_cn_amplifier_max,
		limits_amplifiers_.sfm_cn_amplifier_granularity,
		"Cn parameter of the SFM (dyn. objects)"
	);

	auto sfm_ap_amps = computeAmplifierSamples(
		limits_amplifiers_.sfm_ap_amplifier_min,
		limits_amplifiers_.sfm_ap_amplifier_max,
		limits_amplifiers_.sfm_ap_amplifier_granularity,
		"Ap parameter of the SFM (dyn. objects)"
	);

	auto sfm_bp_amps = computeAmplifierSamples(
		limits_amplifiers_.sfm_bp_amplifier_min,
		limits_amplifiers_.sfm_bp_amplifier_max,
		limits_amplifiers_.sfm_bp_amplifier_granularity,
		"Bp parameter of the SFM (dyn. objects)"
	);

	auto sfm_cp_amps = computeAmplifierSamples(
		limits_amplifiers_.sfm_cp_amplifier_min,
		limits_amplifiers_.sfm_cp_amplifier_max,
		limits_amplifiers_.sfm_cp_amplifier_granularity,
		"Cp parameter of the SFM (dyn. objects)"
	);

	auto sfm_aw_amps = computeAmplifierSamples(
		limits_amplifiers_.sfm_aw_amplifier_min,
		limits_amplifiers_.sfm_aw_amplifier_max,
		limits_amplifiers_.sfm_aw_amplifier_granularity,
		"Aw parameter of the SFM (stat. objects)"
	);

	auto sfm_bw_amps = computeAmplifierSamples(
		limits_amplifiers_.sfm_bw_amplifier_min,
		limits_amplifiers_.sfm_bw_amplifier_max,
		limits_amplifiers_.sfm_bw_amplifier_granularity,
		"Bw parameter of the SFM (stat. objects)"
	);

	auto fis_as_amps = computeAmplifierSamples(
		limits_amplifiers_.fis_as_amplifier_min,
		limits_amplifiers_.fis_as_amplifier_max,
		limits_amplifiers_.fis_as_amplifier_granularity,
		"As parameter of the FIS (people)"
	);

	// prepare vector of amplifiers to investigate
	for (const auto& speed: sfm_speed_amps) {
		for (const auto& an: sfm_an_amps) {
			for (const auto& bn: sfm_bn_amps) {
				for (const auto& cn: sfm_cn_amps) {
					for (const auto& ap: sfm_ap_amps) {
						for (const auto& bp: sfm_bp_amps) {
							for (const auto& cp: sfm_cp_amps) {
								for (const auto& aw: sfm_aw_amps) {
									for (const auto& bw: sfm_bw_amps) {
										for (const auto& as: fis_as_amps) {
											SampleAmplifierSet amp_set {};
											amp_set.sfm_speed_desired_amplifier = speed;
											amp_set.sfm_an_amplifier = an;
											amp_set.sfm_bn_amplifier = bn;
											amp_set.sfm_cn_amplifier = cn;
											amp_set.sfm_ap_amplifier = ap;
											amp_set.sfm_bp_amplifier = bp;
											amp_set.sfm_cp_amplifier = cp;
											amp_set.sfm_aw_amplifier = aw;
											amp_set.sfm_bw_amplifier = bw;
											amp_set.fis_as_amplifier = as;
											sample_amplifier_params_v_.push_back(amp_set);

											ROS_INFO_COND_NAMED(
												log_generation_samples_,
												"SocTrajGen",
												"Sample %3lu - v_i0 %2.5f, An %2.5f, Bn %2.5f, Cn %2.5f, "
												"Ap %2.5f, Bp %2.5f, Cp %2.5f, "
												"Aw %2.5f, Bw %2.5f, "
												"As %2.5f",
												sample_amplifier_params_v_.size(),
												amp_set.sfm_speed_desired_amplifier,
												amp_set.sfm_an_amplifier,
												amp_set.sfm_bn_amplifier,
												amp_set.sfm_cn_amplifier,
												amp_set.sfm_ap_amplifier,
												amp_set.sfm_bp_amplifier,
												amp_set.sfm_cp_amplifier,
												amp_set.sfm_aw_amplifier,
												amp_set.sfm_bw_amplifier,
												amp_set.fis_as_amplifier
											);
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}

	ROS_INFO_COND_NAMED(
		log_generation_samples_,
		"SocTrajGen",
		"Initialized %lu parameter samples "
		"(|v_i0| %lu, |An| %lu, |Bn| %lu, |Cn| %lu, |Ap| %lu, |Bp| %lu, |Cp| %lu, |Aw| %lu, |Bw| %lu, |As| %lu)",
		sample_amplifier_params_v_.size(),
		sfm_speed_amps.size(),
		sfm_an_amps.size(),
		sfm_bn_amps.size(),
		sfm_cn_amps.size(),
		sfm_ap_amps.size(),
		sfm_bp_amps.size(),
		sfm_cp_amps.size(),
		sfm_aw_amps.size(),
		sfm_bw_amps.size(),
		fis_as_amps.size()
	);
}

void SocialTrajectoryGenerator::initialise(
	const World& world_model,
	const geometry::Vector& robot_local_vel,
	std::shared_ptr<const PlannerLimitsParams> limits_lp_ptr,
	const double& robot_mass,
	bool discretize_by_time
) {
	// save copies for later use
	world_model_ = world_model;
	vel_local_ = robot_local_vel;
	limits_planner_ptr_ = limits_lp_ptr;
	discretize_by_time_ = discretize_by_time;

	robot_mass_ = robot_mass;
}

bool SocialTrajectoryGenerator::nextTrajectory(Trajectory& traj) {
	bool result = false;
	if (hasMoreTrajectories()) {
		// try to generate a new trajectory, starting with the initial world model given in `initialise`
		if (generateTrajectory(
			world_model_,
			sample_amplifier_params_v_[next_sample_index_],
			traj
		)) {
			result = true;
		}
	}
	next_sample_index_++;
	return result;
}

bool SocialTrajectoryGenerator::generateTrajectory(
	const World& world_model,
	const SampleAmplifierSet& sample_amplifiers,
	base_local_planner::Trajectory& traj
) {
	ROS_INFO_COND_NAMED(
		log_generation_details_,
		"SocTrajGen",
		"Trajectory generation with: "
		"{v_i0: %3.5f, An: %3.5f, Bn: %3.5f, Cn: %3.5f, Ap: %3.5f, Bp: %3.5f, Cp: %3.5f, Aw: %3.5f, Bw: %3.5f, As: %3.5f}",
		sample_amplifiers.sfm_speed_desired_amplifier,
		sample_amplifiers.sfm_an_amplifier,
		sample_amplifiers.sfm_bn_amplifier,
		sample_amplifiers.sfm_cn_amplifier,
		sample_amplifiers.sfm_ap_amplifier,
		sample_amplifiers.sfm_bp_amplifier,
		sample_amplifiers.sfm_cp_amplifier,
		sample_amplifiers.sfm_aw_amplifier,
		sample_amplifiers.sfm_bw_amplifier,
		sample_amplifiers.fis_as_amplifier
	);

	// speed i.e. velocity magnitude
	double speed_linear = std::hypot(vel_local_.getX(), vel_local_.getY());
	double speed_angular = vel_local_.getZ();

	// placed here in case we return early
	traj.cost_ = -1.0;
	// trajectory might be reused so we'll make sure to reset it
	traj.resetPoints();

	// compute for how many steps forward the simulation will be performed
	int num_steps = computeStepsNumber(speed_linear, speed_angular);

	// compute a timestep
	double dt = sim_time_ / num_steps;
	traj.time_delta_ = dt;

	// copy the initial model of world as it may be reused
	World world_model_plan = world_model;

	for (int i = 0; i < num_steps; ++i) {
		geometry::Vector force_internal;
		geometry::Vector force_interaction_dynamic;
		geometry::Vector force_interaction_static;
		geometry::Vector force_human_action;
		computeForces(
			world_model_plan,
			dt,
			sample_amplifiers,
			force_internal,
			force_interaction_dynamic,
			force_interaction_static,
			force_human_action,
			i == 0 // motion data updated only at first iteration
		);

		// force vectors are already multiplied by proper factors
		geometry::Vector twist_cmd;
		computeTwist(
			world_model_plan.getRobotData().centroid,
			force_internal + force_interaction_dynamic + force_interaction_static + force_human_action,
			world_model_plan.getRobotData().vel,
			dt,
			robot_mass_,
			limits_planner_ptr_->min_vel_x,
			limits_planner_ptr_->max_vel_x,
			limits_planner_ptr_->max_vel_theta,
			limits_planner_ptr_->twist_rotation_compensation,
			twist_cmd
		);

		// find current local velocity
		Vector vel_local_plan;
		computeVelocityLocal(
			world_model_plan.getRobotData().vel,
			world_model_plan.getRobotData().centroid,
			vel_local_plan
		);

		// make sure acceleration limits are not violated with the computed twist command
		adjustTwistWithAccLimits(
			vel_local_plan,
			limits_planner_ptr_->acc_lim_x,
			limits_planner_ptr_->acc_lim_y,
			limits_planner_ptr_->acc_lim_theta,
			limits_planner_ptr_->min_vel_x,
			limits_planner_ptr_->min_vel_y,
			-limits_planner_ptr_->max_vel_theta,
			limits_planner_ptr_->max_vel_x,
			limits_planner_ptr_->max_vel_y,
			limits_planner_ptr_->max_vel_theta,
			dt,
			twist_cmd
		);

		// evaluate effect if the computed forces would be applied -
		// check if velocity limits will be violated after application of the computed velocity
		double sampled_speed_linear = std::hypot(twist_cmd.getX(), twist_cmd.getY());
		double sampled_speed_angular = twist_cmd.getZ();
		if (!areVelocityLimitsFulfilled(sampled_speed_linear, sampled_speed_angular, 1e-4)) {
			ROS_ERROR_COND_NAMED(
				log_generation_fails_,
				"SocTrajGen",
				"Cannot generate trajectory %3u / %3lu due to violated velocity limits ("
				"lin: %3.2f, limits [%3.2f; %3.2f], "
				"ang: %3.2f, limits [%3.2f; %3.2f])",
				next_sample_index_ + 1,
				sample_amplifier_params_v_.size(),
				sampled_speed_linear,
				limits_planner_ptr_->min_vel_trans,
				limits_planner_ptr_->max_vel_trans,
				sampled_speed_angular,
				limits_planner_ptr_->min_vel_theta,
				limits_planner_ptr_->max_vel_theta
			);
			return false;
		}

		// trajectory seems to be valid - seed it with initial velocities
		if (i == 0) {
			traj.xv_ = twist_cmd.getX();
			traj.yv_ = twist_cmd.getY();
			traj.thetav_ = twist_cmd.getZ();
		}

		// extend trajectory with the current pose
		traj.addPoint(
			world_model_plan.getRobotData().centroid.getPosition().getX(),
			world_model_plan.getRobotData().centroid.getPosition().getY(),
			world_model_plan.getRobotData().centroid.getYaw()
		);

		// now, compute robot position if the computed twist command would be applied
		// prepare data for state prediction (global velocity is required, not the base's)
		Vector twist_cmd_glob;
		computeVelocityGlobal(twist_cmd, world_model_plan.getRobotData().centroid, twist_cmd_glob);

		// apply predictions to dynamic objects in the world
		world_model_plan.predict(twist_cmd_glob, dt);

		ROS_INFO_COND_NAMED(
			log_generation_details_,
			"SocTrajGen",
			"Trajectory %3u / %3lu step %3d / %3d: "
			"|f_i0| %3.1f <)%2.1f, |f_ij| %3.1f <)%2.1f, |f_is| %3.1f <)%2.1f, |f_ik| %3.1f <)%2.1f; twist: x %3.3f, y %3.3f, th %3.3f",
			next_sample_index_ + 1,
			sample_amplifier_params_v_.size(),
			i + 1,
			num_steps,
			force_internal.calculateLength(),
			force_internal.calculateDirection().getDegree(),
			force_interaction_dynamic.calculateLength(),
			force_interaction_dynamic.calculateDirection().getDegree(),
			force_human_action.calculateLength(),
			force_human_action.calculateDirection().getDegree(),
			force_interaction_static.calculateLength(),
			force_interaction_static.calculateDirection().getDegree(),
			twist_cmd.getX(),
			twist_cmd.getY(),
			twist_cmd.getZ()
		);
	}  // end for simulation steps

	return true;
}

// static
std::vector<double> SocialTrajectoryGenerator::computeAmplifierSamples(
	double amplifier_min,
	double amplifier_max,
	double granularity,
	const std::string& log_identifier
) {
	std::vector<double> samples;
	int num_amps = ceil((amplifier_max - amplifier_min) / granularity);

	// collect samples based on min/max and granulity
	for (int i = 0; i <= num_amps; i++) {
		double amp_value = amplifier_min + granularity * i;
		// check if amp_value does not exceed limits
		if (amp_value > amplifier_max) {
			samples.push_back(amplifier_max);
			break;
		}
		samples.push_back(amp_value);
	}

	if (samples.empty()) {
		ROS_WARN_NAMED(
			"SocTrajGen", "Discarding amplifier samples for %s"
			" (min %3.2f, max %3.2f, granularity %3.2f). Setting to 0.0",
			log_identifier.c_str(),
			amplifier_min,
			amplifier_max,
			granularity
		);
		samples.push_back(0.0);
	}

	return samples;
}

void SocialTrajectoryGenerator::printFisConfiguration() const {
	fuzzy_processor_.printFisConfiguration();
}

bool SocialTrajectoryGenerator::generateTrajectoryWithoutPlanning(base_local_planner::Trajectory& traj) {
	geometry::Vector force_internal;
	geometry::Vector force_interaction_dynamic;
	geometry::Vector force_interaction_static;
	geometry::Vector force_human_action;

	// for how far into the future the trajectory will be applied
	double dt = sim_period_;

	computeForces(
		world_model_,
		dt,
		SampleAmplifierSet(),
		force_internal,
		force_interaction_dynamic,
		force_interaction_static,
		force_human_action,
		true // update motion data
	);

	// force vectors are already multiplied by proper factors
	geometry::Vector twist_cmd;
	computeTwist(
		world_model_.getRobotData().centroid,
		force_internal + force_interaction_dynamic + force_interaction_static + force_human_action,
		world_model_.getRobotData().vel,
		dt,
		robot_mass_,
		limits_planner_ptr_->min_vel_x,
		limits_planner_ptr_->max_vel_x,
		limits_planner_ptr_->max_vel_theta,
		limits_planner_ptr_->twist_rotation_compensation,
		twist_cmd
	);

	// cost will not be evaluated at all
	traj.cost_ = 0.0;
	traj.time_delta_ = dt;
	traj.xv_ = twist_cmd.getX();
	traj.yv_ = twist_cmd.getY();
	traj.thetav_ = twist_cmd.getZ();

	double sampled_speed_linear = std::hypot(twist_cmd.getX(), twist_cmd.getY());
	double sampled_speed_angular = twist_cmd.getZ();
	return areVelocityLimitsFulfilled(sampled_speed_linear, sampled_speed_angular, 1e-4);
}

bool SocialTrajectoryGenerator::areVelocityLimitsFulfilled(
	const double& speed_linear,
	const double& speed_angular,
	const double& eps
) {
	bool min_vel_trans_positive = limits_planner_ptr_->min_vel_trans >= 0;
	bool vel_trans_below_min = (speed_linear + eps) < limits_planner_ptr_->min_vel_trans;
	bool vel_trans_wrong = min_vel_trans_positive && vel_trans_below_min;

	bool min_vel_theta_positive = limits_planner_ptr_->min_vel_theta >= 0;
	bool vel_theta_below_min = (std::abs(speed_angular) + eps) < limits_planner_ptr_->min_vel_theta;
	bool vel_theta_wrong = min_vel_theta_positive && vel_theta_below_min;

	if (vel_trans_wrong && vel_theta_wrong) {
		return false;
	}

	// make sure we do not exceed max diagonal (x+y) translational velocity (if set)
	bool max_vel_trans_positive = limits_planner_ptr_->max_vel_trans >= 0;
	bool vel_trans_above_max = (speed_linear - eps) > limits_planner_ptr_->max_vel_trans;

	if (max_vel_trans_positive && vel_trans_above_max) {
		return false;
	}

	return true;
}

int SocialTrajectoryGenerator::computeStepsNumber(const double& speed_linear, const double& speed_angular) {
	int num_steps;
	if (discretize_by_time_) {
		num_steps = ceil(sim_time_ / sim_granularity_);
	} else {
		// compute the number of steps we must take along this trajectory to be "safe";
		// the distance the robot would travel in sim_time if it did not change velocity
		double sim_time_distance = speed_linear * sim_time_;
		// the angle the robot would rotate in sim_time
		double sim_time_angle = fabs(speed_angular) * sim_time_;
		num_steps =
			ceil(std::max(sim_time_distance / sim_granularity_,
				sim_time_angle    / angular_sim_granularity_));
	}
	return num_steps;
}

void SocialTrajectoryGenerator::computeForces(
	const World& world_model,
	const double& dt,
	const SampleAmplifierSet& sample_amplifiers,
	geometry::Vector& force_internal,
	geometry::Vector& force_interaction_dynamic,
	geometry::Vector& force_interaction_static,
	geometry::Vector& force_human_action,
	bool update_motion_data
) {
	// store vectors of poses of closest points between robot and other objects; makes use out of obstacles
	// representation (extracted from costmap) and robot's footprint;
	// these, in fact, are used only for visualisation
	std::vector<Distance> meaningful_interaction_static;
	std::vector<Distance> meaningful_interaction_dynamic;

	// backup SFM internal parameters, change them and restore after calculations
	double sfm_param_an = sfm_.getParameterAn();
	double sfm_param_bn = sfm_.getParameterBn();
	double sfm_param_cn = sfm_.getParameterCn();
	double sfm_param_ap = sfm_.getParameterAp();
	double sfm_param_bp = sfm_.getParameterBp();
	double sfm_param_cp = sfm_.getParameterCp();
	double sfm_param_aw = sfm_.getParameterAw();
	double sfm_param_bw = sfm_.getParameterBw();
	double sfm_param_speed_desired = sfm_.getParameterDesiredSpeed();
	sfm_.setEquationParameters(
		sfm_param_an * sample_amplifiers.sfm_an_amplifier,
		sfm_param_bn * sample_amplifiers.sfm_bn_amplifier,
		sfm_param_cn * sample_amplifiers.sfm_cn_amplifier,
		sfm_param_ap * sample_amplifiers.sfm_ap_amplifier,
		sfm_param_bp * sample_amplifiers.sfm_bp_amplifier,
		sfm_param_cp * sample_amplifiers.sfm_cp_amplifier,
		sfm_param_aw * sample_amplifiers.sfm_aw_amplifier,
		sfm_param_bw * sample_amplifiers.sfm_bw_amplifier,
		sfm_param_speed_desired * sample_amplifiers.sfm_speed_desired_amplifier
	);

	// same with FIS
	double fis_param_as = social_conductor_.getParameterAs();
	social_conductor_.setEquationParameters(sample_amplifiers.fis_as_amplifier);

	// begin calculations
	sfm_.computeSocialForce(
		world_model,
		dt,
		meaningful_interaction_static,
		meaningful_interaction_dynamic
	);

	// actual `social` vector
	geometry::Vector human_action_force;

	std::vector<StaticObject> objects_static = world_model.getStaticObjectsData();
	std::vector<DynamicObject> objects_dynamic = world_model.getDynamicObjectsData();

	// evaluate whether more complex forces are supposed to be calculated
	if (!sfm_.areInteractionForcesDisabled() && !social_conductor_.areFuzzyBehavioursDisabled()) {
		// All multi-element data are vectors of the same length whose corresponding elements are related
		// to the same \beta object (i.e. i-th index of each vector variable is related to the same \beta
		// object). Note: \beta objects can be considered as obstacles
		//
		// vector of \beta objects direction of motion
		std::vector<double> dir_beta_dynamic;
		// vector of \beta objects' relative to \f$\alpha\f$ locations
		std::vector<double> rel_loc_dynamic;
		// set of dynamic objects vector directions. Each of these vectors connect \f$\alpha\f$ with \beta_i
		std::vector<double> dist_angle_dynamic;
		// set of lengths of vectors connecting \beta -s and \alpha -s
		std::vector<double> dist_dynamic;
		// set of speeds of \beta -s
		std::vector<double> speed_dynamic;
		// \f$\alpha\f$'s direction of motion expressed in world coordinate system
		double dir_alpha = world_model.getRobotData().heading_dir.getRadian();

		for (const DynamicObject& object: objects_dynamic) {
			dir_beta_dynamic.push_back(object.dir_beta.getRadian());
			rel_loc_dynamic.push_back(object.rel_loc_angle.getRadian());
			dist_angle_dynamic.push_back(object.dist_angle.getRadian());
			dist_dynamic.push_back(object.dist);
			speed_dynamic.push_back(object.speed);
		}

		// execute fuzzy operations block
		fuzzy_processor_.process(dir_alpha, dir_beta_dynamic, rel_loc_dynamic, dist_angle_dynamic);

		// create a force vector according to the activated `social behaviour`
		social_conductor_.computeBehaviourForce(
			world_model.getRobotData().centroid,
			world_model.getRobotData().speed,
			fuzzy_processor_.getOutput(),
			speed_dynamic,
			dist_dynamic
		);

		// assign `social` vector
		human_action_force = social_conductor_.getSocialVector();
	}

	force_internal = sfm_.getForceInternal();
	force_interaction_dynamic = sfm_.getForceInteractionDynamic();
	force_interaction_static = sfm_.getForceInteractionStatic();
	force_human_action = social_conductor_.getSocialVector();

	// restore initial values of params
	sfm_.setEquationParameters(
		sfm_param_an,
		sfm_param_bn,
		sfm_param_cn,
		sfm_param_ap,
		sfm_param_bp,
		sfm_param_cp,
		sfm_param_aw,
		sfm_param_bw,
		sfm_param_speed_desired
	);
	social_conductor_.setEquationParameters(fis_param_as);

	if (update_motion_data) {
		diag_force_internal_ = sfm_.getForceInternal();
		diag_force_interaction_dynamic_ = sfm_.getForceInteractionDynamic();
		diag_force_interaction_static_ = sfm_.getForceInteractionStatic();
		diag_force_social_ = social_conductor_.getSocialVector();
		diag_behaviour_active_ = social_conductor_.getBehaviourActive();

		diag_closest_points_static_.clear();
		for (const auto& dist_static: meaningful_interaction_static) {
			diag_closest_points_static_.push_back(dist_static.object);
			diag_closest_points_static_.push_back(dist_static.robot);
		}

		diag_closest_points_dynamic_.clear();
		for (const auto& dist_dynamic: meaningful_interaction_dynamic) {
			diag_closest_points_dynamic_.push_back(dist_dynamic.object);
			diag_closest_points_dynamic_.push_back(dist_dynamic.robot);
		}
	}
}

} // namespace hubero_local_planner
