#include <humap_local_planner/utils/transformations.h>
#include <humap_local_planner/utils/debug.h>

// debugging macros
#define DEBUG_BASIC 0
#define DEBUG_VERBOSE 0
#define DEBUG_WARN 0
#define debug_print_basic(fmt, ...) _template_debug_print_basic_(DEBUG_BASIC, fmt, ##__VA_ARGS__)
#define debug_print_verbose(fmt, ...) _template_debug_print_basic_(DEBUG_VERBOSE, fmt, ##__VA_ARGS__)
#define debug_print_warn(fmt, ...) _template_debug_print_warn_(DEBUG_WARN, fmt, ##__VA_ARGS__)

namespace humap_local_planner {

using namespace geometry;

geometry::Pose computeNextPoseBaseVel(const geometry::Pose& pose, const geometry::Vector& vel, double dt) {
	// NOTE: calculations based on base_local_planner::SimpleTrajectoryGenerator::computeNewPositions
	double new_x = pose.getX() + (vel.getX() * cos(pose.getYaw()) + vel.getY() * cos(M_PI_2 + pose.getYaw())) * dt;
	double new_y = pose.getY() + (vel.getX() * sin(pose.getYaw()) + vel.getY() * sin(M_PI_2 + pose.getYaw())) * dt;
	double new_yaw = pose.getYaw() + vel.getZ() * dt;

	return geometry::Pose(
		new_x,
		new_y,
		pose.getZ(),
		pose.getRoll(),
		pose.getPitch(),
		Angle(new_yaw).getRadian()
	);
}

geometry::Pose computeNextPose(const geometry::Pose& pose, const geometry::Vector& vel, double dt) {
	double new_x = pose.getX() + vel.getX() * dt;
	double new_y = pose.getY() + vel.getY() * dt;
	double new_yaw = pose.getYaw() + vel.getZ() * dt;

	return geometry::Pose(
		new_x,
		new_y,
		pose.getZ(),
		pose.getRoll(),
		pose.getPitch(),
		Angle(new_yaw).getRadian()
	);
}

void computeTwistHolonomic(
	const geometry::Pose& pose,
	const geometry::Vector& force,
	const geometry::Vector& robot_vel_glob,
	const double& dt,
	const double& robot_mass,
	const double& min_vel_x,
	const double& max_vel_x,
	const double& max_rot_vel,
	geometry::Vector& cmd_vel
) {
	std::cout << "humap_local_planner::computeTwistHolonomic not implemented yet" << std::endl;
}

void computeTwist(
	const Pose& pose,
	const Vector& force,
	const Vector& /*robot_vel_glob*/,
	const double& /*dt*/,
	const double& robot_mass,
	const double& min_vel_x,
	const double& max_vel_x,
	const double& max_rot_vel,
	const double& twist_rotation_compensation,
	Vector& cmd_vel
) {
	/*
	 * Compute how much the acceleration (force) affects global velocity of the robot
	 *
	 * Note that this function is used in connection with artificial force field concepts.
	 * We omit `dt` here since the force actually affects robot motion in each time step
	 * \textbf{without implication from the previous time step}, i.e., acceleration directly affects velocity.
	 */

	// no force - no movement (strain physical laws)
	if (force.calculateLength() <= 1e-08 || robot_mass <= 1e-06) {
		cmd_vel = Vector(0.0, 0.0, 0.0);
		return;
	}

	// global force affects global acceleration directly
	Vector acc_v = force / robot_mass;

	/*
	 * Previous implementation multiplied `acc_v * dt`, see humap_local_planner#55 and explanation above
	 * for details
	 */
	Vector vel_v_new = acc_v;

	// orientation in global coordinate system
	double yaw = pose.getYaw();

	/*
	 * Inverted rotation matrix expressed as 2 equations
	 * @url https://github.com/yinzixuan126/modified_dwa/blob/b379c01e37adc1f6414005750633b05e1a024ae5/iri_navigation/iri_akp_local_planner_companion/local_lib/src/scene_elements/robot.cpp#L91
	 */
	// projection to robot pose (dot product)
	double vv = +cos(yaw) * vel_v_new.getX() + sin(yaw) * vel_v_new.getY();
	// cross product theta x f
	double vw = -sin(yaw) * vel_v_new.getX() + cos(yaw) * vel_v_new.getY();

	/*
	 * Optional heuristics to strengthen rotation - provides better turning in crowded/cluttered environments
	 *
	 * Configurable by `twist_rotation_compensation` parameter via `dynamic_reconfigure`
	 */
	Angle force_dir(force);
	Angle ang_z_force_diff(force_dir.getRadian() - yaw);
	vw += twist_rotation_compensation * ang_z_force_diff.getRadian();

	// saturate velocity if the force-velocity conversion violates the constraints
	cmd_vel = saturateVelocity(
		geometry::Vector(vv, 0.0, vw),
		max_vel_x,
		0.0,
		max_vel_x,
		max_rot_vel,
		(min_vel_x < 0.0) ? std::abs(min_vel_x) : 0.0
	);
}

void computeVelocityGlobal(
	const Vector& vel_local,
	const Pose& pose,
	Vector& vel_global
) {
	double yaw = pose.getYaw();

	// A single approach that handles both holonomic and nonholonomic drives
	// Ref: 2nd to last equation in https://automaticaddison.com/how-to-describe-the-rotation-of-a-robot-in-2d/
	//
	// prepare twist expressed in global coordinates
	vel_global = Vector(
		+vel_local.getX() * std::cos(yaw) - vel_local.getY() * std::sin(yaw),
		+vel_local.getX() * std::sin(yaw) + vel_local.getY() * std::cos(yaw),
		+vel_local.getZ()
	);
}

void computeVelocityLocal(
	const Vector& vel_global,
	const Pose& pose,
	Vector& vel_local,
	bool holonomic
) {
	double yaw = pose.getYaw();

	if (holonomic) {
		// The approach below handles both holonomic and nonholonomic drives; however, with this applied to global velocity,
		// vy is often non-zero, e.g., consider that case
		//     computeVelocityLocal([0.165,0.360,0.1],[0.141,0.306,1.220])
		// which results in
		//     [0.3948, -0.0312, 0.1000]
		//
		// Ref: last equation in https://automaticaddison.com/how-to-describe-the-rotation-of-a-robot-in-2d/
		vel_local = Vector(
			+vel_global.getX() * std::cos(yaw) + vel_global.getY() * std::sin(yaw),
			-vel_global.getX() * std::sin(yaw) + vel_global.getY() * std::cos(yaw),
			+vel_global.getZ()
		);
	} else {
		// simplified version that ignores v_y; see note above for details why calc. methods were divided
		vel_local = Vector(
			+vel_global.getX() * std::cos(yaw) + vel_global.getY() * std::sin(yaw),
			0.0,
			vel_global.getZ()
		);
	}
}

geometry::Vector computeVelocityFromPoses(const geometry::Pose& pose1, const geometry::Pose& pose2, double dt) {
	auto displacement = subtractPoses(pose2, pose1);
	// compute vel from pose difference
	auto vel = geometry::Vector(
		displacement.getX() / dt,
		displacement.getY() / dt,
		displacement.getYaw() / dt
	);
	return vel;
}

geometry::Vector computeBaseVelocityFromPoses(const geometry::Pose& pose1, const geometry::Pose& pose2, double dt) {
	auto displacement = subtractPoses(pose2, pose1);
	double theta = pose1.getYaw();
	auto vel = geometry::Vector(
		+displacement.getX() / dt * cos(theta) + displacement.getY() / dt * sin(theta),
		-displacement.getX() / dt * sin(theta) + displacement.getY() / dt * cos(theta),
		+displacement.getYaw() / dt
	);
	return vel;
}

bool adjustTwistWithAccLimits(
	const geometry::Vector& vel,
	const double& acc_lim_x,
	const double& acc_lim_y,
	const double& acc_lim_th,
	const double& vel_min_x,
	const double& vel_min_y,
	const double& vel_min_th,
	const double& vel_max_x,
	const double& vel_max_y,
	const double& vel_max_th,
	const double& sim_period,
	geometry::Vector& cmd_vel,
	bool maintain_vel_components_rate
) {
	double vel_min_x_acc = std::max(vel_min_x, vel.getX() - acc_lim_x * sim_period);
	double vel_min_y_acc = std::max(vel_min_y, vel.getY() - acc_lim_y * sim_period);
	double vel_min_th_acc = std::max(vel_min_th, vel.getZ() - acc_lim_th * sim_period);

	double vel_max_x_acc = std::min(vel_max_x, vel.getX() + acc_lim_x * sim_period);
	double vel_max_y_acc = std::min(vel_max_y, vel.getY() + acc_lim_y * sim_period);
	double vel_max_th_acc = std::min(vel_max_th, vel.getZ() + acc_lim_th * sim_period);

	auto isCmdVelModified = [](
		const geometry::Vector& cmd_vel_init,
		const geometry::Vector& cmd_vel_mod) -> bool
	{
		return cmd_vel_init.getX() != cmd_vel_mod.getX()
			|| cmd_vel_init.getY() != cmd_vel_mod.getY()
			|| cmd_vel_init.getZ() != cmd_vel_mod.getZ();
	};

	geometry::Vector cmd_vel_backup = cmd_vel;

	// (possibly) trim the velocity command according to the velocities feasible given the @ref vel
	if (!maintain_vel_components_rate) {
		cmd_vel = geometry::Vector(
			std::min(std::max(vel_min_x_acc, cmd_vel_backup.getX()), vel_max_x_acc),
			std::min(std::max(vel_min_y_acc, cmd_vel_backup.getY()), vel_max_y_acc),
			std::min(std::max(vel_min_th_acc, cmd_vel_backup.getZ()), vel_max_th_acc)
		);
		return isCmdVelModified(cmd_vel_backup, cmd_vel);
	}

	// if any velocity component needs to be modified, change other ones proportionally
	cmd_vel = adjustTwistProportional(
		vel,
		cmd_vel_backup,
		vel_min_x_acc,
		vel_min_y_acc,
		vel_min_th_acc,
		vel_max_x_acc,
		vel_max_y_acc,
		vel_max_th_acc
	);
	return isCmdVelModified(cmd_vel_backup, cmd_vel);
}

bool adjustTwistWithAccAndGoalLimits(
	const geometry::Vector& vel,
	const double& acc_lim_x,
	const double& acc_lim_y,
	const double& acc_lim_th,
	const double& vel_min_x,
	const double& vel_min_y,
	const double& vel_min_th,
	const double& vel_max_x,
	const double& vel_max_y,
	const double& vel_max_th,
	const double& sim_period,
	geometry::Vector& cmd_vel,
	bool maintain_vel_components_rate,
	double dist_to_goal
) {
	// Modified version of code from base_local_planner::SimpleTrajectoryGenerator::initialise
	// authored by Thibault Kruse
	// https://github.com/ros-planning/navigation/blob/noetic-devel/base_local_planner/src/simple_trajectory_generator.cpp#L94
	// "we limit the velocities to those that do not overshoot the goal in **sim_time**"
	//
	// formulation below is a transformed equation of a 'constantly accelerated motion' - we know the distance
	// within which the robot must be stopped so we compute the maximum velocity that allows to do so
	double acc_lim_decel = std::hypot(acc_lim_x, acc_lim_y);
	// maximum initial speed allowing to stop after dist_to_goal given a deceleration rate of "acc_lim_decel"
	double speed_init_max = std::sqrt(2.0 * acc_lim_decel * dist_to_goal);

	// to trim both velocity components proportionally
	double vel_vector_angle = 0.0;
	// NOTE: when the robot is stopped, X might be slightly negative and thus, this procedure generates cmd_vel
	// that always forces backward motions;
	// To properly handle this edge case, at least one of the linear component should be significant
	if (std::abs(vel.getX()) >= 1e-04 || std::abs(vel.getY()) >= 1e-04) {
		vel_vector_angle = std::atan2(vel.getY(), vel.getX());
	}
	// updated velocity limits that prevent from overshooting the goal pose
	double vx_safe = std::cos(vel_vector_angle) * speed_init_max;
	double vy_safe = std::sin(vel_vector_angle) * speed_init_max;

	// original kinematic limits related to max. velocity may be reduced to avoid overshooting the goal pose
	double vel_max_x_corrected = std::max(std::min(vel_max_x, vx_safe), vel_min_x);
	double vel_max_y_corrected = std::max(std::min(vel_max_y, vy_safe), vel_min_y);

	return adjustTwistWithAccLimits(
		vel,
		acc_lim_x,
		acc_lim_y,
		acc_lim_th,
		vel_min_x,
		vel_min_y,
		vel_min_th,
		vel_max_x_corrected,
		vel_max_y_corrected,
		vel_max_th,
		sim_period,
		cmd_vel,
		maintain_vel_components_rate
	);
}

geometry::Pose subtractPoses(const geometry::Pose& pose_ref, const geometry::Pose& pose_other) {
	return Pose(
		pose_ref.getX() - pose_other.getX(),
		pose_ref.getY() - pose_other.getY(),
		pose_ref.getZ() - pose_other.getZ(),
		pose_ref.getRoll() - pose_other.getRoll(),
		pose_ref.getPitch() - pose_other.getPitch(),
		pose_ref.getYaw() - pose_other.getYaw()
	);
}

geometry::Pose addPoses(const geometry::Pose& pose_ref, const geometry::Pose& pose_other) {
	return Pose(
		pose_ref.getX() + pose_other.getX(),
		pose_ref.getY() + pose_other.getY(),
		pose_ref.getZ() + pose_other.getZ(),
		pose_ref.getRoll() + pose_other.getRoll(),
		pose_ref.getPitch() + pose_other.getPitch(),
		pose_ref.getYaw() + pose_other.getYaw()
	);
}

geometry::Vector saturateVelocity(
	const geometry::Vector& cmd_vel,
	double max_vel_x,
	double max_vel_y,
	double max_vel_trans,
	double max_vel_theta,
	double max_vel_x_backwards
) {
	double ratio_x = 1.0;
	double ratio_omega = 1.0;
	double ratio_y = 1.0;

	double vx = cmd_vel.getX();
	double vy = cmd_vel.getY();
	double omega = cmd_vel.getZ();

	// Limit translational velocity for forward driving
	if (vx > max_vel_x) {
		ratio_x = max_vel_x / vx;
	}

	// limit strafing velocity
	if (vy > max_vel_y || vy < -max_vel_y) {
		ratio_y = std::abs(vy / max_vel_y);
	}

	// Limit angular velocity
	if (omega > max_vel_theta || omega < -max_vel_theta) {
		ratio_omega = std::abs(max_vel_theta / omega);
	}

	// Limit backwards velocity
	if (vx < -std::abs(max_vel_x_backwards)) {
		ratio_x = -std::abs(max_vel_x_backwards) / vx;
	}

	// TEB sets use_proportional_saturation to false by default, therefore appropriate version applied
	vx *= ratio_x;
	vy *= ratio_y;
	omega *= ratio_omega;

	double vel_linear = std::hypot(vx, vy);
	if (vel_linear > max_vel_trans) {
		double max_vel_trans_ratio = max_vel_trans / vel_linear;
		vx *= max_vel_trans_ratio;
		vy *= max_vel_trans_ratio;
	}
	return geometry::Vector(vx, vy, omega);
}

geometry::Vector adjustTwistProportional(
	const geometry::Vector& vel,
	const geometry::Vector& cmd_vel,
	double min_vel_x,
	double min_vel_y,
	double min_vel_th,
	double max_vel_x,
	double max_vel_y,
	double max_vel_th
) {
	// find feasibility factor of the velocity component, i.e., the percentage of the commanded velocity that can be
	// achieved
	auto findFeasVelComponent = [&](
		double vel_curr,
		double vel_cmd,
		double vel_lim_min,
		double vel_lim_max) -> std::pair<double, double>
	{
		double vel_cmd_diff = vel_cmd - vel_curr;
		if (vel_cmd >= vel_curr) {
			// CASE1 - speeding up along <SOME> direction
			double vel_lim_diff = vel_lim_max - vel_curr;
			// defines a percentage of the velocity delta that is feasible
			double vel_feas_factor = vel_lim_diff / vel_cmd_diff;
			return std::make_pair(vel_feas_factor, vel_cmd_diff);
		}
		// CASE2 - slowing down along <SOME> direction
		double vel_lim_diff = vel_lim_min - vel_curr;
		// defines a percentage of the velocity delta that is feasible
		double vel_feas_factor = vel_lim_diff / vel_cmd_diff;
		return std::make_pair(vel_feas_factor, vel_cmd_diff);
	};

	auto vel_cmd_feas_x = findFeasVelComponent(vel.getX(), cmd_vel.getX(), min_vel_x, max_vel_x);
	auto vel_cmd_feas_y = findFeasVelComponent(vel.getY(), cmd_vel.getY(), min_vel_y, max_vel_y);
	auto vel_cmd_feas_th = findFeasVelComponent(vel.getZ(), cmd_vel.getZ(), min_vel_th, max_vel_th);

	std::vector<double> vel_feasibility_factors = {vel_cmd_feas_x.first, vel_cmd_feas_y.first, vel_cmd_feas_th.first};
	auto feas_factor_min = *std::min_element(vel_feasibility_factors.begin(), vel_feasibility_factors.end());

	// evaluate if trimming 2/3 of components is needed (it is when feas_factor_min < 1.0 since one component cannot
	// be set to the requested value)
	if (std::isnan(feas_factor_min) || feas_factor_min >= 1.0) {
		// velocities within limits - correction is not required (no multiplication)
		return geometry::Vector(
			vel.getX() + vel_cmd_feas_x.second,
			vel.getY() + vel_cmd_feas_y.second,
			vel.getZ() + vel_cmd_feas_th.second
		);
	}

	// correction - commanded velocity exceeded limits (multiplication required)
	// NOTE: trimming to velocity limits is not required here as the factors from findFeasVelComponent already account
	// for feasible velocity bounds
	return geometry::Vector(
		vel.getX() + vel_cmd_feas_x.second * feas_factor_min,
		vel.getY() + vel_cmd_feas_y.second * feas_factor_min,
		vel.getZ() + vel_cmd_feas_th.second * feas_factor_min
	);
}

} // namespace humap_local_planner
