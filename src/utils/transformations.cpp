#include <hubero_local_planner/utils/transformations.h>
#include <hubero_local_planner/utils/debug.h>

// debugging macros
#define DEBUG_BASIC 0
#define DEBUG_VERBOSE 0
#define DEBUG_WARN 0
#define debug_print_basic(fmt, ...) _template_debug_print_basic_(DEBUG_BASIC, fmt, ##__VA_ARGS__)
#define debug_print_verbose(fmt, ...) _template_debug_print_basic_(DEBUG_VERBOSE, fmt, ##__VA_ARGS__)
#define debug_print_warn(fmt, ...) _template_debug_print_warn_(DEBUG_WARN, fmt, ##__VA_ARGS__)

namespace hubero_local_planner {

using namespace geometry;

geometry::Pose computeNextPoseBaseVel(const geometry::Pose& pose, const geometry::Vector& vel, double dt) {
	// NOTE: calculations based on base_local_planner::SimpleTrajectoryGenerator::computeNewPositions
	geometry::Pose pose_new;
	double new_x = pose.getX() + (vel.getX() * cos(pose.getYaw()) + vel.getY() * cos(M_PI_2 + pose.getYaw())) * dt;
	double new_y = pose.getY() + (vel.getX() * sin(pose.getYaw()) + vel.getY() * sin(M_PI_2 + pose.getYaw())) * dt;
	double new_yaw = pose.getYaw() + vel.getZ() * dt;

	pose_new.setPosition(new_x, new_y, pose.getZ());
	pose_new.setOrientation(pose.getRoll(), pose.getPitch(), Angle(new_yaw).getRadian());
	return pose_new;
}

geometry::Pose computeNextPose(const geometry::Pose& pose, const geometry::Vector& vel, double dt) {
	geometry::Pose pose_new;
	double new_x = pose.getX() + vel.getX() * dt;
	double new_y = pose.getY() + vel.getY() * dt;
	double new_yaw = pose.getYaw() + vel.getZ() * dt;

	pose_new.setPosition(new_x, new_y, pose.getZ());
	pose_new.setOrientation(pose.getRoll(), pose.getPitch(), Angle(new_yaw).getRadian());
	return pose_new;
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
	std::cout << "hubero_local_planner::computeTwistHolonomic not implemented yet" << std::endl;
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
	 * Previous implementation multiplied `acc_v * dt`, see hubero_local_planner#55 and explanation above
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

	// logging section
	debug_print_verbose("force - x: %2.3f, y: %2.3f, th: %2.3f, mass - %2.3f\r\n",
		force.getX(), force.getY(), force.getZ(), robot_mass
	);

	debug_print_verbose("accel - x: %2.3f, y: %2.3f, th: %2.3f, vel - x: %2.3f, y: %2.3f, th: %2.3f\r\n",
		acc_v.getX(), acc_v.getY(), acc_v.getZ(),
		vel_v_new.getX(), vel_v_new.getY(), vel_v_new.getZ()
	);

	debug_print_verbose("cmd_vel 'init'  - lin.x: %2.3f, ang.z: %2.3f\r\n", vv, vw);

	// hard non-linearities section:
	// check if within limits: try to maintain path, ignoring trajectory
	if (vv > max_vel_x || vw > max_rot_vel) {
		double excess_x = vv / max_vel_x;
		double excess_z = vw / max_rot_vel;
		double shortening_factor = 1 / std::max(excess_x, excess_z);
		// shorten proportionally
		vv *= shortening_factor;
		vw *= shortening_factor;
		debug_print_warn(
			"Requested velocity not within limits! excess_x %2.3f, excess_z %2.3f, multiplier %2.3f "
			"cmd_vel 'modded' lin.x: %2.3f, ang.z: %2.3f\r\n",
			excess_x, excess_z, shortening_factor, vv, vw
		);
	}

	// trim minimum velocity to `min_vel_x`
	if (vv < min_vel_x) {
		vv = min_vel_x;
		debug_print_warn("Requested velocity is smaller than min! 'modded' lin.x %2.3f\r\n", vv);
	}

	// assign final values
	cmd_vel.setX(vv);
	cmd_vel.setY(0.0);
	cmd_vel.setZ(vw);

	debug_print_verbose(
		"cmd_vel 'final' - lin.x: %2.3f (min %2.3f, max %2.3f), ang.z: %2.3f (max %2.3f)\r\n",
		cmd_vel.getX(), min_vel_x, max_vel_x, cmd_vel.getZ(), max_rot_vel
	);
}

void computeVelocityGlobal(
	const Vector& vel_local,
	const Pose& pose,
	Vector& vel_global
) {
	// slide 38 at https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture3-RobotMotion.pdf
	double yaw = pose.getYaw();
	/* Previously, Eigen Matrix was used, but switched to simplier approach for optimization:
	 *
	 * Eigen::Matrix3d rotation_yaw_inv;
	 * // create a rotation matrix
	 * rotation_yaw_inv <<
	 * 	cos(yaw), 0, 0,
	 * 	sin(yaw), 0, 0,
	 * 	0, 0, 1;
	 * // compute global velocity vector
	 * Eigen::Vector3d vel_global_eigen = rotation_yaw_inv * vel_local.getAsEigen<Eigen::Vector3d>();
	 * vel_global = Vector(vel_global_eigen);
	 *
	 */
	// prepare twist expressed in global coordinates
	vel_global = Vector(
		std::cos(yaw) * vel_local.getX(),
		std::sin(yaw) * vel_local.getX(),
		vel_local.getZ()
	);
}

void computeVelocityLocal(
	const Vector& vel_global,
	const Pose& pose,
	Vector& vel_local
) {
	double yaw = pose.getYaw();
	/* Previously, Eigen Matrix was used, but switched to simplier approach for optimization:
	 *
	 * Eigen::Matrix3d rotation_yaw;
	 * // create a rotation matrix for nonholonomic, https://www.cs.cmu.edu/~rasc/Download/AMRobots3.pdf (3.2)
	 * // pseudoinverted matrix above
	 * rotation_yaw <<
	 * 	cos(yaw), sin(yaw), 0,
	 * 	0, 0, 0,
	 * 	0, 0, 1;
	 * // compute local velocity vector
	 * Eigen::Vector3d vel_local_eigen = rotation_yaw * vel_global.getAsEigen<Eigen::Vector3d>();
	 * // prepare twist expressed in local coordinates
	 * vel_local = Vector(vel_local_eigen);
	 *
	 */
	vel_local = Vector(
		std::cos(yaw) * vel_global.getX() + std::sin(yaw) * vel_global.getY(),
		0.0,
		vel_global.getZ()
	);
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
	geometry::Vector& cmd_vel
) {
	double vel_min_x_acc = std::max(vel_min_x, vel.getX() - acc_lim_x * sim_period);
	double vel_min_y_acc = std::max(vel_min_y, vel.getY() - acc_lim_y * sim_period);
	double vel_min_th_acc = std::max(vel_min_th, vel.getZ() - acc_lim_th * sim_period);

	double vel_max_x_acc = std::min(vel_max_x, vel.getX() + acc_lim_x * sim_period);
	double vel_max_y_acc = std::min(vel_max_y, vel.getY() + acc_lim_y * sim_period);
	double vel_max_th_acc = std::min(vel_max_th, vel.getZ() + acc_lim_th * sim_period);

	// if any component is modified, change other ones proportionally
	geometry::Vector cmd_vel_backup = cmd_vel;
	cmd_vel.setX(std::min(std::max(vel_min_x_acc, cmd_vel_backup.getX()), vel_max_x_acc));
	cmd_vel.setY(std::min(std::max(vel_min_y_acc, cmd_vel_backup.getY()), vel_max_y_acc));
	cmd_vel.setZ(std::min(std::max(vel_min_th_acc, cmd_vel_backup.getZ()), vel_max_th_acc));

	double vel_multiplier = std::min({
		std::abs(cmd_vel.getX() / cmd_vel_backup.getX()),
		std::abs(cmd_vel.getY() / cmd_vel_backup.getY()),
		std::abs(cmd_vel.getZ() / cmd_vel_backup.getZ())
	});
	// check if any component was modified
	if (vel_multiplier >= 1.0) {
		// no action required - acceleration within bounds
		return false;
	}

	// proportionally modify intial cmd_vel to command that is achievable within `dt`
	cmd_vel.setX(vel_multiplier * cmd_vel_backup.getX());
	cmd_vel.setY(vel_multiplier * cmd_vel_backup.getY());
	cmd_vel.setZ(vel_multiplier * cmd_vel_backup.getZ());

	// makes sure that any not investigated 'sign changes' are included and acceleration limits are satisfied
	cmd_vel.setX(std::min(std::max(vel_min_x_acc, cmd_vel.getX()), vel_max_x_acc));
	cmd_vel.setY(std::min(std::max(vel_min_y_acc, cmd_vel.getY()), vel_max_y_acc));
	cmd_vel.setZ(std::min(std::max(vel_min_th_acc, cmd_vel.getZ()), vel_max_th_acc));
	return true;
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

} // namespace hubero_local_planner
